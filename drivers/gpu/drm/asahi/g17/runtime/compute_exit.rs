// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Process-exit controls for closed contexts whose compute commands remain owned.

use super::{now_ns, DeferredBatch};
use crate::g17::{
    channel::Rings,
    context::Context,
    fw::channels::{ContextKill, ControlRecord},
    queue::compute::exit,
    recovery::Memory as _,
    Firmware, MSG_CONTROL_NOTIFY,
};
use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use kernel::{prelude::*, sync::Arc};

impl super::Registry {
    /// ACK storage belongs to the already-retained transaction, so a full event
    /// batch cannot overflow a separate cache. No allocation or wait is needed.
    pub(super) fn acknowledge_compute_exit(&mut self, cookie: u64) {
        for entry in self.compute.iter_mut().flatten() {
            if let Some(queue) = entry.queue.as_deref_mut() {
                if queue.acknowledge_exit(cookie) {
                    return;
                }
            }
        }
    }
}

/// Log-only (G-31) state: when each QID was first seen stuck (0: not stuck), which QIDs
/// were logged, and the lines logged this boot.
const NOT_STUCK: AtomicU64 = AtomicU64::new(0);
static STUCK_SINCE: [AtomicU64; 128] = [NOT_STUCK; 128];
static STUCK_LOGGED: [AtomicU64; 2] = [NOT_STUCK; 2];
static STUCK_LINES: AtomicU32 = AtomicU32::new(0);
/// A queue must stay stuck this long before it is logged, so a kill in progress is not.
const STUCK_LOG_AFTER_NS: u64 = 5_000_000_000;
const STUCK_MAX_LINES: u32 = 16;

impl Firmware {
    /// Log-only (G-31): once per QID, name the gate that keeps a quarantined queue of a
    /// closed owner from being killed or settled. Reads the same state the kill decision
    /// reads under the device mutex; changes nothing.
    fn log_stuck_compute(&self, recovery_closed: bool, now: u64) {
        if STUCK_LINES.load(Ordering::Relaxed) >= STUCK_MAX_LINES {
            return;
        }
        for queue in self
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            let qid = usize::from(queue.qid()) % 128;
            let since = &STUCK_SINCE[qid];
            if !queue.stuck_quarantined() {
                since.store(0, Ordering::Relaxed);
                continue;
            }
            let first = since.load(Ordering::Relaxed);
            if first == 0 {
                since.store(now.max(1), Ordering::Relaxed);
                continue;
            }
            let bit = 1u64 << (qid % 64);
            if now.saturating_sub(first) < STUCK_LOG_AFTER_NS
                || STUCK_LOGGED[qid / 64].load(Ordering::Relaxed) & bit != 0
            {
                continue;
            }
            STUCK_LOGGED[qid / 64].fetch_or(bit, Ordering::Relaxed);
            let n = STUCK_LINES.fetch_add(1, Ordering::Relaxed) + 1;
            if n > STUCK_MAX_LINES {
                return;
            }
            let (reason, scope) = self.stuck_reason(queue, recovery_closed, now);
            queue.log_stuck(n, reason, scope);
        }
    }

    /// Mirrors `service_compute_exit` and `publish_compute_exit` in order; the first gate
    /// that refuses wins. Returns the scheduler page's QID scope once it was read.
    fn stuck_reason(
        &self,
        queue: &crate::g17::queue::compute::Queue,
        recovery_closed: bool,
        now: u64,
    ) -> (&'static str, u128) {
        if self.recovery.pending() || !recovery_closed {
            return ("GPU recovery open", 0);
        }
        let Some(context) = queue.context() else {
            return ("owner context detached", 0);
        };
        let Some(age) = (0..self.queues.teardown.len()).find_map(|index| {
            self.queues
                .teardown
                .closed(index, now)
                .filter(|(closed, _)| Arc::ptr_eq(closed, context))
                .map(|(_, age)| age)
        }) else {
            return ("context not draining in teardown", 0);
        };
        match self.queues.render.teardown_idle(context) {
            Ok(true) => {}
            Ok(false) => return ("render side of the context still busy", 0),
            Err(_) => return ("render idle check failed", 0),
        }
        if !self.queues.render.teardown_pools_released(context) {
            return ("render pools of the context not released", 0);
        }
        if let Some(reason) = queue.exit_refusal(context, age) {
            return (reason, 0);
        }
        if self.render_control_backpressured() {
            return ("render control ring backpressured", 0);
        }
        if queue.exit_owner(context).is_none() {
            return ("commands of several contexts", 0);
        }
        let (mut own, mut foreign, _) = self.queues.render.exit_scope(context);
        for other in self
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
        {
            let bit = 1u128 << other.qid();
            if other
                .context()
                .is_some_and(|bound| Arc::ptr_eq(bound, context))
            {
                own |= bit;
            } else {
                foreign |= bit;
            }
        }
        let Ok(state) = exit::State::read(context) else {
            return ("scheduler page unreadable", 0);
        };
        if state.scope(queue.qid(), own, foreign).is_none() {
            return ("kill scope names QIDs the context does not own", state.qids);
        }
        (
            "kill allowed; render slot hold or publication refused",
            state.qids,
        )
    }

    pub(in crate::g17) fn service_compute_exit(&mut self, deferred: &mut DeferredBatch) -> Result {
        let recovery_closed = self.init.recovery_state()? == 0;
        let view = crate::g17::qos::View::new(self.init.qos()?)?;
        for entry in self.queues.compute.iter_mut().flatten() {
            if let Some(queue) = entry.queue.as_deref_mut() {
                queue.settle_exit(
                    recovery_closed,
                    &mut |owner| self.queues.accounting.complete(&view, &[owner]),
                    &mut |event| deferred.push(event),
                )?;
            }
        }
        self.log_stuck_compute(recovery_closed, now_ns());
        if self.recovery.pending() || !recovery_closed {
            return Ok(());
        }
        let now = now_ns();
        // Publication has already retained each kill. Retry only the doorbell;
        // failed attempts do not extend its bounded polling window.
        for entry in self.queues.compute.iter_mut().flatten() {
            if let Some(queue) = entry.queue.as_deref_mut() {
                let _ = queue.announce_exit(now, || self.primary.notify(MSG_CONTROL_NOTIFY));
            }
        }
        for index in 0..self.queues.teardown.len() {
            let Some((context, age)) = self.queues.teardown.closed(index, now) else {
                continue;
            };
            if !self.queues.render.teardown_idle(&context)?
                || !self.queues.render.teardown_pools_released(&context)
            {
                continue;
            }
            if let Err(error) = self.publish_compute_exit(&context, age, now) {
                dev_err!(
                    self.primary.state.shared.dev.as_ref(),
                    "Scheduler {} exit request failed: {:?}\n",
                    context.id(),
                    error
                );
            }
        }
        Ok(())
    }

    fn publish_compute_exit(&mut self, context: &Arc<Context>, age: u64, now: u64) -> Result {
        let Some(index) = self.queues.compute.iter().position(|entry| {
            entry
                .as_ref()
                .and_then(|entry| entry.queue.as_deref())
                .is_some_and(|queue| queue.exit_candidate(context, age))
        }) else {
            return Ok(());
        };
        if self.render_control_backpressured() {
            return Ok(());
        }
        let queue = self.queues.compute[index]
            .as_ref()
            .and_then(|entry| entry.queue.as_deref())
            .ok_or(EIO)?;
        let Some(owner) = queue.exit_owner(context) else {
            return Ok(());
        };
        let target = queue.qid();
        let (mut own, mut foreign, slots) = self.queues.render.exit_scope(context);
        for entry in self.queues.compute.iter().flatten() {
            if let Some(queue) = entry.queue.as_deref() {
                let bit = 1u128 << queue.qid();
                if queue.context().is_some_and(|bound| Arc::ptr_eq(bound, context)) {
                    own |= bit;
                } else {
                    foreign |= bit;
                }
            }
        }
        let state = exit::State::read(context)?;
        let Some(qids) = state.scope(target, own, foreign) else {
            return Ok(());
        };
        let mut hold = 0u64;
        for slot in 0..crate::hw::t8140::queues::RENDER_SLOTS as u8 {
            if slots & (1u64 << slot) != 0 && self.queues.render.qids_for_slot(slot) & qids != 0 {
                hold |= 1u64 << slot;
            }
        }
        let cookie = exit::next_cookie()?;
        if hold != 0
            && !self
                .queues
                .admission
                .hold_render_slots_for_exit_kill(owner, hold)
        {
            return Ok(());
        }
        let record = ControlRecord::ContextKill(ContextKill::new(context.scheduler_va(), cookie));
        if let Err(error) = Rings::publish_control(&self.init, &record) {
            self.queues.admission.release_exit_kill_hold(hold);
            return if error == EAGAIN { Ok(()) } else { Err(error) };
        }
        // Publication is irrevocable even if its doorbell fails. Keep both
        // the killed graph and the held render identities until processor stop.
        let queue = self.queues.compute[index]
            .as_mut()
            .and_then(|entry| entry.queue.as_deref_mut())
            .ok_or(EIO)?;
        queue.record_exit(cookie, now);
        fence(Ordering::SeqCst);
        if let Err(error) = queue.announce_exit(now, || self.primary.notify(MSG_CONTROL_NOTIFY)) {
            dev_warn!(
                self.primary.state.shared.dev.as_ref(),
                "Scheduler {} exit announcement failed: {:?}\n",
                context.id(),
                error
            );
        }
        Ok(())
    }

    pub(in crate::g17) fn compute_exit_polling(&self, now: u64) -> bool {
        if self
            .queues
            .compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_deref())
            .any(|queue| queue.exit_polling(now))
        {
            return true;
        }
        (0..self.queues.teardown.len()).any(|index| {
            self.queues
                .teardown
                .closed(index, now)
                .is_some_and(|(context, age)| {
                    age < exit::POLL_NS
                        && self
                            .queues
                            .compute
                            .iter()
                            .flatten()
                            .filter_map(|entry| entry.queue.as_deref())
                            .any(|queue| queue.exit_candidate(&context, u64::MAX))
                })
        })
    }
}
