// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! VM admission errors and client failure reporting. No firmware state is accessed here.

use crate::gem;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use kernel::{new_mutex, prelude::*, str::BStr, sync::Mutex, task::Pid};

const MIRROR_MAX_SIZE: usize = 64 * 1024;

/// A VM and its accepted jobs share this status, including after queue destruction.
/// Client failure is separate from hardware admission: recovery may preserve the
/// VM while failed accepted work must still be reported before signalling fences.
#[pin_data]
pub(crate) struct VmStatus {
    error: AtomicI32,
    reported_error: AtomicI32,
    /// The client's VM handle is gone (`release_mirror`).
    closed: AtomicBool,
    /// The process that created the VM, named in the failure report.
    tgid: Pid,
    comm: [u8; 16],
    /// The client's mapped status word, retained with its object only while the
    /// client's VM handle exists; released by `release_mirror` at VM close.
    #[pin]
    mirror: Mutex<Option<Mirror>>,
}

/// One aligned word inside the retained, vmapped mirror object.
struct Mirror {
    word: *mut i32,
    _owner: gem::ObjectRef,
}
// SAFETY: The word points into the mapping the retained owner keeps alive; it is
// only accessed atomically through this pointer while the mutex is held.
unsafe impl Send for Mirror {}

impl VmStatus {
    /// Called from the creating client's ioctl: the VM records who it belongs to.
    pub(crate) fn new() -> impl PinInit<Self> {
        let task = kernel::current!();
        // SAFETY: `comm` is a fixed array inside the live current task. A concurrent rename by
        // another thread of the process may tear the copy, as with C's unlocked `current->comm`.
        let comm: [u8; 16] = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!((*task.as_ptr()).comm).cast())
        };
        let tgid = task.tgid();
        pin_init!(Self {
            error: AtomicI32::new(0),
            reported_error: AtomicI32::new(0),
            closed: AtomicBool::new(false),
            tgid,
            comm,
            mirror <- new_mutex!(None, "Asahi VM status mirror"),
        })
    }

    pub(crate) fn get(&self) -> i32 {
        self.error.load(Ordering::Acquire)
    }

    /// Record permanent admission failure and report it to the client.
    #[track_caller]
    pub(crate) fn record(&self, error: Error) {
        self.report_failure(error);
        let _ = self.error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
    }

    pub(crate) fn reported_error(&self) -> i32 {
        self.reported_error.load(Ordering::Acquire)
    }

    /// Report failed accepted work without preventing recovery of the VM.
    /// Every reporter publishes the first error before returning: a losing
    /// reporter may signal its fence before the winning reporter resumes.
    ///
    /// The first failure reported to a VM is logged once, with the reporting site: the client treats
    /// any reported failure as a lost device, and nothing else in the kernel log names it.
    #[track_caller]
    pub(crate) fn report_failure(&self, error: Error) {
        let error = match self.reported_error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        ) {
            Ok(_) => {
                // ENOENT after close is accepted work of a destroyed VM being cancelled: no
                // client is left to see it.
                if !(error == ENOENT && self.closed.load(Ordering::Acquire)) {
                    let at = core::panic::Location::caller();
                    let len = self.comm.iter().position(|&b| b == 0).unwrap_or(self.comm.len());
                    pr_warn!(
                        "GPU work of a client failed ({:?}, reported at {}:{}, client {}[{}]); the client sees a lost device\n",
                        error,
                        at.file().rsplit("asahi/").next().unwrap_or(at.file()),
                        at.line(),
                        BStr::from_bytes(&self.comm[..len]),
                        self.tgid
                    );
                }
                error.to_errno()
            }
            Err(first) => first,
        };
        let mirror = self.mirror.lock();
        if let Some(mirror) = mirror.as_ref() {
            // SAFETY: attach_mirror stored only an aligned word within a mapping the
            // retained owner keeps alive while this entry exists under the mutex.
            unsafe { AtomicI32::from_ptr(mirror.word) }.store(error, Ordering::SeqCst);
        }
    }

    /// Rejected submissions and contention leave the VM usable.
    #[track_caller]
    pub(crate) fn record_if_device_loss(&self, error: Error) {
        if error == EIO || error == ETIMEDOUT || error == ENODEV {
            self.record(error);
        }
    }

    /// Retain one mapped GEM word until the client's VM handle is dropped.
    pub(crate) fn attach_mirror(&self, mut owner: gem::ObjectRef, offset: usize) -> Result {
        if owner.size() > MIRROR_MAX_SIZE
            || offset % core::mem::align_of::<i32>() != 0
            || offset
                .checked_add(core::mem::size_of::<i32>())
                .is_none_or(|end| end > owner.size())
        {
            return Err(EINVAL);
        }
        let base = owner.vmap()?.addr.cast::<u8>();
        // SAFETY: The checked range is inside the retained, page-aligned mapping.
        let word = unsafe { base.add(offset) }.cast::<i32>();
        let mut mirror = self.mirror.lock();
        if mirror.is_some() {
            return Err(EBUSY);
        }
        // No reporter runs while the mutex is held, so the word holds the first
        // reported error (or zero) when it becomes visible.
        let error = self.reported_error.load(Ordering::SeqCst);
        // SAFETY: The retained owner keeps this checked, aligned word mapped.
        unsafe { AtomicI32::from_ptr(word) }.store(error, Ordering::SeqCst);
        *mirror = Some(Mirror {
            word,
            _owner: owner,
        });
        Ok(())
    }

    /// The client's VM handle is gone and nobody reads the mirror any more. The
    /// mapping and object reference drop outside the mutex (GEM release may sleep).
    pub(crate) fn release_mirror(&self) {
        self.closed.store(true, Ordering::Release);
        let taken = self.mirror.lock().take();
        drop(taken);
    }
}
