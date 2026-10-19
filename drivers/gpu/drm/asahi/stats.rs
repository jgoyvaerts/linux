// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Snapshot of AGX firmware statistics.
//!
//! [`StatsSnapshot`] accumulates the values decoded by `StatsChannel::poll`
//! (see `fw::channels.rs::StatsMsg`) and exposes them to the sysfs file
//! `agx_stats` (registered from `sysfs.c`).
//!
//! The snapshot belongs to one GPU manager. The firmware-stat
//! decoder that feeds it lives in `StatsChannel::poll` (`channel.rs`), inside
//! the versioned context where `StatsMsg` is nameable; `note_job()` is called
//! from the `recv_message` rtkit callback (so the firmware mailbox IRQ
//! context); the sysfs `show` callback runs in arbitrary process context. All
//! fields use `AtomicU*` so neither side takes a lock.
//!
//! Field semantics (firmware units, late-validated):
//! - `util1..util4`: raw `Utilization` u32s from the firmware. Meaning
//!   (firmware tick or %?) is validated against a controlled MLX load; we
//!   publish them unscaled per the producer contract.
//! - `pstate`: current performance state index from `PowerState`.
//! - `avg_power_mw`: average power in milliwatts from `AvgPower` (firmware
//!   field is u32 — units not validated for every chip rev; treat as raw).
//! - `temperature`: raw `Temperature` value, scale, tmin, tmax.
//! - `busy_ns`: cumulative utilization-weighted busy time in nanoseconds,
//!   derived from successive `Utilization` windows (busiest subqueue
//!   percentage x window duration; the firmware timestamps are base-clock
//!   ticks, 24 MHz, converted with `HwConfig::base_clock_hz`). Monotonic and
//!   bounded by elapsed time. Saturates at u64::MAX.
//! - `jobs`: completed submissions since boot, counted at submission
//!   completion (`JobFence::command_complete`).
//!
//! This module is read-only with respect to scheduling or power behaviour:
//! the firmware already sends the messages, the channel already polls them,
//! we only retain the latest values.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Per-device snapshot of AGX firmware stats.
///
/// The manager and bound driver retain Arcs. The driver's Arc outlives the
/// per-device C sysfs registration; repr(C) establishes the shared layout.
#[derive(Default)]
#[repr(C)]
pub(crate) struct StatsSnapshot {
    pub(crate) util1: AtomicU32,
    pub(crate) util2: AtomicU32,
    pub(crate) util3: AtomicU32,
    pub(crate) util4: AtomicU32,
    pub(crate) pstate: AtomicU32,
    pub(crate) avg_power_mw: AtomicU32,
    pub(crate) temperature_raw: AtomicU32,
    pub(crate) temperature_scale: AtomicU32,
    pub(crate) temperature_tmin: AtomicU32,
    pub(crate) temperature_tmax: AtomicU32,
    /// Cumulative utilization-weighted busy time.
    pub(crate) busy_ns: AtomicU64,
    /// Completed submissions since boot, counted at fence signal in
    /// `JobFence::command_complete` when the last command of a submission
    /// completes.
    pub(crate) jobs: AtomicU64,
}

// Keep the Rust half of the C sysfs layout contract checked as well.
const _: () = {
    assert!(core::mem::offset_of!(StatsSnapshot, pstate) == 16);
    assert!(core::mem::offset_of!(StatsSnapshot, busy_ns) == 40);
    assert!(core::mem::offset_of!(StatsSnapshot, jobs) == 48);
    assert!(core::mem::size_of::<StatsSnapshot>() == 56);
};

impl StatsSnapshot {
    /// Bump the completed-submission counter (called from the queue
    /// completion path in `JobFence::command_complete`, NOT from the stats
    /// channel).
    pub(crate) fn note_job(&self) {
        self.jobs.fetch_add(1, Ordering::Relaxed);
    }
}

/// Words kept per statistics record (G17: 18-word ring records).
pub(crate) const RAW_WORDS: usize = 18;
/// Record tags kept (the G17 ring accepts tags below 32).
pub(crate) const RAW_TAGS: usize = 32;

/// The last raw record of every G17 statistics tag and how many arrived, exported read-only as
/// `agx_stats_raw` until the record layouts are decoded. One GPU per system, so one table.
#[repr(C)]
pub(crate) struct RawStats {
    pub(crate) count: [AtomicU32; RAW_TAGS],
    pub(crate) words: [[AtomicU32; RAW_WORDS]; RAW_TAGS],
}

#[allow(clippy::declare_interior_mutable_const)]
const RAW_ZERO: AtomicU32 = AtomicU32::new(0);
#[allow(clippy::declare_interior_mutable_const)]
const RAW_ROW: [AtomicU32; RAW_WORDS] = [RAW_ZERO; RAW_WORDS];

pub(crate) static RAW_STATS: RawStats = RawStats {
    count: [RAW_ZERO; RAW_TAGS],
    words: [RAW_ROW; RAW_TAGS],
};

const _: () = assert!(core::mem::size_of::<RawStats>() == RAW_TAGS * (1 + RAW_WORDS) * 4);

impl RawStats {
    /// Called by the statistics ring consumer for every record it returns to the firmware.
    pub(crate) fn note(&self, words: &[u32; RAW_WORDS]) {
        let tag = words[0] as usize;
        if tag >= RAW_TAGS {
            return;
        }
        for (dst, src) in self.words[tag].iter().zip(words) {
            dst.store(*src, Ordering::Relaxed);
        }
        self.count[tag].fetch_add(1, Ordering::Release);
    }
}
