// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t6040 platforms (M4 Pro).
//!
//! Values come from the G16 driver, which runs this GPU on the macOS 26.6 (25G83) firmware
//! `mBoot-18000.161.10`, whose firmware interface is the A18 Pro's.

use crate::g17::{native::NativeStart, Config, Cores, FixedBuffer, Identity, InitLayout, Start};
use crate::hw::{GpuGen, GpuVariant};
use crate::mmu::UatQuirks;

/// Static configuration of the T6040 GPU.
pub(crate) const CONFIG: Config = Config {
    chip_id: CHIP_ID,
    identity: Identity {
        family: 0xa,
        // ID register 0x0a03_1100: family 0xa, variant 3, revision 0x11 (B1).
        variant: 3,
        num_dies: 1,
        gpu_variant: GpuVariant::S,
        usc_generation: 3,
        hal_generation: 200,
    },
    uat_ias: 42,
    uat_oas: 42,
    // One coprocessor carries both firmware roles; `pagetables` holds only its root.
    secondary_root_offset: None,
    start: Start::Native(NativeStart { pmgr: 0x2_88e8_0000 }),
    init: InitLayout::T6040,
    // Two clusters of ten cores (`apple,m4-core-mask` = 0xfffff).
    cores: Cores::DeviceTree {
        clusters: 2,
        cores: 10,
    },
    // The G16 driver's UAT policy: the firmware's tables live in no-map `pagetables`/`l2`
    // memory, every table change is written back, ranges are invalidated page by page, and the
    // handoff page carries only the host header (current slot 0xffff_ffff, no lock).
    uat_quirks: UatQuirks {
        reserved_tables: true,
        clean_tables: true,
        page_tlbi: true,
        handoff_lock: false,
        handoff_slot: 0xffff_ffff,
        native_gpu_permissions: true,
    },
    // The root is borrowed as the firmware left it: no snapshot, clearing or confirmation.
    adopt_root_entries: false,
    fabric_bridge: false,
    pm_metrics: FixedBuffer {
        va: 0x10_0008_0000,
        size: 0x8000,
    },
    recovery_slot_keys: None,
    gpu_generation: GpuGen::G16,
};

/// Chip identifier written to the hardware data.
pub(crate) const CHIP_ID: u32 = 0x6040;
