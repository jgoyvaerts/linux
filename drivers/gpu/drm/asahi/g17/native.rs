// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Native coprocessor bring-up for G17 targets without power domains (M4 Pro, T6040).
//!
//! The SoC describes the GPU without power domains or an operating-point table. Power and the ASC
//! CPU are driven directly (`t6040_native.c`); RTKit runs on the device-tree ASC mailbox. The
//! firmware asks for its crash-log buffer inside its own reserved
//! `firmware` region, and that request is started early and acknowledged by echoing it (the
//! shared RTKit core's early-crashlog mode, as on the G16 driver).

use core::ffi::c_void;
use core::ptr::NonNull;

use kernel::{
    c_str,
    device,
    io,
    iosys_map::IoSysMapRef,
    prelude::*,
    soc::apple::rtkit,
    sync::Arc,
    types::ForeignOwnable,
};

use super::RoleState;

unsafe extern "C" {
    fn asahi_t6040_native_new(dev: *mut c_void, pmgr: u64) -> *mut c_void;
    fn asahi_t6040_native_free(native: *mut c_void);
    fn asahi_t6040_power_on(native: *mut c_void) -> i32;
    fn asahi_t6040_cpu_cold(native: *mut c_void) -> i32;
    fn asahi_t6040_cpu_start(native: *mut c_void);
    fn asahi_t6040_cpu_stop(native: *mut c_void) -> i32;
}

/// Physical address of the PMGR block holding the GPU power domains, which the device tree does
/// not describe (the ASC CPU control is the GPU node's `asc` region).
pub(crate) struct NativeStart {
    /// PMGR block holding the GPU power domains (domain i at +0x100 + i * 0x10).
    pub(crate) pmgr: u64,
}

/// The powered GPU and its coprocessor CPU.
///
/// # Invariants
///
/// `raw` is a live `asahi_t6040_native` until `Drop` frees it. Once the CPU has been started,
/// freeing it keeps the GPU powered: the firmware may still own memory until reboot.
pub(crate) struct Native {
    raw: NonNull<c_void>,
}

// SAFETY: The C object is only used through `&mut self` or immutable register reads.
unsafe impl Send for Native {}
// SAFETY: As above.
unsafe impl Sync for Native {}

impl Native {
    /// Maps the blocks and powers the GPU on from its cold state.
    pub(crate) fn power_on(dev: &device::Device, start: &NativeStart) -> Result<Native> {
        // SAFETY: `dev` is the GPU's live platform device; the PMGR address is a plain value.
        let raw = unsafe { asahi_t6040_native_new(dev.as_raw().cast(), start.pmgr) };
        let raw = NonNull::new(kernel::error::from_err_ptr(raw)?).ok_or(ENOMEM)?;
        let native = Native { raw };
        // SAFETY: `raw` is live by the type invariant.
        kernel::error::to_result(unsafe { asahi_t6040_power_on(native.raw.as_ptr()) })?;
        Ok(native)
    }

    /// Fails unless the coprocessor CPU is in its cold reset state.
    pub(crate) fn check_cold(&self) -> Result {
        // SAFETY: `raw` is live by the type invariant.
        kernel::error::to_result(unsafe { asahi_t6040_cpu_cold(self.raw.as_ptr()) })
    }

    /// Releases the coprocessor CPU from reset.
    pub(crate) fn start(&mut self) {
        // SAFETY: `raw` is live by the type invariant.
        unsafe { asahi_t6040_cpu_start(self.raw.as_ptr()) };
    }

    /// Stops the coprocessor CPU.
    pub(crate) fn stop(&mut self) -> Result {
        // SAFETY: `raw` is live by the type invariant.
        kernel::error::to_result(unsafe { asahi_t6040_cpu_stop(self.raw.as_ptr()) })
    }
}

impl Drop for Native {
    fn drop(&mut self) {
        // SAFETY: `raw` is live by the type invariant and not used afterwards.
        unsafe { asahi_t6040_native_free(self.raw.as_ptr()) };
    }
}

/// A buffer the firmware placed inside its `firmware` reserved region.
pub(crate) struct FirmwareBuffer {
    map: io::mem::Mem,
    offset: usize,
    size: usize,
    iova: usize,
}

impl rtkit::Buffer for FirmwareBuffer {
    fn iova(&self) -> Result<usize> {
        Ok(self.iova)
    }

    fn buf(&mut self) -> Result<IoSysMapRef<'_, u8>> {
        self.map.as_iosys_map(self.offset, self.size)
    }
}

/// RTKit callbacks of a natively started coprocessor.
pub(crate) struct NativeOps;

#[vtable]
impl rtkit::Operations for NativeOps {
    type Data = Arc<RoleState>;
    type Buffer = FirmwareBuffer;

    fn recv_message_early(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        ep: u8,
        msg: u64,
    ) -> bool {
        if !super::is_events_message(ep, msg) {
            return false;
        }
        data.shared.queue_events();
        true
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, crashlog: Option<&[u8]>) {
        data.signal(true);
        data.shared.lose_device(data.role, crashlog);
    }

    fn shmem_map(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        iova: usize,
        size: usize,
    ) -> Result<FirmwareBuffer> {
        let dev = data.shared.dev.as_ref();
        let node = dev.of_node().ok_or(EINVAL)?;
        let res = node.reserved_mem_region_to_resource_byname(c_str!("firmware"))?;
        let offset = iova.checked_sub(res.start() as usize).ok_or(EINVAL)?;
        if offset.checked_add(size).ok_or(EINVAL)? > res.size() as usize {
            return Err(EINVAL);
        }
        // SAFETY: The bootloader reserved this RAM for the loaded GPU firmware; only the
        // firmware-advertised subrange is exposed.
        let map = unsafe { io::mem::Mem::try_new(res, io::mem::MemFlag::WB.into())? };
        Ok(FirmwareBuffer {
            map,
            offset,
            size,
            iova,
        })
    }
}
