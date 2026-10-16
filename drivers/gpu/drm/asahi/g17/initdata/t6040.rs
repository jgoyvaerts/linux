// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The M4 Pro (T6040) initialization graph.
//!
//! The M4 Pro runs the macOS 26.6 (25G83) GPU firmware `mBoot-18000.161.10`, which has the A18
//! Pro firmware's interface identifier, so the graph is aurora's with the differences of that
//! firmware build and SoC that the G16 driver found running this GPU:
//!
//! - One coprocessor carries both roles: one main configuration with the secondary-only words
//!   (0x300, the power-configuration pointer) and a merged root with the power-state and
//!   configuration pointers.
//! - Status B is larger on 25G83: status A follows it at +0xf180, and the power-state object is
//!   a separate page instead of a tail of status B.
//! - The hardware data, its register map, the power configuration (at bundle+0x5380, built
//!   from the ADT calibration of two clusters) and region C are the G16 encoders' (proven on
//!   this firmware); only the pointers to aurora's QoS object and completion rings are patched.
//! - The five views and the firmware area are separate objects: packed into the bundle they
//!   would overlap the 25G83 hardware data.
//!
//! Everything else (free lists, pools, descriptor tables, rings, roots, regions) is aurora's.

use super::{abi, low, BuildEnv, BundleObjects, CpuMap, DescriptorTables, InitData, KernelObject};
use crate::g16_platform::{put64, Platform};
use crate::g17::{Config, Role};
use crate::mmu;
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;

/// Kernel-window offsets and sizes of the objects whose 25G83 placement differs.
mod layout {
    /// The status and state cluster: aurora's state grid at +0, status B and status A.
    pub(super) const CLUSTER_SIZE: usize = 0xd4000;
    /// Status B in the cluster.
    pub(super) const STATUS_B: usize = 0x1_0000;
    /// Status A in the cluster.
    pub(super) const STATUS_A: usize = 0x1_f180;
    /// The role configuration (root +0xc0) in the cluster; its status follows at +0x80.
    pub(super) const CONFIG: usize = 0xd_3440;
    /// The zeroed page named by main configuration 0x18 (the G16 driver names the word after
    /// its control producer; 26A428 takes 0 there).
    pub(super) const MAIN_HEADER: usize = 0x8000;
    /// The power-state object (root +0xb8), 0x6a8 bytes.
    pub(super) const POWER_STATE: u64 = 0x17_0000;
    /// The views named by the main configuration, with their 25G83 sizes.
    pub(super) const VIEWS: [(u64, usize); 5] = [
        (0x41_4000, 0xc18),
        (0x40_0000, 0x1048),
        (0x40_4000, 0xe10),
        (0x40_8000, 0x68),
        (0x40_c000, 0x800),
    ];
    /// The firmware area named twice by the main configuration (0x88 bytes).
    pub(super) const FIRMWARE_AREA: u64 = 0x41_0000;
    /// The AON temperature-source table named by the power configuration.
    pub(super) const AON_TABLE: u64 = 0xc050_0000;
    /// The power configuration in the bundle.
    pub(super) const POWER_CONFIG: usize = 0x5380;
    /// The completion-ring descriptors in the hardware data (aurora's are 0xc lower).
    pub(super) const COMPLETION_DESCRIPTORS: usize = 0x2620;
    /// The QoS pointer in the hardware data.
    pub(super) const QOS_POINTER: usize = 0x26b8;
}

/// Object indices of the 25G83-specific objects.
struct Objects {
    views: [usize; 5],
    firmware_area: usize,
    power_state: usize,
    aon: usize,
}

/// A completion-ring descriptor of the hardware data.
fn completion_record(index: u32, ring: &abi::CompletionRing) -> [u8; 0x20] {
    let mut out = [0; 0x20];
    out[0..8].copy_from_slice(&ring.low_va.to_le_bytes());
    out[8..16].copy_from_slice(&ring.high_va.to_le_bytes());
    out[16..20].copy_from_slice(&0x800u32.to_le_bytes());
    out[20..24].copy_from_slice(&0x40u32.to_le_bytes());
    out[24..28].copy_from_slice(&index.to_le_bytes());
    out
}

impl InitData {
    /// Constructs the T6040 graph after the firmware has published its root entries.
    pub(super) fn build_t6040(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        metrics: &KernelObject,
    ) -> Result {
        let platform = Platform::new(env.dev.as_ref())?;
        if !platform.t6040 {
            return Err(ENODEV);
        }
        self.status_a = layout::STATUS_A;
        self.status_b = layout::STATUS_B;
        self.map_t6040_registers(env, &platform)?;
        let tables = self.build_descriptor_tables(env, layout::CLUSTER_SIZE)?;
        self.build_render_free_lists(env)?;
        let first_block = self.build_render_pool(env)?;
        self.build_compute_pools(env, first_block)?;
        let bundle = self.build_bundle(env, cfg, metrics)?;
        let objects = self.build_t6040_objects(env)?;
        self.views = Some(objects.views);
        self.initialize_t6040(cfg, &platform, &tables, &bundle, &objects)?;
        // Write back every host-built byte before the firmware may read it: the G16 driver
        // never relies on a coherent path to the firmware, and doing the same costs little.
        for object in self.objects.iter_mut() {
            let bytes = object.bytes_mut();
            crate::pgtable::clean_cache(bytes.as_ptr(), bytes.len());
        }
        crate::mem::sync();
        Ok(())
    }

    /// The register windows of the hardware data's T6040 register map.
    fn map_t6040_registers(&mut self, env: &BuildEnv<'_>, platform: &Platform) -> Result {
        let page = mmu::UAT_PGSZ as u64;
        for r in &platform.registers {
            let prot = if r.flags & 2 != 0 {
                mmu::PROT_FW_MMIO_RW
            } else {
                mmu::PROT_FW_MMIO_RO
            };
            if r.slot == 3 || r.slot == 20 {
                // MCC and AFR: one window per instance, at its stride.
                let physicals = if r.slot == 3 { &platform.mcc } else { &platform.afr };
                for (i, physical) in physicals.iter().enumerate() {
                    let va = r.address + i as u64 * u64::from(r.stride);
                    self.aliases.push(
                        env.vm.map_io(va, (*physical).try_into()?, r.stride as usize, prot)?,
                        GFP_KERNEL,
                    )?;
                }
            } else {
                let offset = r.physical & (page - 1);
                let size = (offset + u64::from(r.size) + page - 1) & !(page - 1);
                self.aliases.push(
                    env.vm.map_io(
                        r.address - offset,
                        (r.physical - offset).try_into()?,
                        size.try_into()?,
                        prot,
                    )?,
                    GFP_KERNEL,
                )?;
            }
        }
        Ok(())
    }

    /// The views, the firmware area, the power-state object and the AON table.
    fn build_t6040_objects(&mut self, env: &BuildEnv<'_>) -> Result<Objects> {
        use mmu::{PROT_FW_PRIV_RW as FP, PROT_FW_SHARED_RW as FS};
        use CpuMap::WriteCombined as WC;
        let mut views = [0; 5];
        for (view, (offset, size)) in views.iter_mut().zip(layout::VIEWS) {
            if size > mmu::UAT_PGSZ {
                return Err(EINVAL);
            }
            *view = self.add(env.fixed(offset, mmu::UAT_PGSZ, FP, WC)?)?;
        }
        let firmware_area = self.add(env.fixed(layout::FIRMWARE_AREA, mmu::UAT_PGSZ, FP, WC)?)?;
        let power_state = self.add(env.fixed(layout::POWER_STATE, mmu::UAT_PGSZ, FS, WC)?)?;
        let aon = self.add(env.fixed(layout::AON_TABLE, mmu::UAT_PGSZ, FP, WC)?)?;
        Ok(Objects {
            views,
            firmware_area,
            power_state,
            aon,
        })
    }

    /// Writes the T6040 firmware objects, then the opening control-ring state.
    fn initialize_t6040(
        &mut self,
        cfg: &Config,
        platform: &Platform,
        tables: &DescriptorTables,
        bundle: &BundleObjects,
        objects: &Objects,
    ) -> Result {
        let bundle_va = self.object(self.bundle)?.gpu_va();
        let cluster_va = self.object(self.cluster)?.gpu_va();
        let config_va = cluster_va + layout::CONFIG as u64;
        let status_a_va = cluster_va + layout::STATUS_A as u64;
        let status_b_va = cluster_va + layout::STATUS_B as u64;
        let aon_va = self.object(objects.aon)?.gpu_va();
        let qos_va = self.object(bundle.qos)?.gpu_va();
        let power_state_va = self.object(objects.power_state)?.gpu_va();
        let firmware_area_va = self.object(objects.firmware_area)?.gpu_va();
        let mut view_va = [0; 5];
        for (va, index) in view_va.iter_mut().zip(objects.views) {
            *va = self.object(index)?.gpu_va();
        }

        // Hardware data and power configuration, then the pointers to aurora's objects.
        // SAFETY: Plain C entropy source.
        let entropy = unsafe { kernel::bindings::get_random_u32() };
        {
            let bytes = self.object_mut(self.bundle)?.bytes_mut();
            platform.power_bundle(bytes, config_va, aon_va, entropy)?;
            put64(bytes, layout::QOS_POINTER, qos_va);
            for (ring, index) in bundle.completion_rings.iter().zip(abi::COMPLETION_RING_INDEX) {
                let offset = layout::COMPLETION_DESCRIPTORS + index as usize * 0x20;
                bytes
                    .get_mut(offset..offset + 0x20)
                    .ok_or(EINVAL)?
                    .copy_from_slice(&completion_record(index, ring));
            }
        }
        let aon = platform.aon_table()?;
        self.object_mut(objects.aon)?
            .bytes_mut()
            .get_mut(..aon.len())
            .ok_or(EINVAL)?
            .copy_from_slice(&aon);
        let region_c = platform.region_c()?;
        let size = self.object(self.region_c)?.size();
        self.object_mut(self.region_c)?
            .bytes_mut()
            .copy_from_slice(region_c.get(..size).ok_or(EINVAL)?);

        // Status A and B: only the host words (+4 = 1) and the firmware-control pointers.
        self.object_mut(self.cluster)?
            .write(layout::STATUS_A, abi::StatusBlock::new())?;
        self.object_mut(self.cluster)?
            .write(layout::STATUS_B, abi::StatusBlock::new())?;
        let fwctl_va = self.object(tables.fwctl)?.gpu_va();
        let cluster = self.object(self.cluster)?;
        cluster
            .dword(layout::STATUS_B + 0x48e0)?
            .store(fwctl_va, Ordering::Relaxed);
        cluster
            .dword(layout::STATUS_B + 0x48e8)?
            .store(fwctl_va + abi::FWCTL_RING, Ordering::Relaxed);

        // One main configuration carrying both roles.
        let main_offset = abi::bundle::MAIN_CONFIG[Role::Primary as usize];
        let main_va = bundle_va + main_offset as u64;
        let args = abi::MainConfigArgs {
            bundle_va,
            main_config_va: main_va,
            state_grid_va: cluster_va + self.state_offset(Role::Primary) as u64,
            status_a_va,
            pb_descriptors_low_va: low::PB_DESCRIPTORS,
            page_pool_descriptors_low_va: low::PAGE_POOL_DESCRIPTORS,
        };
        let views = abi::PrimaryViews {
            zero_page_va: self.object(bundle.zero)?.gpu_va(),
            pb_descriptors_va: self.object(tables.pb)?.gpu_va(),
            page_pool_descriptors_va: self.object(tables.page_pool)?.gpu_va(),
        };
        self.object_mut(self.bundle)?
            .initialize::<abi::MainConfig>(main_offset, |main| {
                main.init_primary(&args, &views);
                main.view_va = view_va;
                main.firmware_area_va = [firmware_area_va; 2];
                main.unk_18 = (cluster_va + layout::MAIN_HEADER as u64).to_le_bytes();
                // The secondary role's words, on the single instance.
                main.unk_300 = 4;
                main.power_config_va = bundle_va + layout::POWER_CONFIG as u64;
                // The 25G83 firmware boots with these zero (G16); 26A428 requires 1/0xabcd_abcd.
                main.unk_32c = 0;
                main.unk_344 = 0;
                Ok(())
            })?;
        self.object_mut(self.bundle)?.write(
            main_offset + abi::MainConfig::CONTROL_RING,
            abi::OpeningRecord::new(abi::Role::Primary),
        )?;

        // The merged root.
        let root = abi::RootArgs {
            region_a_va: self.object(bundle.region_a)?.gpu_va(),
            region_c_va: self.object(self.region_c)?.gpu_va(),
            main_config_va: main_va,
            status_a_va,
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
        };
        self.object_mut(self.roots)?.write(
            0,
            abi::SecondaryRoot::merged(&root, status_b_va, power_state_va, config_va),
        )?;

        // The opening control record is staged in slot 0.
        for (offset, value) in [
            (0, 0),
            (0x10, 0),
            (abi::ChannelState::PRODUCER, abi::OPENING_PRODUCER),
        ] {
            self.object(self.cluster)?
                .word(self.state_offset(Role::Primary) + abi::CONTROL_STATE_OFFSET + offset)?
                .store(value, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Whether the firmware accepted the graph (status A +0x14 = 1). The 25G83 firmware
    /// acknowledges initialization here instead of with a message.
    pub(in crate::g17) fn init_accepted(&self) -> Result<bool> {
        Ok(self.status_word(Role::Primary, 0x14)?.load(Ordering::Acquire) == 1)
    }
}
