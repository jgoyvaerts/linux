// SPDX-License-Identifier: GPL-2.0-only
// Copyright The Gravity Linux Contributors

//! Live platform inputs and initialization builders from g16g_platform.py and
//! g16g_initdata.py. Firmware data is built from explicit bootloader properties.

use kernel::{c_str, device, of, prelude::*};

const PAGE: u64 = 0x4000;
const REGISTER_BASE: u64 = 0xffff_fc21_8020_0000;
const REGISTER_LIMIT: u64 = 0xffff_fc21_8100_0000;
const MCC_BASE: u64 = 0x2201c4000;
const MCC_SIZE: u32 = 0x18000;

pub(crate) fn put32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

pub(crate) fn put64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

pub(crate) fn zeroed(size: usize) -> Result<KVec<u8>> {
    let mut out = KVec::new();
    out.extend_with(size, 0, GFP_KERNEL)?;
    Ok(out)
}

/// The boot boundary preserves original ADT bytes, including float bit patterns.
pub(crate) fn adt_word(node: &of::Node, name: &CStr, default: Option<u32>) -> Result<u32> {
    let bytes: Option<KVec<u8>> = node.get_opt_property(name)?;
    match bytes {
        Some(bytes) => Ok(u32::from_le_bytes(
            bytes.as_slice().try_into().map_err(|_| EINVAL)?,
        )),
        None => default.ok_or(ENOENT),
    }
}

fn ladder(node: &of::Node, name: &CStr) -> Result<[u32; 11]> {
    let values: KVec<u32> = node.get_property(name)?;
    values.as_slice().try_into().map_err(|_| EINVAL)
}

pub(crate) struct Register {
    pub(crate) slot: usize,
    pub(crate) physical: u64,
    pub(crate) address: u64,
    pub(crate) size: u32,
    pub(crate) stride: u32,
    pub(crate) relative: u64,
    pub(crate) flags: u32,
}

pub(crate) struct Platform {
    pub(crate) node: of::Node,
    pub(crate) freq_a: [u32; 11],
    pub(crate) freq_b: [u32; 11],
    core_voltage: [u32; 11],
    memory_voltage: [u32; 11],
    index_a: [u32; 11],
    index_b: [u32; 11],
    relative_a: [u32; 11],
    relative_b: [u32; 11],
    pub(crate) maximum_power: u32,
    pub(crate) core_mask: u32,
    revision: u32,
    pub(crate) period_ms: u32,
    pub(crate) period_clocks: u32,
    pub(crate) registers: KVec<Register>,
    pub(crate) mcc: KVec<u64>,
    pub(crate) afr: KVec<u64>,
    pub(crate) sgx_base: u64,
    pub(crate) t6040: bool,
    cluster_voltages: Option<[[[u32; 2]; 11]; 4]>,
    gpc: bool,
}

impl Platform {
    /// T6040 PMGR publishes these original {sensor, length, coefficients}
    /// records unchanged. G16X firmware uses the sensor bitmap, not a host
    /// evaluation of the calibration polynomial.
    pub(crate) fn mtr_mask(&self) -> Result<u64> {
        let bytes: KVec<u8> = self.node.get_property(c_str!("apple,m4-mtr-polynomial"))?;
        let mut mask = 0u64;
        let mut pos = 0;
        while pos < bytes.len() {
            let header = bytes.get(pos..pos + 8).ok_or(EINVAL)?;
            let id = u32::from_le_bytes(header[..4].try_into().map_err(|_| EINVAL)?);
            let len = u32::from_le_bytes(header[4..].try_into().map_err(|_| EINVAL)?) as usize;
            if id >= 64 || len != 16 || bytes.get(pos + 8..pos + 8 + len).is_none()
                || mask & (1u64 << id) != 0 { return Err(EINVAL); }
            mask |= 1u64 << id;
            pos += 8 + len;
        }
        if mask == 0 { return Err(EINVAL); }
        Ok(mask)
    }

    /// AGX's AON table uses the native PTD rows for its two GPU clusters and
    /// one die. ApplePTD's dictionary conversion preserves all twelve bytes.
    pub(crate) fn aon_table(&self) -> Result<KVec<u8>> {
        let raw: KVec<u8> = self.node.get_property(c_str!("apple,m4-aon-table"))?;
        if raw.is_empty() || raw.len() % 12 != 0 { return Err(EINVAL); }
        let mut out = zeroed(36)?;
        for (index, key) in [*b"g0TR", *b"g1TR", *b"r0TR"].iter().enumerate() {
            let wanted = u32::from_be_bytes(*key);
            let mut found = false;
            for row in raw.chunks_exact(12) {
                let id = u32::from_le_bytes(row[..4].try_into().map_err(|_| EINVAL)?);
                if id != wanted { continue; }
                if found || row[4..6] != 1u16.to_le_bytes() { return Err(EINVAL); }
                out[index * 12..index * 12 + 12].copy_from_slice(row);
                found = true;
            }
            if !found { return Err(ENOENT); }
        }
        Ok(out)
    }

    /// G16X auxiliary clocks use one voltage per die, independently of cores.
    fn auxiliary_table(&self, out: &mut [u8], base: usize, name: &CStr) -> Result {
        let bytes: KVec<u8> = self.node.get_property(name)?;
        let read = |offset: usize| -> Result<u64> {
            Ok(u64::from_le_bytes(bytes.get(offset..offset + 8)
                .ok_or(EINVAL)?.try_into().map_err(|_| EINVAL)?))
        };
        let dies = read(0)?;
        let count = read(8)?;
        // T6040 has one die; don't infer another SKU's table geometry.
        if dies != 1 || count < 2 || count > 16 || bytes.len() != 24 + count as usize * 16 {
            return Err(EINVAL);
        }
        let floor = read(16 + count as usize * 16)? / 1000;
        if floor == 0 || floor > 1500 { return Err(EINVAL); }
        put32(out, base, count as u32 - 1);
        // G16X getPerfStateCap returns the final table index for each domain.
        put32(out, base + 4, count as u32 - 1);
        let mut previous = 0;
        for i in 0..count as usize {
            let mv = read(16 + i * 16)? / 1000;
            let hz = read(24 + i * 16)?;
            if mv == 0 || mv > 1500 || hz <= previous || hz > u32::MAX as u64 {
                return Err(EINVAL);
            }
            previous = hz;
            put32(out, base + 8 + i * 4, (hz / 1_000_000) as u32);
            put32(out, base + 0x48 + i * 8, mv as u32);
            put32(out, base + 0xc8 + i * 8, mv.max(floor) as u32);
        }
        Ok(())
    }

    pub(crate) fn new(dev: &device::Device) -> Result<Self> {
        let node = dev.of_node().ok_or(ENODEV)?;
        let compatible: KVec<u8> = node.get_property(c_str!("compatible"))?;
        let t6040 = compatible.split(|c| *c == 0).any(|c| c == b"apple,agx-t6040");
        // Only the maximum is consumed by the firmware builders. A loader
        // with an observed board maximum need not invent an entire power
        // ladder. Continue accepting the original loader's modeled ladder.
        let maximum_power = match node.get_opt_property::<u32>(c_str!("apple,m4-max-power-mw"))? {
            Some(value) => value,
            None => {
                let power = ladder(&node, c_str!("apple,m4-power-mw"))?;
                if power[0] != 0 || power.windows(2).any(|w| w[0] > w[1]) {
                    return Err(EINVAL);
                }
                power[10]
            }
        };
        let freq_a = ladder(&node, c_str!("apple,m4-freq-a"))?;
        let freq_b = ladder(&node, c_str!("apple,m4-freq-b"))?;
        if maximum_power == 0
            || maximum_power > i32::MAX as u32
            || freq_a[0] != 0
            || freq_a.windows(2).any(|w| w[0] >= w[1])
            || freq_b[0] != 0
            || freq_b.windows(2).any(|w| w[0] >= w[1])
        {
            return Err(EINVAL);
        }
        let period_clocks = match adt_word(
            &node,
            c_str!("apple,sgx-gpu-pwr-sample-period-aic-clks"),
            None,
        ) {
            Ok(value) => value,
            Err(ENOENT) => adt_word(&node, c_str!("apple,sgx-gpu-power-sample-period"), None)?
                .checked_mul(24000)
                .ok_or(EINVAL)?,
            Err(error) => return Err(error),
        };
        let period_ms = period_clocks / 24000;
        if period_ms == 0 {
            return Err(EINVAL);
        }
        let core_mask: u32 = node.get_property(c_str!("apple,m4-core-mask"))?;
        if core_mask == 0 || core_mask & !(if t6040 { 0xfffff } else { 0x3ff }) != 0 {
            return Err(EINVAL);
        }
        let cluster_voltages = if t6040 {
            let mut tables = [[[0; 2]; 11]; 4];
            for (table, name) in tables.iter_mut().zip([
                c_str!("apple,m4-cluster-core-a"), c_str!("apple,m4-cluster-sram-a"),
                c_str!("apple,m4-cluster-core-b"), c_str!("apple,m4-cluster-sram-b"),
            ]) {
                let values: KVec<u32> = node.get_property(name)?;
                if values.len() != 22 { return Err(EINVAL); }
                for (row, values) in table.iter_mut().zip(values.chunks_exact(2)) {
                    row.copy_from_slice(values);
                    if row.iter().any(|v| *v == 0 || *v > 1500) { return Err(EINVAL); }
                }
            }
            if core_mask & 0x3ff == 0 || core_mask >> 10 == 0 { return Err(EINVAL); }
            Some(tables)
        } else { None };
        let mut this = Self {
            core_voltage: ladder(&node, c_str!("apple,m4-core-voltage"))?,
            memory_voltage: ladder(&node, c_str!("apple,m4-memory-voltage"))?,
            index_a: ladder(&node, c_str!("apple,m4-index-a"))?,
            index_b: ladder(&node, c_str!("apple,m4-index-b"))?,
            relative_a: ladder(&node, c_str!("apple,m4-relative-a"))?,
            relative_b: ladder(&node, c_str!("apple,m4-relative-b"))?,
            revision: node.get_property(c_str!("apple,chip-revision"))?,
            gpc: adt_word(&node, c_str!("apple,sgx-no-gpc"), Some(0))? == 0,
            node,
            freq_a,
            freq_b,
            maximum_power,
            core_mask,
            period_ms,
            period_clocks,
            registers: KVec::new(),
            mcc: KVec::new(),
            afr: KVec::new(),
            sgx_base: 0,
            t6040,
            cluster_voltages,
        };
        this.build_registers()?;
        Ok(this)
    }

    fn build_registers(&mut self) -> Result {
        let reg: KVec<u32> = self.node.get_property(c_str!("reg"))?;
        if reg.len() != 8 {
            return Err(EINVAL);
        }
        let base = (u64::from(reg[4]) << 32) | u64::from(reg[5]);
        let span = (u64::from(reg[6]) << 32) | u64::from(reg[7]);
        self.sgx_base = base;
        if self.t6040 && (base != 0x288000000 || span != 0x3758000) { return Err(EINVAL); }
        let irq: KVec<u8> = self
            .node
            .get_property(c_str!("apple,sgx-meta-sw-interrupt"))?;
        if irq.len() != 20 && irq.len() != 40 {
            return Err(EINVAL);
        }
        let irq_address = u64::from_le_bytes(
            irq[irq.len() - 20..irq.len() - 12]
                .try_into()
                .map_err(|_| EINVAL)?,
        );
        if irq_address < PAGE || irq_address >= 1 << 48 {
            return Err(EINVAL);
        }
        let mask = adt_word(
            &self.node,
            c_str!("apple,sgx-active-mcc-bit-vector"),
            Some(if self.t6040 { 15 } else { 3 }),
        )?;
        if mask == 0 {
            return Err(EINVAL);
        }
        for bit in 0..32 {
            if mask & (1 << bit) != 0 {
                self.mcc.push(MCC_BASE + (bit << 25), GFP_KERNEL)?;
            }
        }
        self.registers.push(
            Register {
                slot: 0,
                physical: irq_address & !(PAGE - 1),
                address: 0,
                size: PAGE as u32,
                stride: PAGE as u32,
                relative: 0,
                flags: 2,
            },
            GFP_KERNEL,
        )?;
        self.registers.push(
            Register {
                slot: 3,
                physical: self.mcc[0],
                address: 0,
                size: self.mcc.len() as u32 * MCC_SIZE,
                stride: MCC_SIZE,
                relative: 0,
                flags: 2,
            },
            GFP_KERNEL,
        )?;
        // Each generation uses its own verified HAL declarations.
        let windows: &[(usize, u64, u32, bool)] = if self.t6040 { &[
            (17, 0, 0x21400, true), (26, 0xd04000, 0x8000, true),
            (29, 0xd10000, 0x4000, true), (31, 0xd40000, 0x4000, true),
            (33, 0xd44000, 0x4000, true), (34, 0xd4c000, 0x200, true),
            (28, 0xd58000, 0x8000, true), (32, 0xd60000, 0x20000, true),
            (35, 0xe00000, 0x4000, true), (40, 0xe1c000, 0x4000, true),
            (37, 0xe40000, 0x4000, true), (43, 0xe60000, 0x58, true),
            (22, 0x1000000, 0x8000, false), (39, 0xe08000, 0x8000, false),
            (41, 0xe1f800, 0x4000, false),
        ] } else { &[
            (17, 0, 0x20000, true),
            (26, 0xd04000, 0x8000, true),
            (27, 0xd0d000, 0x1000, true),
            (28, 0xd58000, 0x8000, true),
            (29, 0xd10000, 0x4000, true),
            (31, 0xd40000, 0x4000, true),
            (32, 0xd60000, 0x8000, true),
            (40, 0xe1c000, 0x4000, true),
            (22, 0x1000000, 0x8000, false),
            (39, 0xe08000, 0x8000, false),
            (41, 0xe1f800, 0x4000, false),
        ] };
        for &(slot, offset, size, relative) in windows {
            if offset + u64::from(size) > span {
                return Err(EINVAL);
            }
            self.registers.push(
                Register {
                    slot,
                    physical: base.checked_add(offset).ok_or(EINVAL)?,
                    address: 0,
                    size,
                    stride: size,
                    relative: if relative { offset } else { 0 },
                    flags: 2,
                },
                GFP_KERNEL,
            )?;
        }
        let absolute: &[(usize, u64, u32, u32)] = if self.t6040 { &[
            (4, 0x50244c000, 0x4000, 0), (9, 0x5003d0000, 0x4000, 2),
            (10, 0x5003c0000, 0x4000, 0), (11, 0x5003d8000, 0x4000, 2),
            (12, 0x48565c000, 0x4000, 2), (14, 0x500400000, 0x10000, 0),
            (15, 0x50cdc0000, 0x4000, 0),
        ] } else { &[
            (9, 0x3803d0000, 0x1000, 2),
            (10, 0x3803c0000, 0x2000, 0),
            (12, 0x50165c000, 0x4000, 2),
            (14, 0x380280000, 0x8000, 0),
            (15, 0x38c840000, 0x24000, 0),
        ] };
        for &(slot, physical, size, flags) in absolute {
            self.registers.push(
                Register {
                    slot,
                    physical,
                    address: 0,
                    size,
                    stride: size,
                    relative: 0,
                    flags,
                },
                GFP_KERNEL,
            )?;
        }
        if self.t6040 {
            for physical in [0x28b000000, 0x28b200000, 0x28b400000, 0x28b600000] {
                self.afr.push(physical, GFP_KERNEL)?;
            }
            self.registers.push(Register { slot: 20, physical: self.afr[0], address: 0,
                size: 4 * 0x40000, stride: 0x40000, relative: 0, flags: 2 }, GFP_KERNEL)?;
        }
        self.registers.sort_unstable_by_key(|r| r.slot);
        let mut cursor = REGISTER_BASE;
        for r in &mut self.registers {
            let offset = r.physical & (PAGE - 1);
            r.address = cursor + offset;
            cursor += (offset + u64::from(r.size) + PAGE - 1) & !(PAGE - 1);
            cursor += PAGE;
            if cursor > REGISTER_LIMIT {
                return Err(ENOSPC);
            }
        }
        Ok(())
    }

    pub(crate) fn hwdata(&self) -> Result<KVec<u8>> {
        let mut out = zeroed(0x3db4)?;
        for r in &self.registers {
            let off = 0x640 + r.slot * 0x28;
            put64(&mut out, off, r.physical);
            put64(&mut out, off + 8, r.address);
            put32(&mut out, off + 0x10, r.size);
            put32(&mut out, off + 0x14, r.stride);
            put64(&mut out, off + 0x18, r.relative);
            put32(&mut out, off + 0x20, r.flags);
        }
        for (base, repeats, frequencies) in [(0xfc8, 16, &self.freq_a), (0x1cdc, 1, &self.freq_b)] {
            for state in 0..11 {
                put32(&mut out, base + state * 4, frequencies[state]);
                for word in 0..repeats {
                    put32(
                        &mut out,
                        base + 0x40 + state * 0x40 + word * 4,
                        self.core_voltage[state],
                    );
                    put32(
                        &mut out,
                        base + 0x440 + state * 0x40 + word * 4,
                        self.memory_voltage[state],
                    );
                }
            }
        }
        for (off, values) in [
            (0x1808, &self.freq_b),
            (0x18c8, &self.relative_a),
            (0x1908, &self.relative_b),
            (0x19c8, &self.index_a),
            (0x1a08, &self.index_b),
        ] {
            for (i, value) in values.iter().enumerate() {
                put32(&mut out, off + i * 4, *value);
            }
        }
        for i in 0..11 {
            put32(&mut out, 0x1848 + i * 4, 1.02f32.to_bits());
        }
        if let Some(tables) = &self.cluster_voltages {
            // PerfStateInfo carries an independent voltage for each cluster.
            // Inactive columns are zero, and the lower-frequency table has
            // its own rows rather than borrowing the upper table's voltages.
            for (base, table) in [(0x1008, &tables[0]), (0x1408, &tables[1]),
                                  (0x1d1c, &tables[2]), (0x211c, &tables[3])] {
                out[base..base + 16 * 64].fill(0);
                for (state, row) in table.iter().enumerate() {
                    for (cluster, voltage) in row.iter().enumerate() {
                        put32(&mut out, base + state * 64 + cluster * 4, *voltage);
                    }
                }
            }
            self.auxiliary_table(&mut out, 0x1a48, c_str!("apple,sgx-cs-perf-states"))?;
            self.auxiliary_table(&mut out, 0x1b90, c_str!("apple,sgx-afr-perf-states"))?;
        }
        for (off, value) in [
            (0xe90, if self.t6040 { 0x6040 } else { 0x8132 }),
            (0xe94, self.revision >> 4),
            (0xe98, self.revision & 7),
            (0xea0, u32::from(self.gpc)),
            (0xeb8, 1),
            (0xec8, 1),
            (0xed0, 24000),
            (0xed8, self.period_ms),
            (0xee0, 1),
            (0xee4, 1),
            (0xee8, 1),
            (0xfc4, 10),
            (0x1cd8, 10),
            (0x2604, 1),
        ] {
            put32(&mut out, off, value);
        }
        out[0x258e..0x2590].copy_from_slice(&22u16.to_le_bytes());
        for group in 0..2 {
            for word in 1..4 {
                put32(&mut out, 0x2590 + group * 16 + word * 4, 1);
            }
        }
        for word in 0..12 {
            put32(&mut out, 0x25b4 + word * 4, u32::MAX);
        }
        if self.t6040 {
            // Matching G16X generic/ARM serializers, with native B1 topology.
            // These ranges and feature words differ from the base-M4 shim.
            let revision = match self.revision {
                0x00 => 1, 0x10 => 3, 0x11 => 4, _ => return Err(EINVAL),
            };
            let cores = self.core_mask.count_ones();
            for (off, value) in [
                (0xea4, 1), (0xec0, 1), (0xed4, 1), (0xf04, 0x1f),
                (0xf24, 0x0700_2004), (0xf28, 6), (0xf34, 1), (0xf38, 1),
                (0xf4c, 0x31), (0xf88, cores), (0xf8c, 15),
                (0xfb8, 0x1c), (0xfbc, revision), (0xfc0, cores),
                (0x2544, 5), (0x2548, 6), (0x254c, 3), (0x2550, 7),
                (0x2554, 7), (0x2570, cores), (0x2578, 1),
                (0x258c, 0), (0x2594, 0), (0x2598, 0), (0x25a0, 1),
                (0x25a8, 0), (0x25b0, 1), (0x25b4, 1), (0x25b8, 0),
                (0x25e4, u32::MAX), (0x25e8, u32::MAX),
                (0x25f4, 2), (0x25fc, 1), (0x2604, 0),
                (0x2614, 1), (0x261c, 0x100), (0x26c0, 1), (0x26c4, 1),
                (0x26f4, 1),
            ] { put32(&mut out, off, value); }
            for (off, value) in [(0x00, 0x6f_0000_0000), (0x08, 0xffc0_0000),
                                  (0x10, 0x10_0000_0000), (0x18, 0x10_0000_0000),
                                  (0x20, 0x2ff_ffff_8000),
                                  (0x26b8, crate::g16_fw::QOS_ADDRESS),
                                  (0x28, crate::g16_fw::TIMESTAMP_BASE)] {
                put64(&mut out, off, value);
            }
        }
        Ok(out)
    }

    pub(crate) fn region_c(&self) -> Result<KVec<u8>> {
        let mut out = zeroed(PAGE as usize)?;
        for (off, value) in [
            (0x78, 1),
            (0x7ec, 1),
            (0x7f0, 1),
            (0x24, 4096),
            (0x998, 32),
            (0x99c, 8),
            (0x9a0, 256),
            (0x9a4, 1),
            (0x9c8, 64),
            (0x9cc, 64),
            (0x9d0, 2),
            (0x84, self.maximum_power),
            (0x88, 1000),
            (0x8c, 1000),
            (0x98, 1000),
        ] {
            put32(&mut out, off, value);
        }
        for (off, value) in [(0x54, u16::MAX), (0x56, 40), (0x58, u16::MAX)] {
            out[off..off + 2].copy_from_slice(&value.to_le_bytes());
        }
        if !self.t6040 {
            put32(&mut out, 0xe28, 3);
            put32(&mut out, 0xe2c, 1);
        }
        if self.t6040 {
            // G16X packs these fields at e1d/e21/e25/e29 and e39/e3d.
            // Legacy e28/e2c writes corrupt the e29 parent-debug-check flag.
            // Preserve its native zero default and only retain boot flag 78.
            for (off, value) in [
                (0x24, 3000), (0x30, 1), (0x34, 1), (0x3c, 120),
                (0x54, 0x00280fa0), (0x5c, 0x10000), (0x60, 0x10000),
                (0x64, 0x30000), (0x68, 0x10000),
                (0x7c4, 1500), (0x7c8, 0x3f800000), (0x7cc, 0x3f4ccccd),
                (0x7d0, 0x3e4ccccd), (0x7d4, 0x3f666666), (0x7d8, 0x3dcccccd),
                (0x7dc, 0x3e800000), (0x7e0, 0x3f19999a), (0x7e4, 0x3f666666),
                (0x7e8, 6), (0x7f4, 100), (0x7f8, 100),
                (0x998, 40), (0x99c, 10), (0x9a0, 250), (0x9b8, 2),
                (0x9bc, 40), (0x9c0, 5), (0x9c8, 40), (0x9cc, 50), (0x9d0, 1),
                (0x9e0, 1), (0x9e4, 0x01020200), (0x9e8, 0x010a2801),
                (0x9ec, 0x28020101), (0x9f0, 0x0101010a), (0x9f4, 0x010a2802),
                (0x9f8, 0x28010101), (0x9fc, 0x0001010a),
                (0xe1c, 0x100), (0xe38, 0x300), (0xe3c, 0x100),
            ] { put32(&mut out, off, value); }
        }
        Ok(out)
    }
}
