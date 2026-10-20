// SPDX-License-Identifier: GPL-2.0-only
// Copyright The Gravity Linux Contributors
// Adapted from Niklas Sheth's linux-m4-integration prototype.

//! G16 firmware addresses the hardware data refers to. Fields follow the
//! source-built m1n1 G16 model.

pub(crate) const QOS_ADDRESS: u64 = 0xffff_fc20_0016_0000;
pub(crate) const TIMESTAMP_BASE: u64 = 0xffff_fc21_8140_0000;
