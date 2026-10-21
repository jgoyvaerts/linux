/* SPDX-License-Identifier: GPL-2.0-only */
/* Local J616s qualification. Extends the read-only policy for the Linux
 * partitions. The first enclosing interval is derived from this Mac's original
 * 110 GB M4-Dev physical store, reserving its first 50 GB for the existing APFS
 * container; the second is exactly GPT slot 3 (HP, created by macOS 2026-10-18).
 * Actual partition bounds must additionally be supplied from verified GPT.
 */
#define APPLE_M4_WRITE_RANGES_MAX 2

static inline bool apple_m4_partition_range_valid(u64 first, u64 end)
{
	if (first >= end)
		return false;
	return (first >= 228317114ULL && end <= 242965551ULL) ||
	       (first >= 201461644ULL && end <= 216077314ULL);
}

static inline bool apple_m4_partition_allowed(bool admin, unsigned int opcode,
		unsigned int feature, u32 nsid, unsigned int lba_shift,
		u64 slba, unsigned int nlb, u64 first, u64 end)
{
	if (apple_m4_readonly_allowed(admin, opcode, feature))
		return true;
	if (admin || nsid != 1 || lba_shift != 12 ||
	    !apple_m4_partition_range_valid(first, end))
		return false;
	/* Only flush data written under this same policy. No discard/format. */
	if (opcode == nvme_cmd_flush)
		return true;
	/* NVMe encodes the block count minus one in an unsigned 16-bit field. */
	return opcode == nvme_cmd_write && nlb <= 65535 &&
	       slba >= first && slba < end && (u64)nlb + 1 <= end - slba;
}
