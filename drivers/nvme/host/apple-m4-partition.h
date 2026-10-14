/* SPDX-License-Identifier: GPL-2.0-only */
/* Extends the read-only policy for the Linux partitions.
 *
 * The write ranges come from the boot loader, which checked each one against
 * a GPT entry (exact first/last LBA and PARTUUID) before it booted Linux, and
 * publishes them as /chosen/m4,write-lba-range. Such a range only has to stay
 * clear of the GPT itself (LBA 0..7 in 4 KiB units).
 *
 * Loaders before rev 15 publish nothing; the node's lab,write-lba-range is used
 * then, and only inside the two intervals of the owner's own disk: the first is
 * derived from this Mac's original 110 GB M4-Dev physical store, reserving its
 * first 50 GB for the existing APFS container; the second is exactly GPT slot 3
 * (HP, created by macOS 2026-10-18). Drop that fallback once rev 15 is installed.
 */
#define APPLE_M4_WRITE_RANGES_MAX 4
#define APPLE_M4_GPT_END 8ULL

static inline bool apple_m4_partition_range_valid(u64 first, u64 end, bool from_loader)
{
	if (first >= end)
		return false;
	if (from_loader)
		return first >= APPLE_M4_GPT_END;
	return (first >= 228317114ULL && end <= 242965551ULL) ||
	       (first >= 201461644ULL && end <= 216077314ULL);
}

static inline bool apple_m4_partition_allowed(bool admin, unsigned int opcode,
		unsigned int feature, u32 nsid, unsigned int lba_shift,
		u64 slba, unsigned int nlb, u64 first, u64 end)
{
	if (apple_m4_readonly_allowed(admin, opcode, feature))
		return true;
	/* [first, end) was checked once, at probe */
	if (admin || nsid != 1 || lba_shift != 12)
		return false;
	/* Only flush data written under this same policy. No discard/format. */
	if (opcode == nvme_cmd_flush)
		return true;
	/* NVMe encodes the block count minus one in an unsigned 16-bit field. */
	return opcode == nvme_cmd_write && nlb <= 65535 &&
	       slba >= first && slba < end && (u64)nlb + 1 <= end - slba;
}
