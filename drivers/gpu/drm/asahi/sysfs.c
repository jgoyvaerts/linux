// SPDX-License-Identifier: GPL-2.0-only OR MIT
//
// Per-device sysfs shim. Rust owns the snapshot through an Arc in the bound
// driver. The attribute wrapper is removed and its readers drained before
// that Arc is dropped. Atomic fields may reflect different sample instants.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/pid.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include "sysfs.h"

/* Mirror of repr(C) Rust atomics; READ_ONCE accesses the current values. */
struct asahi_stats_snapshot {
	u32 util1;
	u32 util2;
	u32 util3;
	u32 util4;
	u32 pstate;
	u32 avg_power_mw;
	u32 temperature_raw;
	u32 temperature_scale;
	u32 temperature_tmin;
	u32 temperature_tmax;
	u64 busy_ns;
	u64 jobs;
};

struct asahi_stats_attribute {
	struct device_attribute attr;
	const struct asahi_stats_snapshot *snapshot;
	bool export_enabled;
};

/* Mirrors stats.rs RawStats: per tag a record count, then the last record's words. */
#define ASAHI_RAW_TAGS 32
#define ASAHI_RAW_WORDS 18
struct asahi_raw_stats {
	u32 count[ASAHI_RAW_TAGS];
	u32 words[ASAHI_RAW_TAGS][ASAHI_RAW_WORDS];
};

static const struct asahi_raw_stats *asahi_raw;

#define ASAHI_RAW(tag, word) READ_ONCE(asahi_raw->words[tag][word])

/*
 * G17 (M4 Pro, 25G83) records decoded from live data (2026-10-19): tag 19 is the utilization
 * record (64-bit timestamp, four percentages), tag 25 the power state (timestamp, last-busy
 * timestamp, active, powered off, unknown, p-state, ...). Pseudo-tags 0 and 1 hold the host's
 * fast-die controller state (throttling, temperature x100, limit x100).
 */
static ssize_t agx_stats_show_g17(char *buf)
{
	ssize_t n = 0;

	if (smp_load_acquire(&asahi_raw->count[25])) {
		n += scnprintf(buf + n, PAGE_SIZE - n, "pstate %u\n", ASAHI_RAW(25, 8));
		n += scnprintf(buf + n, PAGE_SIZE - n, "active %u\n", ASAHI_RAW(25, 5));
		n += scnprintf(buf + n, PAGE_SIZE - n, "powered_off %u\n", ASAHI_RAW(25, 6));
	}
	if (smp_load_acquire(&asahi_raw->count[19]))
		n += scnprintf(buf + n, PAGE_SIZE - n, "util %u %u %u %u\n", ASAHI_RAW(19, 3),
			       ASAHI_RAW(19, 4), ASAHI_RAW(19, 5), ASAHI_RAW(19, 6));
	if (smp_load_acquire(&asahi_raw->count[0])) {
		n += scnprintf(buf + n, PAGE_SIZE - n, "fastdie_throttling %u\n", ASAHI_RAW(0, 0));
		n += scnprintf(buf + n, PAGE_SIZE - n, "temperature_centi_c %u\n", ASAHI_RAW(0, 1));
		n += scnprintf(buf + n, PAGE_SIZE - n, "fastdie_limit_centi_c %u\n", ASAHI_RAW(1, 7));
	}
	return n ? n : sysfs_emit(buf, "unsupported\n");
}

static ssize_t agx_stats_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct asahi_stats_attribute *stats =
		container_of(attr, struct asahi_stats_attribute, attr);
	const struct asahi_stats_snapshot *snap = stats->snapshot;
	ssize_t n = 0;

	if (!stats->export_enabled)
		return asahi_raw ? agx_stats_show_g17(buf) : sysfs_emit(buf, "unsupported\n");
	n += scnprintf(buf + n, PAGE_SIZE - n, "busy_ns %llu\n",
		       (unsigned long long)READ_ONCE(snap->busy_ns));
	n += scnprintf(buf + n, PAGE_SIZE - n, "jobs %llu\n",
		       (unsigned long long)READ_ONCE(snap->jobs));
	n += scnprintf(buf + n, PAGE_SIZE - n, "pstate %u\n",
		       (u32)READ_ONCE(snap->pstate));
	n += scnprintf(buf + n, PAGE_SIZE - n, "power_mw %u\n",
		       (u32)READ_ONCE(snap->avg_power_mw));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util1 %u\n",
		       (u32)READ_ONCE(snap->util1));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util2 %u\n",
		       (u32)READ_ONCE(snap->util2));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util3 %u\n",
		       (u32)READ_ONCE(snap->util3));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util4 %u\n",
		       (u32)READ_ONCE(snap->util4));
	n += scnprintf(buf + n, PAGE_SIZE - n, "temperature_raw %u\n",
		       (u32)READ_ONCE(snap->temperature_raw));
	n += scnprintf(buf + n, PAGE_SIZE - n, "temperature_scale %u\n",
		       (u32)READ_ONCE(snap->temperature_scale));

	return n;
}


/*
 * Called from Rust's `AsahiDriver::probe` after the DRM device is
 * registered. Returns 0 on success, or a negative errno.
 */
int asahi_sysfs_register(struct device *dev, const void *snapshot,
			 int export_enabled, void **handle)
{
	struct asahi_stats_attribute *stats;
	int ret;

	if (!dev || !handle || (export_enabled && !snapshot))
		return -ENODEV;

	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, pstate) != 16);
	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, busy_ns) != 40);
	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, jobs) != 48);
	BUILD_BUG_ON(sizeof(struct asahi_stats_snapshot) != 56);

	stats = kzalloc_obj(*stats);
	if (!stats)
		return -ENOMEM;

	sysfs_attr_init(&stats->attr.attr);
	stats->attr.attr.name = "agx_stats";
	stats->attr.attr.mode = 0444;
	stats->attr.show = agx_stats_show;
	stats->snapshot = snapshot;
	stats->export_enabled = export_enabled;

	ret = device_create_file(dev, &stats->attr);
	if (ret) {
		kfree(stats);
		return ret;
	}

	*handle = stats;
	return 0;
}
EXPORT_SYMBOL_GPL(asahi_sysfs_register);

/* The calling process's thread-group leader name and tgid, for the G17 failure logs. */
void asahi_current_owner(char *comm, int *pid)
{
	strscpy(comm, current->group_leader->comm, TASK_COMM_LEN);
	*pid = task_tgid_nr(current);
}
EXPORT_SYMBOL_GPL(asahi_current_owner);

static ssize_t agx_stats_raw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t n = 0;
	int tag, w;

	if (!asahi_raw)
		return sysfs_emit(buf, "unsupported\n");
	for (tag = 0; tag < ASAHI_RAW_TAGS; tag++) {
		u32 count = smp_load_acquire(&asahi_raw->count[tag]);

		if (!count)
			continue;
		n += scnprintf(buf + n, PAGE_SIZE - n, "tag %2d count %u:", tag, count);
		for (w = 0; w < ASAHI_RAW_WORDS; w++)
			n += scnprintf(buf + n, PAGE_SIZE - n, " %08x",
				       READ_ONCE(asahi_raw->words[tag][w]));
		n += scnprintf(buf + n, PAGE_SIZE - n, "\n");
	}
	return n;
}

static DEVICE_ATTR(agx_stats_raw, 0444, agx_stats_raw_show, NULL);

/*
 * G17 only: export the static raw statistics table. The table is static, so only the
 * attribute's lifetime matters; asahi_sysfs_unregister() removes it with agx_stats.
 */
int asahi_sysfs_register_raw(struct device *dev, const void *raw, void **handle)
{
	int ret;

	if (!dev || !raw)
		return -ENODEV;
	BUILD_BUG_ON(sizeof(struct asahi_raw_stats) != ASAHI_RAW_TAGS * (1 + ASAHI_RAW_WORDS) * 4);
	asahi_raw = raw;
	ret = device_create_file(dev, &dev_attr_agx_stats_raw);
	if (ret)
		asahi_raw = NULL;
	if (handle)
		*handle = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(asahi_sysfs_register_raw);

/*
 * G17: GPU idle power-off delay (ms). The firmware powers the GPU cores off after 2 ms idle and
 * reads its own delay only at boot; above 2 the driver holds the cores powered until the GPU has
 * been idle this long (power::IDLE_HOLD_MS). Each wake-up costs about 1 ms, so userspace
 * lengthens it on AC power.
 */
static u32 *asahi_idle_word;

static ssize_t idle_off_delay_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(*asahi_idle_word));
}

static ssize_t idle_off_delay_ms_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	u32 ms;

	if (kstrtou32(buf, 0, &ms) || ms < 1 || ms > 1000)
		return -EINVAL;
	WRITE_ONCE(*asahi_idle_word, ms);
	return count;
}

static DEVICE_ATTR_RW(idle_off_delay_ms);

int asahi_sysfs_register_idle(struct device *dev, u32 *word)
{
	int ret;

	if (!dev || !word)
		return -ENODEV;
	asahi_idle_word = word;
	ret = device_create_file(dev, &dev_attr_idle_off_delay_ms);
	if (ret)
		asahi_idle_word = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(asahi_sysfs_register_idle);

void asahi_sysfs_unregister(struct device *dev, void *handle)
{
	struct asahi_stats_attribute *stats = handle;

	if (!dev || !stats)
		return;

	/* Removal drains active show callbacks before either allocation dies. */
	device_remove_file(dev, &stats->attr);
	if (asahi_raw)
		device_remove_file(dev, &dev_attr_agx_stats_raw);
	if (asahi_idle_word) {
		device_remove_file(dev, &dev_attr_idle_off_delay_ms);
		asahi_idle_word = NULL;
	}
	kfree(stats);
}
EXPORT_SYMBOL_GPL(asahi_sysfs_unregister);

MODULE_AUTHOR("AGX driver maintainer");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Asahi AGX firmware stats sysfs shim");
