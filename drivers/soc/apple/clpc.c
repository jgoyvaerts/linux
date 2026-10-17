// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC host performance controller (CLPC) floors
 *
 * On macOS the CLPC raises the floor of the memory controller (DCS) and the
 * SoC fabric while the CPU clusters or the GPU are busy, through its own
 * voter agent in the PMC. This driver keeps that agent's floor in step with
 * Linux CPU load:
 *
 *  - every sampling period, each cpufreq policy reports how busy its
 *    busiest CPU was and how fast the policy runs relative to its maximum;
 *    a busy policy requests a DCS and fabric state from a table indexed by
 *    that frequency quartile;
 *  - each rail keeps its last request for a hold time after demand stops,
 *    then drops back to 0, so idle releases the floor.
 *
 * The native mapping from CPU demand to DCS and fabric states is not known.
 * The tables below are measured policy, not a copy of macOS.
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/soc/apple/pmc.h>
#include <linux/workqueue.h>

#define APPLE_CLPC_MAX_CLUSTERS	8
#define APPLE_CLPC_LEVELS	4

enum apple_clpc_rail {
	APPLE_CLPC_DCS,
	APPLE_CLPC_SOC,
	APPLE_CLPC_RAILS,
};

/* Sampling period */
#define APPLE_CLPC_SAMPLE_MS	4
/* Busy threshold of a CPU cluster, percent of the sampling period */
#define APPLE_CLPC_BUSY_PCT	50
/* Time a rail keeps its last request after demand stops */
#define APPLE_CLPC_HOLD_MS	32

/**
 * struct apple_clpc_hw - per-SoC CLPC parameters
 * @rail: PMC rail of the DCS and of the fabric
 * @map: state of each rail for a busy cluster below 25, 50, 75 and 100
 *	percent of its maximum frequency
 */
struct apple_clpc_hw {
	unsigned int rail[APPLE_CLPC_RAILS];
	u8 map[APPLE_CLPC_RAILS][APPLE_CLPC_LEVELS];
};

struct apple_clpc_cluster {
	unsigned int policy_cpu;
	cpumask_var_t cpus;
	unsigned int busy;
	unsigned int level;
	unsigned int request[APPLE_CLPC_RAILS];
};

struct apple_clpc_vote {
	unsigned int state;
	unsigned long last_demand;
};

struct apple_clpc {
	struct device *dev;
	struct device *pmc;
	const struct apple_clpc_hw *hw;
	unsigned int agent;
	/* Samples while a vote is held; idle_work defers to busy CPUs. */
	struct delayed_work work;
	struct delayed_work idle_work;
	bool stopped;
	struct apple_clpc_cluster clusters[APPLE_CLPC_MAX_CLUSTERS];
	unsigned int nr_clusters;
	cpumask_var_t covered;
	unsigned long next_scan;
	u64 *idle;
	u64 *wall;
	struct apple_clpc_vote vote[APPLE_CLPC_RAILS];
	u32 mask;
	u32 saved_floor;
	unsigned long samples;
	unsigned long update_errors;
	struct dentry *debugfs;
};

/* cpufreq policies can appear after probe; pick up new ones as they do. */
static void apple_clpc_find_clusters(struct apple_clpc *clpc)
{
	unsigned int cpu, i;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct apple_clpc_cluster *cl;

		if (cpumask_test_cpu(cpu, clpc->covered))
			continue;
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		for (i = 0; i < clpc->nr_clusters; i++)
			if (clpc->clusters[i].policy_cpu == policy->cpu)
				break;
		if (i == clpc->nr_clusters && i < APPLE_CLPC_MAX_CLUSTERS) {
			cl = &clpc->clusters[clpc->nr_clusters++];
			cl->policy_cpu = policy->cpu;
		}
		if (i < clpc->nr_clusters) {
			cl = &clpc->clusters[i];
			cpumask_set_cpu(cpu, cl->cpus);
			cpumask_set_cpu(cpu, clpc->covered);
			clpc->idle[cpu] = get_cpu_idle_time(cpu, &clpc->wall[cpu], 0);
		}
		cpufreq_cpu_put(policy);
	}
}

static void apple_clpc_sample_cluster(struct apple_clpc *clpc,
				      struct apple_clpc_cluster *cl)
{
	struct cpufreq_policy *policy;
	unsigned int cpu, busy = 0, q, rail;

	for_each_cpu(cpu, cl->cpus) {
		u64 wall, idle, dw, di;
		bool valid;

		/*
		 * An offline CPU accrues no idle time while the wall clock runs
		 * on, so it would look fully busy. Skip it, and take a fresh
		 * baseline once it is back.
		 */
		if (!cpu_online(cpu)) {
			clpc->wall[cpu] = 0;
			continue;
		}

		idle = get_cpu_idle_time(cpu, &wall, 0);
		dw = wall - clpc->wall[cpu];
		di = idle - clpc->idle[cpu];
		valid = clpc->wall[cpu];
		clpc->wall[cpu] = wall;
		clpc->idle[cpu] = idle;
		if (valid && dw && di <= dw)
			busy = max_t(unsigned int, busy,
				     div64_u64((dw - di) * 100, dw));
	}
	cl->busy = busy;

	cl->level = 0;
	policy = cpufreq_cpu_get(cl->policy_cpu);
	if (policy) {
		if (policy->cpuinfo.max_freq)
			cl->level = policy->cur * 100 / policy->cpuinfo.max_freq;
		cpufreq_cpu_put(policy);
	}

	q = min_t(unsigned int, cl->level / (100 / APPLE_CLPC_LEVELS),
		  APPLE_CLPC_LEVELS - 1);
	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		cl->request[rail] = busy >= APPLE_CLPC_BUSY_PCT ?
				   clpc->hw->map[rail][q] : 0;
}

static unsigned int apple_clpc_vote(struct apple_clpc_vote *vote,
				    unsigned int demand, unsigned long now)
{
	if (demand) {
		vote->last_demand = now;
		vote->state = demand;
	} else if (time_after_eq(now, vote->last_demand +
					      msecs_to_jiffies(APPLE_CLPC_HOLD_MS))) {
		vote->state = 0;
	}

	return vote->state;
}

static u32 apple_clpc_floor(struct apple_clpc *clpc, const unsigned int *state)
{
	u32 floor = 0;
	unsigned int rail;

	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		floor |= (state[rail] & 0xf) <<
			 APPLE_PMC_RAIL_SHIFT(clpc->hw->rail[rail]);

	return floor;
}

static void apple_clpc_queue(struct apple_clpc *clpc, bool idle)
{
	if (READ_ONCE(clpc->stopped))
		return;

	/*
	 * A held vote has to be sampled on time so that it drops once demand
	 * stops; with no vote held, sampling can wait for a CPU that wakes up
	 * anyway rather than waking an idle one every period.
	 */
	queue_delayed_work(system_freezable_power_efficient_wq,
			   idle ? &clpc->idle_work : &clpc->work,
			   msecs_to_jiffies(APPLE_CLPC_SAMPLE_MS));
}

static void apple_clpc_sample(struct apple_clpc *clpc)
{
	unsigned int demand[APPLE_CLPC_RAILS] = {};
	unsigned int state[APPLE_CLPC_RAILS];
	unsigned long now = jiffies;
	unsigned int i, rail;

	clpc->samples++;
	if (!cpumask_equal(clpc->covered, cpu_possible_mask) &&
	    time_after_eq(now, clpc->next_scan)) {
		apple_clpc_find_clusters(clpc);
		clpc->next_scan = now + HZ;
	}

	for (i = 0; i < clpc->nr_clusters; i++) {
		struct apple_clpc_cluster *cl = &clpc->clusters[i];

		apple_clpc_sample_cluster(clpc, cl);
		for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
			demand[rail] = max(demand[rail], cl->request[rail]);
	}

	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		state[rail] = apple_clpc_vote(&clpc->vote[rail], demand[rail],
					      now);

	if (apple_pmc_floor_update(clpc->pmc, clpc->agent, clpc->mask,
				   apple_clpc_floor(clpc, state)))
		clpc->update_errors++;

	apple_clpc_queue(clpc, !apple_clpc_floor(clpc, state));
}

static void apple_clpc_work(struct work_struct *work)
{
	apple_clpc_sample(container_of(to_delayed_work(work), struct apple_clpc,
				       work));
}

static void apple_clpc_idle_work(struct work_struct *work)
{
	apple_clpc_sample(container_of(to_delayed_work(work), struct apple_clpc,
				       idle_work));
}

static int apple_clpc_status_show(struct seq_file *s, void *unused)
{
	struct apple_clpc *clpc = s->private;
	unsigned int i, rail;

	seq_printf(s, "agent: %u floor: %#010x (saved %#010x)\n", clpc->agent,
		   apple_pmc_floor_read(clpc->pmc, clpc->agent),
		   clpc->saved_floor);
	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		seq_printf(s, "%s: rail %u state %u enabled %u map %u,%u,%u,%u\n",
			   rail == APPLE_CLPC_DCS ? "dcs" : "soc",
			   clpc->hw->rail[rail], clpc->vote[rail].state,
			   apple_pmc_voter_enabled(clpc->pmc, clpc->agent,
						   clpc->hw->rail[rail]),
			   clpc->hw->map[rail][0], clpc->hw->map[rail][1],
			   clpc->hw->map[rail][2], clpc->hw->map[rail][3]);
	seq_printf(s, "samples: %lu update errors: %lu\n", clpc->samples,
		   clpc->update_errors);
	for (i = 0; i < clpc->nr_clusters; i++)
		seq_printf(s, "cluster cpu%u: busy %u%% level %u%% -> dcs %u soc %u\n",
			   clpc->clusters[i].policy_cpu, clpc->clusters[i].busy,
			   clpc->clusters[i].level,
			   clpc->clusters[i].request[APPLE_CLPC_DCS],
			   clpc->clusters[i].request[APPLE_CLPC_SOC]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_clpc_status);

static void apple_clpc_free_clusters(struct apple_clpc *clpc)
{
	unsigned int i;

	for (i = 0; i < APPLE_CLPC_MAX_CLUSTERS; i++)
		free_cpumask_var(clpc->clusters[i].cpus);
	free_cpumask_var(clpc->covered);
}

/* Stop sampling and give the floor back, before suspend and on removal. */
static void apple_clpc_halt(struct apple_clpc *clpc)
{
	unsigned int rail, i;

	/*
	 * Each pass queues one of the two works unless stopped. A pass that
	 * began before the flag was set can still queue the other work once,
	 * which the second round cancels.
	 */
	WRITE_ONCE(clpc->stopped, true);
	for (i = 0; i < 2; i++) {
		cancel_delayed_work_sync(&clpc->work);
		cancel_delayed_work_sync(&clpc->idle_work);
	}
	apple_pmc_floor_update(clpc->pmc, clpc->agent, clpc->mask,
			       clpc->saved_floor);
	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		clpc->vote[rail].state = 0;
}

static void apple_clpc_stop(void *data)
{
	struct apple_clpc *clpc = data;

	apple_clpc_halt(clpc);
	debugfs_remove_recursive(clpc->debugfs);
	apple_clpc_free_clusters(clpc);
}

static int apple_clpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_clpc *clpc;
	unsigned int rail, i;
	int ret;

	clpc = devm_kzalloc(dev, sizeof(*clpc), GFP_KERNEL);
	if (!clpc)
		return -ENOMEM;

	clpc->dev = dev;
	clpc->pmc = dev->parent;
	clpc->hw = of_device_get_match_data(dev);
	if (!apple_pmc_rail_count(clpc->pmc))
		return dev_err_probe(dev, -ENODEV, "parent is not a PMC\n");

	ret = of_property_read_u32(dev->of_node, "reg", &clpc->agent);
	if (ret)
		return dev_err_probe(dev, ret, "missing voter agent\n");

	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++) {
		if (clpc->hw->rail[rail] >= apple_pmc_rail_count(clpc->pmc))
			return -EINVAL;
		clpc->mask |= APPLE_PMC_RAIL_MASK(clpc->hw->rail[rail]);
	}

	clpc->idle = devm_kcalloc(dev, nr_cpu_ids, sizeof(*clpc->idle),
				  GFP_KERNEL);
	clpc->wall = devm_kcalloc(dev, nr_cpu_ids, sizeof(*clpc->wall),
				  GFP_KERNEL);
	if (!clpc->idle || !clpc->wall)
		return -ENOMEM;

	for (i = 0; i < APPLE_CLPC_MAX_CLUSTERS; i++) {
		if (!zalloc_cpumask_var(&clpc->clusters[i].cpus, GFP_KERNEL)) {
			apple_clpc_free_clusters(clpc);
			return -ENOMEM;
		}
	}
	if (!zalloc_cpumask_var(&clpc->covered, GFP_KERNEL)) {
		apple_clpc_free_clusters(clpc);
		return -ENOMEM;
	}

	for (rail = 0; rail < APPLE_CLPC_RAILS; rail++)
		if (!apple_pmc_voter_enabled(clpc->pmc, clpc->agent,
					     clpc->hw->rail[rail]))
			dev_warn(dev, "voter agent %u is not enabled on rail %u\n",
				 clpc->agent, clpc->hw->rail[rail]);

	clpc->saved_floor = apple_pmc_floor_read(clpc->pmc, clpc->agent) &
			    clpc->mask;
	apple_clpc_find_clusters(clpc);
	clpc->next_scan = jiffies + HZ;
	INIT_DELAYED_WORK(&clpc->work, apple_clpc_work);
	INIT_DEFERRABLE_WORK(&clpc->idle_work, apple_clpc_idle_work);

	clpc->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0400, clpc->debugfs, clpc,
			    &apple_clpc_status_fops);

	ret = devm_add_action_or_reset(dev, apple_clpc_stop, clpc);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, clpc);
	apple_clpc_queue(clpc, true);
	return 0;
}

/*
 * Tasks are frozen before devices suspend, and with them the sampling, so
 * the floor would otherwise stay at whatever the last busy sample asked for
 * through the whole sleep.
 */
static int apple_clpc_suspend(struct device *dev)
{
	apple_clpc_halt(dev_get_drvdata(dev));
	return 0;
}

static int apple_clpc_resume(struct device *dev)
{
	struct apple_clpc *clpc = dev_get_drvdata(dev);
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		clpc->wall[cpu] = 0;
	WRITE_ONCE(clpc->stopped, false);
	apple_clpc_queue(clpc, true);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_clpc_pm_ops, apple_clpc_suspend,
				apple_clpc_resume);

/*
 * T8140: DCS states 0-4 on rail 0, fabric states 0-3 on rail 1. A busy
 * cluster raises the DCS one state per frequency quartile and leaves the
 * fabric alone.
 */
static const struct apple_clpc_hw apple_clpc_hw_t8140 = {
	.rail = { [APPLE_CLPC_DCS] = 0, [APPLE_CLPC_SOC] = 1 },
	.map = {
		[APPLE_CLPC_DCS] = { 1, 2, 3, 4 },
		[APPLE_CLPC_SOC] = { 0, 0, 0, 0 },
	},
};

/*
 * T6040: DCS states 0-5 on rail 0, fabric states 0-2 on rail 1. No cluster
 * runs below a quarter of its maximum (P 1260 of 4512 MHz, E 1020 of
 * 2592 MHz), so the first column only completes the table. A busy cluster
 * in the upper half asks for the top states. With the floor held at DCS 5
 * and fabric 2 the CPUs read 237-242 GB/s at 135 ns DRAM latency, against
 * 41 GB/s and 314 ns at 0 and 0. The second column is provisional until the
 * power of each DCS state is measured.
 */
static const struct apple_clpc_hw apple_clpc_hw_t6040 = {
	.rail = { [APPLE_CLPC_DCS] = 0, [APPLE_CLPC_SOC] = 1 },
	.map = {
		[APPLE_CLPC_DCS] = { 1, 2, 5, 5 },
		[APPLE_CLPC_SOC] = { 0, 1, 2, 2 },
	},
};

static const struct of_device_id apple_clpc_of_match[] = {
	{ .compatible = "apple,t8140-clpc", .data = &apple_clpc_hw_t8140 },
	{ .compatible = "apple,t6041-clpc", .data = &apple_clpc_hw_t6040 },
	{}
};
MODULE_DEVICE_TABLE(of, apple_clpc_of_match);

static struct platform_driver apple_clpc_driver = {
	.probe = apple_clpc_probe,
	.driver = {
		.name = "apple-clpc",
		.of_match_table = apple_clpc_of_match,
		.pm = pm_sleep_ptr(&apple_clpc_pm_ops),
	},
};
module_platform_driver(apple_clpc_driver);

MODULE_DESCRIPTION("Apple SoC host performance controller floors");
MODULE_LICENSE("Dual MIT/GPL");
