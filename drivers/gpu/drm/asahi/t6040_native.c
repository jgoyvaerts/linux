// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple M4 Pro (T6040) GPU: power and coprocessor CPU control for the G17
 * runtime.
 *
 * The T6040 device tree describes the GPU without power domains or an
 * operating-point table, so the driver does the few register-level steps
 * itself:
 *  - power: the two PMGR GPU domains, cold and non-automatic before the first
 *    transition, target 15, 10 ms limit; restored children-first only while
 *    the coprocessor has not run;
 *  - the ASC: cold state CPU_CONTROL == 0 and status == 0x6a, RUN (bit 4) in
 *    CPU_CONTROL, stop by clearing it.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define T6040_PMGR_SIZE		0x4000
#define T6040_PMGR_DOMAIN(i)	(0x100 + (i) * 0x10)
#define T6040_PMGR_DOMAINS	2
#define T6040_PMGR_MASK		0x1000030fU
#define T6040_PMGR_COLD_MASK	0x90000cffU
#define T6040_PMGR_ERROR	0x80000c00U
#define T6040_PMGR_ON		15

#define T6040_ASC_CPU_CONTROL	0x44
#define T6040_ASC_CPU_STATUS	0x48
#define T6040_ASC_CPU_RUN	BIT(4)
#define T6040_ASC_COLD_STATUS	0x6a

struct asahi_t6040_native {
	void __iomem *pmgr;
	void __iomem *asc;
	unsigned int powered;	/* domains transitioned by us, in order */
	bool started;		/* RUN was written: power stays on until reboot */
};

struct asahi_t6040_native *asahi_t6040_native_new(struct device *dev, phys_addr_t pmgr);
struct asahi_t6040_native *asahi_t6040_native_new(struct device *dev, phys_addr_t pmgr)
{
	struct resource *asc = platform_get_resource_byname(to_platform_device(dev),
							    IORESOURCE_MEM, "asc");
	struct asahi_t6040_native *n;

	if (!asc || resource_size(asc) < T6040_ASC_CPU_STATUS + 4)
		return ERR_PTR(-EINVAL);
	n = kzalloc(sizeof(*n), GFP_KERNEL);
	if (!n)
		return ERR_PTR(-ENOMEM);
	n->pmgr = ioremap(pmgr, T6040_PMGR_SIZE);
	n->asc = ioremap(asc->start, resource_size(asc));
	if (!n->pmgr || !n->asc) {
		if (n->pmgr)
			iounmap(n->pmgr);
		if (n->asc)
			iounmap(n->asc);
		kfree(n);
		return ERR_PTR(-ENOMEM);
	}
	return n;
}

static int t6040_power_mode(struct asahi_t6040_native *n, unsigned int i, u32 target)
{
	void __iomem *reg = n->pmgr + T6040_PMGR_DOMAIN(i);
	u32 v = readl(reg);
	int ret;

	if (v == ~0U || (v & T6040_PMGR_ERROR))
		return -EIO;
	writel((v & ~T6040_PMGR_MASK) | target, reg);
	ret = readl_poll_timeout_atomic(reg, v,
		v == ~0U || (v & T6040_PMGR_ERROR) || (v & 0xf0) == (target << 4),
		10, 10000);
	if (ret)
		return ret;
	return (v == ~0U || (v & T6040_PMGR_ERROR)) ? -EIO : 0;
}

static void t6040_power_restore(struct asahi_t6040_native *n)
{
	/* Children before parents; keep a parent if its child did not return. */
	while (n->powered) {
		if (t6040_power_mode(n, n->powered - 1, 0)) {
			pr_err("asahi: T6040 GPU power domain %u did not turn off\n",
			       n->powered - 1);
			return;
		}
		n->powered--;
	}
}

int asahi_t6040_power_on(struct asahi_t6040_native *n);
int asahi_t6040_power_on(struct asahi_t6040_native *n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < T6040_PMGR_DOMAINS; i++) {
		u32 v = readl(n->pmgr + T6040_PMGR_DOMAIN(i));

		/* Only the observed cold, non-automatic state is transitioned. */
		if (v & T6040_PMGR_COLD_MASK) {
			pr_err("asahi: T6040 GPU power domain %u not cold: %08x\n", i, v);
			return -EBUSY;
		}
	}
	for (i = 0; i < T6040_PMGR_DOMAINS; i++) {
		ret = t6040_power_mode(n, i, T6040_PMGR_ON);
		if (ret) {
			pr_err("asahi: T6040 GPU power domain %u failed to turn on: %d\n", i, ret);
			t6040_power_restore(n);
			return ret;
		}
		n->powered = i + 1;
	}
	return 0;
}

int asahi_t6040_cpu_cold(struct asahi_t6040_native *n);
int asahi_t6040_cpu_cold(struct asahi_t6040_native *n)
{
	u32 control = readl(n->asc + T6040_ASC_CPU_CONTROL);
	u32 status = readl(n->asc + T6040_ASC_CPU_STATUS);

	if (control || status != T6040_ASC_COLD_STATUS) {
		pr_err("asahi: T6040 GPU coprocessor not cold: %08x/%08x\n", control, status);
		return -EBUSY;
	}
	return 0;
}

void asahi_t6040_cpu_start(struct asahi_t6040_native *n);
void asahi_t6040_cpu_start(struct asahi_t6040_native *n)
{
	n->started = true;
	dsb(sy);
	writel(T6040_ASC_CPU_RUN, n->asc + T6040_ASC_CPU_CONTROL);
	dsb(sy);
}

int asahi_t6040_cpu_stop(struct asahi_t6040_native *n);
int asahi_t6040_cpu_stop(struct asahi_t6040_native *n)
{
	u32 control, status;
	int ret;

	writel(0, n->asc + T6040_ASC_CPU_CONTROL);
	ret = readl_poll_timeout(n->asc + T6040_ASC_CPU_CONTROL, control,
				 !(control & T6040_ASC_CPU_RUN), 50, 100000);
	if (ret)
		return ret;
	return readl_poll_timeout(n->asc + T6040_ASC_CPU_STATUS, status,
				  (status & 3) == 2, 50, 100000);
}

void asahi_t6040_native_free(struct asahi_t6040_native *n);
void asahi_t6040_native_free(struct asahi_t6040_native *n)
{
	if (IS_ERR_OR_NULL(n))
		return;
	/* Once the coprocessor has run it may still own memory: keep it powered. */
	if (!n->started)
		t6040_power_restore(n);
	iounmap(n->asc);
	iounmap(n->pmgr);
	kfree(n);
}
