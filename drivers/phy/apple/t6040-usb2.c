// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple T6040 USB2 PHY -- host-only slice
 *
 * This deliberately does not share the T8103/T8122 ATC PHY compatible. T6040
 * has CIO4/TB5-era resources and a different event-bank layout. The only
 * implemented operation is the exact USB2 host sequence decoded from the
 * paired AppleT6040TypeCPhy driver.
 *
 * The driver performs no ATC-register writes during probe. The power
 * sequences only cover the PHY: the host controller must be stopped before
 * power_off, which dwc3-apple does on every role change.
 *
 * Runtime PM: the PHY core keeps this device active from power_on to
 * power_off, and the DWC3 reset below holds it while the controller is out of
 * reset (the controller runs on the pipehandler's PIPE clock). In between,
 * the PHY's common_dp power state can drop with an idle port.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset-controller.h>

#define T6040_USB2_USBCTL		0x00
#define T6040_USB2_USBCTL_MODE		GENMASK(2, 0)
#define T6040_USB2_USBCTL_MODE_HOST	2

#define T6040_USB2_CTL			0x04
#define T6040_USB2_CTL_RESET		BIT(0)
#define T6040_USB2_CTL_PORT_RESET	BIT(1)
#define T6040_USB2_CTL_APB_RESET_N	BIT(2)
#define T6040_USB2_CTL_SIDDQ		BIT(3)

#define T6040_USB2_SIG			0x08
#define T6040_USB2_SIG_VBUSDET_FORCE_VAL	BIT(0)
#define T6040_USB2_SIG_VBUSDET_FORCE_EN	BIT(1)
#define T6040_USB2_SIG_HOST_0		BIT(12)
#define T6040_USB2_SIG_HOST_1		BIT(13)
#define T6040_USB2_SIG_HOST_2		BIT(14)

#define T6040_USB2_MISCTUNE		0x1c
#define T6040_USB2_MISCTUNE_APBCLK_GATE_OFF	BIT(29)
#define T6040_USB2_MISCTUNE_REFCLK_GATE_OFF	BIT(30)

#define T6040_USB2_EVT_CTL		0x00
#define T6040_USB2_EVT_CTL_EN		BIT(0)
#define T6040_USB2_EVT_CTL_LOAD_CNT	BIT(3)
#define T6040_USB2_EVT_STATUS		0x20

/*
 * DWC3 reset and clamp in the pipehandler AON general register: bit 0 is the
 * controller's VCC reset (cleared = reset asserted), bit 4 forces the clamp.
 * Same layout as PIPEHANDLER_AON_GEN on T8103/T8122 (atc.c).
 */
#define T6040_PIPEHANDLER_AON_GEN		0x1c
#define T6040_PIPEHANDLER_AON_GEN_DWC3_RESET_N	BIT(0)
#define T6040_PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN	BIT(4)

struct t6040_usb2_phy {
	struct device *dev;
	void __iomem *usb2;
	void __iomem *evt;
	struct phy *phy;
	void __iomem *pipehandler;
	struct reset_controller_dev rcdev;
	struct mutex lock;
	enum phy_mode mode;
	bool powered;
	bool shutdown_failed;
	bool dwc3_running;	/* reset deasserted: holds a runtime PM reference */
};

static inline void t6040_usb2_mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);

	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void t6040_usb2_set32(void __iomem *reg, u32 set)
{
	t6040_usb2_mask32(reg, 0, set);
}

static inline void t6040_usb2_clear32(void __iomem *reg, u32 clear)
{
	t6040_usb2_mask32(reg, clear, 0);
}

static int t6040_usb2_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);

	if (mode != PHY_MODE_USB_HOST)
		return -EOPNOTSUPP;

	guard(mutex)(&tphy->lock);
	if (tphy->shutdown_failed)
		return -EIO;
	if (tphy->powered && tphy->mode != mode)
		return -EBUSY;

	tphy->mode = mode;
	return 0;
}

static int t6040_usb2_power_on(struct phy *phy)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);
	u32 status, ctl, sig, tune, mode;

	guard(mutex)(&tphy->lock);
	if (tphy->shutdown_failed)
		return -EIO;

	if (tphy->mode != PHY_MODE_USB_HOST)
		return -EINVAL;
	if (tphy->powered)
		return 0;

	/*
	 * AppleT6040TypeCPhy::eusb2phy_init(false, false), in exact order.
	 * Keep the five signal mutations separate: the paired driver performs
	 * five read/modify/write operations rather than one combined write.
	 */
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_SIG,
			 T6040_USB2_SIG_HOST_2);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_SIG,
			 T6040_USB2_SIG_HOST_1);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_SIG,
			 T6040_USB2_SIG_HOST_0);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_SIG,
			 T6040_USB2_SIG_VBUSDET_FORCE_VAL);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_SIG,
			 T6040_USB2_SIG_VBUSDET_FORCE_EN);

	msleep(10);

	t6040_usb2_clear32(tphy->usb2 + T6040_USB2_CTL,
			   T6040_USB2_CTL_SIDDQ);
	udelay(10);
	t6040_usb2_clear32(tphy->usb2 + T6040_USB2_CTL,
			   T6040_USB2_CTL_RESET);
	t6040_usb2_clear32(tphy->usb2 + T6040_USB2_CTL,
			   T6040_USB2_CTL_PORT_RESET);

	t6040_usb2_set32(tphy->evt + T6040_USB2_EVT_CTL,
			 T6040_USB2_EVT_CTL_LOAD_CNT |
			 T6040_USB2_EVT_CTL_EN);

	t6040_usb2_set32(tphy->usb2 + T6040_USB2_CTL,
			 T6040_USB2_CTL_APB_RESET_N);
	t6040_usb2_clear32(tphy->usb2 + T6040_USB2_MISCTUNE,
			   T6040_USB2_MISCTUNE_APBCLK_GATE_OFF);
	t6040_usb2_clear32(tphy->usb2 + T6040_USB2_MISCTUNE,
			   T6040_USB2_MISCTUNE_REFCLK_GATE_OFF);

	udelay(30);
	status = readl(tphy->evt + T6040_USB2_EVT_STATUS);
	if (status == U32_MAX)
		goto failed;
	dev_info(tphy->dev, "USB2 event status %#x\n", status);

	msleep(5);
	t6040_usb2_mask32(tphy->usb2 + T6040_USB2_USBCTL,
			  T6040_USB2_USBCTL_MODE,
			  T6040_USB2_USBCTL_MODE_HOST);

	/* Confirm the programmed fields before the core can start the host. */
	ctl = readl(tphy->usb2 + T6040_USB2_CTL);
	sig = readl(tphy->usb2 + T6040_USB2_SIG);
	tune = readl(tphy->usb2 + T6040_USB2_MISCTUNE);
	mode = readl(tphy->usb2 + T6040_USB2_USBCTL);
	status = readl(tphy->evt + T6040_USB2_EVT_CTL);
	if (ctl == U32_MAX || sig == U32_MAX || tune == U32_MAX ||
	    mode == U32_MAX || status == U32_MAX ||
	    (ctl & (T6040_USB2_CTL_RESET | T6040_USB2_CTL_PORT_RESET |
		    T6040_USB2_CTL_SIDDQ | T6040_USB2_CTL_APB_RESET_N)) !=
		   T6040_USB2_CTL_APB_RESET_N ||
	    (sig & (T6040_USB2_SIG_HOST_0 | T6040_USB2_SIG_HOST_1 |
		    T6040_USB2_SIG_HOST_2 | T6040_USB2_SIG_VBUSDET_FORCE_VAL |
		    T6040_USB2_SIG_VBUSDET_FORCE_EN)) !=
		   (T6040_USB2_SIG_HOST_0 | T6040_USB2_SIG_HOST_1 |
		    T6040_USB2_SIG_HOST_2 | T6040_USB2_SIG_VBUSDET_FORCE_VAL |
		    T6040_USB2_SIG_VBUSDET_FORCE_EN) ||
	    (tune & (T6040_USB2_MISCTUNE_APBCLK_GATE_OFF |
		     T6040_USB2_MISCTUNE_REFCLK_GATE_OFF)) ||
	    (mode & T6040_USB2_USBCTL_MODE) != T6040_USB2_USBCTL_MODE_HOST ||
	    !(status & T6040_USB2_EVT_CTL_EN))
		goto failed;

	tphy->powered = true;
	dev_info(tphy->dev, "USB2 host sequence complete\n");
	return 0;
failed:
	tphy->shutdown_failed = true;
	return -EIO;
}

/*
 * Power the PHY down: port reset (held 5 ms), USBCTL mode 4, SIDDQ, PHY
 * reset, then gate the APB and reference clocks. The host controller must be
 * halted first; this is not a substitute for an xHCI halt.
 */
static int t6040_usb2_power_off(struct phy *phy)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);
	u32 ctl, mode, tune;

	guard(mutex)(&tphy->lock);
	if (tphy->shutdown_failed)
		return -EIO;
	if (!tphy->powered)
		return 0;
	tphy->shutdown_failed = true;

	if (!(readl(tphy->usb2 + T6040_USB2_CTL) & T6040_USB2_CTL_PORT_RESET)) {
		t6040_usb2_set32(tphy->usb2 + T6040_USB2_CTL,
				 T6040_USB2_CTL_PORT_RESET);
		msleep(5);
	}
	t6040_usb2_mask32(tphy->usb2 + T6040_USB2_USBCTL,
			  T6040_USB2_USBCTL_MODE, 4);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_CTL,
			 T6040_USB2_CTL_SIDDQ);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_CTL,
			 T6040_USB2_CTL_RESET);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_MISCTUNE,
			 T6040_USB2_MISCTUNE_APBCLK_GATE_OFF);
	t6040_usb2_set32(tphy->usb2 + T6040_USB2_MISCTUNE,
			 T6040_USB2_MISCTUNE_REFCLK_GATE_OFF);
	/* The vendor driver waits 500 ns after gating the clocks. */
	ndelay(500);

	/*
	 * Do not report the PHY off, or allow another power cycle, after a
	 * write that did not land.
	 */
	ctl = readl(tphy->usb2 + T6040_USB2_CTL);
	mode = readl(tphy->usb2 + T6040_USB2_USBCTL);
	tune = readl(tphy->usb2 + T6040_USB2_MISCTUNE);
	if (ctl == U32_MAX || mode == U32_MAX || tune == U32_MAX ||
	    (ctl & (T6040_USB2_CTL_PORT_RESET | T6040_USB2_CTL_SIDDQ |
		    T6040_USB2_CTL_RESET)) !=
		   (T6040_USB2_CTL_PORT_RESET | T6040_USB2_CTL_SIDDQ |
		    T6040_USB2_CTL_RESET) ||
	    (mode & T6040_USB2_USBCTL_MODE) != 4 ||
	    (tune & (T6040_USB2_MISCTUNE_APBCLK_GATE_OFF |
		     T6040_USB2_MISCTUNE_REFCLK_GATE_OFF)) !=
		    (T6040_USB2_MISCTUNE_APBCLK_GATE_OFF |
		     T6040_USB2_MISCTUNE_REFCLK_GATE_OFF))
		return -EIO;
	tphy->shutdown_failed = false;
	tphy->powered = false;
	return 0;
}

static const struct phy_ops t6040_usb2_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = t6040_usb2_set_mode,
	.power_on = t6040_usb2_power_on,
	.power_off = t6040_usb2_power_off,
};

/*
 * dwc3-apple asserts this at probe, deasserts it before dwc3_core_init()
 * (which then powers this PHY on), and asserts it again on stop. As in
 * atc.c, assert drops RESET_N and then sets the clamp; deassert releases
 * the clamp and then sets RESET_N.
 */
static int t6040_usb2_reset_assert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct t6040_usb2_phy *tphy =
		container_of(rcdev, struct t6040_usb2_phy, rcdev);
	int ret;

	PM_RUNTIME_ACQUIRE(tphy->dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&tphy->lock);
	t6040_usb2_clear32(tphy->pipehandler + T6040_PIPEHANDLER_AON_GEN,
			   T6040_PIPEHANDLER_AON_GEN_DWC3_RESET_N);
	t6040_usb2_set32(tphy->pipehandler + T6040_PIPEHANDLER_AON_GEN,
			 T6040_PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
	if (tphy->dwc3_running) {
		tphy->dwc3_running = false;
		pm_runtime_put(tphy->dev);
	}
	return 0;
}

static int t6040_usb2_reset_deassert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct t6040_usb2_phy *tphy =
		container_of(rcdev, struct t6040_usb2_phy, rcdev);
	u32 value;
	int ret;

	PM_RUNTIME_ACQUIRE(tphy->dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&tphy->lock);
	if (tphy->shutdown_failed)
		return -EIO;
	t6040_usb2_clear32(tphy->pipehandler + T6040_PIPEHANDLER_AON_GEN,
			   T6040_PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
	t6040_usb2_set32(tphy->pipehandler + T6040_PIPEHANDLER_AON_GEN,
			 T6040_PIPEHANDLER_AON_GEN_DWC3_RESET_N);

	value = readl(tphy->pipehandler + T6040_PIPEHANDLER_AON_GEN);
	if (value == U32_MAX ||
	    (value & (T6040_PIPEHANDLER_AON_GEN_DWC3_RESET_N |
		      T6040_PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN)) !=
		    T6040_PIPEHANDLER_AON_GEN_DWC3_RESET_N)
		return -EIO;
	if (!tphy->dwc3_running) {
		pm_runtime_get_noresume(tphy->dev);
		tphy->dwc3_running = true;
	}
	return 0;
}

static const struct reset_control_ops t6040_usb2_reset_ops = {
	.assert = t6040_usb2_reset_assert,
	.deassert = t6040_usb2_reset_deassert,
};

static int t6040_usb2_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

static struct phy *t6040_usb2_xlate(struct device *dev,
				    const struct of_phandle_args *args)
{
	struct t6040_usb2_phy *tphy = dev_get_drvdata(dev);

	if (args->args_count != 1 || args->args[0] != PHY_TYPE_USB2)
		return ERR_PTR(-ENODEV);

	return tphy->phy;
}

static int t6040_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct t6040_usb2_phy *tphy;
	struct phy_provider *provider;
	int ret;

	tphy = devm_kzalloc(dev, sizeof(*tphy), GFP_KERNEL);
	if (!tphy)
		return -ENOMEM;

	tphy->dev = dev;
	/*
	 * dwc3 powers on its phys from dwc3_core_init(), which runs inside
	 * dwc3_core_probe() -- before any consumer can deliver phy_set_mode()
	 * to this phy. dwc3-apple's early phy_set_mode() call happens while
	 * dwc.usb2_generic_phy[] is still unpopulated, so it is a silent no-op
	 * on a NULL phy, and a PHY_MODE_INVALID default then fails power_on
	 * with -EINVAL ("failed to initialize core", ticket 108). This
	 * provider implements exactly one mode, so start in it; set_mode
	 * still rejects every request that is not PHY_MODE_USB_HOST.
	 */
	tphy->mode = PHY_MODE_USB_HOST;
	mutex_init(&tphy->lock);
	platform_set_drvdata(pdev, tphy);

	tphy->usb2 = devm_platform_ioremap_resource_byname(pdev, "bank0");
	if (IS_ERR(tphy->usb2))
		return dev_err_probe(dev, PTR_ERR(tphy->usb2),
				     "failed to map USB2 bank 0\n");

	tphy->evt = devm_platform_ioremap_resource_byname(pdev, "bank1");
	if (IS_ERR(tphy->evt))
		return dev_err_probe(dev, PTR_ERR(tphy->evt),
				     "failed to map USB2 event bank 1\n");

	/* Before devm_phy_create(): the PHY core then manages the references. */
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	tphy->phy = devm_phy_create(dev, NULL, &t6040_usb2_phy_ops);
	if (IS_ERR(tphy->phy))
		return PTR_ERR(tphy->phy);
	phy_set_drvdata(tphy->phy, tphy);

	tphy->pipehandler = devm_platform_ioremap_resource_byname(pdev, "pipehandler");
	if (IS_ERR(tphy->pipehandler))
		return dev_err_probe(dev, PTR_ERR(tphy->pipehandler),
				     "failed to map pipehandler bank\n");

	tphy->rcdev.owner = THIS_MODULE;
	tphy->rcdev.nr_resets = 1;
	tphy->rcdev.ops = &t6040_usb2_reset_ops;
	tphy->rcdev.of_node = dev->of_node;
	tphy->rcdev.of_reset_n_cells = 0;
	tphy->rcdev.of_xlate = t6040_usb2_reset_xlate;
	ret = devm_reset_controller_register(dev, &tphy->rcdev);
	if (ret)
		return ret;

	provider = devm_of_phy_provider_register(dev, t6040_usb2_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id t6040_usb2_of_match[] = {
	{ .compatible = "apple,t6040-atcphy" },
	{}
};
MODULE_DEVICE_TABLE(of, t6040_usb2_of_match);

static struct platform_driver t6040_usb2_driver = {
	.probe = t6040_usb2_probe,
	.driver = {
		.name = "phy-apple-t6040-usb2",
		.of_match_table = t6040_usb2_of_match,
	},
};
module_platform_driver(t6040_usb2_driver);

MODULE_AUTHOR("The Asahi Linux Contributors");
MODULE_DESCRIPTION("Experimental Apple T6040 USB2 PHY");
MODULE_LICENSE("Dual BSD/GPL");
