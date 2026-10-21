// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple T6040 Type-C PHY -- USB2 and USB3 host
 *
 * This deliberately does not share the T8103/T8122 ATC PHY compatible. T6040
 * has CIO4/TB5-era resources and a different event-bank layout. The
 * implemented operations are the USB2 host sequence and the USB3 host attach
 * (mode 4) decoded from the paired AppleT6040TypeCPhy driver.
 *
 * USB3: the Type-C orientation switch records the plug orientation; power_on
 * of the USB3 PHY wakes the PHY, applies the tunables and the crossbar for
 * that orientation and starts the PHY coprocessor (RTKit), which signals
 * that it has tuned the PHY; set_mode then moves the controller's PIPE from
 * the dummy PHY to this one, which dwc3-apple does after the core init and
 * before xHCI starts. power_off parks the PIPE again and puts the
 * coprocessor to sleep with the Power Down message, as macOS does on detach.
 * Without a plug orientation, or when the coprocessor does not come up, the
 * PIPE stays on the dummy PHY and the port keeps working at USB2.
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
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset-controller.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/tunable.h>
#include <linux/usb/typec_mux.h>

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

/* The controller's PIPE mux: the M1-M3 layout of atc.c. */
#define T6040_PIPEHANDLER_OVERRIDE		0x00
#define T6040_PIPEHANDLER_OVERRIDE_EN		BIT(0)
#define T6040_PIPEHANDLER_OVERRIDE_EN2		BIT(2)
#define T6040_PIPEHANDLER_OVERRIDE_VALUES	0x04
#define T6040_PIPEHANDLER_MUX_CTRL		0x0c
#define T6040_PIPEHANDLER_MUX_CTRL_CLK		GENMASK(5, 3)
#define T6040_PIPEHANDLER_MUX_CTRL_DATA		GENMASK(1, 0)
#define T6040_PIPEHANDLER_MUX_CLK_USB3		1
#define T6040_PIPEHANDLER_MUX_CLK_DUMMY		4
#define T6040_PIPEHANDLER_MUX_DATA_USB3		0
#define T6040_PIPEHANDLER_MUX_DATA_DUMMY	2
#define T6040_PIPEHANDLER_LOCK_REQ		0x10
#define T6040_PIPEHANDLER_LOCK_ACK		0x14
#define T6040_PIPEHANDLER_LOCK_EN		BIT(0)
#define T6040_PIPEHANDLER_NONSELECTED_OVERRIDE	0x20
#define T6040_PIPEHANDLER_NONSELECTED_MODE	GENMASK(3, 0)
#define T6040_PIPEHANDLER_NONSELECTED_BUSY	BIT(12)
#define T6040_PIPEHANDLER_DUMMY_PHY_READY	BIT(15)

/* PHY core ("core"), atc-phy reg[2] and the blocks behind it */
#define T6040_CORE_CMN_OVRD		0x0c
#define T6040_CORE_UC_CTRL		0x260	/* ACIOPHY_TOP_BLK_UC_CTRL_REG */
#define T6040_CORE_UC_CTRL_TNBL_DONE	BIT(10)	/* the coprocessor has tuned the PHY */
#define T6040_CORE_XBAR			0x6c
#define T6040_CORE_XBAR_LANES		0x78
#define T6040_CORE_BIST			0x29000	/* ACIOPHY_BIST_BLK, atc-phy reg[35] */
#define T6040_BIST_CFG0			0x004
#define T6040_BIST_CFG1			0x008
#define T6040_BIST_STATUS		0x104
#define T6040_BIST_STATUS_PHY		BIT(6)
#define T6040_BIST_STATUS_PHY_2		BIT(8)
#define T6040_BIST_OVRD0		0x10c
#define T6040_BIST_OVRD1		0x110

/* ATCPHY_CFG ("cfg"), atc-phy reg[39]: power requests and their acks */
#define T6040_CFG_CTRL			0x00
#define T6040_CFG_CTRL_SLEEP_B_SML	BIT(0)
#define T6040_CFG_CTRL_SLEEP_B_BIG	BIT(1)
#define T6040_CFG_CTRL_CLAMP		BIT(2)
#define T6040_CFG_CTRL_APB_RESET_N	BIT(3)
#define T6040_CFG_CTRL_PHY_RESET_N	BIT(4)
#define T6040_CFG_STATUS		0x04
#define T6040_CFG_STATUS_SLEEP_B_SML	BIT(0)
#define T6040_CFG_STATUS_SLEEP_B_BIG	BIT(1)
#define T6040_CFG_STATUS_AHB_CLK	BIT(8)
#define T6040_CFG_SPARE0		0x08
#define T6040_CFG_SPARE0_USB_LN_SWAP	BIT(1)
#define T6040_CFG_AHB			0x28
#define T6040_CFG_AHB_CLK_REQ_N		BIT(0)

/* MXWrap CPU control ("cpu"), atc-phy reg[38], of the PHY coprocessor */
#define T6040_CPU_RUN			0x28
#define T6040_CPU_RUN_START		0x10	/* AppleMxWrap::_runCPU */
#define T6040_CPU_IDLE_CTRL		0x40	/* MXWRAP_BLK_IDLE_CTRL */
#define T6040_CPU_IDLE_CTRL_EN		BIT(0)

/* ADT acio-phy-cpuN segment-ranges: the coprocessor sees its __DATA SRAM ("cpu-data") here */
#define T6040_CPU_DATA_IOVA		0x10000000
/* The coprocessor's application endpoint, and the one message macOS sends it */
#define T6040_CPU_EP_APP		32
#define T6040_CPU_MSG_POWER_DOWN	0x0100000000000000ULL

enum t6040_usb3_region {
	T6040_USB3_AXI2AF,
	T6040_USB3_AXI2AF_LIOA,
	T6040_USB3_FABRIC,
	T6040_USB3_COMMON_CFG,
	T6040_USB3_CORE,
	T6040_USB3_CFG,
	T6040_USB3_CPU,
	T6040_USB3_NR_REGIONS,
};

static const char * const t6040_usb3_region_names[T6040_USB3_NR_REGIONS] = {
	[T6040_USB3_AXI2AF] = "axi2af",
	[T6040_USB3_AXI2AF_LIOA] = "axi2af-lioa",
	[T6040_USB3_FABRIC] = "fabric",
	[T6040_USB3_COMMON_CFG] = "common-cfg",
	[T6040_USB3_CORE] = "core",
	[T6040_USB3_CFG] = "cfg",
	[T6040_USB3_CPU] = "cpu",
};

/*
 * The ADT tables AppleT6040TypeCPhy applies at every USB3 power-on (mode 4),
 * in its order; the device tree carries them per port, as calibration.
 */
static const struct {
	const char *prop;
	enum t6040_usb3_region region;
} t6040_usb3_tunable_list[] = {
	{ "apple,tunable-axi2af", T6040_USB3_AXI2AF },
	{ "apple,tunable-fabric", T6040_USB3_FABRIC },
	{ "apple,tunable-common-cfg", T6040_USB3_COMMON_CFG },
	{ "apple,tunable-axi2af-lioa", T6040_USB3_AXI2AF_LIOA },
	{ "apple,tunable-lane0", T6040_USB3_CORE },
	{ "apple,tunable-lane1", T6040_USB3_CORE },
	{ "apple,tunable-aus40cmn-shm", T6040_USB3_CORE },
	{ "apple,tunable-auspll", T6040_USB3_CORE },
	{ "apple,tunable-cio4pll", T6040_USB3_CORE },
};

#define T6040_USB3_TUNABLE_AUSPLL	7
#define T6040_USB3_NR_TUNABLES		ARRAY_SIZE(t6040_usb3_tunable_list)

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

	/* USB3; usb3_ok is false when the device tree lacks part of it */
	struct phy *usb3;
	bool usb3_ok;
	void __iomem *regs[T6040_USB3_NR_REGIONS];
	struct resource *cpu_data;
	struct apple_tunable *tunables[T6040_USB3_NR_TUNABLES];
	struct apple_tunable *tunable_pipehandler;
	struct apple_rtkit *rtk;
	struct completion app_reply;
	enum typec_orientation orientation;
	bool usb3_up;		/* tuned by the coprocessor this power cycle */
	bool pipe_switched;	/* the controller's PIPE runs on this PHY */
	bool cpu_booted;	/* the coprocessor ran under this driver this boot */
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

/* ---- USB3 ---- */

#define t6040_core(tphy)	((tphy)->regs[T6040_USB3_CORE])
#define t6040_cfg(tphy)		((tphy)->regs[T6040_USB3_CFG])
#define t6040_cpu(tphy)		((tphy)->regs[T6040_USB3_CPU])
#define t6040_bist(tphy)	(t6040_core(tphy) + T6040_CORE_BIST)

static int t6040_usb3_cfg_ack(struct t6040_usb2_phy *tphy, const char *what,
			      u32 bit, bool set)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(t6040_cfg(tphy) + T6040_CFG_STATUS, status,
				 !!(status & bit) == set, 1, 1000);
	if (ret)
		dev_err(tphy->dev, "USB3: no %s acknowledge (status %#x)\n", what, status);
	udelay(1);
	return ret;
}

/* A: powerup_rst, every acknowledge within 1 ms. */
static int t6040_usb3_wake(struct t6040_usb2_phy *tphy)
{
	void __iomem *cfg = t6040_cfg(tphy);

	t6040_usb2_set32(cfg + T6040_CFG_CTRL, T6040_CFG_CTRL_SLEEP_B_SML);
	udelay(1);
	if (t6040_usb3_cfg_ack(tphy, "SLEEP_B_SML", T6040_CFG_STATUS_SLEEP_B_SML, true))
		return -ETIMEDOUT;
	t6040_usb2_set32(cfg + T6040_CFG_CTRL, T6040_CFG_CTRL_SLEEP_B_BIG);
	udelay(1);
	if (t6040_usb3_cfg_ack(tphy, "SLEEP_B_BIG", T6040_CFG_STATUS_SLEEP_B_BIG, true))
		return -ETIMEDOUT;
	t6040_usb2_clear32(cfg + T6040_CFG_CTRL, T6040_CFG_CTRL_CLAMP);
	udelay(8);
	t6040_usb2_clear32(cfg + T6040_CFG_AHB, T6040_CFG_AHB_CLK_REQ_N);
	if (t6040_usb3_cfg_ack(tphy, "AHB clock", T6040_CFG_STATUS_AHB_CLK, false))
		return -ETIMEDOUT;
	t6040_usb2_set32(cfg + T6040_CFG_CTRL, T6040_CFG_CTRL_APB_RESET_N);
	return 0;
}

/*
 * B1: the tables, then apply_dptx_tunables' fixed writes (atc-phy reg[] offsets
 * relative to the core) and AUSPLL_CORE once more.
 */
static void t6040_usb3_tunables(struct t6040_usb2_phy *tphy)
{
	static const struct { u32 off, clear, set; } dptx[] = {
		{ 0x00800, 0x000003f0, 0x000001c0 },	/* AUS40CMN_DIG RCAL0 */
		{ 0x00818, 0x0000f000, 0x00003000 },	/* AUS40CMN_DIG RCAL5 */
		{ 0x03888, 0x000000ff, 0x00000027 },	/* CIO4PLL FREQDESCRIPTOR_0_C */
		{ 0x28004, 0x00000038, 0x00000008 },	/* AUX_LDO_CTRL */
		{ 0x03adc, 0x00003ff0, 0x00001760 },	/* CIO4PLL CFG_FRACN_FUNC */
		{ 0x0e0cc, 0x01f80000, 0x00380000 },	/* LANE0 TX_EQ_CTRL_REG11 */
		{ 0x1e0cc, 0x01f80000, 0x00380000 },	/* LANE1 TX_EQ_CTRL_REG11 */
		{ 0x0d090, 0x01f80000, 0x00380000 },	/* LANE0 RX_CFG TX_EQ_CTRL_REG11 */
		{ 0x1d090, 0x01f80000, 0x00380000 },	/* LANE1 RX_CFG TX_EQ_CTRL_REG11 */
	};
	size_t i;

	for (i = 0; i < T6040_USB3_NR_TUNABLES; i++)
		apple_tunable_apply(tphy->regs[t6040_usb3_tunable_list[i].region],
				    tphy->tunables[i]);
	for (i = 0; i < ARRAY_SIZE(dptx); i++)
		t6040_usb2_mask32(t6040_core(tphy) + dptx[i].off, dptx[i].clear, dptx[i].set);
	apple_tunable_apply(t6040_core(tphy), tphy->tunables[T6040_USB3_TUNABLE_AUSPLL]);
}

/* B2: the crossbar for USB3 on the plug's lane; B3: release that lane's overrides. */
static void t6040_usb3_lanes(struct t6040_usb2_phy *tphy, bool swap)
{
	static const struct { u32 off, bit; } lane_ovrd[] = {
		{ 0x10, 0x4 }, { 0x10, 0x40 }, { 0x0c, 0x10000 }, { 0x0c, 0x100 },
		{ 0x0c, 0x1000 }, { 0x120, 0x40 }, { 0x120, 0x4 }, { 0x120, 0x400 },
	};
	void __iomem *core = t6040_core(tphy);
	size_t i;

	if (swap)
		t6040_usb2_mask32(core + T6040_CORE_XBAR, ~0xfebfff03u, 0x28);
	else
		t6040_usb2_mask32(core + T6040_CORE_XBAR, ~0xfd7fff03u, 0x44);
	t6040_usb2_mask32(core + T6040_CORE_XBAR_LANES, 0x1ffff, swap ? 0x111 : 0x110);
	t6040_usb2_mask32(core + T6040_CORE_XBAR, 0x3, 0x2);

	/* cmn_rm_ovrd, then txrx_rm_ovrd of the lane */
	t6040_usb2_clear32(core + T6040_CORE_CMN_OVRD, 0x2a);
	udelay(1);
	for (i = 0; i < ARRAY_SIZE(lane_ovrd); i++) {
		t6040_usb2_clear32(core + lane_ovrd[i].off, lane_ovrd[i].bit << swap);
		udelay(1);
	}
}

/*
 * C: runFirmware. A stopped coprocessor is run and RTKit booted; one this
 * driver put to sleep on the last detach is woken. A coprocessor that runs
 * without this driver having booted it is not taken over. Then wait for the
 * coprocessor to have tuned the PHY and release the PHY reset (D).
 */
static int t6040_usb3_run_firmware(struct t6040_usb2_phy *tphy, bool swap)
{
	void __iomem *cpu = t6040_cpu(tphy);
	bool running;
	u32 uc;
	int ret;

	t6040_usb2_mask32(t6040_cfg(tphy) + T6040_CFG_SPARE0, T6040_CFG_SPARE0_USB_LN_SWAP,
			  swap ? T6040_CFG_SPARE0_USB_LN_SWAP : 0);
	fsleep(20);
	t6040_usb2_set32(cpu + T6040_CPU_IDLE_CTRL, T6040_CPU_IDLE_CTRL_EN);
	fsleep(10);

	running = readl(cpu + T6040_CPU_RUN) & T6040_CPU_RUN_START;
	if (running && !tphy->cpu_booted) {
		dev_err(tphy->dev, "USB3: the PHY coprocessor was started outside this driver\n");
		return -EBUSY;
	}
	dev_dbg(tphy->dev, "USB3: %s the PHY coprocessor\n", running ? "waking" : "starting");
	if (running) {
		ret = apple_rtkit_wake(tphy->rtk);
	} else {
		writel(T6040_CPU_RUN_START, cpu + T6040_CPU_RUN);
		usleep_range(5000, 6000);
		tphy->cpu_booted = true;
		ret = apple_rtkit_boot(tphy->rtk);
	}
	if (ret) {
		dev_err(tphy->dev, "USB3: PHY coprocessor %s failed: %d\n",
			running ? "wake-up" : "start", ret);
		return ret;
	}

	ret = readl_poll_timeout(t6040_core(tphy) + T6040_CORE_UC_CTRL, uc,
				 uc & T6040_CORE_UC_CTRL_TNBL_DONE, 1000, USEC_PER_SEC);
	if (ret) {
		dev_err(tphy->dev, "USB3: the PHY coprocessor did not tune the PHY (%#x)\n", uc);
		return ret;
	}

	t6040_usb2_set32(t6040_cfg(tphy) + T6040_CFG_CTRL, T6040_CFG_CTRL_PHY_RESET_N);
	fsleep(100);
	return 0;
}

static void t6040_usb3_sleep_firmware(struct t6040_usb2_phy *tphy)
{
	int ret;

	if (!apple_rtkit_is_running(tphy->rtk))
		return;

	reinit_completion(&tphy->app_reply);
	ret = apple_rtkit_start_ep(tphy->rtk, T6040_CPU_EP_APP);
	if (!ret)
		ret = apple_rtkit_send_message(tphy->rtk, T6040_CPU_EP_APP,
					       T6040_CPU_MSG_POWER_DOWN, NULL, false);
	if (!ret && !wait_for_completion_timeout(&tphy->app_reply, msecs_to_jiffies(500)))
		ret = -ETIMEDOUT;
	if (ret)
		dev_warn(tphy->dev, "USB3: no Power Down acknowledge: %d\n", ret);

	ret = apple_rtkit_shutdown(tphy->rtk);
	if (ret)
		dev_warn(tphy->dev, "USB3: PHY coprocessor sleep failed: %d\n", ret);
}

/* Sets PHY_STATUS(_2) handshake bits in the BIST block and waits as macOS does, without failing. */
static void t6040_usb3_bist_pulse(struct t6040_usb2_phy *tphy, u32 reg, u32 bit)
{
	t6040_usb2_set32(t6040_bist(tphy) + reg, bit);
	t6040_usb2_clear32(t6040_bist(tphy) + reg, bit);
}

static void t6040_usb3_bist_wait(struct t6040_usb2_phy *tphy, u32 bit, bool set,
				 unsigned int ms)
{
	u32 status;

	if (readl_poll_timeout(t6040_bist(tphy) + T6040_BIST_STATUS, status,
			       !!(status & bit) == set, 1000, ms * 1000))
		dev_dbg(tphy->dev, "USB3: BIST status %#x\n", status);
}

static void t6040_usb3_bist_on(struct t6040_usb2_phy *tphy)
{
	void __iomem *bist = t6040_bist(tphy);

	t6040_usb2_set32(bist + T6040_BIST_CFG0, BIT(0));
	t6040_usb2_mask32(bist + T6040_BIST_OVRD0, ~0xe1ffffffu, 0x4100000);
	t6040_usb2_mask32(bist + T6040_BIST_OVRD1, ~0xfffff99fu, 0x220);
	t6040_usb3_bist_pulse(tphy, T6040_BIST_CFG0, BIT(3));
	t6040_usb3_bist_wait(tphy, T6040_BIST_STATUS_PHY_2, false, 20);
	t6040_usb3_bist_pulse(tphy, T6040_BIST_CFG1, BIT(5));
	t6040_usb2_mask32(bist + T6040_BIST_OVRD0, ~0xe1ffffffu, 0x6000000);
	t6040_usb2_mask32(bist + T6040_BIST_OVRD1, ~0xfffff9ffu, 0x200);
	t6040_usb3_bist_pulse(tphy, T6040_BIST_CFG0, BIT(3));
	t6040_usb3_bist_wait(tphy, T6040_BIST_STATUS_PHY, true, 130);
	t6040_usb3_bist_wait(tphy, T6040_BIST_STATUS_PHY_2, false, 1);
	t6040_usb3_bist_pulse(tphy, T6040_BIST_CFG1, BIT(5));
}

static void t6040_usb3_bist_off(struct t6040_usb2_phy *tphy)
{
	t6040_usb2_clear32(t6040_bist(tphy) + T6040_BIST_OVRD1, 0x660);
	t6040_usb3_bist_pulse(tphy, T6040_BIST_CFG0, BIT(3));
	t6040_usb2_clear32(t6040_bist(tphy) + T6040_BIST_CFG0, BIT(0));
}

static int t6040_usb3_pipe_lock(struct t6040_usb2_phy *tphy, unsigned int ms)
{
	u32 ack;

	t6040_usb2_set32(tphy->pipehandler + T6040_PIPEHANDLER_LOCK_REQ,
			 T6040_PIPEHANDLER_LOCK_EN);
	return readl_poll_timeout(tphy->pipehandler + T6040_PIPEHANDLER_LOCK_ACK, ack,
				  ack & T6040_PIPEHANDLER_LOCK_EN, 500, ms * 1000);
}

static void t6040_usb3_pipe_unlock(struct t6040_usb2_phy *tphy)
{
	u32 ack;

	t6040_usb2_clear32(tphy->pipehandler + T6040_PIPEHANDLER_LOCK_REQ,
			   T6040_PIPEHANDLER_LOCK_EN);
	if (readl_poll_timeout(tphy->pipehandler + T6040_PIPEHANDLER_LOCK_ACK, ack,
			       !(ack & T6040_PIPEHANDLER_LOCK_EN), 500, 6000))
		dev_dbg(tphy->dev, "USB3: PIPE lock acknowledge did not clear\n");
}

static void t6040_usb3_pipe_mux(struct t6040_usb2_phy *tphy, u32 clk, u32 data)
{
	void __iomem *mux = tphy->pipehandler + T6040_PIPEHANDLER_MUX_CTRL;

	t6040_usb2_clear32(mux, T6040_PIPEHANDLER_MUX_CTRL_CLK);
	udelay(5);
	t6040_usb2_mask32(mux, T6040_PIPEHANDLER_MUX_CTRL_DATA,
			  FIELD_PREP(T6040_PIPEHANDLER_MUX_CTRL_DATA, data));
	t6040_usb2_mask32(mux, T6040_PIPEHANDLER_MUX_CTRL_CLK,
			  FIELD_PREP(T6040_PIPEHANDLER_MUX_CTRL_CLK, clk));
}

/*
 * E: move the PIPE from the dummy PHY to this one, inside macOS's BIST
 * bracket. The lock is never acknowledged on T6040 (the dummy PHY clocks the
 * handshake, and so far it has not answered); the switch works without it.
 */
static void t6040_usb3_pipe_switch(struct t6040_usb2_phy *tphy)
{
	void __iomem *ph = tphy->pipehandler;
	u32 val;

	/* macOS's controller reset() sets DUMMY_PHY_READY once; Linux's dwc3 does not. */
	if (!(readl(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE) &
	      T6040_PIPEHANDLER_DUMMY_PHY_READY)) {
		t6040_usb2_set32(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE,
				 T6040_PIPEHANDLER_DUMMY_PHY_READY);
		usleep_range(1000, 1500);
	}
	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE_VALUES, 0xe);
	t6040_usb2_set32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN);
	if (t6040_usb3_pipe_lock(tphy, 20))
		dev_dbg(tphy->dev, "USB3: PIPE lock not acknowledged, switching anyway\n");

	t6040_usb3_bist_on(tphy);
	t6040_usb2_mask32(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE, ~0xffffeff0u, 0x3);
	if (readl_poll_timeout(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE, val,
			       (val & T6040_PIPEHANDLER_NONSELECTED_MODE) == 3 &&
			       !(val & T6040_PIPEHANDLER_NONSELECTED_BUSY), 5, 1000))
		dev_dbg(tphy->dev, "USB3: non-selected override %#x\n", val);
	t6040_usb3_bist_off(tphy);

	t6040_usb3_pipe_mux(tphy, T6040_PIPEHANDLER_MUX_CLK_USB3,
			    T6040_PIPEHANDLER_MUX_DATA_USB3);
	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN);
	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN2);
	t6040_usb3_pipe_unlock(tphy);
}

/* macOS setUSB3Mode(0) from 2: the PIPE back on the dummy PHY. */
static void t6040_usb3_pipe_park(struct t6040_usb2_phy *tphy)
{
	void __iomem *ph = tphy->pipehandler;
	u32 val;

	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE_VALUES, 0xe);
	t6040_usb2_set32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN);
	t6040_usb2_set32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN2);
	if (t6040_usb3_pipe_lock(tphy, 150))
		dev_dbg(tphy->dev, "USB3: PIPE lock not acknowledged, parking anyway\n");
	t6040_usb2_mask32(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE,
			  T6040_PIPEHANDLER_NONSELECTED_MODE, 0x1002);
	if (readl_poll_timeout(ph + T6040_PIPEHANDLER_NONSELECTED_OVERRIDE, val,
			       (val & T6040_PIPEHANDLER_NONSELECTED_MODE) == 2 &&
			       (val & T6040_PIPEHANDLER_NONSELECTED_BUSY), 5, 1000))
		dev_dbg(tphy->dev, "USB3: non-selected override %#x\n", val);
	t6040_usb3_pipe_mux(tphy, T6040_PIPEHANDLER_MUX_CLK_DUMMY,
			    T6040_PIPEHANDLER_MUX_DATA_DUMMY);
	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN);
	t6040_usb2_clear32(ph + T6040_PIPEHANDLER_OVERRIDE, T6040_PIPEHANDLER_OVERRIDE_EN2);
	t6040_usb3_pipe_unlock(tphy);
}

/*
 * dwc3 powers this on in its core init, with the controller out of reset and
 * its PIPE on the dummy PHY. A failure here leaves the port at USB2, so it is
 * logged and not returned: the USB2 PHY and the core must still come up.
 */
static int t6040_usb3_power_on(struct phy *phy)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);
	bool swap;
	int ret;

	guard(mutex)(&tphy->lock);
	if (!tphy->usb3_ok || tphy->usb3_up)
		return 0;

	switch (tphy->orientation) {
	case TYPEC_ORIENTATION_NORMAL:
		swap = false;
		break;
	case TYPEC_ORIENTATION_REVERSE:
		swap = true;
		break;
	default:
		dev_info(tphy->dev, "USB3: no plug orientation, USB2 only\n");
		return 0;
	}

	/* PIPE_HANDLER_DEFAULT: the dummy PIPE clock the controller keeps using at USB2 */
	apple_tunable_apply(tphy->pipehandler, tphy->tunable_pipehandler);

	ret = t6040_usb3_wake(tphy);
	if (ret)
		goto usb2_only;
	t6040_usb3_tunables(tphy);
	t6040_usb3_lanes(tphy, swap);
	ret = t6040_usb3_run_firmware(tphy, swap);
	if (ret)
		goto usb2_only;

	tphy->usb3_up = true;
	dev_dbg(tphy->dev, "USB3 PHY up, lane %d\n", swap);
	return 0;

usb2_only:
	/* A coprocessor left running could not be woken on the next attach. */
	t6040_usb3_sleep_firmware(tphy);
	dev_warn(tphy->dev, "USB3 PHY bring-up failed (%d), USB2 only\n", ret);
	return 0;
}

static int t6040_usb3_power_off(struct phy *phy)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);

	guard(mutex)(&tphy->lock);
	if (tphy->pipe_switched) {
		t6040_usb3_pipe_park(tphy);
		tphy->pipe_switched = false;
	}
	/*
	 * shortcut: macOS also powers the lanes and the common block down here
	 * (aciophy_phy_shutdown); its lane masks are not decoded, so the PHY
	 * stays awake until the next attach reprograms it or its power domain
	 * drops. Upgrade once the shutdown masks are known.
	 */
	if (tphy->usb3_up) {
		t6040_usb3_sleep_firmware(tphy);
		tphy->usb3_up = false;
	}
	return 0;
}

/*
 * Host only. dwc3-apple calls this after the core init and before xHCI
 * starts; the xHCI root hub calls it again as it starts, also for a port
 * that stays at USB2, which must not fail.
 */
static int t6040_usb3_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct t6040_usb2_phy *tphy = phy_get_drvdata(phy);

	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_HOST_SS)
		return -EOPNOTSUPP;

	guard(mutex)(&tphy->lock);
	if (!tphy->usb3_up || tphy->pipe_switched)
		return 0;
	t6040_usb3_pipe_switch(tphy);
	tphy->pipe_switched = true;
	dev_info(tphy->dev, "USB3 PIPE on the PHY\n");
	return 0;
}

static const struct phy_ops t6040_usb3_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = t6040_usb3_set_mode,
	.power_on = t6040_usb3_power_on,
	.power_off = t6040_usb3_power_off,
};

/* tipd sets the orientation before the USB role, and NONE after the role is dropped. */
static int t6040_usb3_sw_set(struct typec_switch_dev *sw, enum typec_orientation orientation)
{
	struct t6040_usb2_phy *tphy = typec_switch_get_drvdata(sw);

	guard(mutex)(&tphy->lock);
	tphy->orientation = orientation;
	return 0;
}

static void t6040_usb3_switch_unregister(void *sw)
{
	typec_switch_unregister(sw);
}

/* ---- the coprocessor's RTKit ---- */

/* The coprocessor places its buffers in its own __DATA SRAM. */
static int t6040_cpu_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct t6040_usb2_phy *tphy = cookie;
	resource_size_t size = resource_size(tphy->cpu_data);

	if (!bfr->iova || bfr->iova < T6040_CPU_DATA_IOVA || bfr->size > size ||
	    bfr->iova - T6040_CPU_DATA_IOVA > size - bfr->size) {
		dev_err(tphy->dev, "USB3: coprocessor buffer %pad+%#zx outside its SRAM\n",
			&bfr->iova, bfr->size);
		return -EINVAL;
	}
	bfr->iomem = ioremap_np(tphy->cpu_data->start + (bfr->iova - T6040_CPU_DATA_IOVA),
				bfr->size);
	if (!bfr->iomem)
		return -ENOMEM;
	bfr->is_mapped = true;
	return 0;
}

static void t6040_cpu_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	if (bfr->iomem)
		iounmap(bfr->iomem);
}

static void t6040_cpu_crashed(void *cookie, const void *crashlog, size_t size)
{
	struct t6040_usb2_phy *tphy = cookie;

	dev_err(tphy->dev, "USB3: the PHY coprocessor crashed\n");
}

static void t6040_cpu_recv_message(void *cookie, u8 ep, u64 msg)
{
	struct t6040_usb2_phy *tphy = cookie;

	if (ep == T6040_CPU_EP_APP)
		complete(&tphy->app_reply);
	else
		dev_dbg(tphy->dev, "USB3: coprocessor message on endpoint %u: %#llx\n", ep, msg);
}

static const struct apple_rtkit_ops t6040_cpu_rtkit_ops = {
	.flags = APPLE_RTKIT_COPROC_PLACES_BUFFERS,
	.crashed = t6040_cpu_crashed,
	.recv_message = t6040_cpu_recv_message,
	.shmem_setup = t6040_cpu_shmem_setup,
	.shmem_destroy = t6040_cpu_shmem_destroy,
};

/*
 * Everything USB3 needs from the device tree. A missing piece leaves the
 * port at USB2; only a mailbox that has not probed yet defers.
 */
static int t6040_usb3_probe(struct t6040_usb2_phy *tphy, struct platform_device *pdev)
{
	struct device *dev = tphy->dev;
	struct typec_switch_desc sw_desc = {
		.drvdata = tphy,
		.fwnode = dev_fwnode(dev),
		.set = t6040_usb3_sw_set,
	};
	struct typec_switch_dev *sw;
	struct resource *res[T6040_USB3_NR_REGIONS];
	struct resource *ph;
	size_t i;
	int ret;

	tphy->usb3 = devm_phy_create(dev, NULL, &t6040_usb3_phy_ops);
	if (IS_ERR(tphy->usb3))
		return PTR_ERR(tphy->usb3);
	phy_set_drvdata(tphy->usb3, tphy);
	init_completion(&tphy->app_reply);

	if (!of_property_present(dev->of_node, "orientation-switch"))
		return 0;

	for (i = 0; i < T6040_USB3_NR_REGIONS; i++) {
		res[i] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      t6040_usb3_region_names[i]);
		if (!res[i]) {
			dev_warn(dev, "USB3: no \"%s\" region, USB2 only\n",
				 t6040_usb3_region_names[i]);
			return 0;
		}
		tphy->regs[i] = devm_ioremap_resource(dev, res[i]);
		if (IS_ERR(tphy->regs[i])) {
			dev_warn(dev, "USB3: \"%s\" region: %pe, USB2 only\n",
				 t6040_usb3_region_names[i], tphy->regs[i]);
			return 0;
		}
	}
	tphy->cpu_data = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cpu-data");
	ph = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pipehandler");
	if (!tphy->cpu_data || !ph) {
		dev_warn(dev, "USB3: no coprocessor SRAM region, USB2 only\n");
		return 0;
	}

	for (i = 0; i < T6040_USB3_NR_TUNABLES; i++) {
		tphy->tunables[i] =
			devm_apple_tunable_parse(dev, dev->of_node, t6040_usb3_tunable_list[i].prop,
						 res[t6040_usb3_tunable_list[i].region]);
		if (IS_ERR(tphy->tunables[i])) {
			dev_warn(dev, "USB3: %s: %pe, USB2 only\n",
				 t6040_usb3_tunable_list[i].prop, tphy->tunables[i]);
			return 0;
		}
	}
	tphy->tunable_pipehandler = devm_apple_tunable_parse(dev, dev->of_node,
							     "apple,tunable-pipehandler", ph);
	if (IS_ERR(tphy->tunable_pipehandler)) {
		dev_warn(dev, "USB3: apple,tunable-pipehandler: %pe, USB2 only\n",
			 tphy->tunable_pipehandler);
		return 0;
	}

	tphy->rtk = devm_apple_rtkit_init(dev, tphy, "phy-cpu", 0, &t6040_cpu_rtkit_ops);
	if (IS_ERR(tphy->rtk)) {
		ret = PTR_ERR(tphy->rtk);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev, "USB3: no PHY coprocessor mailbox (%d), USB2 only\n", ret);
		return 0;
	}

	sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(sw))
		return PTR_ERR(sw);
	ret = devm_add_action_or_reset(dev, t6040_usb3_switch_unregister, sw);
	if (ret)
		return ret;

	tphy->usb3_ok = true;
	return 0;
}

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

	if (args->args_count != 1)
		return ERR_PTR(-ENODEV);

	switch (args->args[0]) {
	case PHY_TYPE_USB2:
		return tphy->phy;
	case PHY_TYPE_USB3:
		return tphy->usb3;
	default:
		return ERR_PTR(-ENODEV);
	}
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

	ret = t6040_usb3_probe(tphy, pdev);
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
MODULE_DESCRIPTION("Apple T6040 Type-C PHY, USB2 and USB3 host");
MODULE_LICENSE("Dual BSD/GPL");
