// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_INTERNAL_H__
#define __APPLE_DCP_INTERNAL_H__

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/usb/typec_mux.h>

#include "dptxep.h"
#include "iomfb.h"
#include "iomfb_v12_3.h"
#include "iomfb_v13_3.h"
#include "iomfb_v26_6.h"
#include "epic/dpavservep.h"

#define DCP_MAX_PLANES 4

struct apple_dcp;
struct apple_dcp_afkep;

extern bool dcp_disable_rt_bandwidth;
/* M4: 0 = RTKit/IOMFB start handshake only, 1 = full DRM takeover. */
extern bool dcp_takeover;
/* M4 (R38): one M4_DCP_SWAP_STATS line per 2 s window while swaps complete (default on). */
extern bool dcp_swap_stats;
/* M4 (R39 prep): page-flip event policy (bit0 raw completion time, bit1 running sequence) and fake swap timestamps */
extern unsigned int dcp_flip_event_mode;
extern bool dcp_swap_timestamps;
extern unsigned int dcp_backlight_default_nits;
extern unsigned long dcp_clk_hz;

struct dcpav_service_epic;

enum dcp_firmware_version {
	DCP_FIRMWARE_UNKNOWN,
	DCP_FIRMWARE_V_12_3,
	DCP_FIRMWARE_V_13_5,
	DCP_FIRMWARE_V_26_6,
};

enum {
	SYSTEM_ENDPOINT = 0x20,
	TEST_ENDPOINT = 0x21,
	DCP_EXPERT_ENDPOINT = 0x22,
	DISP0_ENDPOINT = 0x23,
	DPAVCTRL_ENDPOINT = 0x24,
	EPIC25_ENDPOINT = 0x25,
	DPSAC_ENDPOINT = 0x26,
	DPDEV_ENDPOINT = 0x27,
	DPAVSERV_ENDPOINT = 0x28,
	AV_ENDPOINT = 0x29,
	DPTX_ENDPOINT = 0x2a,
	HDCP_ENDPOINT = 0x2b,
	EPIC2C_ENDPOINT = 0x2c,
	REMOTE_ALLOC_ENDPOINT = 0x2d,
	IOMFB_ENDPOINT = 0x37,
};

/* Temporary backing for a chunked transfer via setDCPAVPropStart/Chunk/End */
struct dcp_chunks {
	size_t length;
	void *data;
};

#define DCP_MAX_MAPPINGS (128) /* should be enough */
#define MAX_DISP_REGISTERS (7)

struct dcp_mem_descriptor {
	size_t size;
	void *buf;
	dma_addr_t dva;
	struct sg_table map;
	u64 reg;
};

/* Limit on call stack depth (arbitrary). Some nesting is required */
#define DCP_MAX_CALL_DEPTH 8

typedef void (*dcp_callback_t)(struct apple_dcp *, void *, void *);

struct dcp_channel {
	dcp_callback_t callbacks[DCP_MAX_CALL_DEPTH];
	void *cookies[DCP_MAX_CALL_DEPTH];
	void *output[DCP_MAX_CALL_DEPTH];
	u16 start[DCP_MAX_CALL_DEPTH];
	u16 end[DCP_MAX_CALL_DEPTH];
	bool host_call[DCP_MAX_CALL_DEPTH];

	/* Current depth of the call stack. Less than DCP_MAX_CALL_DEPTH */
	u8 depth;
	/* Already warned about busy channel */
	bool warned_busy;
};

struct dcp_fb_reference {
	struct list_head head;
	struct drm_framebuffer *fb;
	u32 swap_id;
};

#define MAX_NOTCH_HEIGHT 160

struct dcp_brightness {
	struct backlight_device *bl_dev;
	u32 maximum;
	u32 dac;
	int nits;
	int scale;
	bool update;
	/* M4: userspace wrote a brightness through the backlight device. */
	bool requested;
};

struct audiosrv_data;

/** laptop/AiO integrated panel parameters from DT */
struct dcp_panel {
	/// panel width in millimeter
	int width_mm;
	/// panel height in millimeter
	int height_mm;
	/// panel has a mini-LED backlight
	bool has_mini_led;
};

struct apple_dcp_hw_data {
	u32 num_dptx_ports;
};

/* TODO: move IOMFB members to its own struct */
struct apple_dcp {
	struct device *dev;
	struct platform_device *piodma;
	struct iommu_domain *iommu_dom;
	struct apple_rtkit *rtk;
	struct apple_crtc *crtc;
	struct apple_connector *connector;

	struct apple_dcp_hw_data hw;

	/* firmware version and compatible firmware version */
	enum dcp_firmware_version fw_compat;

	/* Coprocessor control register */
	void __iomem *coproc_reg;

	/* DCP has crashed */
	bool crashed;

	DECLARE_BITMAP(iomfb_surfaces, DCP_MAX_PLANES);

	/************* IOMFB **************************************************
	 * everything below is mostly used inside IOMFB but it could make     *
	 * sense to keep some of the members in apple_dcp.                    *
	 **********************************************************************/

	/* clock rate request by dcp in */
	struct clk *clk;

	/* DCP shared memory */
	void *shmem;

	/* Display registers mappable to the DCP */
	struct resource *disp_registers[MAX_DISP_REGISTERS];
	unsigned int nr_disp_registers;

	struct resource disp_bw_scratch_res;
	struct resource disp_bw_doorbell_res;
	u32 disp_bw_scratch_index;
	u32 disp_bw_scratch_offset;
	u32 disp_bw_doorbell_index;
	u32 disp_bw_doorbell_offset;

	u32 index;

	/* Bitmap of memory descriptors used for mappings made by the DCP */
	DECLARE_BITMAP(memdesc_map, DCP_MAX_MAPPINGS);

	/* Indexed table of memory descriptors */
	struct dcp_mem_descriptor memdesc[DCP_MAX_MAPPINGS];

	struct dcp_channel ch_cmd, ch_oobcmd;
	struct dcp_channel ch_cb, ch_oobcb, ch_async, ch_oobasync;

	/* iomfb EP callback handlers */
	const struct iomfb_cb_entry *cb_handlers;

	/* Active chunked transfer. There can only be one at a time. */
	struct dcp_chunks chunks;
	unsigned long *chunks_received;
	/* Per-transfer diagnostics for the 26.6 property path (M4_DCP_PROP_*). */
	u32 chunk_calls, chunk_last_offset, chunk_last_length;

	/* Queued swap. Owned by the DCP to avoid per-swap memory allocation */
	union {
		struct dcp_swap_submit_req_v12_3 v12_3;
		struct dcp_swap_submit_req_v13_3 v13_3;
		struct dcp_swap_submit_req_v26_6_0 v26_6_0;
	} swap;

	/* swap id of the last completed swap */
	u32 last_swap_id;
	ktime_t swap_start;

	/* Current display mode */
	bool during_modeset;
	bool valid_mode;
	/*
	 * Diagnostic 26.6 external-hotplug path: the link can disappear while
	 * IOMFB remains powered.  Remember that atomic_disable deliberately
	 * skipped the firmware power-off sequence so atomic_enable does not send
	 * a mismatched power-on sequence on reconnect.
	 */
	bool hotplug_keepalive;
	/*
	 * M4 takeover: the pre-eviction trial already ran poweron and a modeset.
	 * The first real atomic enable reuses them instead of repeating them.
	 */
	bool trial_powered;
	bool trial_modeset_ok;
	u32 trial_timing_mode_id, trial_color_mode_id;
	/* M4: outcome of the last iomfb_poweron(): 0 ok, 1 timeout, 2 refused, 3 error. */
	u8 poweron_result;
	/* M4: first swaps logged (M4_DCP_SWAP_*). */
	u32 m4_swaps_started, m4_swaps_ok, m4_swaps_complete, m4_flushes;
	/* M4 (R38): swap completion statistics for the present-rate question (R37 measured 65-80 fps at 120 Hz) */
	ktime_t m4_stats_window, m4_stats_last;
	u32 m4_stats_n, m4_stats_gap_max_us, m4_stats_lat_max_us;
	u64 m4_stats_gap_sum_us, m4_stats_lat_sum_us;
	/* M4 (R39 prep): commit-to-complete latency, which surfaces each flush carried, event sequence */
	ktime_t m4_commit_time;
	u32 m4_stats_commit_max_us, m4_stats_mask[4];
	u64 m4_stats_commit_sum_us, m4_flip_seq;
	/* M4 (R51): vblank clock for flip_event_mode bit2: frame period of the current mode, first event, last sequence,
	 * time of the last swap_complete_intent_gated callback and of the last swap_complete, intent statistics */
	u64 m4_frame_ns, m4_vbl_seq;
	ktime_t m4_vbl_epoch, m4_intent_time, m4_complete_time;
	u32 m4_stats_intent_n, m4_stats_intent_max_us;
	u64 m4_stats_intent_sum_us;
	bool use_timestamps;
	struct dcp_set_digital_out_mode_req mode;

	/* completion for active turning true */
	struct completion start_done;

	/* Is the DCP booted? */
	bool active;

	/* eDP display without DP-HDMI conversion */
	bool main_display;

	/* clear all surfaces on init */
	bool surfaces_cleared;

	/* enable CRC calculation */
	bool crc_enabled;

	/* Modes valid for the connected display */
	struct mutex modes_lock;
	struct dcp_display_mode *modes;
	unsigned int nr_modes;

	/* Attributes of the connector */
	int connector_type;

	/* Attributes of the connected display */
	int width_mm, height_mm;

	unsigned notch_height;

	/* Workqueue for sending vblank events when a dcp swap is not possible */
	struct work_struct vblank_wq;

	/* List of referenced drm_framebuffers which can be unreferenced
	 * on the next successfully completed swap.
	 */
	struct list_head swapped_out_fbs;

	struct dcp_brightness brightness;
	/* Workqueue for updating the initial brightness */
	struct work_struct bl_register_wq;
	struct mutex bl_register_mutex;
	/* Workqueue for updating the brightness */
	struct work_struct bl_update_wq;

	/* integrated panel if present */
	struct dcp_panel panel;

	struct apple_dcp_afkep *systemep;
	struct completion systemep_done;

	struct apple_dcp_afkep *ibootep;
	struct apple_epic_service *iboot_service;
	struct apple_dcp_afkep *dpavctrlep;
	struct apple_dcp_afkep *dpdevep;
	struct apple_epic_service *dpdev_service;
	struct completion dpdev_ready;
	struct apple_dcp_afkep *avauxep;
	struct apple_dcp_afkep *dpsacep;
	struct apple_dcp_afkep *remoteallocep;
	struct apple_dcp_afkep *epicep2c;
	struct apple_dcp_afkep *expertep;
	struct apple_dcp_afkep *epicep25;
	/* M4: panel DCP graph endpoints started / skipped (bit = endpoint - 0x20) */
	u32 graph_started;
	u32 graph_skipped;
	struct apple_dcp_afkep *hdcpep;
	struct completion hdcp_ready;
	struct apple_epic_service *hdcp_service[2];
	u32 hdcp_teardown_interface;
	bool hdcp_mprime_attempted;
	struct completion dpavctrl_ready;
	struct apple_epic_service *dpavctrl_av_controller[2];
	struct apple_epic_service *dpavctrl_late_service;
	struct apple_dcp_afkep *dcpavservep;
	struct dcpavserv dcpavserv;

	struct apple_dcp_afkep *avep;
	struct audiosrv_data *audiosrv;

	struct apple_dcp_afkep *dptxep;

	struct dptx_port dptxport[2];

	/* debugfs entries */
	struct dentry *ep_debugfs[0x20];

	/* these fields are output port specific */
	struct phy *phy;
	struct mux_control *xbar;
	struct typec_mux *typec_mux;

	struct gpio_desc *hdmi_hpd;
	struct gpio_desc *hdmi_pwren;
	struct gpio_desc *dp2hdmi_pwren;

	struct mutex hpd_mutex;

	u32 dptx_phy;
	u32 dptx_die;
	int hdmi_hpd_irq;
};

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now);

int dcp_backlight_register(struct apple_dcp *dcp);
int dcp_backlight_update(struct apple_dcp *dcp);
bool dcp_has_panel(struct apple_dcp *dcp);

#define DCP_AUDIO_MAX_CHANS 15

#endif /* __APPLE_DCP_INTERNAL_H__ */
