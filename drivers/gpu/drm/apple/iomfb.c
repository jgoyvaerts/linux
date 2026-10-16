// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>

#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "parser.h"
#include "trace.h"

static int dcp_tx_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_CB:
	case DCP_CONTEXT_CMD:
		return 0x00000;
	case DCP_CONTEXT_OOBCB:
	case DCP_CONTEXT_OOBCMD:
		return 0x08000;
	default:
		return -EINVAL;
	}
}

static int dcp_channel_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_ASYNC:
		return 0x40000;
	case DCP_CONTEXT_OOBASYNC:
		return 0x48000;
	case DCP_CONTEXT_CB:
		return 0x60000;
	case DCP_CONTEXT_OOBCB:
		return 0x68000;
	default:
		return dcp_tx_offset(id);
	}
}

static inline u64 dcpep_set_shmem(u64 dart_va)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_SET_SHMEM) |
	       FIELD_PREP(IOMFB_SHMEM_FLAG, IOMFB_SHMEM_FLAG_VALUE) |
	       FIELD_PREP(IOMFB_SHMEM_DVA, dart_va);
}

static inline u64 dcpep_msg(enum dcp_context_id id, u32 length, u16 offset)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_MSG) |
		FIELD_PREP(IOMFB_MSG_CONTEXT, id) |
		FIELD_PREP(IOMFB_MSG_OFFSET, offset) |
		FIELD_PREP(IOMFB_MSG_LENGTH, length);
}

static inline u64 dcpep_ack(enum dcp_context_id id)
{
	return dcpep_msg(id, 0, 0) | IOMFB_MSG_ACK;
}

/*
 * A channel is busy if we have sent a message that has yet to be
 * acked. The driver must not sent a message to a busy channel.
 */
static bool dcp_channel_busy(struct dcp_channel *ch)
{
	return (ch->depth != 0);
}

/*
 * Get the context ID passed to the DCP for a command we push. The rule is
 * simple: callback contexts are used when replying to the DCP, command
 * contexts are used otherwise. That corresponds to a non/zero call stack
 * depth. This rule frees the caller from tracking the call context manually.
 */
static enum dcp_context_id dcp_call_context(struct apple_dcp *dcp, bool oob)
{
	u8 depth = oob ? dcp->ch_oobcmd.depth : dcp->ch_cmd.depth;

	if (depth)
		return oob ? DCP_CONTEXT_OOBCB : DCP_CONTEXT_CB;
	else
		return oob ? DCP_CONTEXT_OOBCMD : DCP_CONTEXT_CMD;
}

/* Get a channel for a context */
static struct dcp_channel *dcp_get_channel(struct apple_dcp *dcp,
					   enum dcp_context_id context)
{
	switch (context) {
	case DCP_CONTEXT_CB:
		return &dcp->ch_cb;
	case DCP_CONTEXT_CMD:
		return &dcp->ch_cmd;
	case DCP_CONTEXT_OOBCB:
		return &dcp->ch_oobcb;
	case DCP_CONTEXT_OOBCMD:
		return &dcp->ch_oobcmd;
	case DCP_CONTEXT_ASYNC:
		return &dcp->ch_async;
	case DCP_CONTEXT_OOBASYNC:
		return &dcp->ch_oobasync;
	default:
		return NULL;
	}
}

/* Get the start of a packet: after the end of the previous packet */
static u16 dcp_packet_start(struct dcp_channel *ch, u8 depth)
{
	if (depth > 0)
		return ch->end[depth - 1];
	else
		return 0;
}

/* Pushes and pops the depth of the call stack with safety checks */
static bool dcp_packet_range(u16 offset, u32 length)
{
	return IS_ALIGNED(offset, DCP_PACKET_ALIGNMENT) &&
	       offset <= DCP_CHANNEL_SIZE &&
	       length >= sizeof(struct dcp_packet_header) &&
	       length <= DCP_CHANNEL_SIZE - offset &&
	       ALIGN(length, DCP_PACKET_ALIGNMENT) <= DCP_CHANNEL_SIZE - offset;
}

static bool dcp_packet_overlaps(struct apple_dcp *dcp, int bank,
				u16 start, u16 end)
{
	const enum dcp_context_id ids[] = { DCP_CONTEXT_CB, DCP_CONTEXT_CMD,
		DCP_CONTEXT_ASYNC, DCP_CONTEXT_OOBCB, DCP_CONTEXT_OOBCMD,
		DCP_CONTEXT_OOBASYNC };
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		struct dcp_channel *ch = dcp_get_channel(dcp, ids[i]);

		if (ch->depth > DCP_MAX_CALL_DEPTH)
			return true;
		for (j = 0; j < ch->depth; j++) {
			int owner_bank = ch->host_call[j] ? dcp_tx_offset(ids[i]) :
							 dcp_channel_offset(ids[i]);

			if (owner_bank < 0 || ch->start[j] >= ch->end[j] ||
			    ch->end[j] > DCP_CHANNEL_SIZE)
				return true;
			if (owner_bank == bank && start < ch->end[j] && ch->start[j] < end)
				return true;
		}
	}
	return false;
}

static int dcp_push_depth(u8 *depth)
{
	if (WARN_ON(*depth >= DCP_MAX_CALL_DEPTH))
		return -ENOSPC;
	return (*depth)++;
}

static int dcp_pop_depth(u8 *depth)
{
	if (WARN_ON(!*depth || *depth > DCP_MAX_CALL_DEPTH))
		return -EINVAL;
	return --(*depth);
}

/* False means no host-call slot was published; true is not a firmware ACK. */
bool dcp_push(struct apple_dcp *dcp, bool oob, const struct dcp_method_entry *call,
		     u32 in_len, u32 out_len, void *data, dcp_callback_t cb,
		     void *cookie)
{
	enum dcp_context_id context = dcp_call_context(dcp, oob);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	struct dcp_packet_header header;
	int bank = dcp_tx_offset(context), depth;
	u32 data_len;
	u16 offset, end;
	void *out, *out_data;

	if (!dcp->shmem || !call || !ch || bank < 0 ||
	    ch->depth >= DCP_MAX_CALL_DEPTH || (in_len && !data) ||
	    in_len > DCP_CHANNEL_SIZE - sizeof(header) ||
	    out_len > DCP_CHANNEL_SIZE - sizeof(header) - in_len)
		goto refuse;

	data_len = sizeof(header) + in_len + out_len;
	offset = dcp_packet_start(ch, ch->depth);
	if (!dcp_packet_range(offset, data_len))
		goto refuse;
	end = offset + ALIGN(data_len, DCP_PACKET_ALIGNMENT);
	if (dcp_packet_overlaps(dcp, bank, offset, end))
		goto refuse;

	header = (struct dcp_packet_header) {
		.in_len = in_len, .out_len = out_len,
		.tag = { call->tag[3], call->tag[2], call->tag[1], call->tag[0] },
	};
	depth = dcp_push_depth(&ch->depth);
	if (depth < 0)
		return false;
	out = dcp->shmem + bank + offset;
	out_data = out + sizeof(header);
	memcpy(out, &header, sizeof(header));
	if (in_len)
		memcpy(out_data, data, in_len);

	trace_iomfb_push(dcp, call, context, offset, depth);
	ch->callbacks[depth] = cb;
	ch->cookies[depth] = cookie;
	ch->output[depth] = out_data + in_len;
	ch->start[depth] = offset;
	ch->end[depth] = end;
	ch->host_call[depth] = true;
	dma_wmb();
	dcp_send_message(dcp, IOMFB_ENDPOINT, dcpep_msg(context, data_len, offset));
	return true;
refuse:
	dev_warn(dcp->dev, "refusing invalid IOMFB outgoing packet\n");
	return false;
}

/* Parse a callback tag "D123" into the ID 123. Returns -EINVAL on failure. */
int dcp_parse_tag(char tag[4])
{
	u32 d[3];
	int i;

	if (tag[3] != 'D')
		return -EINVAL;

	for (i = 0; i < 3; ++i) {
		d[i] = (u32)(tag[i] - '0');

		if (d[i] > 9)
			return -EINVAL;
	}

	return d[0] + (d[1] * 10) + (d[2] * 100);
}

/* Ack a callback from the DCP */
void dcp_ack(struct apple_dcp *dcp, enum dcp_context_id context)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	int depth;

	if (!dcp->shmem || !ch || !ch->depth || ch->depth > DCP_MAX_CALL_DEPTH ||
	    ch->host_call[ch->depth - 1]) {
		dev_warn(dcp->dev, "refusing invalid IOMFB callback ACK\n");
		return;
	}
	depth = dcp_pop_depth(&ch->depth);
	if (depth < 0)
		return;
	ch->output[depth] = NULL;
	ch->start[depth] = ch->end[depth] = 0;
	dma_wmb();
	dcp_send_message(dcp, IOMFB_ENDPOINT, dcpep_ack(context));
}

/*
 * Helper to send a DRM hotplug event. The DCP is accessed from a single
 * (RTKit) thread. To handle hotplug callbacks, we need to call
 * drm_kms_helper_hotplug_event, which does an atomic commit (via DCP) and
 * waits for vblank (a DCP callback). That means we deadlock if we call from
 * the RTKit thread! Instead, move the call to another thread via a workqueue.
 */
void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct apple_dcp *dcp;

	connector = container_of(work, struct apple_connector, hotplug_wq);

	dcp = platform_get_drvdata(connector->dcp);
	dev_info(dcp->dev, "%s() connected:%d valid_mode:%d nr_modes:%u\n", __func__,
		 connector->connected, dcp->valid_mode, READ_ONCE(dcp->nr_modes));

	if (!connector->connected) {
		drm_edid_free(connector->drm_edid);
		connector->drm_edid = NULL;
	}

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	if (connector->base.state && !dcp->valid_mode && connector->connected)
		drm_connector_set_link_status_property(&connector->base,
						       DRM_MODE_LINK_STATUS_BAD);

	drm_kms_helper_connector_hotplug_event(&connector->base);
}

static void dcpep_handle_cb(struct apple_dcp *dcp, enum dcp_context_id context,
			    void *data, u32 length, u16 offset)
{
	struct dcp_packet_header hdr;
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	const struct iomfb_cb_entry *entry;
	void *in, *out;
	int tag, depth, bank = dcp_channel_offset(context);
	u16 end;

	if (!dcp->shmem || !ch || bank < 0 || ch->depth >= DCP_MAX_CALL_DEPTH ||
	    !dcp_packet_range(offset, length) || data != dcp->shmem + bank + offset)
		goto refuse;
	memcpy(&hdr, data, sizeof(hdr));
	if (hdr.in_len > length - sizeof(hdr) ||
	    hdr.out_len > length - sizeof(hdr) - hdr.in_len)
		goto refuse;
	tag = dcp_parse_tag(hdr.tag);
	if (tag < 0 || tag >= IOMFB_MAX_CB || !dcp->cb_handlers)
		goto refuse;
	entry = &dcp->cb_handlers[tag];
	if (!entry->handler || hdr.in_len < entry->min_in || hdr.out_len < entry->min_out)
		goto refuse;
	end = offset + ALIGN(length, DCP_PACKET_ALIGNMENT);
	if (dcp_packet_overlaps(dcp, bank, offset, end))
		goto refuse;

	in = data + sizeof(hdr);
	out = in + hdr.in_len;
	if (hdr.out_len)
		memset(out, 0, hdr.out_len);
	depth = dcp_push_depth(&ch->depth);
	if (depth < 0)
		return;
	ch->output[depth] = out;
	ch->callbacks[depth] = NULL;
	ch->cookies[depth] = NULL;
	ch->start[depth] = offset;
	ch->end[depth] = end;
	ch->host_call[depth] = false;
	if (entry->handler(dcp, tag, out, in))
		dcp_ack(dcp, context);
	return;
refuse:
	dev_warn(dcp->dev, "refusing invalid IOMFB callback packet\n");
}

static void dcpep_handle_ack(struct apple_dcp *dcp, enum dcp_context_id context,
			     void *data, u32 length)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	int bank = dcp_tx_offset(context), depth;
	unsigned long base, output;
	void *cookie, *out;
	dcp_callback_t cb;

	/* Mailbox ACKs can have zero length/offset. The submitted slot owns output. */
	if (!dcp->shmem || !ch || bank < 0 || !ch->depth ||
	    ch->depth > DCP_MAX_CALL_DEPTH)
		goto refuse;
	depth = ch->depth - 1;
	if (!ch->host_call[depth] || ch->start[depth] >= ch->end[depth] ||
	    ch->end[depth] > DCP_CHANNEL_SIZE ||
	    ch->end[depth] - ch->start[depth] < sizeof(struct dcp_packet_header))
		goto refuse;
	base = (unsigned long)dcp->shmem + bank;
	output = (unsigned long)ch->output[depth];
	if (output < base + ch->start[depth] + sizeof(struct dcp_packet_header) ||
	    output > base + ch->end[depth])
		goto refuse;

	cb = ch->callbacks[depth];
	cookie = ch->cookies[depth];
	out = ch->output[depth];
	if (dcp_pop_depth(&ch->depth) < 0)
		return;
	ch->callbacks[depth] = NULL;
	ch->cookies[depth] = NULL;
	ch->output[depth] = NULL;
	ch->start[depth] = ch->end[depth] = 0;
	ch->host_call[depth] = false;
	if (cb)
		cb(dcp, out, cookie);
	return;
refuse:
	dev_warn(dcp->dev, "refusing unowned IOMFB command ACK\n");
}

static void dcpep_got_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcp_context_id ctx_id = FIELD_GET(IOMFB_MSG_CONTEXT, message);
	u16 offset = FIELD_GET(IOMFB_MSG_OFFSET, message);
	u32 length = FIELD_GET(IOMFB_MSG_LENGTH, message);
	int channel_offset = dcp_channel_offset(ctx_id);
	void *data;

	if (!dcp->shmem || channel_offset < 0) {
		dev_warn(dcp->dev, "invalid IOMFB receive context\n");
		return;
	}
	if (FIELD_GET(IOMFB_MSG_ACK, message)) {
		dma_rmb();
		dcpep_handle_ack(dcp, ctx_id, NULL, 0);
		return;
	}
	if (!dcp_packet_range(offset, length)) {
		dev_warn(dcp->dev, "invalid IOMFB receive aperture\n");
		return;
	}
	data = dcp->shmem + channel_offset + offset;
	dma_rmb();
	dcpep_handle_cb(dcp, ctx_id, data, length, offset);
}

int dcp_get_modes(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	int i;
	unsigned int nr_modes;

	mutex_lock(&dcp->modes_lock);
	nr_modes = dcp->nr_modes;
	for (i = 0; i < nr_modes; ++i) {
		mode = drm_mode_duplicate(dev, &dcp->modes[i].mode);

		if (!mode) {
			dev_err(dev->dev, "Failed to duplicate display mode\n");
			mutex_unlock(&dcp->modes_lock);
			return 0;
		}

		drm_mode_probed_add(connector, mode);
	}
	mutex_unlock(&dcp->modes_lock);

	if (nr_modes && dcp->dcpavserv.enabled &&
	    !apple_connector->drm_edid) {
		const struct drm_edid *edid;
		edid = dcpavserv_copy_edid(dcp->dcpavserv.service);
		if (IS_ERR_OR_NULL(edid)) {
			dev_info(dcp->dev, "copy_edid failed: %pe\n", edid);
		} else {
			drm_edid_free(apple_connector->drm_edid);
			apple_connector->drm_edid = edid;
		}
	}
	if (nr_modes && apple_connector->drm_edid)
		drm_edid_connector_update(connector, apple_connector->drm_edid);

	return nr_modes;
}

/* Copy a matching mode while locked; return its array count, or zero. */
unsigned int lookup_mode(struct apple_dcp *dcp,
			 const struct drm_display_mode *mode,
			 struct dcp_display_mode *out)
{
	unsigned int count = 0;
	int i;

	mutex_lock(&dcp->modes_lock);
	for (i = 0; i < dcp->nr_modes; ++i) {
		if (drm_mode_match(mode, &dcp->modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK)) {
			if (out)
				*out = dcp->modes[i];
			count = dcp->nr_modes;
			break;
		}
	}
	mutex_unlock(&dcp->modes_lock);

	return count;
}

enum drm_mode_status dcp_mode_valid(struct drm_connector *connector,
				    const struct drm_display_mode *mode)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return lookup_mode(dcp, mode, NULL) ? MODE_OK : MODE_BAD;
}

static int dcp_modeset_fw(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state)
{
	int ret = -EIO;

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		ret = iomfb_modeset_v12_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_13_5:
		ret = iomfb_modeset_v13_3(dcp, crtc_state);
		break;
	case DCP_FIRMWARE_V_26_6:
		ret = iomfb_modeset_v26_6_0(dcp, crtc_state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n",
			  dcp->fw_compat);
		break;
	}

	return ret;
}

int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	int ret = -EIO;
	bool modeset;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		return 0;

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) || !dcp->valid_mode;

	if (!modeset)
		return 0;

	/* ignore no mode, poweroff is handled elsewhere */
	if (crtc_state->mode.hdisplay == 0 && crtc_state->mode.vdisplay == 0)
		return 0;

	/*
	 * M4: the pre-eviction trial already set exactly this mode on a DCP
	 * that has been powered on; do not run the sequence a second time.
	 */
	if (dcp->trial_modeset_ok) {
		struct dcp_display_mode matched;

		dcp->trial_modeset_ok = false;
		if (dcp->valid_mode && lookup_mode(dcp, &crtc_state->mode, &matched) &&
		    matched.timing_mode_id == dcp->trial_timing_mode_id &&
		    matched.color_mode_id == dcp->trial_color_mode_id) {
			dev_info(dcp->dev,
				 "M4_DCP_TRIAL_REUSE modeset timing=%u color=%u " DRM_MODE_FMT "\n",
				 matched.timing_mode_id, matched.color_mode_id,
				 DRM_MODE_ARG(&crtc_state->mode));
			return 0;
		}
		dev_info(dcp->dev,
			 "M4_DCP_TRIAL_REUSE not-applicable (requested mode differs from the trial mode)\n");
	}

	ret = dcp_modeset_fw(dcp, crtc_state);
	dev_info(dcp->dev,
		 "M4_DCP_MODESET rc=%d valid_mode=%d " DRM_MODE_FMT "\n",
		 ret, dcp->valid_mode, DRM_MODE_ARG(&crtc_state->mode));

	return ret;
}

bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct platform_device *pdev = apple_crtc->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	/* TODO: support synthesized modes through scaling */
	return lookup_mode(dcp, mode, NULL) != 0;
}


void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp_channel_busy(&dcp->ch_cmd))
	{
		if (!dcp->ch_cmd.warned_busy) {
			dev_err(dcp->dev, "unexpected busy command channel\n");
			dcp->ch_cmd.warned_busy = true;
		}
		/* HACK: issue a delayed vblank event to avoid timeouts in
		 * drm_atomic_helper_wait_for_vblanks().
		 */
		schedule_work(&dcp->vblank_wq);
		return;
	} else if (dcp->ch_cmd.warned_busy) {
		dcp->ch_cmd.warned_busy = false;
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_flush_v12_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_flush_v13_3(dcp, crtc, state);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_flush_v26_6_0(dcp, crtc, state);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

static bool trial_modeset = true;
module_param(trial_modeset, bool, 0444);
MODULE_PARM_DESC(trial_modeset,
		 "Takeover: power on the DCP and set a mode before the boot framebuffer is evicted; on failure simpledrm keeps the screen (default 1)");

/*
 * M4 (P2, 2026-10-16): try the highest refresh rate at the panel's full resolution first. With the boot-like 60 Hz
 * trial the panel goes 60 -> 120 (greeter highrr) -> 60 (console restore at login) -> 120 (session), each a full
 * DCP power cycle of 0.2-0.25 s. With the 120 Hz timing as the trial, and so the console, mode, the later commits
 * find the mode already set: dcp_crtc_atomic_modeset() runs no firmware modeset unless DRM flags one. The
 * boot-like mode stays the next candidate. Default off (attended first).
 */
static bool trial_highrr;
module_param(trial_highrr, bool, 0444);
MODULE_PARM_DESC(trial_highrr,
		 "Takeover trial: try the highest refresh rate at full resolution first, the boot-like mode next (default 0)");

/* iBoot's panel mode: the pixel clock the firmware falls back to (O10). */
#define M4_BOOT_PIXEL_CLOCK_KHZ	521065
#define M4_TRIAL_MAX_CAND	3

/*
 * M4 takeover gate, called after the DCP started and before the boot
 * framebuffer is evicted. Everything that can still be refused is tried here,
 * while simpledrm still owns the screen: DCP ready, modes known, panel
 * connected, power on, one mode set. Returns 0 when it is safe to evict;
 * otherwise a negative errno and a short reason. Each failure leaves the DCP
 * running (its shutdown sequence can crash it) and the caller keeps simpledrm.
 */
int dcp_takeover_trial(struct platform_device *pdev, const char **reason)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_display_mode cand[M4_TRIAL_MAX_CAND];
	struct drm_crtc_state *st;
	unsigned int i, k, n = 0, pref = UINT_MAX, boot = UINT_MAX, hr = UINT_MAX;
	int win = -1, ret = -EIO, hr_rate = 0;
	u32 best = U32_MAX, hr_area = 0;

	*reason = "none";
	if (!dcp) {
		*reason = "no-dcp";
		return -ENODEV;
	}
	if (dcp->crashed) {
		*reason = "dcp-crashed";
		return -ENODEV;
	}
	if (!dcp->active) {
		*reason = "dcp-not-ready";
		return -ETIMEDOUT;
	}
	if (!READ_ONCE(dcp->nr_modes)) {
		*reason = "no-modes";
		return -ENODEV;
	}
	if (!dcp->connector || !dcp->connector->connected) {
		*reason = "not-connected";
		return -ENODEV;
	}
	dev_info(dcp->dev,
		 "M4_DCP_TRIAL_READY modes=%u main_display=%d panel=%d connector_type=%d trial=%d\n",
		 READ_ONCE(dcp->nr_modes), dcp->main_display,
		 dcp_has_panel(dcp), dcp->connector_type, trial_modeset);
	if (!trial_modeset)
		return 0;

	mutex_lock(&dcp->modes_lock);
	for (i = 0; i < dcp->nr_modes; i++) {
		const struct dcp_display_mode *m = &dcp->modes[i];

		u32 area = (u32)m->mode.hdisplay * m->mode.vdisplay;
		int rate = drm_mode_vrefresh(&m->mode);

		if ((m->mode.type & DRM_MODE_TYPE_PREFERRED) && pref == UINT_MAX)
			pref = i;
		if (area > hr_area || (area == hr_area && rate > hr_rate)) {
			hr = i;
			hr_area = area;
			hr_rate = rate;
		}
		if (!m->vrr) {
			u32 d = abs((int)m->mode.clock - M4_BOOT_PIXEL_CLOCK_KHZ);

			if (d < best) {
				best = d;
				boot = i;
			}
		}
	}
	if (pref == UINT_MAX)
		pref = 0;
	if (!trial_highrr)
		hr = UINT_MAX;
	if (hr != UINT_MAX)
		cand[n++] = dcp->modes[hr].mode;
	/* Only a mode within 1% of the firmware's default video clock counts as boot-like. */
	if (boot != UINT_MAX && best <= M4_BOOT_PIXEL_CLOCK_KHZ / 100) {
		if (boot != hr)
			cand[n++] = dcp->modes[boot].mode;
	} else {
		boot = UINT_MAX;
	}
	if (pref != boot && pref != hr)
		cand[n++] = dcp->modes[pref].mode;
	mutex_unlock(&dcp->modes_lock);

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		*reason = "no-memory";
		return -ENOMEM;
	}

	dcp->poweron_result = 0;
	dcp_poweron(pdev);
	dev_info(dcp->dev, "M4_DCP_TRIAL_POWERON result=%u (0 ok 1 timeout 2 refused 3 error)\n",
		 dcp->poweron_result);
	if (dcp->crashed || dcp->poweron_result >= 2) {
		*reason = dcp->crashed ? "dcp-crashed-in-poweron" : "poweron-refused";
		goto out;
	}

	for (k = 0; k < n; k++) {
		int r;

		st->mode = cand[k];
		dev_info(dcp->dev, "M4_DCP_TRIAL_MODESET try=%u " DRM_MODE_FMT "\n",
			 k, DRM_MODE_ARG(&st->mode));
		r = dcp_modeset_fw(dcp, st);
		dev_info(dcp->dev,
			 "M4_DCP_TRIAL_MODESET try=%u rc=%d valid_mode=%d timing=%u color=%u\n",
			 k, r, dcp->valid_mode, dcp->mode.timing_mode_id,
			 dcp->mode.color_mode_id);
		if (!r && dcp->valid_mode) {
			win = k;
			break;
		}
		if (dcp->crashed)
			break;
	}
	if (win < 0) {
		*reason = dcp->crashed ? "dcp-crashed-in-modeset" : "modeset-failed";
		goto out;
	}

	dcp->trial_timing_mode_id = dcp->mode.timing_mode_id;
	dcp->trial_color_mode_id = dcp->mode.color_mode_id;
	dcp->trial_powered = true;
	dcp->trial_modeset_ok = true;

	/* Make the mode that worked the one the console client picks. */
	mutex_lock(&dcp->modes_lock);
	for (i = 0; i < dcp->nr_modes; i++) {
		struct dcp_display_mode *m = &dcp->modes[i];

		if (m->timing_mode_id == dcp->trial_timing_mode_id &&
		    m->color_mode_id == dcp->trial_color_mode_id)
			m->mode.type |= DRM_MODE_TYPE_PREFERRED;
		else
			m->mode.type &= ~DRM_MODE_TYPE_PREFERRED;
	}
	mutex_unlock(&dcp->modes_lock);
	dev_info(dcp->dev, "M4_DCP_TRIAL_OK mode=" DRM_MODE_FMT " timing=%u color=%u\n",
		 DRM_MODE_ARG(&cand[win]), dcp->trial_timing_mode_id,
		 dcp->trial_color_mode_id);
	ret = 0;
out:
	kfree(st);
	return ret;
}

static void iomfb_start(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_start_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_start_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_start_v26_6_0(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

bool dcp_is_initialized(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->active;
}

void iomfb_recv_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcpep_type type = FIELD_GET(IOMFB_MESSAGE_TYPE, message);

	if (type == IOMFB_MESSAGE_TYPE_INITIALIZED)
		iomfb_start(dcp);
	else if (type == IOMFB_MESSAGE_TYPE_MSG)
		dcpep_got_msg(dcp, message);
	else
		dev_warn(dcp->dev, "Ignoring unknown message %llx\n", message);
}

int iomfb_start_rtkit(struct apple_dcp *dcp)
{
	dma_addr_t shmem_iova;

	if (dcp->fw_compat != DCP_FIRMWARE_V_26_6)
		apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);
	dcp->shmem = dma_alloc_coherent(dcp->dev, DCP_SHMEM_SIZE,
					 &shmem_iova, GFP_KERNEL);
	if (!dcp->shmem)
		return -ENOMEM;

	memset(dcp->shmem, 0, DCP_SHMEM_SIZE);
	dma_wmb();

	dcp_send_message(dcp, IOMFB_ENDPOINT, dcpep_set_shmem(shmem_iova));

	return 0;
}

void iomfb_shutdown(struct apple_dcp *dcp)
{
	/* We're going down */
	dcp->active = false;
	dcp->valid_mode = false;

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_shutdown_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_shutdown_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_shutdown_v26_6_0(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}
