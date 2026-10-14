// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright 2021 Alyssa Rosenzweig
 * Copyright The Asahi Linux Contributors
 */

#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "afk.h"
#include "iomfb.h"
#include "iomfb_internal.h"
#include "parser.h"
#include "trace.h"
#include "version_utils.h"

enum dcp_power_transition {
	DCP_POWER_TRANSITION_NONE,
	DCP_POWER_TRANSITION_DISPLAY_OFF,
	DCP_POWER_TRANSITION_RESTARTABLE_SLEEP,
	DCP_POWER_TRANSITION_SHUTDOWN,
};

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
typedef struct dcp_set_power_state_req_v26_6 dcp_power_state_req_t;
typedef struct iomfb_abort_swaps_dcp_req_v26_6 dcp_abort_swaps_req_t;
typedef struct iomfb_abort_swaps_dcp_resp_v26_6 dcp_abort_swaps_resp_t;
typedef struct dcp_read_edt_data_resp_v26_6 dcp_read_edt_data_resp_t;
typedef struct dcp_swap_complete_intent_gated_v26_6 dcp_swap_intent_t;
typedef struct dcp_bool_arg dcp_bool_arg_t;

static dcp_power_state_req_t dcp_power_state_req(u64 state,
						 enum dcp_power_transition transition)
{
	dcp_power_state_req_t req = {
		.state = state,
		.padding = { 0xaa, 0xaa, 0xaa },
	};

	switch (transition) {
	case DCP_POWER_TRANSITION_RESTARTABLE_SLEEP:
		req.flags[0] = 1;
		req.flags[3] = 1;
		req.result_is_null = 1;
		break;
	case DCP_POWER_TRANSITION_SHUTDOWN:
		req.flags[1] = 1;
		req.result_is_null = 1;
		break;
	case DCP_POWER_TRANSITION_NONE:
	case DCP_POWER_TRANSITION_DISPLAY_OFF:
		break;
	}

	return req;
}

static dcp_bool_arg_t dcp_bool_arg(bool value)
{
	return (dcp_bool_arg_t){
		.value = value,
		.padding = { 0xaa, 0xaa, 0xaa },
	};
}

/* Native 26.6 brightness and display-off swaps use binary64 nits, not iDAC. */
static void dcp_swap_set_brightness(struct DCP_FW_NAME(dcp_swap) *swap, u32 nits)
{
	u64 bits = 0;

	if (nits) {
		unsigned int exponent = fls(nits) - 1;

		/* All u32 integers are exactly representable; no kernel FP use. */
		bits = (u64)(exponent + 1023) << 52;
		bits |= ((u64)nits << (52 - exponent)) & GENMASK_ULL(51, 0);
	}

	swap->bl_update = 1;
	swap->bl_flags[1] = 1;
	put_unaligned_le64(bits, &swap->bl_nits);
}
#else
typedef struct dcp_set_power_state_req dcp_power_state_req_t;
typedef struct iomfb_abort_swaps_dcp_req dcp_abort_swaps_req_t;
typedef struct iomfb_abort_swaps_dcp_resp dcp_abort_swaps_resp_t;
typedef struct dcp_read_edt_data_resp dcp_read_edt_data_resp_t;
typedef struct dcp_swap_complete_intent_gated dcp_swap_intent_t;
typedef u32 dcp_bool_arg_t;

static dcp_power_state_req_t dcp_power_state_req(u64 state,
						 enum dcp_power_transition transition)
{
	return (dcp_power_state_req_t){
		.unklong = state,
		.unkbool = transition != DCP_POWER_TRANSITION_NONE,
		.unkint_null = transition != DCP_POWER_TRANSITION_NONE,
	};
}

static dcp_bool_arg_t dcp_bool_arg(bool value)
{
	return value;
}
#endif

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static u64 dcp_iomfb_client(struct apple_dcp *dcp)
{
	/* Native passes its IOUserClient pointer as an opaque client identity. */
	return (u64)(unsigned long)dcp;
}
#endif

static struct DCP_FW_NAME(dcp_swap_start_req)
dcp_swap_start_request(struct apple_dcp *dcp)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	return (struct DCP_FW_NAME(dcp_swap_start_req)){
		.client = dcp_iomfb_client(dcp),
		.padding = { 0xaa, 0xaa, 0xaa },
	};
#else
	return (struct DCP_FW_NAME(dcp_swap_start_req)){ 0 };
#endif
}

static dcp_abort_swaps_req_t dcp_abort_swaps_request(struct apple_dcp *dcp)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	return (dcp_abort_swaps_req_t){ .client = dcp_iomfb_client(dcp) };
#else
	return (dcp_abort_swaps_req_t){ .client.flag2 = 1 };
#endif
}

/* Register defines used in bandwidth setup structure */
#define REG_DOORBELL_BIT(idx) (2 + (idx))

struct dcp_wait_cookie {
	struct kref refcount;
	struct completion done;
	bool failed;
};

static void release_wait_cookie(struct kref *ref)
{
	struct dcp_wait_cookie *cookie;
	cookie = container_of(ref, struct dcp_wait_cookie, refcount);

        kfree(cookie);
}

/* Only a rejected enqueue has no receiver left to complete this cookie. */
static void dcp_wait_refused(struct apple_dcp *dcp, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	dev_warn(dcp->dev, "request publication refused\n");
	if (wait) {
		wait->failed = true;
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static void dcp_start_refused(struct apple_dcp *dcp)
{
	dev_warn(dcp->dev, "startup request publication refused\n");
	dcp->active = false;
	dcp->crashed = true;
	complete(&dcp->start_done);
}

DCP_THUNK_OUT(iomfb_a131_pmu_service_matched, iomfbep_a131_pmu_service_matched, u32);
DCP_THUNK_OUT(iomfb_a132_backlight_service_matched, iomfbep_a132_backlight_service_matched, u32);
DCP_THUNK_OUT(iomfb_a358_vi_set_temperature_hint, iomfbep_a358_vi_set_temperature_hint, u32);

IOMFB_THUNK_INOUT(set_matrix);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
IOMFB_THUNK_INOUT(get_gamma_table);
#endif
IOMFB_THUNK_INOUT(get_color_remap_mode);
IOMFB_THUNK_INOUT(last_client_close);
DCP_THUNK_INOUT(iomfb_abort_swaps_dcp, iomfbep_abort_swaps_dcp,
		dcp_abort_swaps_req_t, dcp_abort_swaps_resp_t);

DCP_THUNK_INOUT(dcp_swap_submit, dcpep_swap_submit,
		struct DCP_FW_NAME(dcp_swap_submit_req),
		struct DCP_FW_NAME(dcp_swap_submit_resp));

DCP_THUNK_INOUT(dcp_swap_start, dcpep_swap_start,
		struct DCP_FW_NAME(dcp_swap_start_req),
		struct DCP_FW_NAME(dcp_swap_start_resp));

DCP_THUNK_INOUT(dcp_set_power_state, dcpep_set_power_state,
		dcp_power_state_req_t,
		struct dcp_set_power_state_resp);

DCP_THUNK_INOUT(dcp_set_digital_out_mode, dcpep_set_digital_out_mode,
		struct dcp_set_digital_out_mode_req, u32);

DCP_THUNK_INOUT(dcp_set_display_device, dcpep_set_display_device, u32, u32);

DCP_THUNK_OUT(dcp_set_display_refresh_properties,
	      dcpep_set_display_refresh_properties, u32);

#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
DCP_THUNK_INOUT(dcp_late_init_signal, dcpep_late_init_signal,
		dcp_bool_arg_t, u32);
#else
DCP_THUNK_OUT(dcp_late_init_signal, dcpep_late_init_signal, u32);
#endif
DCP_THUNK_IN(dcp_flush_supports_power, dcpep_flush_supports_power,
	     dcp_bool_arg_t);
DCP_THUNK_OUT(dcp_create_default_fb, dcpep_create_default_fb, u32);
DCP_THUNK_OUT(dcp_start_signal, dcpep_start_signal, u32);
DCP_THUNK_VOID(dcp_setup_video_limits, dcpep_setup_video_limits);
DCP_THUNK_VOID(dcp_set_create_dfb, dcpep_set_create_dfb);
DCP_THUNK_VOID(dcp_first_client_open, dcpep_first_client_open);

DCP_THUNK_INOUT(dcp_set_parameter_dcp, dcpep_set_parameter_dcp,
		struct dcp_set_parameter_dcp, u32);

DCP_THUNK_INOUT(dcp_enable_disable_video_power_savings,
		dcpep_enable_disable_video_power_savings, u32, int);

DCP_THUNK_OUT(dcp_is_main_display, dcpep_is_main_display, u32);

/* DCP callback handlers */
static void dcpep_cb_nop(struct apple_dcp *dcp)
{
	/* No operation */
}

static u8 dcpep_cb_true(struct apple_dcp *dcp)
{
	return true;
}

static u8 dcpep_cb_false(struct apple_dcp *dcp)
{
	return false;
}

static u32 dcpep_cb_zero(struct apple_dcp *dcp)
{
	return 0;
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static void dcp_gamma_probe_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_gamma_table_resp *resp = out;
	u32 max = 0, ch, i;
	bool mono = true;

	for (ch = 0; ch < 3; ch++) {
		u32 prev = 0;

		for (i = 0; i < IOMFB_GAMMA_ENTRIES; i++) {
			u32 v = le32_to_cpu(resp->table[ch][i]);

			if (v > max)
				max = v;
			if (v < prev)
				mono = false;
			prev = v;
		}
	}
	dev_info(dcp->dev,
		 "M4_DCP_GAMMA_TABLE null=%u max=0x%x monotonic=%d r[0,1,2,128,255,256]=%x,%x,%x,%x,%x,%x g[1,128,256]=%x,%x,%x b[1,128,256]=%x,%x,%x\n",
		 resp->table_null, max, mono,
		 le32_to_cpu(resp->table[0][0]), le32_to_cpu(resp->table[0][1]), le32_to_cpu(resp->table[0][2]),
		 le32_to_cpu(resp->table[0][128]), le32_to_cpu(resp->table[0][255]), le32_to_cpu(resp->table[0][256]),
		 le32_to_cpu(resp->table[1][1]), le32_to_cpu(resp->table[1][128]), le32_to_cpu(resp->table[1][256]),
		 le32_to_cpu(resp->table[2][1]), le32_to_cpu(resp->table[2][128]), le32_to_cpu(resp->table[2][256]));
}

static void dcp_gamma_probe_start(struct apple_dcp *dcp)
{
	static struct iomfb_gamma_table_req req;

	if (!dcp_gamma_probe || dcp->m4_gamma_probed || !dcp->main_display || !dcp_has_panel(dcp))
		return;
	dcp->m4_gamma_probed = true;
	memset(&req, 0, sizeof(req));
	if (!iomfb_get_gamma_table(dcp, false, &req, dcp_gamma_probe_done, NULL))
		dev_warn(dcp->dev, "M4_DCP_GAMMA_TABLE request publication refused\n");
}
#else
static void dcp_gamma_probe_start(struct apple_dcp *dcp) {}
#endif

static void dcpep_cb_swap_complete(struct apple_dcp *dcp,
				   struct DCP_FW_NAME(dc_swap_complete_resp) *resp)
{
	ktime_t now = ktime_get();
	trace_iomfb_swap_complete(dcp, resp->swap_id);
	dcp->last_swap_id = resp->swap_id;
	if (dcp->m4_swaps_complete < 3) {
		dcp->m4_swaps_complete++;
		dev_info(dcp->dev, "M4_DCP_SWAP_COMPLETE n=%u swap_id=%u\n",
			 dcp->m4_swaps_complete, resp->swap_id);
		if (dcp->m4_swaps_complete == 3)
			dcp_gamma_probe_start(dcp);
	}

	if (dcp_swap_stats) {
		ktime_t start = dcp->swap_start;
		s64 span = ktime_us_delta(now, dcp->m4_stats_window);

		if (!dcp->m4_stats_window) {
			dcp->m4_stats_window = now;
		} else {
			if (dcp->m4_stats_last) {
				s64 gap = ktime_us_delta(now, dcp->m4_stats_last);

				dcp->m4_stats_gap_sum_us += gap;
				if (gap > dcp->m4_stats_gap_max_us)
					dcp->m4_stats_gap_max_us = gap;
			}
			if (start != KTIME_MIN) {
				s64 lat = ktime_us_delta(now, start);

				dcp->m4_stats_lat_sum_us += lat;
				if (lat > dcp->m4_stats_lat_max_us)
					dcp->m4_stats_lat_max_us = lat;
			}
			if (dcp->m4_commit_time) {
				s64 clat = ktime_us_delta(now, dcp->m4_commit_time);

				dcp->m4_stats_commit_sum_us += clat;
				if (clat > dcp->m4_stats_commit_max_us)
					dcp->m4_stats_commit_max_us = clat;
			}
			if (dcp->m4_intent_time && ktime_after(dcp->m4_intent_time, dcp->m4_complete_time) &&
			    !ktime_after(dcp->m4_intent_time, now)) {
				s64 lead = ktime_us_delta(now, dcp->m4_intent_time);

				dcp->m4_stats_intent_n++;
				dcp->m4_stats_intent_sum_us += lead;
				if (lead > dcp->m4_stats_intent_max_us)
					dcp->m4_stats_intent_max_us = lead;
			}
			dcp->m4_stats_n++;
			if (span >= 2000000 && dcp->m4_stats_n) {
				dev_info(dcp->dev,
					 "M4_DCP_SWAP_STATS window_ms=%lld swaps=%u rate_x10=%llu gap_us_avg=%llu gap_us_max=%u lat_us_avg=%llu lat_us_max=%u vrr_timestamps=%d commit_us_avg=%llu commit_us_max=%u flush_masks=%u/%u/%u intent_n=%u intent_lead_us_avg=%llu intent_lead_us_max=%u frame_ns=%llu\n",
					 span / 1000, dcp->m4_stats_n,
					 div64_u64(10000000ULL * dcp->m4_stats_n, span),
					 div64_u64(dcp->m4_stats_gap_sum_us, dcp->m4_stats_n),
					 dcp->m4_stats_gap_max_us,
					 div64_u64(dcp->m4_stats_lat_sum_us, dcp->m4_stats_n),
					 dcp->m4_stats_lat_max_us,
					 dcp->use_timestamps,
					 div64_u64(dcp->m4_stats_commit_sum_us, dcp->m4_stats_n),
					 dcp->m4_stats_commit_max_us,
					 dcp->m4_stats_mask[1], dcp->m4_stats_mask[2],
					 dcp->m4_stats_mask[3],
					 dcp->m4_stats_intent_n,
					 dcp->m4_stats_intent_n ?
						div64_u64(dcp->m4_stats_intent_sum_us, dcp->m4_stats_intent_n) : 0,
					 dcp->m4_stats_intent_max_us, dcp->m4_frame_ns);
				dcp->m4_stats_intent_n = dcp->m4_stats_intent_max_us = 0;
				dcp->m4_stats_intent_sum_us = 0;
				dcp->m4_stats_commit_sum_us = 0;
				dcp->m4_stats_commit_max_us = 0;
				memset(dcp->m4_stats_mask, 0, sizeof(dcp->m4_stats_mask));
				dcp->m4_stats_window = now;
				dcp->m4_stats_n = 0;
				dcp->m4_stats_gap_sum_us = dcp->m4_stats_lat_sum_us = 0;
				dcp->m4_stats_gap_max_us = dcp->m4_stats_lat_max_us = 0;
			}
		}
		dcp->m4_stats_last = now;
	}

	dcp_drm_crtc_page_flip(dcp, now);
	if (dcp->crc_enabled) {
		u32 crc32 = 0;
		drm_crtc_add_crc_entry(&dcp->crtc->base, true, resp->swap_id, &crc32);
	}
}

/* special */
static void complete_vi_set_temperature_hint(struct apple_dcp *dcp, void *out, void *cookie)
{
	// ack D100 cb_match_pmu_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

IOMFB_CB_SIZE(iomfbep_cb_match_pmu_service, 0, 0);
static bool iomfbep_cb_match_pmu_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	/* Refused calls own no nested slot; ACK this callback immediately. */
	return !iomfb_a358_vi_set_temperature_hint(dcp, false,
					    complete_vi_set_temperature_hint,
					    NULL);
}

static void complete_pmu_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;

	// ack D206 cb_match_pmu_service_2
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

IOMFB_CB_SIZE(iomfbep_cb_match_pmu_service_2, 0, 1);
static bool iomfbep_cb_match_pmu_service_2(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	return !iomfb_a131_pmu_service_matched(dcp, false,
					     complete_pmu_service_matched, out);
}

static void complete_backlight_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = cookie;

	/* The deferred reply must still own the pending firmware callback. */
	if (!out || !succ || !ch->depth || ch->depth > DCP_MAX_CALL_DEPTH ||
	    ch->host_call[ch->depth - 1] || ch->output[ch->depth - 1] != succ) {
		dev_warn(dcp->dev, "refusing stale backlight match completion\n");
		return;
	}

	/* A132 is a link status, not proof of panel illumination. */
	*succ = get_unaligned_le32(out) == 0;

	// ack D207 cb_match_backlight_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

IOMFB_CB_SIZE(iomfbep_cb_match_backlight_service, 0, 1);
static bool iomfbep_cb_match_backlight_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	if (!dcp_has_panel(dcp)) {
		u8 *succ = out;
		*succ = true;
		return true;
	}

	return !iomfb_a132_backlight_service_matched(dcp, false,
						   complete_backlight_service_matched,
						   out);
}

static void iomfb_cb_pr_publish(struct apple_dcp *dcp, struct iomfb_property *prop)
{
	switch (prop->id) {
#if DCP_FW_VER < DCP_FW_VERSION(26, 6, 0)
	case IOMFB_PROPERTY_NITS:
	{
		if (dcp_has_panel(dcp)) {
			u32 nits;

			if (dcp->brightness.scale <= 0)
				return;
			nits = prop->value / dcp->brightness.scale;
			if (nits > INT_MAX || nits > dcp->brightness.maximum)
				return;
			dcp->brightness.nits = nits;
			/* notify backlight device of the initial brightness */
			if (dcp_takeover && !dcp->brightness.bl_dev &&
			    dcp->brightness.maximum > 0)
				schedule_work(&dcp->bl_register_wq);
			trace_iomfb_brightness(dcp, prop->value);
		}
		break;
	}
#endif
	default:
		dev_dbg(dcp->dev, "pr_publish: id: %d = %u\n", prop->id, prop->value);
	}
}

static struct dcp_get_uint_prop_resp
dcpep_cb_get_uint_prop(struct apple_dcp *dcp, struct dcp_get_uint_prop_req *req)
{
	struct dcp_get_uint_prop_resp resp = (struct dcp_get_uint_prop_resp){
	    .value = 0
	};

	if (dcp->panel.has_mini_led &&
	    memcmp(req->obj, "SUMP", sizeof(req->obj)) == 0) { /* "PMUS */
	    if (strncmp(req->key, "Temperature", sizeof(req->key)) == 0) {
		/*
		 * TODO: value from j314c, find out if it is temperature in
		 *       centigrade C and which temperature sensor reports it
		 */
		resp.value = 3029;
		resp.ret = true;
	    }
	}

	return resp;
}

static u8 iomfbep_cb_sr_set_property_int(struct apple_dcp *dcp,
					 struct iomfb_sr_set_property_int_req *req)
{
	if (memcmp(req->obj, "FMOI", sizeof(req->obj)) == 0) { /* "IOMF */
		if (strncmp(req->key, "Brightness_Scale", sizeof(req->key)) == 0) {
			if (!req->value_null) {
				/* The wire integer is 64-bit; our divisor is positive int. */
				if (!req->value || req->value > INT_MAX)
					return 0;
				dcp->brightness.scale = req->value;
			}
		}
	}

	return 1;
}

static void iomfbep_cb_set_fx_prop(struct apple_dcp *dcp, struct iomfb_set_fx_prop_req *req)
{
    // TODO: trace this, see if there properties which needs to used later
}

/*
 * Callback to map a buffer allocated with allocate_buf for PIODMA usage.
 * PIODMA is separate from the main DCP and uses own IOVA space on a dedicated
 * stream of the display DART, rather than the expected DCP DART.
 */
static struct DCP_FW_NAME(dcp_map_buf_resp)
dcpep_cb_map_piodma(struct apple_dcp *dcp, struct dcp_map_buf_req *req)
{
	struct dcp_mem_descriptor *memdesc;
	struct sg_table *map;
	ssize_t ret;

	if (req->buffer >= ARRAY_SIZE(dcp->memdesc))
		goto reject;

	memdesc = &dcp->memdesc[req->buffer];
	map = &memdesc->map;

	if (!map->sgl)
		goto reject;

	/* use the piodma iommu domain to map against the right IOMMU */
	ret = iommu_map_sgtable(dcp->iommu_dom, memdesc->dva, map,
				IOMMU_READ | IOMMU_WRITE);

	/* HACK: expect size to be 16K aligned since the iommu API only maps
	 *       full pages
	 */
	if (ret < 0 || ret != ALIGN(memdesc->size, SZ_16K)) {
		dev_err(dcp->dev, "iommu_map_sgtable() returned %zd instead of expected buffer size of %zu\n", ret, memdesc->size);
		goto reject;
	}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){
		.buffer = req->buffer,
		.dva = memdesc->dva,
	};
#else
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){ .dva = memdesc->dva };
#endif

reject:
	dev_err(dcp->dev, "denying map of invalid buffer %llx for piodma\n",
		req->buffer);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){
		.buffer = req->buffer,
		.ret = EINVAL,
	};
#else
	return (struct DCP_FW_NAME(dcp_map_buf_resp)){ .ret = EINVAL };
#endif
}

static void dcpep_cb_unmap_piodma(struct apple_dcp *dcp,
				  struct dcp_unmap_buf_resp *resp)
{
	struct dcp_mem_descriptor *memdesc;

	if (resp->buffer >= ARRAY_SIZE(dcp->memdesc)) {
		dev_warn(dcp->dev, "unmap request for out of range buffer %llu\n",
			 resp->buffer);
		return;
	}

	memdesc = &dcp->memdesc[resp->buffer];

	if (!memdesc->buf) {
		dev_warn(dcp->dev,
			 "unmap for non-mapped buffer %llu iova:0x%08llx\n",
			 resp->buffer, resp->dva);
		return;
	}

	if (memdesc->dva != resp->dva) {
		dev_warn(dcp->dev, "unmap buffer %llu address mismatch "
			 "memdesc.dva:%llx dva:%llx\n", resp->buffer,
			 memdesc->dva, resp->dva);
		return;
	}

	/* use the piodma iommu domain to unmap from the right IOMMU */
	/* HACK: expect size to be 16K aligned since the iommu API only maps
	 *       full pages
	 */
	iommu_unmap(dcp->iommu_dom, memdesc->dva, ALIGN(memdesc->size, SZ_16K));
}

/*
 * Allocate an IOVA contiguous buffer mapped to the DCP. The buffer need not be
 * physically contiguous, however we should save the sgtable in case the
 * buffer needs to be later mapped for PIODMA.
 */
static struct dcp_allocate_buffer_resp
dcpep_cb_allocate_buffer(struct apple_dcp *dcp,
			 struct dcp_allocate_buffer_req *req)
{
	struct dcp_allocate_buffer_resp resp = { 0 };
	struct dcp_mem_descriptor *memdesc;
	size_t size;
	u32 id;

	resp.dva_size = ALIGN(req->size, 4096);
	resp.mem_desc_id =
		find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);

	if (resp.mem_desc_id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev, "DCP overflowed mapping table, ignoring\n");
		resp.dva_size = 0;
		resp.mem_desc_id = 0;
		return resp;
	}
	id = resp.mem_desc_id;
	set_bit(id, dcp->memdesc_map);

	memdesc = &dcp->memdesc[id];

	memdesc->size = resp.dva_size;
	/* HACK: align size to 16K since the iommu API only maps full pages */
	size = ALIGN(resp.dva_size, SZ_16K);
	memdesc->buf = dma_alloc_coherent(dcp->dev, size,
					  &memdesc->dva, GFP_KERNEL);

	dma_get_sgtable(dcp->dev, &memdesc->map, memdesc->buf, memdesc->dva,
			size);
	resp.dva = memdesc->dva;

	return resp;
}

static u8 dcpep_cb_release_mem_desc(struct apple_dcp *dcp, u32 *mem_desc_id)
{
	struct dcp_mem_descriptor *memdesc;
	size_t size;
	u32 id = *mem_desc_id;

	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev,
			 "unmap request for out of range mem_desc_id %u", id);
		return 0;
	}

	if (!test_and_clear_bit(id, dcp->memdesc_map)) {
		dev_warn(dcp->dev, "unmap request for unused mem_desc_id %u\n",
			 id);
		return 0;
	}

	memdesc = &dcp->memdesc[id];
	size = ALIGN(memdesc->size, SZ_16K);
	if (memdesc->buf) {
		dma_free_coherent(dcp->dev, size, memdesc->buf, memdesc->dva);
		memdesc->buf = NULL;
		memset(&memdesc->map, 0, sizeof(memdesc->map));
	} else {
		memdesc->reg = 0;
	}

	memdesc->size = 0;

	return 1;
}

/* Validate that the specified region is a display register */
static bool is_disp_register(struct apple_dcp *dcp, u64 start, u64 end)
{
	int i;

	for (i = 0; i < dcp->nr_disp_registers; ++i) {
		struct resource *r = dcp->disp_registers[i];

		if ((start >= r->start) && (end <= r->end))
			return true;
	}

	return false;
}

/*
 * Map contiguous physical memory into the DCP's address space. The firmware
 * uses this to map the display registers we advertise in
 * sr_map_device_memory_with_index, so we bounds check against that to guard
 * safe against malicious coprocessors.
 */
static struct dcp_map_physical_resp
dcpep_cb_map_physical(struct apple_dcp *dcp, struct dcp_map_physical_req *req)
{
	int size = ALIGN(req->size, 4096);
	dma_addr_t dva;
	u32 id;

	if (!is_disp_register(dcp, req->paddr, req->paddr + size - 1)) {
		dev_err(dcp->dev, "refusing to map phys address %llx size %llx\n",
			req->paddr, req->size);
		return (struct dcp_map_physical_resp){};
	}

	id = find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	set_bit(id, dcp->memdesc_map);
	dcp->memdesc[id].size = size;
	dcp->memdesc[id].reg = req->paddr;

	dva = dma_map_resource(dcp->dev, req->paddr, size, DMA_BIDIRECTIONAL, 0);
	WARN_ON(dva == DMA_MAPPING_ERROR);

	return (struct dcp_map_physical_resp){
		.dva_size = size,
		.mem_desc_id = id,
		.dva = dva,
	};
}

static u64 dcpep_cb_get_frequency(struct apple_dcp *dcp)
{
	if (!dcp->clk) {
		dev_warn_once(dcp->dev,
			      "M4_DCP_CLK_ABSENT reporting %lu Hz to firmware\n",
			      dcp_clk_hz);
		return dcp_clk_hz;
	}
	return clk_get_rate(dcp->clk);
}

static struct DCP_FW_NAME(dcp_map_reg_resp) dcpep_cb_map_reg(struct apple_dcp *dcp,
						struct DCP_FW_NAME(dcp_map_reg_req) *req)
{
	if (req->index >= dcp->nr_disp_registers) {
		dev_warn(dcp->dev, "attempted to read invalid reg index %u\n",
			 req->index);

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){ .ret = 1 };
	} else {
		struct resource *rsrc = dcp->disp_registers[req->index];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		dma_addr_t dva;
		size_t size = resource_size(rsrc);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
		/*
		 * 26.6 returns register-window physical addresses directly. DCP
		 * issues these firmware-owned MMIO transactions on its dedicated
		 * DART-bypass stream; installing a mapping in the normal DMA stream
		 * does not affect them.
		 */
		dva = rsrc->start;
		dev_dbg(dcp->dev,
			 "D411: physical reg[%u] DVA/PA %#llx size %#zx\n",
			 req->index, (unsigned long long)dva, size);
#else
		dva = dma_map_resource(dcp->dev, rsrc->start, size,
				       DMA_BIDIRECTIONAL, 0);
		WARN_ON(dva == DMA_MAPPING_ERROR);
#endif
#endif

		return (struct DCP_FW_NAME(dcp_map_reg_resp)){
			.addr = rsrc->start,
			.length = resource_size(rsrc),
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
			.dva = dva,
#endif
		};
	}
}

static dcp_read_edt_data_resp_t
dcpep_cb_read_edt_data(struct apple_dcp *dcp, struct dcp_read_edt_data_req *req)
{
	return (dcp_read_edt_data_resp_t){
		.value[0] = req->value[0],
		.ret = 0,
	};
}

static void iomfbep_cb_enable_backlight_message_ap_gated(struct apple_dcp *dcp,
							 u8 *enabled)
{
	/*
	 * update backlight brightness on next swap, on non mini-LED displays
	 * DCP seems to set an invalid iDAC value after coming out of DPMS.
	 * syslog: "[BrightnessLCD.cpp:743][AFK]nitsToDBV: iDAC out of range"
	 */
	if (!dcp_takeover)
		return;
	dcp->brightness.update = true;
	schedule_work(&dcp->bl_update_wq);
}

/* Chunked data transfer for property dictionaries */
static u8 dcpep_cb_prop_start(struct apple_dcp *dcp, u32 *length)
{
	if (dcp->chunks.data != NULL || dcp->chunks_received != NULL) {
		dev_warn(dcp->dev, "ignoring spurious transfer start\n");
		return false;
	}

	if (!*length || *length > INT_MAX) {
		dev_warn(dcp->dev, "ignoring empty transfer\n");
		return false;
	}

	dcp->chunks.data = devm_kzalloc(dcp->dev, *length, GFP_KERNEL);

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "failed to allocate chunks\n");
		return false;
	}

	/*
	 * devm_bitmap_zalloc() registers a devres *action*, which devm_kfree()
	 * cannot find (WARN, nothing freed).  A plain devres allocation can be
	 * released with devm_kfree().
	 */
	dcp->chunks_received = devm_kcalloc(dcp->dev, BITS_TO_LONGS(*length),
					    sizeof(unsigned long), GFP_KERNEL);
	if (!dcp->chunks_received) {
		devm_kfree(dcp->dev, dcp->chunks.data);
		dcp->chunks.data = NULL;
		return false;
	}
	dcp->chunks.length = *length;
	dcp->chunk_calls = 0;
	dcp->chunk_last_offset = 0;
	dcp->chunk_last_length = 0;
	dev_info(dcp->dev, "M4_DCP_PROP_START length=%u\n", *length);
	return true;
}

static u8 dcpep_cb_prop_chunk(struct apple_dcp *dcp,
			      struct dcp_set_dcpav_prop_chunk_req *req)
{
	if (!dcp->chunks.data || !dcp->chunks_received) {
		dev_warn(dcp->dev, "ignoring spurious chunk\n");
		return false;
	}

	if (req->length > sizeof(req->data) ||
	    req->offset > dcp->chunks.length ||
	    req->length > dcp->chunks.length - req->offset) {
		dev_warn(dcp->dev, "ignoring overflowing chunk\n");
		return false;
	}

	memcpy(dcp->chunks.data + req->offset, req->data, req->length);
	if (req->length)
		bitmap_set(dcp->chunks_received, req->offset, req->length);
	if (dcp->chunk_calls < 4)
		dev_info(dcp->dev, "M4_DCP_PROP_CHUNK #%u offset=%u length=%u of %zu\n",
			 dcp->chunk_calls, req->offset, req->length,
			 dcp->chunks.length);
	dcp->chunk_calls++;
	dcp->chunk_last_offset = req->offset;
	dcp->chunk_last_length = req->length;
	return true;
}

static bool dcpep_process_chunks(struct apple_dcp *dcp,
				 struct dcp_set_dcpav_prop_end_req *req)
{
	struct dcp_parse_ctx ctx;
	int ret;

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious end\n");
		return false;
	}

	/* used just as opaque pointer for tracing */
	ctx.dcp = dcp;

	ret = parse(dcp->chunks.data, dcp->chunks.length, &ctx);

	if (ret) {
		dev_warn(dcp->dev, "bad header on dcpav props\n");
		return false;
	}

	if (!strcmp(req->key, "TimingElements")) {
		struct dcp_display_mode *modes;
		unsigned int nr_modes;

		modes = enumerate_modes(&ctx, &nr_modes, dcp->width_mm,
					dcp->height_mm, dcp->notch_height);
		if (IS_ERR(modes)) {
			dev_warn(dcp->dev, "failed to parse modes\n");
			return false;
		}

		mutex_lock(&dcp->modes_lock);
		kfree(dcp->modes);
		dcp->modes = modes;
		WRITE_ONCE(dcp->nr_modes, nr_modes);
		mutex_unlock(&dcp->modes_lock);
		dev_info(dcp->dev, "M4_DCP_MODES n=%u\n", nr_modes);
		for (unsigned int i = 0; i < nr_modes && i < 4; i++)
			dev_info(dcp->dev, "M4_DCP_MODE #%u %ux%u clock=%d vrefresh=%d\n",
				 i, modes[i].mode.hdisplay, modes[i].mode.vdisplay,
				 modes[i].mode.clock, drm_mode_vrefresh(&modes[i].mode));
		if (nr_modes == 0)
			dev_warn(dcp->dev, "TimingElements without valid modes!\n");
	} else if (!strcmp(req->key, "DisplayAttributes")) {
		ret = parse_display_attributes(&ctx, &dcp->width_mm,
					&dcp->height_mm);

		if (ret) {
			dev_warn(dcp->dev, "failed to parse display attribs\n");
			return false;
		}

		dev_info(dcp->dev, "M4_DCP_DISPLAY_ATTRS width_mm=%d height_mm=%d\n",
			 dcp->width_mm, dcp->height_mm);
		dcp_set_dimensions(dcp);
	}

	return true;
}

static u8 dcpep_cb_prop_end(struct apple_dcp *dcp,
			    struct dcp_set_dcpav_prop_end_req *req)
{
	bool named, complete = false, relaxed = false;
	unsigned long got = 0, gap = 0;
	size_t len = dcp->chunks.length;
	u8 resp;

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious end\n");
		return false;
	}

	named = memchr(req->key, '\0', sizeof(req->key));
	if (dcp->chunks_received) {
		got = bitmap_weight(dcp->chunks_received, len);
		gap = find_first_zero_bit(dcp->chunks_received, len);
		complete = gap >= len;
		/*
		 * 26.6 declares N + 1 bytes and sends N: only the final
		 * (terminator) byte is missing.  Everything before it must be
		 * there; the zeroed backing store supplies the terminator.
		 */
		relaxed = !complete && len > 1 && gap == len - 1;
	}

	if (!named || !(complete || relaxed)) {
		dev_warn(dcp->dev,
			 "M4_DCP_PROP_INCOMPLETE key=%.*s named=%d length=%zu received=%lu first_gap=%lu chunks=%u last=%u+%u\n",
			 named ? (int)sizeof(req->key) : 0, req->key, named,
			 len, got, gap, dcp->chunk_calls,
			 dcp->chunk_last_offset, dcp->chunk_last_length);
		print_hex_dump(KERN_INFO, "M4_DCP_PROP_END_REQ ", DUMP_PREFIX_OFFSET,
			       16, 1, req, sizeof(req->key), false);
		dev_warn(dcp->dev, "ignoring incomplete or unnamed property\n");
		devm_kfree(dcp->dev, dcp->chunks.data);
		dcp->chunks.data = NULL;
		dcp->chunks.length = 0;
		devm_kfree(dcp->dev, dcp->chunks_received);
		dcp->chunks_received = NULL;
		return false;
	}

	if (relaxed)
		dev_info(dcp->dev,
			 "M4_DCP_PROP_RELAXED key=%s declared=%zu covered=%lu missing=1 trailing\n",
			 req->key, len, got);

	resp = dcpep_process_chunks(dcp, req);

	dev_info(dcp->dev,
		 "M4_DCP_PROP_END key=%s declared=%zu covered=%lu chunks=%u relaxed=%d parsed=%d\n",
		 req->key, len, got, dcp->chunk_calls, relaxed, resp);

	/* Debugfs takes data ownership, never the receipt bitmap. */
	if (dcp->connector)
		dcp_connector_update_dict(dcp->connector, req->key, &dcp->chunks);
	else
		devm_kfree(dcp->dev, dcp->chunks.data);
	devm_kfree(dcp->dev, dcp->chunks_received);
	dcp->chunks_received = NULL;
	dcp->chunks.data = NULL;
	dcp->chunks.length = 0;
	return resp;
}

/* Boot sequence */
static bool boot_reply_pending(struct apple_dcp *dcp, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;

	return cookie && ch->depth && ch->depth <= DCP_MAX_CALL_DEPTH &&
	       !ch->host_call[ch->depth - 1] &&
	       ch->output[ch->depth - 1] == cookie;
}

static void boot_refused(struct apple_dcp *dcp, void *cookie)
{
	if (!boot_reply_pending(dcp, cookie))
		return;
	*(u8 *)cookie = false;
	dcp_start_refused(dcp);
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static void boot_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (!boot_reply_pending(dcp, cookie))
		return;
	dev_dbg(dcp->dev, "boot done\n");
	*(u8 *)cookie = !dcp->crashed;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static void boot_5(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (dcp->crashed) {
		boot_refused(dcp, cookie);
		return;
	}
	if (!dcp_set_display_refresh_properties(dcp, false, boot_done, cookie))
		boot_refused(dcp, cookie);
}

static void boot_4(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	dcp_bool_arg_t v_true = dcp_bool_arg(true);
#endif
	if (dcp->crashed) {
		boot_refused(dcp, cookie);
		return;
	}
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	if (!dcp_late_init_signal(dcp, false, &v_true, boot_5, cookie))
#else
	if (!dcp_late_init_signal(dcp, false, boot_5, cookie))
#endif
		boot_refused(dcp, cookie);
}

static void boot_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_bool_arg_t v_true = dcp_bool_arg(true);

	if (dcp->crashed) {
		boot_refused(dcp, cookie);
		return;
	}
	if (!dcp_flush_supports_power(dcp, false, &v_true, boot_4, cookie))
		boot_refused(dcp, cookie);
}

static void boot_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (dcp->crashed) {
		boot_refused(dcp, cookie);
		return;
	}
	if (!dcp_setup_video_limits(dcp, false, boot_3, cookie))
		boot_refused(dcp, cookie);
}

static void boot_1_5(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (dcp->crashed) {
		boot_refused(dcp, cookie);
		return;
	}
	if (!dcp_create_default_fb(dcp, false, boot_2, cookie))
		boot_refused(dcp, cookie);
}

/* Use special function signature to defer the ACK */
IOMFB_CB_SIZE(dcpep_cb_boot_1, 0, 1);
static bool dcpep_cb_boot_1(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	if (dcp->crashed)
		return true;
	if (!dcp_set_create_dfb(dcp, false, boot_1_5, out)) {
		dcp_start_refused(dcp);
		return true;
	}
	return false;
}

static struct dcp_allocate_bandwidth_resp dcpep_cb_allocate_bandwidth(struct apple_dcp *dcp,
						struct dcp_allocate_bandwidth_req *req)
{
	return (struct dcp_allocate_bandwidth_resp){
		.unk1 = req->unk1,
		.unk2 = req->unk2,
		.ret = 1,
	};
}

static struct dcp_rt_bandwidth dcpep_cb_rt_bandwidth(struct apple_dcp *dcp)
{
	struct dcp_rt_bandwidth rt_bw = (struct dcp_rt_bandwidth){
			.reg_scratch = 0,
			.reg_doorbell = 0,
			.doorbell_bit = 0,
	};

	if (!dcp_disable_rt_bandwidth && dcp->disp_bw_scratch_index) {
		u32 offset = dcp->disp_bw_scratch_offset;
		u32 index = dcp->disp_bw_scratch_index;
		rt_bw.reg_scratch = dcp->disp_registers[index]->start + offset;
	}

	if (!dcp_disable_rt_bandwidth && dcp->disp_bw_doorbell_index) {
		u32 index = dcp->disp_bw_doorbell_index;
		rt_bw.reg_doorbell = dcp->disp_registers[index]->start;
		rt_bw.doorbell_bit = REG_DOORBELL_BIT(dcp->index);
		/*
		 * This is most certainly not padding. t8103-dcp crashes without
		 * setting this immediately during modeset on 12.3 and 13.5
		 * firmware.
		 */
		rt_bw.padding[3] = 0x4;
	}

	return rt_bw;
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static struct dcp_set_frame_sync_props_resp_v26_6_0
dcpep_cb_set_frame_sync_props(
	struct apple_dcp *dcp,
	struct dcp_set_frame_sync_props_req_v26_6_0 *req)
{
	struct dcp_set_frame_sync_props_resp_v26_6_0 resp;

	memcpy(resp.props, req->props, sizeof(resp.props));
	memset(&resp.props[73], 0, 3);
	return resp;
}
#else
static struct dcp_set_frame_sync_props_resp
dcpep_cb_set_frame_sync_props(struct apple_dcp *dcp,
			      struct dcp_set_frame_sync_props_req *req)
{
	return (struct dcp_set_frame_sync_props_resp){};
}
#endif

/* Callback to get the current time as milliseconds since the UNIX epoch */
static u64 dcpep_cb_get_time(struct apple_dcp *dcp)
{
	return ktime_to_ms(ktime_get_real());
}

struct dcp_swap_cookie {
	struct kref refcount;
	struct completion done;
	bool failed;
	u32 swap_id;
};

static void release_swap_cookie(struct kref *ref)
{
	struct dcp_swap_cookie *cookie;
	cookie = container_of(ref, struct dcp_swap_cookie, refcount);

        kfree(cookie);
}

static void dcp_clear_refused(struct apple_dcp *dcp, void *cookie)
{
	struct dcp_swap_cookie *info = cookie;

	dev_warn(dcp->dev, "clear-swap publication refused\n");
	if (info) {
		info->failed = true;
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
	}
}

static void dcp_swap_cleared(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
	}

	if (resp->ret) {
		dev_err(dcp->dev, "swap_clear failed! status %u\n", resp->ret);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	while (!list_empty(&dcp->swapped_out_fbs)) {
		struct dcp_fb_reference *entry;
		entry = list_first_entry(&dcp->swapped_out_fbs,
					 struct dcp_fb_reference, head);
		if (entry->swap_id == dcp->last_swap_id)
			break;
		if (entry->fb)
			drm_framebuffer_put(entry->fb);
		list_del(&entry->head);
		kfree(entry);
	}
}

static void dcp_swap_clear_started(struct apple_dcp *dcp, void *data,
				   void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;
	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		info->swap_id = resp->swap_id;
	}

	if (!dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swap_cleared, cookie))
		dcp_clear_refused(dcp, cookie);
}

static void dcp_on_final(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out,
				 void *cookie);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static const struct dcp_method_entry dcp_prepare_wake_method = {
	.name = "dcpep_prepare_wake",
	.tag = { 'A', '4', '1', '4' },
};

static const struct dcp_method_entry dcp_finish_wake_method = {
	.name = "dcpep_finish_wake",
	.tag = { 'A', '4', '7', '4' },
};

static const struct dcp_method_entry dcp_get_mode_info_method = {
	.name = "dcpep_get_mode_info",
	.tag = { 'A', '3', '6', '4' },
};
#endif

static void dcp_on_set_power_state(struct apple_dcp *dcp, void *out, void *cookie)
{
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	u32 handle = dcp->main_display ? 0 : 2;

	if (!dcp_set_display_device(dcp, false, &handle,
			       dcp->main_display ? dcp_on_final : dcp_on_set_parameter,
			       cookie))
		dcp_wait_refused(dcp, cookie);
#else
	dcp_power_state_req_t req =
		dcp_power_state_req(1, DCP_POWER_TRANSITION_NONE);

	if (!dcp_set_power_state(dcp, false, &req, dcp_on_final, cookie))
		dcp_wait_refused(dcp, cookie);
#endif
}

static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_set_parameter_dcp param;
	u32 count =
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
		3;
#else
		1;
#endif

	memset(&param, 0xaa, sizeof(param));
	param.param = IOMFBPARAM_ADAPTIVE_SYNC;
	memset(param.value, 0, count * sizeof(param.value[0]));
	param.count = count;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	if (dcp_adaptive_sync_min_hz) {
		if (dcp_adaptive_sync_min_hz == 24 && dcp->main_display && dcp_has_panel(dcp)) {
			/* value[0] minRR (16.16 Hz), value[1] mediaTargetRate = 0 (no media mode), value[2] fractional = 0 */
			param.value[0] = (u64)dcp_adaptive_sync_min_hz << 16;
			dev_info(dcp->dev, "M4_DCP_ADAPTIVE_SYNC min_rr=0x%llx media_rate=0 fractional=0\n",
				 param.value[0]);
		} else {
			dev_warn(dcp->dev,
				 "M4_DCP_ADAPTIVE_SYNC_IGNORED value=%u main_display=%d panel=%d (accepted: 0, 24 on the internal panel)\n",
				 dcp_adaptive_sync_min_hz, dcp->main_display, dcp_has_panel(dcp));
		}
	}
#endif

	if (!dcp_set_parameter_dcp(dcp, false, &param,
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
			      dcp_on_final,
#else
			      dcp_on_set_power_state,
#endif
			      cookie))
		dcp_wait_refused(dcp, cookie);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static void dcp_poweron_after_main_display(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	dcp_power_state_req_t power_req =
		dcp_power_state_req(1, DCP_POWER_TRANSITION_NONE);

	if (!dcp_set_power_state(dcp, false, &power_req, dcp_on_set_power_state,
			    cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_poweron_after_finish_wake(struct apple_dcp *dcp, void *out,
					  void *cookie)
{
	if (!dcp_is_main_display(dcp, false, dcp_poweron_after_main_display, cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_poweron_after_first_client(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	if (!dcp_push(dcp, false, &dcp_finish_wake_method, 0, sizeof(u32), NULL,
		 dcp_poweron_after_finish_wake, cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_poweron_after_prepare_wake(struct apple_dcp *dcp, void *out,
					   void *cookie)
{
	if (!dcp_first_client_open(dcp, false, dcp_poweron_after_first_client,
			      cookie))
		dcp_wait_refused(dcp, cookie);
}
#endif

void DCP_FW_NAME(iomfb_poweron)(struct apple_dcp *dcp)
{
	struct dcp_wait_cookie *cookie;
	int ret;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	const u8 prepare_wake[8] = { 0, 0, 0, 0, 0, 0xaa, 0xaa, 0xaa };
#else
	u32 handle;
#endif
	dev_info(dcp->dev, "dcp_poweron() starting\n");
	dcp->poweron_result = 0;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie) {
		dcp->poweron_result = 3;
		return;
	}

	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	/* This is the wake ordering observed on 26.6 after A500 put the
	 * external-display pipeline to sleep.  Omitting A414/A474, or selecting
	 * the display before A473, leaves fControllerPowerState at zero and DCP
	 * silently swallows every scanout swap. */
	if (!dcp_push(dcp, false, &dcp_prepare_wake_method, sizeof(prepare_wake),
		 sizeof(u64), (void *)prepare_wake,
		 dcp_poweron_after_prepare_wake, cookie))
		dcp_wait_refused(dcp, cookie);
#else
	if (dcp->main_display) {
		handle = 0;
		if (!dcp_set_display_device(dcp, false, &handle, dcp_on_set_power_state,
				       cookie))
			dcp_wait_refused(dcp, cookie);
	} else {
		handle = 2;
		if (!dcp_set_display_device(dcp, false, &handle,
				       dcp_on_set_parameter, cookie))
			dcp_wait_refused(dcp, cookie);
	}
#endif
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(10000));

	if (cookie->failed) {
		dcp->poweron_result = 2;
		drm_connector_set_link_status_property(&dcp->connector->base,
						       DRM_MODE_LINK_STATUS_BAD);
	} else if (ret == 0) {
		dcp->poweron_result = 1;
		dev_warn(dcp->dev, "wait for power timed out, connector will be broken\n");
	} else if (ret > 0) {
		int msecs = jiffies_to_msecs(ret);
		if (msecs > 6000)
			dev_info(dcp->dev, "dcp_set_power_state_req returned, %d ms remaining\n", msecs);
		else
			dev_warn(dcp->dev, "dcp_set_power_state_req returned, %d ms remaining\n", msecs);
	} else {
		dcp->poweron_result = 3;
		drm_connector_set_link_status_property(&dcp->connector->base,
						       DRM_MODE_LINK_STATUS_BAD);
		dev_warn(dcp->dev, "wait for completion error: %d\n", ret);
	}
	dev_info(dcp->dev, "M4_DCP_POWERON result=%u (0 ok 1 timeout 2 refused 3 error)\n",
		 dcp->poweron_result);

	kref_put(&cookie->refcount, release_wait_cookie);;

	/* Force a brightness update after poweron, to restore the brightness */
	dcp->brightness.update = true;
}

static void complete_set_powerstate(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static void dcp_poweroff_after_main_display(struct apple_dcp *dcp, void *out,
					     void *cookie)
{
	dcp_power_state_req_t power_req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_DISPLAY_OFF);

	if (!dcp_set_power_state(dcp, false, &power_req, complete_set_powerstate,
			    cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_poweroff_after_prepare_state(struct apple_dcp *dcp, void *out,
					      void *cookie)
{
	if (!dcp_is_main_display(dcp, false, dcp_poweroff_after_main_display, cookie))
		dcp_wait_refused(dcp, cookie);
}
#endif

#if DCP_FW_VER < DCP_FW_VERSION(26, 6, 0)
static void last_client_closed_poff(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	dcp_power_state_req_t power_req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_DISPLAY_OFF);
	if (!dcp_set_power_state(dcp, false, &power_req, complete_set_powerstate,
			    cookie))
		dcp_wait_refused(dcp, cookie);
}

static void aborted_swaps_dcp_poff(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_last_client_close_req last_client_req = {};
	if (!iomfb_last_client_close(dcp, false, &last_client_req,
				last_client_closed_poff, cookie))
		dcp_wait_refused(dcp, cookie);
}
#endif

void DCP_FW_NAME(iomfb_poweroff)(struct apple_dcp *dcp)
{
	int ret, swap_id;
#if DCP_FW_VER < DCP_FW_VERSION(26, 6, 0)
	dcp_abort_swaps_req_t abort_req = dcp_abort_swaps_request(dcp);
#endif
	struct dcp_swap_cookie *cookie;
	struct dcp_wait_cookie *poff_cookie;
	struct DCP_FW_NAME(dcp_swap_start_req) swap_req =
		dcp_swap_start_request(dcp);
	struct DCP_FW_NAME(dcp_swap_submit_req) *swap = &DCP_FW_UNION(dcp->swap);

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	// clear surfaces
	memset(swap, 0, sizeof(*swap));

	swap->swap.swap_enabled =
		swap->swap.swap_completed = IOMFB_SET_BACKGROUND | 0x7;
	swap->swap.bg_color = 0xFF000000;

	/*
	 * Turn off the backlight. This matters because the DCP's idea of
	 * backlight brightness gets desynced after a power change, and it
	 * needs to be told it's going to turn off so it will consider the
	 * subsequent update on poweron an actual change and restore the
	 * brightness.
	 */
	if (dcp_has_panel(dcp)) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
		dcp_swap_set_brightness(&swap->swap, 0);
#else
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
		swap->swap.bl_update = 1;
#else
		swap->swap.bl_unk = 1;
#endif
		swap->swap.bl_value = 0;
		swap->swap.bl_power = 0;
#endif
	}

	/* Null all surfaces */
	for (int l = 0; l < SWAP_SURFACES; l++)
		swap->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	for (int l = 0; l < ARRAY_SIZE(swap->surf2_null); l++)
		swap->surf2_null[l] = true;
	swap->unkU32Ptr_null = true;
	swap->unkU32out_null = true;
#endif

	if (!dcp_swap_start(dcp, false, &swap_req, dcp_swap_clear_started, cookie))
		dcp_clear_refused(dcp, cookie);

	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(50));
	swap_id = cookie->swap_id;
	if (cookie->failed)
		ret = 0;
	kref_put(&cookie->refcount, release_swap_cookie);
	if (ret <= 0) {
		dcp->crashed = true;
		return;
	}

	dev_dbg(dcp->dev, "%s: clear swap submitted: %u\n", __func__, swap_id);

	poff_cookie = kzalloc(sizeof(*poff_cookie), GFP_KERNEL);
	if (!poff_cookie)
		return;
	init_completion(&poff_cookie->done);
	kref_init(&poff_cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&poff_cookie->refcount);

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	/* Native 26.6 ordinary hot-unplug goes directly from its clear
	 * transaction through A414/A412/A473.  A469 abort-swaps and A458
	 * last-client-close only appear in full shutdown/sleep traces, and may
	 * tear down state which the link power-off callbacks still require. */
	{
		const u8 prepare_state[8] = {
			0, 0, 0, 0, 0, 0xaa, 0xaa, 0xaa,
		};

		if (!dcp_push(dcp, false, &dcp_prepare_wake_method,
			 sizeof(prepare_state), sizeof(u64), (void *)prepare_state,
			 dcp_poweroff_after_prepare_state, poff_cookie))
			dcp_wait_refused(dcp, poff_cookie);
	}
#else
	if (!iomfb_abort_swaps_dcp(dcp, false, &abort_req,
				aborted_swaps_dcp_poff, poff_cookie))
		dcp_wait_refused(dcp, poff_cookie);
#endif
	ret = wait_for_completion_timeout(&poff_cookie->done,
					  msecs_to_jiffies(1000));

	if (poff_cookie->failed)
		dev_warn(dcp->dev, "setPowerState(0) publication refused\n");
	else if (ret == 0)
		dev_warn(dcp->dev, "setPowerState(0) timeout %u ms\n", 1000);
	else if (ret > 0)
		dev_dbg(dcp->dev,
			"setPowerState(0) finished with %d ms to spare",
			jiffies_to_msecs(ret));

	if (poff_cookie->failed) {
		kref_put(&poff_cookie->refcount, release_wait_cookie);
		return;
	}
	kref_put(&poff_cookie->refcount, release_wait_cookie);

	dev_info(dcp->dev, "dcp_poweroff() done\n");
}

static void last_client_closed_sleep(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_power_state_req_t power_req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_RESTARTABLE_SLEEP);
	if (!dcp_set_power_state(dcp, false, &power_req, complete_set_powerstate, cookie))
		dcp_wait_refused(dcp, cookie);
}

static void aborted_swaps_dcp_sleep(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct iomfb_last_client_close_req req = { 0 };
	if (!iomfb_last_client_close(dcp, false, &req, last_client_closed_sleep, cookie))
		dcp_wait_refused(dcp, cookie);
}

void DCP_FW_NAME(iomfb_sleep)(struct apple_dcp *dcp)
{
	int ret;
	dcp_abort_swaps_req_t req = dcp_abort_swaps_request(dcp);

	struct dcp_wait_cookie *cookie;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (!iomfb_abort_swaps_dcp(dcp, false, &req, aborted_swaps_dcp_sleep,
				cookie))
		dcp_wait_refused(dcp, cookie);
	ret = wait_for_completion_timeout(&cookie->done,
					  msecs_to_jiffies(1000));

	if (cookie->failed)
		dev_warn(dcp->dev, "setDCPPower(0) publication refused\n");
	else if (ret == 0)
		dev_warn(dcp->dev, "setDCPPower(0) timeout %u ms\n", 1000);

	if (cookie->failed) {
		kref_put(&cookie->refcount, release_wait_cookie);
		return;
	}
	kref_put(&cookie->refcount, release_wait_cookie);
	dev_info(dcp->dev, "dcp_sleep() done\n");
}

static void dcpep_cb_hotplug_state(struct apple_dcp *dcp, u64 connected)
{
	struct apple_connector *connector = dcp->connector;

	/* DCP issues hotplug_gated callbacks after SetPowerState() calls on
	 * devices with display (macbooks, imacs). This must not result in
	 * connector state changes on DRM side. Some applications won't enable
	 * a CRTC with a connector in disconnected state. Weston after DPMS off
	 * is one example. dcp_is_main_display() returns true on devices with
	 * integrated display. Ignore the hotplug_gated() callbacks there.
	 */
	if (dcp->main_display)
		return;

	if (dcp->during_modeset) {
		dev_info(dcp->dev,
			 "cb_hotplug() ignored during modeset connected:%llu\n",
			 connected);
		return;
	}

	dev_info(dcp->dev, "cb_hotplug() connected:%llu, valid_mode:%d\n",
		 connected, dcp->valid_mode);

	/* Hotplug invalidates mode. DRM doesn't always handle this. */
	if (!connected) {
		dcp->valid_mode = false;
		/* after unplug swap will not complete until the next
		 * set_digital_out_mode */
		schedule_work(&dcp->vblank_wq);
	}

	if (connector && connector->connected != !!connected) {
		connector->connected = !!connected;
		dcp->valid_mode = false;
		schedule_work(&connector->hotplug_wq);
	}
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
static struct dcp_hotplug_resp_v26_6
dcpep_cb_hotplug(struct apple_dcp *dcp, struct dcp_hotplug_req_v26_6 *req)
{
	struct dcp_hotplug_resp_v26_6 resp = { 0 };

	dcpep_cb_hotplug_state(dcp, req->connected);

	/* Every captured 26.6 connected response has an unaligned 0xffffffff
	 * field at offsets 71..74.  Byte 75 is padding and varies in native
	 * traces, so leave it initialized to zero.  The disconnected response
	 * contains additional state that is not decoded yet. */
	if (req->connected)
		memset(&resp.opaque[0x47], 0xff, sizeof(u32));

	return resp;
}
#else
static void dcpep_cb_hotplug(struct apple_dcp *dcp, u64 *connected)
{
	dcpep_cb_hotplug_state(dcp, *connected);
}
#endif

static void
dcpep_cb_swap_complete_intent_gated(struct apple_dcp *dcp,
				    dcp_swap_intent_t *info)
{
	trace_iomfb_swap_complete_intent_gated(dcp, info->swap_id,
		info->width, info->height);
	dcp->m4_intent_time = ktime_get();
}

static void
dcpep_cb_abort_swap_ap_gated(struct apple_dcp *dcp, u32 *swap_id)
{
	trace_iomfb_abort_swap_ap_gated(dcp, *swap_id);
}

static struct dcpep_get_tiling_state_resp
dcpep_cb_get_tiling_state(struct apple_dcp *dcp,
			  struct dcpep_get_tiling_state_req *req)
{
	return (struct dcpep_get_tiling_state_resp){
		.value = 0,
		.ret = 1,
	};
}

static u8 dcpep_cb_create_backlight_service(struct apple_dcp *dcp)
{
	return dcp_has_panel(dcp);
}

TRAMPOLINE_VOID(trampoline_nop, dcpep_cb_nop);
TRAMPOLINE_OUT(trampoline_true, dcpep_cb_true, u8);
TRAMPOLINE_OUT(trampoline_false, dcpep_cb_false, u8);
TRAMPOLINE_OUT(trampoline_zero, dcpep_cb_zero, u32);
TRAMPOLINE_IN(trampoline_swap_complete, dcpep_cb_swap_complete,
	      struct DCP_FW_NAME(dc_swap_complete_resp));
TRAMPOLINE_INOUT(trampoline_get_uint_prop, dcpep_cb_get_uint_prop,
		 struct dcp_get_uint_prop_req, struct dcp_get_uint_prop_resp);
TRAMPOLINE_IN(trampoline_set_fx_prop, iomfbep_cb_set_fx_prop,
	      struct iomfb_set_fx_prop_req)
TRAMPOLINE_INOUT(trampoline_map_piodma, dcpep_cb_map_piodma,
		 struct dcp_map_buf_req,
		 struct DCP_FW_NAME(dcp_map_buf_resp));
TRAMPOLINE_IN(trampoline_unmap_piodma, dcpep_cb_unmap_piodma,
	      struct dcp_unmap_buf_resp);
TRAMPOLINE_INOUT(trampoline_sr_set_property_int, iomfbep_cb_sr_set_property_int,
		 struct iomfb_sr_set_property_int_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_buffer, dcpep_cb_allocate_buffer,
		 struct dcp_allocate_buffer_req,
		 struct dcp_allocate_buffer_resp);
TRAMPOLINE_INOUT(trampoline_map_physical, dcpep_cb_map_physical,
		 struct dcp_map_physical_req, struct dcp_map_physical_resp);
TRAMPOLINE_INOUT(trampoline_release_mem_desc, dcpep_cb_release_mem_desc, u32,
		 u8);
TRAMPOLINE_INOUT(trampoline_map_reg, dcpep_cb_map_reg,
		 struct DCP_FW_NAME(dcp_map_reg_req),
		 struct DCP_FW_NAME(dcp_map_reg_resp));
TRAMPOLINE_INOUT(trampoline_read_edt_data, dcpep_cb_read_edt_data,
		 struct dcp_read_edt_data_req, dcp_read_edt_data_resp_t);
TRAMPOLINE_INOUT(trampoline_prop_start, dcpep_cb_prop_start, u32, u8);
TRAMPOLINE_INOUT(trampoline_prop_chunk, dcpep_cb_prop_chunk,
		 struct dcp_set_dcpav_prop_chunk_req, u8);
TRAMPOLINE_INOUT(trampoline_prop_end, dcpep_cb_prop_end,
		 struct dcp_set_dcpav_prop_end_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_bandwidth, dcpep_cb_allocate_bandwidth,
	       struct dcp_allocate_bandwidth_req, struct dcp_allocate_bandwidth_resp);
TRAMPOLINE_OUT(trampoline_rt_bandwidth, dcpep_cb_rt_bandwidth,
	       struct dcp_rt_bandwidth);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props,
		 dcpep_cb_set_frame_sync_props,
		 struct dcp_set_frame_sync_props_req_v26_6_0,
		 struct dcp_set_frame_sync_props_resp_v26_6_0);
#else
TRAMPOLINE_INOUT(trampoline_set_frame_sync_props, dcpep_cb_set_frame_sync_props,
		 struct dcp_set_frame_sync_props_req,
		 struct dcp_set_frame_sync_props_resp);
#endif
TRAMPOLINE_OUT(trampoline_get_frequency, dcpep_cb_get_frequency, u64);
TRAMPOLINE_OUT(trampoline_get_time, dcpep_cb_get_time, u64);
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
TRAMPOLINE_INOUT(trampoline_hotplug, dcpep_cb_hotplug,
		 struct dcp_hotplug_req_v26_6,
		 struct dcp_hotplug_resp_v26_6);
#else
TRAMPOLINE_IN(trampoline_hotplug, dcpep_cb_hotplug, u64);
#endif
TRAMPOLINE_IN(trampoline_swap_complete_intent_gated,
	      dcpep_cb_swap_complete_intent_gated,
	      dcp_swap_intent_t);
TRAMPOLINE_IN(trampoline_abort_swap_ap_gated, dcpep_cb_abort_swap_ap_gated, u32);
TRAMPOLINE_IN(trampoline_enable_backlight_message_ap_gated,
	      iomfbep_cb_enable_backlight_message_ap_gated, u8);
TRAMPOLINE_IN(trampoline_pr_publish, iomfb_cb_pr_publish,
	      struct iomfb_property);
TRAMPOLINE_INOUT(trampoline_get_tiling_state, dcpep_cb_get_tiling_state,
		 struct dcpep_get_tiling_state_req, struct dcpep_get_tiling_state_resp);
TRAMPOLINE_OUT(trampoline_create_backlight_service, dcpep_cb_create_backlight_service, u8);

/*
 * Callback for swap requests. If a swap failed, we'll never get a swap
 * complete event so we need to fake a vblank event early to avoid a hang.
 */

static void dcp_swap_refused(struct apple_dcp *dcp)
{
	dev_warn(dcp->dev, "swap publication refused\n");
	dev_err(dcp->dev, "M4_DCP_SWAP_REFUSED\n");
	/* No scanout was published; retain buffers and retry pending state. */
	dcp->brightness.update = true;
	dcp->surfaces_cleared = false;
	dcp_drm_crtc_vblank(dcp->crtc);
}

static void dcp_swapped(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_submit_resp) *resp = data;

	if (resp->ret) {
		dev_err(dcp->dev, "swap failed! status %u\n", resp->ret);
		dev_err(dcp->dev, "M4_DCP_SWAP_FAILED status=%u\n", resp->ret);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	if (dcp->m4_swaps_ok < 3) {
		dcp->m4_swaps_ok++;
		dev_info(dcp->dev, "M4_DCP_SWAP_SUBMIT_OK n=%u\n", dcp->m4_swaps_ok);
	}
	dcp->swap_start = ktime_get();

	while (!list_empty(&dcp->swapped_out_fbs)) {
		struct dcp_fb_reference *entry;
		entry = list_first_entry(&dcp->swapped_out_fbs,
					 struct dcp_fb_reference, head);
		if (entry->swap_id == dcp->last_swap_id)
			break;
		if (entry->fb)
			drm_framebuffer_put(entry->fb);
		list_del(&entry->head);
		kfree(entry);
	}
}

static void dcp_swap_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_resp) *resp = data;

	DCP_FW_UNION(dcp->swap).swap.swap_id = resp->swap_id;

	trace_iomfb_swap_submit(dcp, resp->swap_id);
	if (!dcp_swap_submit(dcp, false, &DCP_FW_UNION(dcp->swap), dcp_swapped, NULL))
		dcp_swap_refused(dcp);
}

/* Helpers to modeset and swap, used to flush */
static void do_swap(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct DCP_FW_NAME(dcp_swap_start_req) start_req =
		dcp_swap_start_request(dcp);

	if (dcp->connector && dcp->connector->connected) {
		if (!dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL))
			dcp_swap_refused(dcp);
	} else
		dcp_drm_crtc_vblank(dcp->crtc);
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
struct dcp_modeset_cookie {
	/* Must be first: release_wait_cookie() frees this allocation. */
	struct dcp_wait_cookie wait;
	struct dcp_set_digital_out_mode_req actual;
	struct dcp_set_digital_out_mode_req bootstrap;
	u32 result;
};
#endif

static void complete_set_digital_out_mode(struct apple_dcp *dcp, void *data,
					  void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		if (dcp->crashed)
			wait->failed = true;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
		((struct dcp_modeset_cookie *)cookie)->result =
			get_unaligned_le32(data);
#endif
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)

static bool dcp_get_mode_info(struct apple_dcp *dcp,
			      const struct dcp_set_digital_out_mode_req *mode,
			      dcp_callback_t cb, void *cookie)
{
	struct dcp_get_mode_info_req_v26_6 req;

	/* Match the initialized bytes in native 26.6 requests.  A364 fills the
	 * mode record before A413 consumes the selected timing/color pair. */
	memset(&req, 0xaa, sizeof(req));
	memset(req.mode, 0, sizeof(req.mode));
	req.mode_null = 0;
	req.timing_mode_id = mode->timing_mode_id;
	req.color_mode_id = mode->color_mode_id;
	req.result_null = 0;
	memset(req.padding1, 0, sizeof(req.padding1));

	return dcp_push(dcp, false, &dcp_get_mode_info_method, sizeof(req),
		 sizeof(struct dcp_get_mode_info_resp_v26_6), &req, cb, cookie);
}

static void dcp_modeset_after_actual_info(struct apple_dcp *dcp, void *data,
					   void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (dcp->crashed || !dcp_set_digital_out_mode(dcp, false, &modeset->actual,
				 complete_set_digital_out_mode, cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_modeset_after_bootstrap(struct apple_dcp *dcp, void *data,
					 void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (get_unaligned_le32(data)) {
		complete_set_digital_out_mode(dcp, data, cookie);
		return;
	}

	if (dcp->crashed || !dcp_get_mode_info(dcp, &modeset->actual,
			  dcp_modeset_after_actual_info, cookie))
		dcp_wait_refused(dcp, cookie);
}

static void dcp_modeset_after_bootstrap_info(struct apple_dcp *dcp, void *data,
					      void *cookie)
{
	struct dcp_modeset_cookie *modeset = cookie;

	if (dcp->crashed || !dcp_set_digital_out_mode(dcp, false, &modeset->bootstrap,
				 dcp_modeset_after_bootstrap, cookie))
		dcp_wait_refused(dcp, cookie);
}
#endif

int DCP_FW_NAME(iomfb_modeset)(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state)
{
	struct dcp_display_mode matched_mode;
	struct dcp_display_mode *mode = &matched_mode;
	unsigned int nr_modes;
	struct dcp_wait_cookie *cookie;
	struct dcp_color_mode *cmode = NULL;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	struct dcp_modeset_cookie *modeset_cookie;
#endif
	int ret;

	if (dcp->crashed)
		return -ENODEV;

	nr_modes = lookup_mode(dcp, &crtc_state->mode, mode);
	if (!nr_modes) {
		dev_err(dcp->dev, "no match for " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(&crtc_state->mode));
		return -EIO;
	}

	dev_info(dcp->dev,
		 "select_mode(color:%d timing:%d) " DRM_MODE_FMT "\n",
		 mode->color_mode_id, mode->timing_mode_id,
		 DRM_MODE_ARG(&crtc_state->mode));
	if (mode->color_mode_id == mode->sdr_rgb.id)
		cmode = &mode->sdr_rgb;
	else if (mode->color_mode_id == mode->sdr_444.id)
		cmode = &mode->sdr_444;
	else if (mode->color_mode_id == mode->sdr.id)
		cmode = &mode->sdr;
	else if (mode->color_mode_id == mode->best.id)
		cmode = &mode->best;
	if (cmode)
		dev_info(dcp->dev,
			"select_mode() color mode depth:%hhu format:%u "
			"colorimetry:%u eotf:%u range:%u vrr:%u\n", cmode->depth,
			cmode->format, cmode->colorimetry, cmode->eotf,
			cmode->range, mode->vrr);
	dev_info(dcp->dev, "M4_DCP_MODE_VRR timing:%u vrr_marked:%d min_vrr:%u max_vrr:%u\n",
		 mode->timing_mode_id, mode->vrr, mode->min_vrr, mode->max_vrr);

	dcp->mode = (struct dcp_set_digital_out_mode_req){
		.color_mode_id = mode->color_mode_id,
		.timing_mode_id = mode->timing_mode_id
	};

	/* Keep track of suspected vrr modes */
	dcp->use_timestamps = mode->vrr;
	/* M4 (R51): frame period for the flip_event_mode vblank clock; a new mode restarts the sequence epoch */
	if (crtc_state->mode.clock && crtc_state->mode.htotal && crtc_state->mode.vtotal)
		dcp->m4_frame_ns = div64_u64(1000000000ULL * crtc_state->mode.htotal * crtc_state->mode.vtotal,
					     crtc_state->mode.clock * 1000ULL);
	dcp->m4_vbl_epoch = 0;
	dcp->m4_vbl_seq = 0;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	/*
	 * Native fixed-panel mode changes use A407 source/destination geometry,
	 * not the external digital-output A364/A413 sequence. Accept only the
	 * single firmware-advertised non-VRR timing, already matched above.
	 */
	if (dcp_has_panel(dcp) && dcp->main_display &&
	    nr_modes == 1 && !mode->vrr) {
		dcp->valid_mode = true;
		return 0;
	}

	modeset_cookie = kzalloc(sizeof(*modeset_cookie), GFP_KERNEL);
	if (!modeset_cookie)
		return -ENOMEM;
	cookie = &modeset_cookie->wait;
	modeset_cookie->actual = dcp->mode;
#else
	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
#endif
	if (!cookie) {
		return -ENOMEM;
	}

	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	dcp->during_modeset = true;

#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	if (!dcp->valid_mode && mode->bootstrap.score >= 0) {
		modeset_cookie->bootstrap =
			(struct dcp_set_digital_out_mode_req) {
				.color_mode_id = mode->bootstrap.id,
				.timing_mode_id = mode->timing_mode_id,
			};
		dev_info(dcp->dev,
			 "bootstrapping generic pipe (color:%u timing:%u)\n",
			 modeset_cookie->bootstrap.color_mode_id,
			 modeset_cookie->bootstrap.timing_mode_id);
		if (dcp->crashed || !dcp_get_mode_info(dcp, &modeset_cookie->bootstrap,
				  dcp_modeset_after_bootstrap_info,
				  modeset_cookie))
			dcp_wait_refused(dcp, cookie);
	} else {
		if (dcp->crashed || !dcp_get_mode_info(dcp, &modeset_cookie->actual,
				  dcp_modeset_after_actual_info,
				  modeset_cookie))
			dcp_wait_refused(dcp, cookie);
	}
#else
	if (dcp->crashed || !dcp_set_digital_out_mode(dcp, false, &dcp->mode,
				 complete_set_digital_out_mode, cookie))
		dcp_wait_refused(dcp, cookie);
#endif

	/*
	 * The DCP firmware has an internal timeout of ~8 seconds for
	 * modesets. Add an extra 500ms to safe side that the modeset
	 * call has returned.
	 */
	ret = wait_for_completion_timeout(&cookie->done,
					  msecs_to_jiffies(8500));
	if (cookie->failed)
		ret = -EIO;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	if (ret > 0 && modeset_cookie->result) {
		dev_err(dcp->dev, "set_digital_out_mode returned 0x%08x\n",
			modeset_cookie->result);
		ret = -EIO;
	}
#endif
	if (ret <= 0)
		dcp->valid_mode = false;

	kref_put(&cookie->refcount, release_wait_cookie);
	dcp->during_modeset = false;
	dev_info(dcp->dev, "set_digital_out_mode finished:%d\n", ret);

	if (ret == 0) {
		dev_info(dcp->dev, "set_digital_out_mode timed out\n");
		return -EIO;
	} else if (ret < 0) {
		dev_info(dcp->dev,
			 "set_digital_out_mode failed:%d\n", ret);
		return -EIO;

	} else if (ret > 0) {
		dev_dbg(dcp->dev,
			"set_digital_out_mode finished with %d to spare\n",
			jiffies_to_msecs(ret));
	}
	dcp->valid_mode = true;

	return 0;
}

void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *new_state, *old_state;
	struct drm_crtc_state *crtc_state;
	struct DCP_FW_NAME(dcp_swap_submit_req) *req = &DCP_FW_UNION(dcp->swap);
	int plane_idx, l;
	int has_surface = 0;
	u32 m4_mask = 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	dcp->m4_commit_time = ktime_get();
	if (dcp->m4_flushes < 3) {
		dcp->m4_flushes++;
		dev_info(dcp->dev,
			 "M4_DCP_FLUSH n=%u valid_mode=%d connected=%d active=%d mode_changed=%d planes_changed=%d\n",
			 dcp->m4_flushes, dcp->valid_mode,
			 dcp->connector ? dcp->connector->connected : -1,
			 crtc_state ? crtc_state->active : -1,
			 crtc_state ? crtc_state->mode_changed : -1,
			 crtc_state ? crtc_state->planes_changed : -1);
	}

	/* Reset all surfaces to defaults */
	memset(req, 0, sizeof(*req));
	for (l = 0; l < SWAP_SURFACES; l++)
		req->surf_null[l] = true;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	for (l = 0; l < ARRAY_SIZE(req->surf2_null); l++)
		req->surf2_null[l] = true;
	req->unkU32Ptr_null = true;
	req->unkU32out_null = true;
#endif
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	req->swap.presents_surface = 1;
	req->swap.has_completion = 1;
#endif

	/*
	 * Clear all surfaces on startup. The boot framebuffer in surface 0
	 * sticks around.
	 */
	if (!dcp->surfaces_cleared) {
		req->swap.swap_enabled = IOMFB_SET_BACKGROUND | 0x7;
		req->swap.bg_color = 0xFF000000;
		dcp->surfaces_cleared = true;
	}

	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, plane_idx) {
		struct apple_plane_state *apple_state = to_apple_plane_state(new_state);
		struct apple_plane *apl_plane = to_apple_plane(plane);

		/* skip planes not for this crtc */
		if (old_state->crtc != crtc && new_state->crtc != crtc)
			continue;

		l = apl_plane->iomfb_surf;
		req->swap.swap_enabled |= BIT(l);

		if (old_state->fb && new_state->fb != old_state->fb) {
			/*
			 * Race condition between a framebuffer unbind getting
			 * swapped out and GEM unreferencing a framebuffer. If
			 * we lose the race, the display gets IOVA faults and
			 * the DCP crashes. We need to extend the lifetime of
			 * the drm_framebuffer (and hence the GEM object) until
			 * after we get a swap complete for the swap unbinding
			 * it.
			 */
			struct dcp_fb_reference *entry =
				kzalloc(sizeof(*entry), GFP_KERNEL);
			if (entry) {
				entry->fb = old_state->fb;
				entry->swap_id = dcp->last_swap_id;
				list_add_tail(&entry->head,
					      &dcp->swapped_out_fbs);
			}
			drm_framebuffer_get(old_state->fb);
		}

		if (!new_state->fb || !new_state->visible)
			continue;

		req->surf_null[l] = false;
		has_surface = 1;
		m4_mask |= BIT(l & 1);
		req->swap.src_rect[l] = apple_state->src_rect;
		req->swap.dst_rect[l] = apple_state->dst_rect;

		if (dcp->notch_height > 0)
			req->swap.dst_rect[l].y += dcp->notch_height;

		req->surf_iova[l] = apple_state->iova;
		req->surf[l].base = apple_state->surf;

		if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP &&
		    req->surf[l].base.colorspace == DCP_COLORSPACE_BG_SRGB)
			req->surf[l].base.colorspace = DCP_COLORSPACE_NATIVE;
	}

	dcp->m4_stats_mask[m4_mask & 3]++;

	if (!has_surface && !crtc_state->color_mgmt_changed) {
		if (crtc_state->enable && crtc_state->active &&
		    !crtc_state->planes_changed) {
			schedule_work(&dcp->vblank_wq);
			return;
		}

		/* Set black background */
		req->swap.swap_enabled |= IOMFB_SET_BACKGROUND;
		req->swap.bg_color = 0xFF000000;
		req->clear = 1;
	}

	if (has_surface && dcp->use_timestamps && dcp_swap_timestamps) {
		/*
		 * Fake timstamps to get 120hz refresh rate. It looks
		 * like the actual value does not matter, as long  as it is non zero.
		 */
		req->swap.ts1 = 120;
		req->swap.ts2 = 120;
		req->swap.ts3 = 120;
	}

	/* These fields should be set together */
	req->swap.swap_completed = req->swap.swap_enabled;

	/* update brightness if changed */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	/*
	 * M4: 26.6 publishes no initial nits, so nits is 0 until userspace writes
	 * the backlight. Do not push "0 nits" from the forced post-poweron update:
	 * it turns the panel off while the swap itself works. Leave the level
	 * iBoot set until a brightness was really requested.
	 */
	if (dcp_has_panel(dcp) && dcp->brightness.update &&
	    !dcp->brightness.requested) {
		dev_info_once(dcp->dev,
			      "M4_DCP_BRIGHTNESS_SKIP nits=%d requested=0 (panel keeps the firmware level)\n",
			      dcp->brightness.nits);
		dcp->brightness.update = false;
	}
#endif
	if (dcp_has_panel(dcp) && dcp->brightness.update) {
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
		dcp_swap_set_brightness(&req->swap, dcp->brightness.nits);
#else
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
		req->swap.bl_update = 1;
#else
		req->swap.bl_unk = 1;
#endif
		req->swap.bl_value = dcp->brightness.dac;
		req->swap.bl_power = 0x40;
#endif
		dcp->brightness.update = false;
	}

	if (crtc_state->color_mgmt_changed) {
		struct iomfb_set_matrix_req mat = {
			.location = 9,
		};

		if (crtc_state->ctm) {
			struct drm_color_ctm *ctm = (struct drm_color_ctm *)crtc_state->ctm->data;
			memcpy(mat.matrix, ctm->matrix, sizeof(mat.matrix));
		} else {
			mat.matrix[0] = mat.matrix[4] = mat.matrix[8] = 1LLU << 32;
		}

		if (!iomfb_set_matrix(dcp, false, &mat, do_swap, NULL))
			dcp_swap_refused(dcp);
	} else
		do_swap(dcp, NULL, NULL);
}

static void res_is_main_display(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct apple_connector *connector;
	int result = *(int *)out;

	if (dcp->crashed)
		return;
	dev_info(dcp->dev, "DCP is_main_display: %d\n", result);

	dcp->main_display = result != 0;

	connector = dcp->connector;
	if (connector) {
		connector->connected = READ_ONCE(dcp->nr_modes) > 0;
		schedule_work(&connector->hotplug_wq);
	}

	dcp->active = true;
#if DCP_FW_VER >= DCP_FW_VERSION(26, 6, 0)
	/* 26.6 no longer publishes the legacy initial-nits property (D300/15).
	 * Start with the requested value zero; userspace sets its desired nits.
	 *
	 * M4 (R37): the backlight device is registered by the DRM master after
	 * drm_dev_register() (dcp_backlight_takeover_registered()), not here: a
	 * handshake-only load has no CRTC and an aborted gate has no DRM device,
	 * and a write to the device would reach neither.
	 */
#endif
	complete(&dcp->start_done);
}

static void init_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (dcp->crashed)
		return;
	if (!dcp_is_main_display(dcp, false, res_is_main_display, NULL))
		dcp_start_refused(dcp);
}

static void init_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	if (dcp->crashed)
		return;
	if (!dcp_first_client_open(dcp, false, init_3, NULL))
		dcp_start_refused(dcp);
}

static void init_1(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 val = 0;
	if (dcp->crashed)
		return;
	if (!dcp_enable_disable_video_power_savings(dcp, false, &val, init_2, NULL))
		dcp_start_refused(dcp);
}

static void dcp_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct iomfb_get_color_remap_mode_req color_remap =
		(struct iomfb_get_color_remap_mode_req){
			.mode = 6,
		};

	if (dcp->crashed)
		return;
	dev_info(dcp->dev, "DCP booted\n");
	dev_info(dcp->dev, "M4_DCP_IOMFB_STARTED\n");

	if (!dcp_takeover) {
		/*
		 * Handshake only: start_signal was acknowledged.  Do not run
		 * the client-open / main-display init chain.
		 */
		complete(&dcp->start_done);
		return;
	}

	if (!iomfb_get_color_remap_mode(dcp, false, &color_remap, init_1, cookie))
		dcp_start_refused(dcp);
}

void DCP_FW_NAME(iomfb_shutdown)(struct apple_dcp *dcp)
{
	dcp_power_state_req_t req =
		dcp_power_state_req(0, DCP_POWER_TRANSITION_SHUTDOWN);

	if (!dcp_set_power_state(dcp, false, &req, NULL, NULL)) {
		dev_warn(dcp->dev, "shutdown request publication refused\n");
		dcp->crashed = true;
		complete(&dcp->start_done);
	}
}
