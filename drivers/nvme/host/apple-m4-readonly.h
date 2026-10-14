/* SPDX-License-Identifier: GPL-2.0-only */
/* Qualification-only command policy. No runtime switch to enable writes. */
static inline bool apple_m4_readonly_allowed(bool admin, unsigned int opcode, unsigned int feature)
{
	if (!admin)
		return opcode == nvme_cmd_read;
	/* Only volatile, standard host/runtime setup; the Save bit is refused. */
	if (opcode == nvme_admin_set_features)
		return feature == NVME_FEAT_ASYNC_EVENT ||
		       feature == NVME_FEAT_AUTO_PST ||
		       feature == NVME_FEAT_TIMESTAMP ||
		       feature == NVME_FEAT_HOST_BEHAVIOR;
	switch (opcode) {
	case nvme_admin_delete_sq:
	case nvme_admin_create_sq:
	case nvme_admin_get_log_page:
	case nvme_admin_delete_cq:
	case nvme_admin_create_cq:
	case nvme_admin_identify:
	case nvme_admin_abort_cmd:
	case nvme_admin_get_features:
	case nvme_admin_async_event:
		return true;
	default:
		return false;
	}
}
