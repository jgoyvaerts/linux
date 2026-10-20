/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_APPLE_IDLE_H
#define __ASM_APPLE_IDLE_H

/* WFI that preserves x18-x30 on affected Apple cores (T6040, T8140, T8142 and T8152). */
void apple_cpu_wfi(void);

#endif
