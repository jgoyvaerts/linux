/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_APPLE_IDLE_H
#define __ASM_APPLE_IDLE_H

/* WFI that preserves x18-x30 on affected Apple cores (T6040, T8140, T8142 and T8152). */
void apple_cpu_wfi(void);

#ifdef CONFIG_ARCH_APPLE
/* Called from smp_prepare_boot_cpu(): before the boot CPU's first idle. */
void apple_wfi_quirk_init(void);
#else
static inline void apple_wfi_quirk_init(void) { }
#endif

#endif
