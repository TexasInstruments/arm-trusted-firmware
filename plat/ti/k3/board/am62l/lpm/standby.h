/*
 * Copyright (c) 2026, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LPM_STANDBY_H__
#define __LPM_STANDBY_H__

#include <plat/common/platform.h>

/* power domain indices */
#define PD_MPU_CLST		4
#define PD_MPU_CLST_CORE_0	5
#define PD_MPU_CLST_CORE_1	6

/* lpsc indices */
#define LPSC_MAIN_MPU_CLST		38
#define LPSC_MAIN_MPU_CLST_PBIST	39
#define LPSC_MAIN_MPU_CLST_CORE_0	40
#define LPSC_MAIN_MPU_CLST_CORE_1	41

#define PSC_SYNCRESETDISABLE	(0x0)
#define PSC_SYNCRESET		(0x1)
#define PSC_DISABLE		(0x2)
#define PSC_ENABLE		(0x3)
#define PSC_PD_OFF		(0x0)
#define PSC_PD_ON		(0x1)

// Standby idle states
#define CORE_IDLE_STATE 0x1
#define LOW_LATENCY_IDLE_STATE 0x2
#define HIGH_LATENCY_IDLE_STATE 0x3

void am62l_save_state(void);
void am62l_restore_state(void);
void am62l_low_latency_standby(void);
void set_main_psc_state(uint32_t, uint32_t, uint32_t, uint32_t);

#endif /* __LPM_STANDBY_H__ */
