/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SOC_CLK_AM6_H
#define SOC_CLK_AM6_H

#include <cdefs.h>

#include <clk.h>

/* Board-specific configuration */
#define HFOSC0_WKUP_CTRL_MMR_BASE    0x43010000UL
#define HFOSC0_MAIN_DEVSTAT_OFFSET   0x30UL
#define HFOSC0_MAIN_BOOTCFG_OFFSET   0x34UL
#define HFOSC0_DEVSTAT_FREQ_MASK     0x7U

/* AM62LX supported HFOSC0 frequencies */
#define HFOSC0_FREQ_0	FREQ_MHZ(24.0)
#define HFOSC0_FREQ_1	FREQ_MHZ(25.0)
#define HFOSC0_FREQ_2	FREQ_MHZ(26.0)

extern const struct clk_drv clk_drv_soc_hfosc0;

#endif
