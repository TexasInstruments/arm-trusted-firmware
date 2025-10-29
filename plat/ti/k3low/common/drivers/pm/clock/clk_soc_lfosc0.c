/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cdefs.h>

#include <clk_soc_lfosc0.h>
#include <lib/io.h>

/*
 * LFXOSC_CTRL - Low Frequency Oscillator Control Register
 * Generic implementation using board-specific configuration from header
 */
static uint32_t clk_soc_lfosc0_get_freq(struct clk *clk_ptr __unused)
{
	uint32_t ret = 0U;
	uint32_t v;

	v = readl(LFOSC0_RTC_BASE + LFOSC0_LFXOSC_CTRL_OFFSET)
		& LFOSC0_32K_DISABLE_VAL;
	if (v == 0U) {
		ret = LFOSC0_FREQ_HZ;
	}

	return ret;
}

static uint32_t clk_soc_lfosc0_get_state(struct clk *clk_ptr __unused)
{
	return CLK_HW_STATE_ENABLED;
}

const struct clk_drv clk_drv_soc_lfosc0 = {
	.get_freq = clk_soc_lfosc0_get_freq,
	.get_state = clk_soc_lfosc0_get_state,
};
