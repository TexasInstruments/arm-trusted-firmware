/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cdefs.h>

#include <lib/io.h>

#include <clk_fixed.h>

static uint32_t clk_fixed_get_freq(struct clk *clkp)
{
	const struct clk_data *clk_datap;
	const struct clk_range *range;

	clk_datap = clk_get_data(clkp);
	range = clk_get_range(clk_datap->range_idx);

	return range->min_hz;
}

static uint32_t clk_fixed_get_state(struct clk *clkp __unused)
{
	return CLK_HW_STATE_ENABLED;
}

const struct clk_drv clk_drv_fixed = {
	.get_freq = clk_fixed_get_freq,
	.get_state = clk_fixed_get_state,
};
