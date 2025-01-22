/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <limits.h>

#include <common/debug.h>

#include <clk.h>
#include <clk_div.h>
#include <clk_mux.h>
#include <device.h>
#include <device_clk.h>
#include <device_prepare.h>
#include <clk_handler.h>
#include <hosts.h>
#include <types/errno.h>

int32_t set_clock_handler(uint32_t dev_id, uint32_t clk_id, bool enable)
{
	struct device *dev = NULL;
	bool gated = false;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	int32_t ret = SUCCESS;
	uint32_t flags = CLOCK_ALLOW_FREQ_CHANGE;
	uint8_t host_id = HOST_ID_TIFS;
	uint8_t state;

	if (enable) {
		state = CLOCK_SW_STATE_REQ;
	} else {
		state = CLOCK_SW_STATE_AUTO;
	}

	VERBOSE("SET_CLOCK: clk_id=%d dev_id=%d state=%d\n",
		clkidx, dev_id, state);

	ret = device_prepare_exclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		switch (state) {
		case CLOCK_SW_STATE_UNREQ:
			gated = true;
			break;
		case CLOCK_SW_STATE_AUTO:
		case CLOCK_SW_STATE_REQ:
			gated = false;
			break;
		default:
			VERBOSE("INVALID_STATE: state=%d\n", state);
			ret = -EINVAL;
			break;
		}
	}

	if (ret == SUCCESS) {
		if ((flags & CLOCK_ALLOW_SSC) != 0UL) {
			device_clk_set_ssc(dev, clkidx, true);
		} else {
			device_clk_set_ssc(dev, clkidx, false);
		}

		if ((flags & CLOCK_ALLOW_FREQ_CHANGE) != 0UL) {
			device_clk_set_freq_change(dev, clkidx, true);
		} else {
			device_clk_set_freq_change(dev, clkidx, false);
		}

		if ((flags & CLOCK_INPUT_TERM) != 0UL) {
			device_clk_set_input_term(dev, clkidx, true);
		} else {
			device_clk_set_input_term(dev, clkidx, false);
		}

		if (!device_clk_set_gated(dev, clkidx, gated)) {
			ret = -EINVAL;
		}
	}

	return ret;
}

int32_t get_clock_handler(uint32_t dev_id, uint32_t clk_id)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("GET_CLOCK: clk_id=%d dev_id=%d\n", clkidx, dev_id);

	ret = device_prepare_nonexclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		uint8_t prog;

		prog = (uint8_t) (device_clk_get_sw_gated(dev, clkidx) ?
				  CLOCK_SW_STATE_UNREQ :
				  CLOCK_SW_STATE_AUTO);

		return (int32_t)prog;
	}

	return 0;
}

int32_t set_clock_parent_handler(uint32_t dev_id, uint32_t clk_id, uint32_t parent_id)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	dev_clk_idx_t parent = (dev_clk_idx_t) parent_id;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("SET_CLOCK_PARENT: clk_id=%d dev_id=%d parent=%d\n",
		clkidx, dev_id, parent);

	ret = device_prepare_exclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		if (!device_clk_set_parent(dev, clkidx, parent)) {
			ret = -EINVAL;
		}
	}

	return ret;
}

int32_t get_clock_parent_handler(uint32_t dev_id, uint32_t clk_id, uint32_t *parent_id)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("GET_CLOCK_PARENT: clk_id=%d dev_id=%d\n",
		clkidx, dev_id);

	ret = device_prepare_nonexclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		dev_clk_idx_t parent;

		parent = device_clk_get_parent(dev, clkidx);

		if (parent == DEV_CLK_ID_NONE) {
			ret = -EINVAL;
		} else {
			*parent_id = (uint32_t) parent;
		}
	}

	return ret;
}

int32_t get_num_clock_parents_handler(uint32_t dev_id, uint32_t clk_id)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("GET_NUM_CLOCK_PARENTS: clk_id=%d dev_id=%d\n",
		clkidx, dev_id);

	ret = device_prepare_nonexclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		dev_clk_idx_t num_parents;

		num_parents = device_clk_get_num_parents(dev, clkidx);

		if (num_parents == DEV_CLK_ID_NONE) {
			return 0;
		} else {
			return (int32_t) num_parents;
		}
	}

	return 0;
}

int32_t set_freq_handler(uint32_t dev_id, uint32_t clk_id, uint64_t target_freq)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	uint64_t min_freq_hz = target_freq / 10U * 9U;
	uint64_t max_freq_hz = target_freq / 10U * 11U;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("SET_FREQ: clk_id=%d dev_id=%d\n", clkidx, dev_id);

	ret = device_prepare_exclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		if ((min_freq_hz > target_freq)
		    || (target_freq > max_freq_hz)) {
			ret = -EINVAL;
		}
	}

	if (ret == SUCCESS) {
		if (!device_clk_set_freq(dev, clkidx, (uint32_t) min_freq_hz,
					 (uint32_t) target_freq,
					 (uint32_t) max_freq_hz)) {
			ret = -EINVAL;
		}
	}

	return ret;
}

uint64_t get_freq_handler(uint32_t dev_id, uint32_t clk_id)
{
	struct device *dev = NULL;
	dev_clk_idx_t clkidx = (dev_clk_idx_t) clk_id;
	int32_t ret = SUCCESS;
	uint8_t host_id = HOST_ID_TIFS;

	VERBOSE("GET_FREQ: clk_id=%d dev_id=%d\n", clkidx, dev_id);

	ret = device_prepare_nonexclusive(host_id, dev_id, NULL, &dev);
	if (ret == SUCCESS) {
		return device_clk_get_freq(dev, clkidx);
	}

	return 0U;
}
