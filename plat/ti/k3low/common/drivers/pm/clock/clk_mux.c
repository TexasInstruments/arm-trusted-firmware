/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <lib/container_of.h>
#include <lib/io.h>
#include <lib/trace.h>

#include <clk_mux.h>

static uint32_t clk_mux_get_parent_value(struct clk *clkp)
{
	const struct clk_data *clk_datap = clk_get_data(clkp);
	const struct clk_data_mux *mux;
	const struct clk_data_mux_reg *reg;
	uint32_t v;
	uint32_t n_minus_one;

	mux = container_of(clk_datap->data, const struct clk_data_mux, data);
	reg = container_of(mux, const struct clk_data_mux_reg, data_mux);

	/*
	 * Hack, temporarily return parent 0 for muxes without register
	 * assignments.
	 */
	if (reg->reg == 0U) {
		v = 0U;
	} else {
		v = readl(reg->reg);
		v >>= (uint32_t) reg->bit;

		n_minus_one = mux->n - 1U;
		v &= MASK_COVER_FOR_NUMBER(n_minus_one);
	}

	return v;
}

static const struct clk_parent *clk_mux_get_parent(struct clk *clkp)
{
	const struct clk_data *clk_datap = clk_get_data(clkp);
	const struct clk_data_mux *mux;
	uint32_t v;
	bool valid_parent;
	bool valid_div;

	mux = container_of(clk_datap->data, const struct clk_data_mux, data);
	v = clk_mux_get_parent_value(clkp);

	valid_parent = (v < mux->n);
	valid_div = (mux->parents[v].div != 0U);

	return (valid_parent && valid_div) ? &mux->parents[v] : NULL;
}

static bool clk_mux_set_parent(struct clk *clkp, uint8_t new_parent)
{
	const struct clk_data *clk_datap = clk_get_data(clkp);
	const struct clk_data_mux *mux;
	const struct clk_data_mux_reg *reg;
	uint32_t v;
	uint32_t parent_val;
	uint32_t mask;
	uint32_t inverted_mask;
	uint32_t trace_val;
	uint32_t trace_id;
	uint32_t n_minus_one;
	uint32_t reg_bit;
	bool ret = true;

	mux = container_of(clk_datap->data, const struct clk_data_mux, data);
	reg = container_of(mux, const struct clk_data_mux_reg, data_mux);

	if (reg->reg == 0U) {
		/*
		 * Hack, temporarily ignore assignments for muxes without
		 * register assignments.
		 */
	} else {
		v = readl(reg->reg);
		n_minus_one = mux->n - 1U;
		reg_bit = (uint32_t)reg->bit;
		mask = (MASK_COVER_FOR_NUMBER(n_minus_one) << reg_bit);
		inverted_mask = ~mask;
		v &= inverted_mask;
		parent_val = (uint32_t)new_parent << reg_bit;
		v |= parent_val;
		writel(v, reg->reg);

		trace_val = ((uint32_t)new_parent << TRACE_PM_VAL_CLOCK_VAL_SHIFT) &
			    TRACE_PM_VAL_CLOCK_VAL_MASK;
		trace_id = (clk_id(clkp) << TRACE_PM_VAL_CLOCK_ID_SHIFT) &
			   TRACE_PM_VAL_CLOCK_ID_MASK;

		pm_trace(TRACE_PM_ACTION_CLOCK_SET_PARENT, trace_val | trace_id);
	}

	return ret;
}

const struct clk_drv_mux clk_drv_mux_reg_ro = {
	.get_parent = clk_mux_get_parent,
};

const struct clk_drv_mux clk_drv_mux_reg = {
	.set_parent = clk_mux_set_parent,
	.get_parent = clk_mux_get_parent,
};

const struct clk_parent *clk_get_parent(struct clk *clkp)
{
	const struct clk_data *clk_datap = clk_get_data(clkp);
	const struct clk_parent *ret = NULL;
	bool is_mux_type;

	is_mux_type = (clk_datap->type == CLK_TYPE_MUX);

	if (is_mux_type) {
		const struct clk_drv_mux *mux;

		mux = container_of(clk_datap->drv, const struct clk_drv_mux, drv);
		ret = mux->get_parent(clkp);
	} else {
		bool valid_div = (clk_datap->parent.div > 0U);

		ret = valid_div ? &clk_datap->parent : NULL;
	}

	return ret;
}

/* FIXME: freq change ok/notify? new freq in range? */
bool clk_set_parent(struct clk *clkp, uint8_t new_parent)
{
	const struct clk_data *clk_datap = clk_get_data(clkp);
	const struct clk_drv_mux *mux_drv = NULL;
	const struct clk_data_mux *mux_data = NULL;
	const struct clk_parent *op;
	struct clk *parent;
	bool ret = true;
	bool done = false;

	if (clk_datap->type != CLK_TYPE_MUX) {
		ret = false;
		done = true;
	}

	if (!done) {
		mux_data = container_of(clk_datap->data,
					const struct clk_data_mux, data);
		if (new_parent >= mux_data->n) {
			ret = false;
			done = true;
		} else if (mux_data->parents[new_parent].div == 0U) {
			ret = false;
			done = true;
		} else {
			/* Do Nothing */
		}
	}

	if (!done) {
		mux_drv = container_of(clk_datap->drv,
				       const struct clk_drv_mux, drv);
		if (!mux_drv->set_parent) {
			ret = false;
			done = true;
		}
	}

	if (!done) {
		op = mux_drv->get_parent(clkp);
		if (op && (op->clk == mux_data->parents[new_parent].clk)
		    && (op->div == mux_data->parents[new_parent].div)) {
			ret = true;
			done = true;
		}
	}

	if (!done) {
		parent = clk_lookup((clk_idx_t) mux_data->parents[new_parent].clk);
		if (!parent) {
			ret = false;
			done = true;
		} else if ((clkp->flags & CLK_FLAG_INITIALIZED) == 0U) {
			ret = false;
			done = true;
		} else if (clkp->ref_count == 0U) {
			/* No get necessary */
		} else if (!clk_get(parent)) {
			ret = false;
			done = true;
		} else {
			/* Do Nothing */
		}
	}

	if (!done) {
		if (!mux_drv->set_parent(clkp, new_parent)) {
			if (clkp->ref_count != 0U) {
				clk_put(parent);
			}
			ret = false;
			done = true;
		}
	}

	if (!done) {
		if (op && (clkp->ref_count != 0U)) {
			struct clk *op_parent;

			op_parent = clk_lookup((clk_idx_t) op->clk);
			if (op_parent != NULL) {
				clk_put(op_parent);
			}
		}
	}

	return ret;
}
