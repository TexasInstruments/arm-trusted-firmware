/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cdefs.h>

#include <drivers/delay_timer.h>

#include <clk_mux.h>
#include <clk_pll.h>
#include <clk_pll_deskew.h>
#include <device.h>
#include <lib/container_of.h>
#include <lib/io.h>

static inline uint32_t PLL_DESKEW_PID(uint32_t idx)
{
	return (0x1000UL * idx) + 0x00UL;
}

static inline uint32_t PLL_DESKEW_CFG(uint32_t idx)
{
	return (0x1000UL * idx) + 0x08UL;
}

static inline uint32_t PLL_DESKEW_LOCKKEY0(uint32_t idx)
{
	return (0x1000UL * idx) + 0x10UL;
}

#define PLL_DESKEW_LOCKKEY0_VALUE		0x68EF3490UL

static inline uint32_t PLL_DESKEW_LOCKKEY1(uint32_t idx)
{
	return (0x1000UL * idx) + 0x14UL;
}

#define PLL_DESKEW_LOCKKEY1_VALUE		0xD172BC5AUL

static inline uint32_t PLL_DESKEW_CTRL(uint32_t idx)
{
	return (0x1000UL * idx) + 0x20UL;
}
#define PLL_DESKEW_CTRL_BYPASS_EN		BIT(31)
#define PLL_DESKEW_CTRL_BYP_ON_LOCKLOSS		BIT(16)
#define PLL_DESKEW_CTRL_INTL_BYP_EN		BIT(8)
#define PLL_DESKEW_CTRL_PD_EN			BIT(4)
#define PLL_DESKEW_CTRL_VCO_SEL			BIT(0)

static inline uint32_t PLL_DESKEW_STAT(uint32_t idx)
{
	return (0x1000UL * idx) + 0x24UL;
}

#define PLL_DESKEW_STAT_LOCK			BIT(0)

static inline uint32_t PLL_DESKEW_DIV_CTRL(uint32_t idx)
{
	return (0x1000UL * idx) + 0x38UL;
}
#define PLL_DESKEW_DIV_CTRL_FB_DIV_SHIFT	12UL
#define PLL_DESKEW_DIV_CTRL_FB_DIV_MASK		(0x3UL << 12UL)
#define PLL_DESKEW_DIV_CTRL_POST_DIV_SHIFT	8UL
#define PLL_DESKEW_DIV_CTRL_POST_DIV_MASK	(0x7UL << 8UL)
#define PLL_DESKEW_DIV_CTRL_REF_DIV_SHIFT	0UL
#define PLL_DESKEW_DIV_CTRL_REF_DIV_MASK	(0x3UL << 0UL)

static inline uint32_t PLL_DESKEW_TEST_CTRL(uint32_t idx)
{
	return (0x1000UL * idx) + 0x50UL;
}

static inline uint32_t PLL_DESKEW_CAL_CTRL(uint32_t idx)
{
	return (0x1000UL * idx) + 0x60UL;
}

static inline uint32_t PLL_DESKEW_CAL_STAT(uint32_t idx)
{
	return (0x1000UL * idx) + 0x64UL;
}

static inline uint32_t PLL_DESKEW_HSDIV_CTRL(uint32_t idx, uint32_t n)
{
	return (0x1000UL * idx) + 0x80UL + (n * 4UL);
}
#define PLL_DESKEW_HSDIV_CTRL_RESET		BIT(31)
#define PLL_DESKEW_HSDIV_CTRL_CLKOUT_EN		BIT(15)
#define PLL_DESKEW_HSDIV_CTRL_SYNC_DIS		BIT(8)
#define PLL_DESKEW_HSDIV_CTRL_HSDIV_SHIFT	0UL
#define PLL_DESKEW_HSDIV_CTRL_HSDIV_MASK	(0x7fUL << 0UL)

/*
 * \brief Check if the pllm value is valid
 *
 * \param clk The PLL clock.
 * \param pllm The multiplier value
 *
 * \return true if pllm value is valid, false otherwise
 */
static bool pll_deskew_pllm_valid(struct clk *clock_ptr __unused,
				  uint32_t pllm,
				  bool is_frac __unused)
{
	return (pllm == 1U) || (pllm == 2U) || (pllm == 4U);
}

static bool pll_deskew_plld_valid(struct clk *clock_ptr __unused, uint32_t plld)
{
	return (plld == 1U) || (plld == 2U) || (plld == 4U);
}

static bool pll_deskew_clkod_valid(struct clk *clock_ptr __unused,
				   uint32_t clkod)
{
	return (clkod == 4U) || (clkod == 8U) || (clkod == 16U) ||
		(clkod == 32U) || (clkod == 64U) || (clkod == 128U) ||
		(clkod == 256U) || (clkod == 512U);
}

/* Binning not yet supported/needed */
static int32_t pll_deskew_bin(struct clk *clock_ptr __unused,
			      uint32_t plld __unused, uint32_t pllm __unused,
			      bool is_frac __unused, uint32_t clkod __unused)
{
	return 0;
}

/* Prefer higher VCO frequencies */
static uint32_t pll_deskew_vco_fitness(struct clk *clock_ptr __unused,
				       uint32_t vco, bool is_frac __unused)
{
	return vco;
}

static const struct pll_data pll_deskew_data = {
	.plld_max	= 4U,
	.pllm_max	= 4U,
	.pllfm_bits	= 0U,
	.clkod_max	= 512U,
	.pllm_valid	= pll_deskew_pllm_valid,
	.plld_valid	= pll_deskew_plld_valid,
	.clkod_valid	= pll_deskew_clkod_valid,
	.bin		= pll_deskew_bin,
	.vco_fitness	= pll_deskew_vco_fitness,
};

/*
 * \brief Check if the PLL VCO/DCO is locked.
 *
 * \param clk The PLL clock.
 *
 * \return true if VCO/DCO is locked, false otherwise
 */
static bool clk_pll_deskew_check_lock(struct clk *clock_ptr)
{
	const struct clk_data *clock_data;
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	uint32_t stat;

	clock_data = clk_get_data(clock_ptr);
	data_pll = container_of(clock_data->data, const struct clk_data_pll,
				data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew,
			   data_pll);

	stat = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_STAT(pll->idx));
	return (stat & PLL_DESKEW_STAT_LOCK) != 0U;
}

static bool clk_pll_deskew_wait_for_lock(struct clk *clock_ptr)
{
	uint64_t i;

	/*
	 * Minimum VCO input freq is 2.08MHz, and the longest a lock should
	 * be consider to be timed out after 120 cycles. That's 58.69us. Be
	 * conservative and assume each loop takes 10 cycles and we run at a
	 * max of 1GHz. That gives 5869 loop cycles. We may end up waiting
	 * longer than necessary for timeout, but that should be ok.
	 */
	for (i = 0UL; i < 5869UL; i++) {
		if (clk_pll_deskew_check_lock(clock_ptr)) {
			break;
		}
	}

	return (i != 5869UL);
}

/*
 * \brief Query bypass state of PLL
 *
 * \param clk The PLL clock.
 *
 * \return true if PLL is in bypass.
 */
static bool clk_pll_deskew_is_bypass(struct clk *clock_ptr)
{
	const struct clk_data *clock_data;
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	bool ret = false;
	uint32_t ctrl;

	clock_data = clk_get_data(clock_ptr);
	data_pll = container_of(clock_data->data, const struct clk_data_pll,
				data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew,
			   data_pll);

	/* IDLE Bypass */
	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	ret = (ctrl & PLL_DESKEW_CTRL_BYPASS_EN) != 0U;

	return ret;
}

static int32_t clk_pll_deskew_bypass(struct clk *clock_ptr, bool bypass)
{
	const struct clk_data *clock_data;
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	uint32_t ctrl;

	clock_data = clk_get_data(clock_ptr);
	data_pll = container_of(clock_data->data, const struct clk_data_pll,
				data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew,
			   data_pll);

	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	if (bypass) {
		/* Enable bypass */
		ctrl |= (uint32_t)PLL_DESKEW_CTRL_BYPASS_EN;
	} else {
		/* Disable bypass */
		ctrl &= (uint32_t)~PLL_DESKEW_CTRL_BYPASS_EN;
	}
	writel(ctrl, (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	return SUCCESS;
}

static bool clk_pll_deskew_program_freq(struct clk *clock_ptr,
					const struct clk_data_pll_deskew *pll,
					uint32_t plld,
					uint32_t pllm,
					uint32_t clkod)
{
	bool ret = true;
	uint64_t div_ctrl;
	uint32_t ctrl;
	uint64_t i;
	uint64_t pllm_val_p = (uint64_t) pllm;
	uint64_t plld_val_p = (uint64_t) plld;
	uint64_t clkod_val_p = (uint64_t) clkod;

	clock_ptr->flags &= (uint8_t) ~CLK_FLAG_CACHED;

	if (!clk_pll_deskew_is_bypass(clock_ptr)) {
		/* Put the PLL into bypass */
		(void)clk_pll_deskew_bypass(clock_ptr, true);
	}

	/* Program the new rate */
	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	div_ctrl = (uint64_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_DIV_CTRL(pll->idx));

	div_ctrl &= ~PLL_DESKEW_DIV_CTRL_FB_DIV_MASK;
	switch (pllm_val_p) {
	case 1UL:
		pllm_val_p = 2UL;
		break;
	case 2UL:
		pllm_val_p = 1UL;
		break;
	case 4UL:
		pllm_val_p = 0UL;
		break;
	default:
		/* Do Nothing */
		break;
	}
	div_ctrl |= pllm_val_p << PLL_DESKEW_DIV_CTRL_FB_DIV_SHIFT;

	div_ctrl &= ~PLL_DESKEW_DIV_CTRL_REF_DIV_MASK;
	switch (plld_val_p) {
	case 1UL:
		plld_val_p = 2UL;
		break;
	case 2UL:
		plld_val_p = 1UL;
		break;
	case 4UL:
		plld_val_p = 0UL;
		break;
	default:
		/* Do Nothing */
		break;
	}
	div_ctrl |= plld_val_p << PLL_DESKEW_DIV_CTRL_REF_DIV_SHIFT;

	/* Start with vco output divider */
	if (clkod_val_p > 2UL) {
		clkod_val_p /= 2UL;
		ctrl &= (uint32_t)~PLL_DESKEW_CTRL_VCO_SEL;
	} else {
		ctrl |= (uint32_t)PLL_DESKEW_CTRL_VCO_SEL;
	}

	/* Find the log2 for clkod setting */
	div_ctrl &= ~PLL_DESKEW_DIV_CTRL_POST_DIV_MASK;
	for (i = 0UL; (((uint64_t) (2UL << i) & clkod_val_p) == 0UL); i++) {
		/* No loop action needed */
	}
	div_ctrl |= i << PLL_DESKEW_DIV_CTRL_POST_DIV_SHIFT;

	if (ret) {
		writel(ctrl, (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	}

	writel((uint32_t) div_ctrl,
	       (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_DIV_CTRL(pll->idx));

	if (ret && (clock_ptr->ref_count != 0U)) {
		/* Take the PLL out of bypass */
		ret = clk_pll_deskew_wait_for_lock(clock_ptr);
		if (ret) {
			(void)clk_pll_deskew_bypass(clock_ptr, false);
		}
	}

	return ret;
}

static uint32_t clk_pll_deskew_set_freq(struct clk *clock_ptr,
					uint32_t target_hz,
					uint32_t min_hz,
					uint32_t max_hz,
					bool query, bool *changed)
{
	const struct clk_data *pll_clk_data;
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	uint32_t ctrl;
	uint32_t div_ctrl;
	uint64_t prev_pllm;
	uint64_t prev_plld;
	uint64_t prev_clkod;
	uint32_t pllm = 0U;
	uint32_t pllfm = 0U;
	uint32_t plld = 0U;
	uint32_t clkod = 0U;
	uint32_t freq;
	uint32_t parent_freq_hz;
	bool was_bypass;

	was_bypass = clk_pll_deskew_is_bypass(clock_ptr);

	pll_clk_data = clk_get_data(clock_ptr);
	data_pll = container_of(pll_clk_data->data, const struct clk_data_pll,
				data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew,
			   data_pll);

	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	div_ctrl = (uint64_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_DIV_CTRL(pll->idx));

	/* Check current values */
	prev_pllm = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_FB_DIV_MASK;
	prev_pllm >>= PLL_DESKEW_DIV_CTRL_FB_DIV_SHIFT;
	switch (prev_pllm) {
	case 0UL:
		prev_pllm = 4UL;
		break;
	case 1UL:
		prev_pllm = 2UL;
		break;
	default:
		prev_pllm = 1UL;
		break;
	}

	prev_plld = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_REF_DIV_MASK;
	prev_plld >>= PLL_DESKEW_DIV_CTRL_REF_DIV_SHIFT;
	switch (prev_plld) {
	case 0UL:
		prev_plld = 4UL;
		break;
	case 1UL:
		prev_plld = 2UL;
		break;
	default:
		prev_plld = 1UL;
		break;
	}

	prev_clkod = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_POST_DIV_MASK;
	prev_clkod >>= PLL_DESKEW_DIV_CTRL_POST_DIV_SHIFT;
	prev_clkod = (uint64_t) 2UL << prev_clkod;
	if ((ctrl & PLL_DESKEW_CTRL_VCO_SEL) == 0UL) {
		prev_clkod *= 2UL;
	}

	/* Compute new values */
	parent_freq_hz = clk_get_parent_freq(clock_ptr);
	freq = pll_calc(clock_ptr, &pll_deskew_data, parent_freq_hz,
			target_hz, min_hz, max_hz,
			&plld, &pllm, &pllfm, &clkod);

	*changed = true;
	/* No need to do anything if they haven't changed */
	if ((plld == (uint32_t) prev_plld) &&
	    (pllm == (uint32_t) prev_pllm) &&
	    (clkod == (uint32_t) prev_clkod) &&
	    !was_bypass) {
		*changed = false;
	}

	if ((freq != 0U) && !query && *changed) {
		if (!clk_pll_deskew_program_freq(clock_ptr, pll, plld, pllm, clkod)) {
			freq = 0U;
		}
	}

	return freq;
}

static uint32_t clk_pll_deskew_get_freq_internal(struct clk *clock_ptr)
{
	const struct clk_data *clock_data = clk_get_data(clock_ptr);
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	uint32_t ret = 0U;
	uint32_t ctrl;
	uint32_t div_ctrl;
	uint64_t pllm;
	uint64_t plld;
	uint64_t clkod;
	uint64_t clkod_plld;

	data_pll = container_of(clock_data->data, const struct clk_data_pll, data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew, data_pll);
	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
	div_ctrl = (uint64_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_DIV_CTRL(pll->idx));

	pllm = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_FB_DIV_MASK;
	pllm >>= PLL_DESKEW_DIV_CTRL_FB_DIV_SHIFT;
	switch (pllm) {
	case 0UL:
		pllm = 4UL;
		break;
	case 1UL:
		pllm = 2UL;
		break;
	default:
		pllm = 1UL;
		break;
	}

	plld = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_REF_DIV_MASK;
	plld >>= PLL_DESKEW_DIV_CTRL_REF_DIV_SHIFT;
	switch (plld) {
	case 0UL:
		plld = 4UL;
		break;
	case 1UL:
		plld = 2UL;
		break;
	default:
		plld = 1UL;
		break;
	}

	clkod = (uint64_t) div_ctrl & PLL_DESKEW_DIV_CTRL_POST_DIV_MASK;
	clkod >>= PLL_DESKEW_DIV_CTRL_POST_DIV_SHIFT;
	clkod = (uint64_t) 2UL << clkod;
	if ((ctrl & PLL_DESKEW_CTRL_VCO_SEL) == 0U) {
		clkod *= 2UL;
	}

	clkod_plld = clkod * plld;

	if (clkod_plld != 0UL) {
		uint64_t parent_freq_hz;
		uint64_t ret64;
		uint64_t rem;

		/* Calculate non-fractional part */
		parent_freq_hz = (uint64_t) clk_get_parent_freq(clock_ptr);
		ret64 = (parent_freq_hz / clkod_plld) * pllm;
		rem = (parent_freq_hz % clkod_plld) * pllm;
		/* Remainder should always fit within 32 bits based on bit analysis */
		ret64 += rem / clkod_plld;

		/* Based on calculation constraints, overflow is not possible */
		ret = (uint32_t) ret64;
	}

	return ret;
}

/*
 * \brief Return the DESKEW PLL DCO frequency.
 *
 * \param clk The DESKEW PLL clock.
 *
 * \return The DCO frequency of the DESKEW in Hz.
 */
static uint32_t clk_pll_deskew_get_freq(struct clk *clock_ptr)
{
	const struct clk_data *clock_data = clk_get_data(clock_ptr);
	uint32_t ret;

	/*
	 * If disabled, return the frequency we would be running at once
	 * we bring it out of bypass. If enabled and in bypass, return 0.
	 */
	if (clock_ptr->ref_count == 0U) {
		ret = 0U;
	} else if ((clock_ptr->flags & CLK_FLAG_CACHED) != 0U) {
		ret = soc_clock_values[clock_data->freq_idx];
	} else {
		if (clk_pll_deskew_is_bypass(clock_ptr)) {
			ret = 0U;
		} else {
			ret = clk_pll_deskew_get_freq_internal(clock_ptr);
		}
		soc_clock_values[clock_data->freq_idx] = ret;
		clock_ptr->flags |= CLK_FLAG_CACHED;
	}

	return ret;
}

static bool clk_pll_deskew_set_state(struct clk *clock_ptr, bool enabled)
{
	bool ret = true;

	clock_ptr->flags &= (uint8_t) ~CLK_FLAG_CACHED;

	if (enabled) {
		ret = clk_pll_deskew_wait_for_lock(clock_ptr);
	}

	if (ret) {
		(void)clk_pll_deskew_bypass(clock_ptr, !enabled);
	}

	return ret;
}

/*
 * \brief Get the state of the DESKEW PLL.
 *
 * \param clk The DESKEW PLL clock.
 *
 * \return Returns HW state using the CLK_HW_STATE macro.
 */
static uint32_t clk_pll_deskew_get_state(struct clk *clock_ptr)
{
	struct clk *clock_parent = NULL;
	uint32_t ret = CLK_HW_STATE_ENABLED;

	if (clock_ptr->ref_count == 0U) {
		ret = CLK_HW_STATE_DISABLED;
	}

	/* PLLs can't function without an enabled parent */
	if (ret == CLK_HW_STATE_ENABLED) {
		const struct clk_parent *p;

		p = clk_get_parent(clock_ptr);
		if (p != NULL) {
			clock_parent = clk_lookup(p->clk);
		}
		if (clock_parent != NULL) {
			ret = clk_get_state(clock_parent);
		} else {
			ret = CLK_HW_STATE_DISABLED;
		}
	}

	if (ret == CLK_HW_STATE_ENABLED) {
		if (!clk_pll_deskew_is_bypass(clock_ptr)
		    && !clk_pll_deskew_check_lock(clock_ptr)) {
			ret = CLK_HW_STATE_TRANSITION;
		}
	}

	return ret;
}

static int32_t clk_pll_deskew_init_internal(struct clk *clock_ptr)
{
	const struct clk_data *clock_data = clk_get_data(clock_ptr);
	const struct clk_data_pll_deskew *pll;
	const struct clk_data_pll *data_pll;
	uint32_t ctrl;
	uint32_t cfg;
	uint32_t i;
	int32_t ret = SUCCESS;

	data_pll = container_of(clock_data->data, const struct clk_data_pll,
				data);
	pll = container_of(data_pll, const struct clk_data_pll_deskew,
			   data_pll);

	/*
	 * Unlock write access. Note this register does not readback the
	 * written value.
	 */
	writel(PLL_DESKEW_LOCKKEY0_VALUE,
	       (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_LOCKKEY0(pll->idx));
	writel(PLL_DESKEW_LOCKKEY1_VALUE,
	       (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_LOCKKEY1(pll->idx));

	ctrl = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));

	/* Make sure PLL is enabled */
	if ((ctrl & PLL_DESKEW_CTRL_PD_EN) == 0U) {
		ctrl |= PLL_DESKEW_CTRL_PD_EN;
		writel(ctrl, (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));
		udelay(1U); /* Wait 1us */
	}

	/* Always bypass if we lose lock */
	ctrl |= PLL_DESKEW_CTRL_BYP_ON_LOCKLOSS;

	/* Prefer glitchless bypass */
	if ((ctrl & PLL_DESKEW_CTRL_INTL_BYP_EN) != 0U) {
		ctrl |= (uint32_t)PLL_DESKEW_CTRL_BYPASS_EN;
		ctrl &= (uint32_t)~PLL_DESKEW_CTRL_INTL_BYP_EN;
	}

	writel(ctrl, (uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CTRL(pll->idx));

	/* Enable all HSDIV outputs */
	cfg = (uint32_t)readl((uintptr_t)pll->base + (uintptr_t)PLL_DESKEW_CFG(pll->idx));
	for (i = 0U; (i < 16U); i++) {
		/* Enable HSDIV output if present */
		if (((1UL << (i + 16UL)) & cfg) != 0UL) {
			ctrl = (uint32_t)readl((uintptr_t)pll->base
					       + (uintptr_t)PLL_DESKEW_HSDIV_CTRL(pll->idx, i));
			ctrl |= PLL_DESKEW_HSDIV_CTRL_CLKOUT_EN;
			writel(ctrl, (uintptr_t)pll->base +
			       (uintptr_t)PLL_DESKEW_HSDIV_CTRL(pll->idx, i));
		}
	}

	ret = pll_init(clock_ptr);

	clock_ptr->flags &= (uint8_t) ~CLK_FLAG_PWR_UP_EN;

	return ret;
}

static int32_t clk_pll_deskew_init(struct clk *clock_ptr)
{
	int32_t ret;

	ret = clk_pll_deskew_init_internal(clock_ptr);

	return ret;
}

const struct clk_drv clk_drv_pll_deskew = {
	.init = clk_pll_deskew_init,
	.get_freq = clk_pll_deskew_get_freq,
	.set_freq = clk_pll_deskew_set_freq,
	.get_state = clk_pll_deskew_get_state,
	.set_state = clk_pll_deskew_set_state,
};
