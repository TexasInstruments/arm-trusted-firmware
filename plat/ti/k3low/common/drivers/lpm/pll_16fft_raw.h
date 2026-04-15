/*
 * Copyright (c) 2024-2026, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PLL_16FFT_RAW_H__
#define __PLL_16FFT_RAW_H__

#include <plat/common/platform.h>

struct pll_raw_data {
	uint64_t	base;
	uint32_t	freq_ctrl0;
	uint32_t	freq_ctrl1;
	uint32_t	div_ctrl;
	uint32_t	hsdiv[16];
};

/**
 * @brief Restore a saved pll configuration from k3low_pll_save
 *
 * @param pll Pointer to location to restore saved PLL context
 * @return 0 on success, error code otherwise
 */
int32_t k3low_pll_restore(struct pll_raw_data *pll);

/**
 * @brief Save a pll configuration
 *
 * @param pll Pointer to location to save PLL context, with base set
 *            to address for PLL to save
 */
void k3low_pll_save(struct pll_raw_data *pll);

/**
 * @brief Disable a PLL after bypassing
 *
 * @param pll Pointer to PLL context, with base set to address for pll
 *            to manipulate
 */
void k3low_pll_disable(struct pll_raw_data *pll);

/**
 * @brief Unbypass a PLL
 *
 * @param pll Pointer to PLL context, with base set to address for pll
 *            to manipulate
 */
void k3low_pll_unbypass(struct pll_raw_data *pll);

/**
 * @brief Bypass a PLL's HSDIVs
 *
 * @param pll Pointer to PLL context, with base set to address for pll
 *            to manipulate
 */
void k3low_pll_bypass_hsdivs(struct pll_raw_data *pll);

/**
 * \brief Program specific HSDIV divider value
 * \param pll Pointer to PLL context, with base set to address for pll to
 *            manipulate
 * \param hsdiv HSDIV index to program
 * \param value New divider value to program (7-bit value)
 */
void k3low_pll_program_hsdiv(struct pll_raw_data *pll, uint8_t hsdiv, uint8_t value);

/**
 * \brief Disable multiple HSDIVs of PLL
 * \param pll Pointer to PLL context, with base set to address for pll to
 *            manipulate
 * \param hsdiv_indices Array of HSDIV indices to disable
 * \param count Number of HSDIVs in the array
 */
void k3low_pll_disable_hsdivs(struct pll_raw_data *pll, const uint8_t *hsdiv_indices,
				uint8_t count);

/**
 * \brief Enable multiple HSDIVs of PLL
 * \param pll Pointer to PLL context, with base set to address for pll to
 *            manipulate
 * \param hsdiv_indices Array of HSDIV indices to enable
 * \param count Number of HSDIVs in the array
 */
void k3low_pll_enable_hsdivs(struct pll_raw_data *pll, const uint8_t *hsdiv_indices, uint8_t count);

#endif /* __PLL_16FFT_RAW_H__ */

