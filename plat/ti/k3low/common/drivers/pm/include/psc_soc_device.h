/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PSC_SOC_DEVICE_H
#define PSC_SOC_DEVICE_H

#include <pm_types.h>

/**
 * \brief SoC specific const device data.
 *
 * This stores the SoC specific const data for each device.
 */
struct soc_device_data {
	uint8_t psc_idx : 3;    /** PSC index */
	uint8_t pd : 5;         /** PSC powerdomain */
	lpsc_idx_t mod;         /** PSC module index */
};

#endif
