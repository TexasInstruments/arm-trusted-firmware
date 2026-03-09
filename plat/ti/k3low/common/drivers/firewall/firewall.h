/*
 * Copyright (c) 2026, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FIREWALL_H
#define FIREWALL_H

#include <stdint.h>

#include <lib/utils_def.h>

#define TFA_HOST_ID		10U
#define A53_PRIV_ID		4U
#define FW_BACKGROUND_BIT	BIT(8U)

#define DDR_BASE		0x80000000
#define DDR_SIZE		0x7FFFFFFF

/* Firewall IDs */
#define OSPI_FWL_ID		97U
#define ADC_MCASP_FWL_ID	160U
#define DDR_FWL_ID		1U

/* Firewall regions to disable */
#define OSPI_FWL_REGION		0U
#define ADC_MCASP_FWL_REGION	0U

/* Firewall Regions to configure */
#define DDR_BG_REGION		0U

/* Firewall Control Register Values */
#define FWL_CTRL_EN		0xA
#define FWL_CTRL_EN_BG		(FWL_CTRL_EN | FW_BACKGROUND_BIT)

/* Firewall permission values */
#define FWL_PERM_ALL_RW		0xC3BBBB /* RW access to ALL hosts */

struct fwl_data {
	uint16_t fwl_id;
	uint8_t region;
};

/*
 * ROM makes bad firewall configurations that cause boot to fail. As a
 * workaround, make these configurations permissive for all and keep them
 * enabled. Keeping them permissive has the same effect as disabling them, and
 * keeping them enabled causes TIFS to save their context during suspend and
 * restore it during resume. Disabling those firewalls would lead to TIFS not
 * restoring context correctly after ROM sets those firewalls again, during
 * resume.
 */
void open_rom_fwl(void);

/*
 * Protects BL31 and BL32 memory regions by configuring restrictive foreground
 * firewalls over them. Configures a permissive background firewall over rest of
 * the DDR.
 */
void update_fwl_configs(void);

#endif /* FIREWALL_H */
