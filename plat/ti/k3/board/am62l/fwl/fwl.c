/*
 * Copyright (c) 2026, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <fwl.h>
#include <ti_sci.h>

const struct fwl_data fwls[] = {
	{DDR_FWL_ID, DDR_FWL_NUM_REGIONS},	/* DDR */
	{OSPI_FWL_ID, OSPI_FWL_NUM_REGIONS},	/* OSPI */
	{ADC_MCASP_FWL_ID, ADC_MCASP_FWL_NUM_REGIONS},	/* ADC and MCASP */
};

const size_t fwls_count = ARRAY_SIZE(fwls);

/*
 * Removes firewall configurations for a given firewall and region type.
 * This function iterates through all regions of the specified firewall,
 * takes ownership, reads the current configuration, and disables any
 * active firewall regions of the requested type (foreground or background).
 *
 * @fwl: Firewall data containing the firewall ID and number of regions
 * @fwl_type: Type of firewall region to remove (foreground or background)
 */
static void remove_fwl_configs(struct fwl_data fwl, enum k3_fwl_region_type fwl_type)
{
	uint8_t owner_index = TFA_HOST_ID;
	uint8_t owner_privid = A53_PRIV_ID;
	uint16_t owner_permission_bits = 0;
	uint32_t control = 0;
	uint32_t permissions[FWL_MAX_PRIVID_SLOTS] = { };
	uint32_t n_permission_regs = FWL_MAX_PRIVID_SLOTS;
	uint64_t start_address = 0;
	uint64_t end_address = 0;
	int ret = 0;

	for (int i = 0; i < fwl.num_regions; i++) {
		ret = ti_sci_change_fwl_owner(fwl.fwl_id, i, owner_index,
				&owner_privid, &owner_permission_bits);
		if (ret) {
			ERROR("Could not change firewall owner (%d)\n", ret);
			continue;
		}

		ret = ti_sci_get_fwl_region(fwl.fwl_id, i, n_permission_regs,
			&control, permissions, &start_address, &end_address);
		if (ret) {
			ERROR("Could not get firewall region information (%d)\n", ret);
			continue;
		}

		if (control != 0 && (control & (1 << FW_BACKGROUND_BIT)) == fwl_type) {
			control = 0;

			ret = ti_sci_set_fwl_region(fwl.fwl_id, i, n_permission_regs,
				control, permissions, start_address, end_address);
			if (ret) {
				ERROR("Could not disable firewall region information (%d)\n", ret);
				panic();
			}
		}
	}
}

void update_fwl_configs(void)
{
	/* Disable firewalls that were configured by ROM for boot phase */
	for (int i = 0; i < fwls_count; i++) {
		remove_fwl_configs(fwls[i], K3_FWL_REGION_FOREGROUND);
		remove_fwl_configs(fwls[i], K3_FWL_REGION_BACKGROUND);
	}
}
