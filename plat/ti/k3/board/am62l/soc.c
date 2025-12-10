/*
 * Copyright (c) 2025, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <device_wrapper.h>
#include <lpm_stub.h>
#include <plat_private.h>
#include <plat_scmi_def.h>
#include <rtc.h>
#include <ti_sci.h>
#include <ti_sci_transport.h>

#define TFA_HOST_ID		10U
#define A53_PRIV_ID		4U
#define FW_BACKGROUND_BIT	8U

/* Firewall IDs */
#define DDR_FWL_ID		1U
#define OSPI_FWL_ID		97U
#define ADC_MCASP_FWL_ID	160U

/* Number of firewall regions */
#define DDR_FWL_NUM_REGIONS		16U
#define OSPI_FWL_NUM_REGIONS		8U
#define ADC_MCASP_FWL_NUM_REGIONS 	16U

enum k3_fwl_region_type {
	K3_FWL_REGION_FOREGROUND = 0,
	K3_FWL_REGION_BACKGROUND = BIT(FW_BACKGROUND_BIT),
};

static struct fwl_data {
	uint16_t fwl_id;
	uint8_t num_regions;
} const fwls[] = {
	{DDR_FWL_ID, DDR_FWL_NUM_REGIONS},	/* DDR */
	{OSPI_FWL_ID, OSPI_FWL_NUM_REGIONS},	/* OSPI */
	{ADC_MCASP_FWL_ID, ADC_MCASP_FWL_NUM_REGIONS},	/* ADC and MCASP */
};

/* Table of regions to map using the MMU */
/* TODO: Add AM62L specific mapping such that K3 devices don't break */
const mmap_region_t plat_k3_mmap[] = {
	MAP_REGION_FLAT(0x0, 0x80000000, MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(K3_FUSE_WRITEBUFF_BASE, K3_FUSE_WRITEBUFF_SIZE, MT_MEMORY | MT_RW | MT_NS),
#ifdef K3_AM62L_LPM
	MAP_REGION_FLAT(DEVICE_WKUP_SRAM_BASE, DEVICE_WKUP_SRAM_SIZE, MT_MEMORY | MT_RW | MT_SECURE),
#endif
	{ /* sentinel */ }
};

void remove_fwl_configs(struct fwl_data fwl, enum k3_fwl_region_type fwl_type)
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
					    &control, permissions,
					    &start_address, &end_address);
		if (ret) {
			ERROR("Could not get firewall region information (%d)\n", ret);
			continue;
		}

		if (control != 0 && (control & (1 << FW_BACKGROUND_BIT)) == fwl_type) {
			control = 0;

			ret = ti_sci_set_fwl_region(fwl.fwl_id, i, n_permission_regs,
						    control, permissions,
						    start_address, end_address);
			if (ret) {
				ERROR("Could not disable firewall region information (%d)\n", ret);
				panic();
			}
		}
	}
}

int ti_soc_init(void)
{
	struct ti_sci_msg_version version;
	int ret;

	generic_delay_timer_init();
	ti_init_scmi_server();
#ifdef K3_AM62L_LPM
	if (k3_lpm_stub_copy_to_sram()) {
		WARN("A53 stub copy failed!\n");
	} else {
		INFO("A53 stub copy passed\n");
	}
#endif
	ret = ti_sci_get_revision(&version);
	if (ret) {
		ERROR("Unable to communicate with the control firmware (%d)\n", ret);
		return ret;
	}

	NOTICE("SYSFW ABI: %d.%d (firmware rev 0x%04x '%s')\n",
	     version.abi_major, version.abi_minor,
	     version.firmware_revision,
	     version.firmware_description);

	ret = ti_sci_proc_request(PLAT_PROC_START_ID);
	if (ret) {
		ERROR("Unable to request host (%d)\n", ret);
		return ret;
	}

	/* Enable ACP based coherency */
	ret = ti_sci_proc_set_boot_ctrl(PLAT_PROC_START_ID, 0,
									PROC_BOOT_CTRL_FLAG_ARMV8_AINACTS);
	if (ret) {
		ERROR("Unable to set boot control (%d)\n", ret);
		return ret;
	}

	/* Update firewall configurations */
	for (int i = 0; i < ARRAY_SIZE(fwls); i++) {
		remove_fwl_configs(fwls[i], K3_FWL_REGION_FOREGROUND);
		remove_fwl_configs(fwls[i], K3_FWL_REGION_BACKGROUND);
	}

	return 0;
}
