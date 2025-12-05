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
#define FW_ENABLE_REGION        0x0a
#define FW_CACHE_MODE		BIT(9)
#define FW_WILDCARD_PRIVID      0xc3
#define FW_NON_SECURE           GENMASK_32(15, 0)

/* Firewall IDs */
#define DDR_FWL_ID                      1U
#define OSPI_FWL_ID                     97U
#define ADC_FWL_ID                      160U
#define MCASP_FWL_ID                    160U

/* Firewall Regions */
#define DDR_FWL_REGION                  1U
#define OSPI_FWL_REGION                 2U
#define ADC_FWL_REGION                  1U
#define MCASP_FWL_REGION                2U

/* Firewall Start Addresses */
#define DDR_START_ADDR                  0x80a00000ULL
#define OSPI_START_ADDR                 0x500000000ULL
#define ADC_START_ADDR                  0x28001000ULL
#define MCASP_START_ADDR                0x02b00000ULL

/* Firewall End Addresses */
#define DDR_END_ADDR                    0x100000000ULL
#define OSPI_END_ADDR                   0x5ffffffffULL
#define ADC_END_ADDR                    0x280013ffULL
#define MCASP_END_ADDR                  0x02b01fffULL

static struct fwl_data {
	uint16_t fwl_id;
	uint16_t fwl_region;
	uint64_t start_address;
	uint64_t end_address;
} const fwls[] = {
	{DDR_FWL_ID, DDR_FWL_REGION, DDR_START_ADDR, DDR_END_ADDR},	/* DDR. Start addr is BL32 base + sizeof(OP-TEE), */
						/* end addr is +2GB from start of DDR */
	{OSPI_FWL_ID, OSPI_FWL_REGION, OSPI_START_ADDR, OSPI_END_ADDR},	/* OSPI */
	{ADC_FWL_ID, ADC_FWL_REGION, ADC_START_ADDR, ADC_END_ADDR},	/* ADC */
	{MCASP_FWL_ID, MCASP_FWL_REGION, MCASP_START_ADDR, MCASP_END_ADDR},	/* MCASP */
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

void update_fwl_configs(struct fwl_data fwl)
{
	int ret;
	uint8_t owner_index = TFA_HOST_ID;
	uint8_t owner_privid = A53_PRIV_ID;
	uint16_t owner_permission_bits = 0xffff;
	uint32_t control = 0;
	uint32_t permissions[FWL_MAX_PRIVID_SLOTS] = { };

	ret = ti_sci_change_fwl_owner(fwl.fwl_id, fwl.fwl_region, owner_index,
					&owner_privid, &owner_permission_bits);
	if (ret) {
		ERROR("Could not change firewall owner (%d)\n", ret);
		panic();
	}

	permissions[0] = (FW_WILDCARD_PRIVID << 16) | FW_NON_SECURE;
	permissions[1] = (FW_WILDCARD_PRIVID << 16) | FW_NON_SECURE;
	permissions[2] = (FW_WILDCARD_PRIVID << 16) | FW_NON_SECURE;
	control = (FW_CACHE_MODE | FW_ENABLE_REGION);

	ret = ti_sci_set_fwl_region(fwl.fwl_id, fwl.fwl_region, 3,
				    control, permissions,
				    fwl.start_address, fwl.end_address);
	if (ret) {
		ERROR("Could not set firewall region information (%d)\n", ret);
		panic();
	}

}

static enum k3_device_type get_device_type(void)
{
	uint32_t sys_status = mmio_read_32(K3_SEC_MGR_SYS_STATUS);

	uint32_t sys_dev_type = (sys_status & SYS_STATUS_DEV_TYPE_MASK) >>
			SYS_STATUS_DEV_TYPE_SHIFT;

	uint32_t sys_sub_type = (sys_status & SYS_STATUS_SUB_TYPE_MASK) >>
			SYS_STATUS_SUB_TYPE_SHIFT;

	INFO("System Status: 0x%x\n", sys_status);

	switch (sys_dev_type) {
	case SYS_STATUS_DEV_TYPE_GP:
		return K3_DEVICE_TYPE_GP;
	case SYS_STATUS_DEV_TYPE_TEST:
		return K3_DEVICE_TYPE_TEST;
	case SYS_STATUS_DEV_TYPE_EMU:
		return K3_DEVICE_TYPE_EMU;
	case SYS_STATUS_DEV_TYPE_HS:
		if (sys_sub_type == SYS_STATUS_SUB_TYPE_VAL_FS)
			return K3_DEVICE_TYPE_HS_FS;
		else
			return K3_DEVICE_TYPE_HS_SE;
	default:
		return K3_DEVICE_TYPE_BAD;
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

	if (get_device_type() == K3_DEVICE_TYPE_HS_SE) {
		/* Update firewall configurations */
		for (int i = 0; i < ARRAY_SIZE(fwls); i++)
			update_fwl_configs(fwls[i]);
	}

	return 0;
}
