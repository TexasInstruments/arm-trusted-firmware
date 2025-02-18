/*
 * Copyright (c) 2025, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <plat_scmi_def.h>
#include <device_wrapper.h>
#include <plat_private.h>
#include <lpm_stub.h>
#include <rtc.h>
#include <common/debug.h>

/* Table of regions to map using the MMU */
/* TODO: Add AM62L specific mapping such that K3 devices don't break */
const mmap_region_t plat_k3_mmap[] = {
	MAP_REGION_FLAT(0x0, 0x80000000, MT_DEVICE | MT_RW | MT_SECURE),
	{ /* sentinel */ }
};

void ti_soc_init(void)
{
	generic_delay_timer_init();
	ti_init_scmi_server();
#ifdef TI_AM62L_LPM
	if (k3_lpm_stub_copy_to_sram()) {
		WARN("A53 stub copy failed!\n");
	} else {
		INFO("A53 stub copy passed\n");
	}
	rtc_init();
#endif
}
