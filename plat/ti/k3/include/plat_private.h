/*
 * Copyright (c) 2025, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef PLATFORM_PRIVATE_H
#define PLATFORM_PRIVATE_H

#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <drivers/generic_delay_timer.h>
#include <board_def.h>

#define ADDR_DOWN(_adr) (_adr & XLAT_ADDR_MASK(2U))
#define SIZE_UP(_adr, _sz) (round_up((_adr + _sz), XLAT_BLOCK_SIZE(2U)) - ADDR_DOWN(_adr))

#define K3_MAP_REGION_FLAT(_adr, _sz, _attr) \
	MAP_REGION_FLAT(ADDR_DOWN(_adr), SIZE_UP(_adr, _sz), _attr)

extern const mmap_region_t plat_k3_mmap[];

enum k3_device_type {
	K3_DEVICE_TYPE_BAD,
	K3_DEVICE_TYPE_GP,
	K3_DEVICE_TYPE_TEST,
	K3_DEVICE_TYPE_EMU,
	K3_DEVICE_TYPE_HS_FS,
	K3_DEVICE_TYPE_HS_SE,
};

#define K3_SEC_MGR_SYS_STATUS		0x44234100
#define SYS_STATUS_DEV_TYPE_SHIFT	0
#define SYS_STATUS_DEV_TYPE_MASK	(0xf)
#define SYS_STATUS_DEV_TYPE_GP		0x3
#define SYS_STATUS_DEV_TYPE_TEST	0x5
#define SYS_STATUS_DEV_TYPE_EMU		0x9
#define SYS_STATUS_DEV_TYPE_HS		0xa
#define SYS_STATUS_SUB_TYPE_SHIFT	8
#define SYS_STATUS_SUB_TYPE_MASK	(0xf << 8)
#define SYS_STATUS_SUB_TYPE_VAL_FS	0xa

void ti_init_scmi_server(void);

/* Any kind of SOC specific init can be done here */
int ti_soc_init(void);

#endif /* PLATFORM_PRIVATE_H */

