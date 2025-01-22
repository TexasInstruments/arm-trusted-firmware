/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RESOURCE_H
#define RESOURCE_H

#include <common/debug.h>

#include <device_clk.h>
#include <lib/io.h>
#include <pm_types.h>
#include <soc_device.h>
/*
 * A compact method of storing device resources. These are meant to allow
 * SoC independent interaction with devices. For instance, for device foo,
 * clk 0 is always the functional clock and clk 1 is always the interface
 * clock, even though their device/clock framework IDs may differ. Only
 * resources utilized by drivers within the System Firmware are included.
 *
 * Not all devices will have the full set of resources supported by a driver,
 * if the missing resources are at the end of the resource list, the
 * resoure table can just define fewer resources. If the resource is in
 * the middle of the resource list, the resource table should include
 * RESOURCE_xxx_NONE for the missing resources.
 */

#define RESOURCE_IRQ	0x00U
#define RESOURCE_CLK	0x40U
#define RESOURCE_MEM	0x80U
#define RESOURCE_RST	0xc0U

/*
 * Each resource set includes a set of resources including a resource
 * header followed by count resources. The last resource header has
 * a it's last flag set. Resource types can be defined in any order.
 */
#define RESOURCE_TYPE_MASK	0xc0U
#define RESOURCE_LAST		0x20U
#define RESOURCE_COUNT_MASK	0x1fU

#define RESOURCE_MEM_NONE	0xffffffffU

struct resource_mem {
	uint32_t addr;
} __packed;

#define RDAT_HDR(type, count, last) ((uint8_t) (type) | (uint8_t) (count) | \
				     (uint8_t) ((last) ? RESOURCE_LAST : 0U))

#define RDAT_MEM(mem)		(uint8_t) ((uint32_t) (mem) & 0xffU),	     \
				(uint8_t) (((uint32_t) (mem) >> 8) & 0xffU), \
				(uint8_t) (((uint32_t) (mem) >> 16) & 0xffU),\
				(uint8_t) ((uint32_t) (mem) >> 24)

struct device;
const void *resource_get(struct device *dev, uint8_t type, uint8_t idx);

#endif
