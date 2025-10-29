/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <types/errno.h>
#include <lib/mmio.h>

#define readl(REG) mmio_read_32(REG)

#define writel(VAL, REG) mmio_write_32((uintptr_t)REG, VAL)
