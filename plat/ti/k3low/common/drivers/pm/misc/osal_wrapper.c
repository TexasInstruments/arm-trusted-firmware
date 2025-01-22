/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <drivers/delay_timer.h>

void osal_delay(uint64_t usecs)
{
	udelay((uint32_t)usecs);
}
