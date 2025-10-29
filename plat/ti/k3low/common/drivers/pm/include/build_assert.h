/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BUILD_ASSERT_H
#define BUILD_ASSERT_H

#define BUILD_ASSERT_OR_ZERO(cond) \
	(sizeof(char[(cond) ? 1 : -1]) - 1)

#endif /* BUILD_ASSERT_H */
