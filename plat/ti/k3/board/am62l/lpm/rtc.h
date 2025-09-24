/*
 * Copyright (c) 2024-2025, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LPM_RTC_H__
#define __LPM_RTC_H__

#include <plat/common/platform.h>

struct rtc_time {
	uint32_t	sub_sec;
	uint32_t	sec_lo;
	uint32_t	sec_hi;
};

/**
 *  \brief  Read RTC counter
 */
void lpm_rtc_read_time(struct rtc_time *rtc);

#endif /* __LPM_RTC_H__ */
