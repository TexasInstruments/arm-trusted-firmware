/*
 * System Firmware
 *
 * Cortex-M3 (CM3) firmware for power management
 *
 * Copyright (C) 2015-2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <soc/am62lx/clk_soc_lfosc0.h>
#include <types/array_size.h>
#include <lib/io.h>
#include <compiler.h>

#define AM62L_WKUP_RTCSS0_RTC_BASE    0x2b1f0000UL
#define AM62L_WKUP_RTCSS0_LFXOSC_CTRL 0x80UL
#define LFXOSC_32K_DISABLE_VAL        0x80UL

/*
 * LFXOSC_CTRL - Low Frequency Oscillator Control Register
 * 32.768 KHz
 */
static uint32_t clk_soc_lfosc0_get_freq(struct clk *clk_ptr UNUSED)
{
	uint32_t ret = 0;
	uint32_t v;

	v = readl(AM62L_WKUP_RTCSS0_RTC_BASE + AM62L_WKUP_RTCSS0_LFXOSC_CTRL)
                & LFXOSC_32K_DISABLE_VAL;
	if (v == 0) {
		ret = FREQ_KHZ(32.768);
	}

	return ret;
}

static uint32_t clk_soc_lfosc0_get_state(struct clk *clk_ptr UNUSED)
{
	return CLK_HW_STATE_ENABLED;
}

const struct clk_drv clk_drv_soc_lfosc0 = {
	.get_freq	= clk_soc_lfosc0_get_freq,
	.get_state	= clk_soc_lfosc0_get_state,
};
