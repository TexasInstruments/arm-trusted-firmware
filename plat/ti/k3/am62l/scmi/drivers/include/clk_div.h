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

#ifndef CLK_DIV_H
#define CLK_DIV_H

#include <clk.h>

struct clk_data_div {
	struct clk_drv_data	data;
	uint16_t			n;
	uint16_t			default_div;
};

struct clk_data_div_reg {
	uint32_t			reg;
	struct clk_data_div	data_div;
	uint8_t			bit : 7;
	uint8_t			start_at_1 : 1;
};

struct clk_data_div_reg_go {
	uint32_t			reg;
	struct clk_data_div	data_div;
	uint8_t			bit : 7;
	uint8_t			start_at_1 : 1;
	uint8_t			go;
};

struct clk_drv_div {
	struct clk_drv	drv;
	bool		(*set_div)(struct clk *clkp, uint32_t d);
	uint32_t		(*get_div)(struct clk *clkp);
	bool		(*valid_div)(struct clk *clkp, uint32_t d);
};

extern const struct clk_drv_div clk_drv_div_reg;
extern const struct clk_drv_div clk_drv_div_reg_ro;
extern const struct clk_drv_div clk_drv_div_reg_go;
extern const struct clk_drv_div clk_drv_div_fixed;

bool clk_div_notify_freq(struct clk *clkp, uint32_t parent_freq_hz, bool query);
uint32_t clk_div_set_freq(struct clk *clkp, uint32_t target_hz, uint32_t min_hz, uint32_t max_hz, bool query, bool *changed);
uint32_t clk_div_get_freq(struct clk *clkp);
uint32_t clk_div_set_freq_static_parent(struct clk *clkp, uint32_t target_hz, uint32_t min_hz, uint32_t max_hz, bool query, bool *changed);

uint32_t clk_div_reg_go_get_div(struct clk *clkp);
bool clk_div_reg_go_set_div(struct clk *clkp, uint32_t d);
uint32_t clk_div_reg_get_div(struct clk *clkp);
bool clk_div_reg_set_div(struct clk *clkp, uint32_t d);

uint32_t clk_get_div(struct clk *clkp);
bool clk_set_div(struct clk *clkp, uint32_t d);
int32_t clk_div_init(struct clk *clkp);

#endif
