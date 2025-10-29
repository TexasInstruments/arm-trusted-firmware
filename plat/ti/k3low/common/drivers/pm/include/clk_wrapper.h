/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

int32_t scmi_handler_clock_prepare(uint32_t dev_id, uint32_t clk_id);

int32_t scmi_handler_clock_unprepare(uint32_t dev_id, uint32_t clk_id);

int32_t scmi_handler_clock_get_state(uint32_t dev_id, uint32_t clk_id);

int32_t scmi_handler_clock_set_rate(uint32_t dev_id, uint32_t clk_id,
				    uint64_t target_freq);

uint64_t scmi_handler_clock_get_rate(uint32_t dev_id, uint32_t clk_id);

int32_t scmi_handler_clock_get_clock_parent(uint32_t dev_id, uint32_t clk_id,
					    uint32_t *parent_id);

int32_t scmi_handler_clock_set_clock_parent(uint32_t dev_id, uint32_t clk_id,
					    uint32_t parent_id);

int32_t scmi_handler_clock_get_num_clock_parents(uint32_t dev_id,
						 uint32_t clk_id);
