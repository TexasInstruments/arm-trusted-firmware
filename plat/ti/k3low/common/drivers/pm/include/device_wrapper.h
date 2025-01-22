/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

int32_t scmi_handler_device_state_set_on(uint32_t dev_id);

int32_t scmi_handler_device_state_set_on_exclusive(uint32_t dev_id);

int32_t scmi_handler_device_state_set_off(uint32_t dev_id);

unsigned int scmi_handler_device_state_get(uint32_t dev_id);

