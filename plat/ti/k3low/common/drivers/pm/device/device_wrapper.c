/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <tisci/pm/tisci_pm_device.h>
#include <hosts.h>
#include <pm.h>
#include <common/debug.h>
#include <device_wrapper.h>
#include <string.h>

#define POWER_STATE_ON	(0U << 30U)
#define POWER_STATE_OFF (1U << 30U)

int32_t scmi_handler_device_state_set_on(uint32_t dev_id)
{
	struct tisci_msg_set_device_req req;
	uint32_t flags = 0;

	req.id = dev_id;
	req.hdr.flags = flags;
	req.hdr.host = HOST_ID_TIFS;
	req.state = TISCI_MSG_VALUE_DEVICE_SW_STATE_ON;

	return set_device_handler(&req);
}

int32_t scmi_handler_device_state_set_off(uint32_t dev_id)
{
	struct tisci_msg_set_device_req req;
	uint32_t flags = 0;

	req.id = dev_id;
	req.hdr.flags = flags;
	req.hdr.host = HOST_ID_TIFS;
	req.state = TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF;

	return set_device_handler(&req);
}

uint32_t scmi_handler_device_state_get(uint32_t dev_id)
{
	struct tisci_msg_get_device_resp resp;
	struct tisci_msg_get_device_req req;
	uint32_t flags = 0;
	int32_t ret = 0/* SUCCESS */;
	uint32_t state = POWER_STATE_ON;

	req.id = dev_id;
	req.hdr.flags = flags;
	req.hdr.host = HOST_ID_TIFS;

	(void)memcpy(&resp, &req, sizeof(req));
	ret = get_device_handler(&resp);
	if (ret == 0/* SUCCESS */) {
		if ((resp.programmed_state == (uint8_t)TISCI_MSG_VALUE_DEVICE_SW_STATE_ON) &&
		   (resp.current_state == (uint8_t)TISCI_MSG_VALUE_DEVICE_HW_STATE_ON)) {
			state = POWER_STATE_ON;
		} else {
			state = POWER_STATE_OFF;
		}
	}
	return state;
}
