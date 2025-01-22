/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include <hosts.h>
#include <pm.h>
#include <tisci/pm/tisci_pm_clock.h>

#include <common/debug.h>

#include <clk_wrapper.h>

int32_t scmi_handler_clock_prepare(uint32_t dev_id, uint32_t clk_id)
{
	struct tisci_msg_set_clock_req req;
	uint32_t flags = 0;

	flags |= TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.flags = flags;
	req.hdr.host = HOST_ID_TIFS;
	req.state = TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ;

	return set_clock_handler(&req);
}

int32_t scmi_handler_clock_unprepare(uint32_t dev_id, uint32_t clk_id)
{
	struct tisci_msg_set_clock_req req;
	uint32_t flags = 0;

	flags |= TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.flags = flags;
	req.hdr.host = HOST_ID_TIFS;
	req.state = TISCI_MSG_VALUE_CLOCK_SW_STATE_AUTO;

	return set_clock_handler(&req);
}

int32_t scmi_handler_clock_get_state(uint32_t dev_id, uint32_t clk_id)
{
	struct tisci_msg_get_clock_req req;
	struct tisci_msg_get_clock_resp resp;
	int32_t ret;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.host = HOST_ID_TIFS;

	ret = get_clock_handler(&req);
	if (ret == 0) {
		/* Copy response data from req buffer to resp structure */
		(void)memcpy(&resp, &req, sizeof(resp));
		return (int32_t)resp.programmed_state;
	}
	return 0;
}

int32_t scmi_handler_clock_set_rate(uint32_t dev_id, uint32_t clk_id,
				    uint64_t target_freq)
{
	struct tisci_msg_set_freq_req req;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.min_freq_hz = target_freq / 10U * 9U;
	req.target_freq_hz = target_freq;
	req.max_freq_hz = target_freq / 10U * 11U;
	req.hdr.host = HOST_ID_TIFS;

	return set_freq_handler(&req);
}
uint64_t scmi_handler_clock_get_rate(uint32_t dev_id, uint32_t clk_id)
{
	struct tisci_msg_get_freq_req req;
	struct tisci_msg_get_freq_resp resp;
	int32_t ret;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.host = HOST_ID_TIFS;

	ret = get_freq_handler(&req);
	if (ret == 0) {
		/* Copy response data from req buffer to resp structure */
		(void)memcpy(&resp, &req, sizeof(resp));
		return resp.freq_hz;
	}
	return 0U;
}

int32_t scmi_handler_clock_get_num_clock_parents(uint32_t dev_id,
						 uint32_t clk_id)
{
	struct tisci_msg_get_num_clock_parents_req req;
	struct tisci_msg_get_num_clock_parents_resp resp;
	int32_t ret;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.host = HOST_ID_TIFS;

	ret = get_num_clock_parents_handler(&req);
	if (ret == 0) {
		/* Copy response data from req buffer to resp structure */
		(void)memcpy(&resp, &req, sizeof(resp));
		return (int32_t)resp.num_parents;
	}
	return 0;
}

int32_t scmi_handler_clock_set_clock_parent(uint32_t dev_id, uint32_t clk_id,
					    uint32_t parent_id)
{
	struct tisci_msg_set_clock_parent_req req;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.host = HOST_ID_TIFS;
	req.parent = (uint8_t)parent_id;

	return set_clock_parent_handler(&req);
}

int32_t scmi_handler_clock_get_clock_parent(uint32_t dev_id, uint32_t clk_id,
					    uint32_t *parent_id)
{
	struct tisci_msg_get_clock_parent_req req;
	struct tisci_msg_get_clock_parent_resp resp;
	int32_t status = 0;

	req.device = (uint8_t)dev_id;
	req.clk = (uint8_t)clk_id;
	req.hdr.host = HOST_ID_TIFS;

	status = get_clock_parent_handler(&req);
	if (status == 0) {
		/* Copy response data from req buffer to resp structure */
		(void)memcpy(&resp, &req, sizeof(resp));
		*parent_id = (uint32_t)resp.parent;
	} else {
		*parent_id = 0U;
	}

	return status;
}
