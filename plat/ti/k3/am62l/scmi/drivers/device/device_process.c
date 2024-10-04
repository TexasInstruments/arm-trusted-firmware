/*
 * System Firmware
 *
 * Cortex-M3 (CM3) firmware for power management
 *
 * Copyright (C) 2020-2023, Texas Instruments Incorporated
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

#include <device.h>
#include <device_pm.h>
#include <tisci/pm/tisci_pm_device.h>
#include <types/errno.h>
#include <lib/trace.h>
#include <lib/mmr_lock.h>
#include <device_prepare.h>
#include <soc/host_idx_mapping.h>
#include <pm.h>
#include <common/debug.h>

int32_t set_device_handler(struct tisci_msg_set_device_req *msg_recv)
{
	struct tisci_msg_set_device_req *req =
		(struct tisci_msg_set_device_req *) msg_recv;
	struct tisci_msg_set_device_resp *resp =
		(struct tisci_msg_set_device_resp *) msg_recv;
	struct device *dev = NULL;
	uint32_t id = req->id;
	uint8_t state = req->state;
	uint32_t flags = req->hdr.flags;
	uint8_t host_id = req->hdr.host;
	bool enable, retention;
	int32_t ret = SUCCESS;
	uint8_t host_idx;

	pm_trace(TRACE_PM_ACTION_MSG_RECEIVED, TISCI_MSG_SET_DEVICE);
	pm_trace(TRACE_PM_ACTION_MSG_PARAM_DEV_CLK_ID, id);
	pm_trace(TRACE_PM_ACTION_MSG_PARAM_VAL, state);

	resp->hdr.flags = 0U;

	mmr_unlock_all();

	ret = device_prepare_exclusive(host_id, id, &host_idx, &dev);
	if (ret == SUCCESS) {
		switch (state) {
		case TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF:
                        enable = false;
			retention = false;
			break;
		case TISCI_MSG_VALUE_DEVICE_SW_STATE_RETENTION:
			enable = false;
			retention = true;
			break;
		case TISCI_MSG_VALUE_DEVICE_SW_STATE_ON:
			enable = true;
			retention = true;
			break;
		default:
			pm_trace(TRACE_PM_ACTION_INVALID_STATE, state);
			ret = -EINVAL;
			break;
		}
	}

	if (ret == SUCCESS) {
		if ((flags & TISCI_MSG_FLAG_DEVICE_EXCLUSIVE) != 0UL) {
			/* Make sure no one else has the device enabled */
			uint64_t mask = DEV_FLAG_ENABLED_MASK;
			uint64_t enabled;
			/* It's ok if we already have the device enabled */
			mask &= ~DEV_FLAG_ENABLED(host_idx);
			/* It's also ok if the device in on due to power up en */
			mask &= (uint64_t) ~DEV_FLAG_POWER_ON_ENABLED;
			enabled = (dev->flags & mask) >> DEV_FLAG_ENABLED_BIT;
			if (enabled != 0UL) {
				uint8_t i;
				/*
				 * Note, rather than trying to fit the enabled
				 * bit field in the trace message, just pick
				 * single host to include.
				 */
				for (i = 0U; i < (sizeof(enabled) * 8U); i++) {
					if ((enabled & 1UL) != 0UL) {
						break;
					}
					enabled >>= 1UL;
				}
#ifdef CONFIG_TRACE
				{
					uint32_t enabled_host_id = TRACE_PM_VAL_EXCLUSIVE_BUSY_EHOST_ID_MASK >>
							      TRACE_PM_VAL_EXCLUSIVE_BUSY_EHOST_ID_SHIFT;
					/*
					 * Do a reverse lookup. Find host ID from
					 * host index.
					 */
					if (i != (sizeof(enabled) * 8U)) {
						uint8_t j;
						for (j = 0U; j < soc_host_indexes_sz; j++) {
							if (soc_host_indexes[j] == i) {
								enabled_host_id = j;
								break;
							}
						}
					}
					pm_trace(TRACE_PM_ACTION_EXCLUSIVE_BUSY,
						 (((uint32_t) id << (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_DEVICE_ID_SHIFT) &
						  (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_DEVICE_ID_MASK) |
						 (((uint32_t) host_id << (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_RHOST_ID_SHIFT) &
						  (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_RHOST_ID_MASK) |
						 (((uint32_t) enabled_host_id << (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_EHOST_ID_SHIFT) &
						  (uint32_t) TRACE_PM_VAL_EXCLUSIVE_BUSY_EHOST_ID_MASK));
				}
#endif
				ret = -EINVAL;
			}
		}
	}

	if (ret == SUCCESS) {
		if ((flags & TISCI_MSG_FLAG_DEVICE_WAKE_ENABLED) != 0UL) {
			/* FIXME: Not supported */
		}

		if ((flags & TISCI_MSG_FLAG_DEVICE_EXCLUSIVE) != 0UL) {
			/* Only this host may modify device */
			dev->exclusive = host_idx;
		} else {
			/* Allow any host to modify device */
			dev->exclusive = 0U;
		}

		if ((flags & TISCI_MSG_FLAG_DEVICE_RESET_ISO) != 0UL) {
			device_set_reset_iso(dev, true);
		} else {
			device_set_reset_iso(dev, false);
		}

		/* Ordering to void unnecessary PD transations */
		if (retention) {
			device_set_retention(dev, retention);
		}
		device_set_state(dev, host_idx, enable);
		if (!retention) {
			device_set_retention(dev, retention);
		}
	}

	mmr_lock_all();

	return ret;
}

int32_t get_device_handler(struct tisci_msg_get_device_resp *msg_recv)
{
	struct tisci_msg_get_device_req *req =
		(struct tisci_msg_get_device_req *) msg_recv;
	struct tisci_msg_get_device_resp *resp =
		(struct tisci_msg_get_device_resp *) msg_recv;
	struct device *dev = NULL;
	uint32_t id = req->id;
	int32_t ret = SUCCESS;
	uint8_t host_idx;

	pm_trace(TRACE_PM_ACTION_MSG_RECEIVED, TISCI_MSG_GET_DEVICE);
	pm_trace(TRACE_PM_ACTION_MSG_PARAM_DEV_CLK_ID, id);

	resp->hdr.flags = 0U;

	mmr_unlock_all();

	ret = device_prepare_nonexclusive(req->hdr.host, id, &host_idx, &dev);
	if (ret == SUCCESS) {
		uint32_t context_loss_count;
		uint32_t resets;
		uint8_t programmed_state;
		uint8_t current_state;

		context_loss_count = device_get_context_loss_count(dev);
		resets = device_get_resets(dev);

		if ((dev->flags & DEV_FLAG_ENABLED(host_idx)) != 0UL) {
			programmed_state = TISCI_MSG_VALUE_DEVICE_SW_STATE_ON;
		} else if ((dev->flags & DEV_FLAG_RETENTION) != 0UL) {
			programmed_state = TISCI_MSG_VALUE_DEVICE_SW_STATE_RETENTION;
		} else {
			programmed_state = TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF;
		}

		switch (device_get_state(dev)) {
		case 0:
			current_state = TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF;
			break;
		case 1:
			current_state = TISCI_MSG_VALUE_DEVICE_HW_STATE_ON;
			break;
		default:
			current_state = TISCI_MSG_VALUE_DEVICE_HW_STATE_TRANS;
			break;
		}

		if (device_get_reset_iso(dev)) {
			resp->hdr.flags = TISCI_MSG_FLAG_DEVICE_RESET_ISO;
		}

		resp->context_loss_count        = context_loss_count;
		resp->resets                    = resets;
		resp->programmed_state          = programmed_state;
		resp->current_state             = current_state;
	}

	mmr_lock_all();

	return ret;
}

int32_t set_device_resets_handler(uint32_t *msg_recv)
{
	struct tisci_msg_set_device_resets_req *req =
		(struct tisci_msg_set_device_resets_req *) msg_recv;
	struct tisci_msg_set_device_resets_resp *resp =
		(struct tisci_msg_set_device_resets_resp *) msg_recv;
	struct device *dev = NULL;
	uint32_t id = req->id;
	uint32_t resets = req->resets;
	int32_t ret = SUCCESS;

	pm_trace(TRACE_PM_ACTION_MSG_RECEIVED, TISCI_MSG_SET_DEVICE_RESETS);
	pm_trace(TRACE_PM_ACTION_MSG_PARAM_DEV_CLK_ID, id);
	pm_trace(TRACE_PM_ACTION_MSG_PARAM_VAL, resets);

	resp->hdr.flags = 0U;

	mmr_unlock_all();

	ret = device_prepare_exclusive(req->hdr.host, id, NULL, &dev);

	if (ret == SUCCESS) {
		if (resets <= 3U) {
			ret = SUCCESS;
		} else {
			pm_trace(TRACE_PM_ACTION_INVALID_STATE, resets);
			ret = EFAIL;
		}
	}

	if (ret == SUCCESS) {
		device_set_resets(dev, resets);
	}

	mmr_lock_all();

	return ret;
}

int32_t device_drop_powerup_ref_handler(uint32_t *msg_recv)
{
	struct tisci_msg_device_drop_powerup_ref_resp *resp =
		(struct tisci_msg_device_drop_powerup_ref_resp *) msg_recv;
	int32_t ret = SUCCESS;

	pm_trace(TRACE_PM_ACTION_MSG_RECEIVED, TISCI_MSG_DEVICE_DROP_POWERUP_REF);

	resp->hdr.flags = 0U;

	mmr_unlock_all();

	devices_drop_power_up_ref();

	mmr_lock_all();

	return ret;
}
