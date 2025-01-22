/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>

#include <device.h>
#include <device_handler.h>
#include <device_pm.h>
#include <device_prepare.h>
#include <host_idx_mapping.h>
#include <hosts.h>
#include <types/errno.h>

int32_t set_device_handler(uint32_t dev_id, bool enable)
{
	struct device *dev = NULL;
	uint32_t flags = 0U;
	uint8_t host_id = HOST_ID_TIFS;
	bool retention;
	int32_t ret = SUCCESS;
	uint8_t host_idx;
	uint8_t state;

	if (enable) {
		state = DEVICE_SW_STATE_ON;
	} else {
		state = DEVICE_SW_STATE_AUTO_OFF;
	}

	VERBOSE("SET_DEVICE: dev_id=%d state=%d\n", dev_id, state);

	ret = device_prepare_exclusive(host_id, dev_id, &host_idx, &dev);
	if (ret == SUCCESS) {
		switch (state) {
		case DEVICE_SW_STATE_AUTO_OFF:
			enable = false;
			retention = false;
			break;
		case DEVICE_SW_STATE_RETENTION:
			enable = false;
			retention = true;
			break;
		case DEVICE_SW_STATE_ON:
			enable = true;
			retention = true;
			break;
		default:
			VERBOSE("INVALID_STATE: state=%d\n", state);
			ret = -EINVAL;
			break;
		}
	}

	if (ret == SUCCESS) {
		if ((flags & DEVICE_EXCLUSIVE) != 0UL) {
			/* Make sure no one else has the device enabled */
			uint64_t mask = DEV_FLAG_ENABLED_MASK;
			uint64_t enabled;
			/* It's ok if we already have the device enabled */
			mask &= ~DEV_FLAG_ENABLED(host_idx);
			/* It's also ok if the device in on due to power up en */
			mask &= ~((uint64_t)DEV_FLAG_POWER_ON_ENABLED);
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
#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
				{
					uint32_t enabled_host_id = 0U;
					/*
					 * Do a reverse lookup. Find host ID from
					 * host index.
					 */
					if (i != (sizeof(enabled) * 8U)) {
						uint8_t j;

						for (j = 0U; j < soc_host_indexes_count; j++) {
							if (soc_host_indexes[j] == i) {
								enabled_host_id = j;
								break;
							}
						}
					}
					VERBOSE("EXCLUSIVE_BUSY: dev_id=%d req_host=%d holder_host=%d\n",
						dev_id, host_id, enabled_host_id);
				}
#endif
				ret = -EINVAL;
			}
		}
	}

	if (ret == SUCCESS) {
		if ((flags & DEVICE_EXCLUSIVE) != 0UL) {
			/* Only this host may modify device */
			dev->exclusive = host_idx;
		} else {
			/* Allow any host to modify device */
			dev->exclusive = 0U;
		}

		if ((flags & DEVICE_RESET_ISO) != 0UL) {
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

	return ret;
}

bool get_device_handler(uint32_t dev_id)
{
	struct device *dev = NULL;
	uint8_t host_id = HOST_ID_TIFS;
	int32_t ret = SUCCESS;
	uint8_t host_idx;
	bool is_enabled = false;

	VERBOSE("GET_DEVICE: dev_id=%d\n", dev_id);

	ret = device_prepare_nonexclusive(host_id, dev_id, &host_idx, &dev);
	if (ret == SUCCESS) {
		uint8_t programmed_state;
		uint8_t current_state;

		if ((dev->flags & DEV_FLAG_ENABLED(host_idx)) != 0UL) {
			programmed_state = DEVICE_SW_STATE_ON;
		} else if ((dev->flags & DEV_FLAG_RETENTION) != 0UL) {
			programmed_state = DEVICE_SW_STATE_RETENTION;
		} else {
			programmed_state = DEVICE_SW_STATE_AUTO_OFF;
		}

		switch (device_get_state(dev)) {
		case 0:
			current_state = DEVICE_HW_STATE_OFF;
			break;
		case 1:
			current_state = DEVICE_HW_STATE_ON;
			break;
		default:
			current_state = DEVICE_HW_STATE_TRANS;
			break;
		}

		if ((programmed_state == (uint8_t)DEVICE_SW_STATE_ON) &&
		   (current_state == (uint8_t)DEVICE_HW_STATE_ON)) {
			is_enabled = true;
		} else {
			is_enabled = false;
		}
	} else {
		is_enabled = false;
	}

	return is_enabled;
}
