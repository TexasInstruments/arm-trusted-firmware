/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>

#include <device.h>
#include <device_prepare.h>
#include <host_idx_mapping.h>
#include <types/errno.h>

static int32_t device_prepare(uint8_t host_id, uint32_t id, uint8_t *host_idx,
			      struct device **dev, bool exclusive)
{
	int32_t ret = SUCCESS;
	struct device *local_device = NULL;
	uint8_t local_host_idx = HOST_IDX_NONE;

	local_device = device_api_lookup(id);
	if (!local_device) {
		VERBOSE("BAD_DEVICE: dev_id=%d\n", id);
		ret = -EINVAL;
	}

	if (ret == SUCCESS) {
		if (local_device->initialized == 0U) {
			ret = -EINVAL;
		}
	}

	if (ret == SUCCESS) {
		local_host_idx = host_idx_lookup(host_id);
		if (local_host_idx == HOST_IDX_NONE) {
			ret = -EINVAL;
		}
	}

	if ((ret == SUCCESS) && exclusive) {
		if ((local_device->exclusive != 0U) &&
		    (local_device->exclusive != local_host_idx)) {
			VERBOSE("EXCLUSIVE_DEVICE: dev_id=%d holder_host=%d\n",
				id, host_id);
			ret = -EINVAL;
		}
	}

	if (dev != NULL) {
		*dev = local_device;
	}
	if (host_idx != NULL) {
		*host_idx = local_host_idx;
	}

	return ret;
}

int32_t device_prepare_exclusive(uint8_t host_id, uint32_t id, uint8_t *host_idx,
				 struct device **device_ptr)
{
	int32_t ret;

	/* Ensure devices are fully initialized to allow modification */
	ret = devices_init_rw();

	if (ret == SUCCESS) {
		ret = device_prepare(host_id, id, host_idx, device_ptr, true);
	}
	return ret;
}

int32_t device_prepare_nonexclusive(uint8_t host_id, uint32_t id, uint8_t *host_idx,
				    struct device **device_ptr)
{
	return device_prepare(host_id, id, host_idx, device_ptr, false);
}
