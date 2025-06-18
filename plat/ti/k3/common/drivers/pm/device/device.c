/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <clk.h>
#include <device.h>
#include <device_clk.h>
#include <device_pm.h>
#include <hosts.h>
#include <lib/trace.h>
#include <stddef.h>
#include <types/errno.h>

/** The bitmask of currently enabled devgroups. */
static uint32_t pm_devgroups_enabled;


static bool devices_rw;

void pm_devgroup_set_enabled(devgrp_t groups)
{
	pm_devgroups_enabled |= (uint32_t) groups;
}

bool pm_devgroup_is_enabled(devgrp_t groups)
{
	return true;
}














static int32_t device_init(struct device *dev)
{
	const struct dev_data *data = get_dev_data(dev);
	int32_t ret = SUCCESS;
	uint16_t i;

	if ((data != NULL) && ((data->flags & DEVD_FLAG_DRV_DATA) != 0U) &&
	    ((data->flags & DEVD_FLAG_DO_INIT) != 0U)) {
		const struct drv *drvp = to_drv_data(data)->drv;

		if ((drvp != NULL) && (drvp->pre_init != NULL)) {
			ret = drvp->pre_init(dev);
		}
	}

	if (!devices_rw) {

		data = NULL;
	}

	if ((data != NULL) && (ret == SUCCESS)) {
		ret = soc_device_init(dev);
	}


	if ((data != NULL) && (ret == SUCCESS)) {
		for (i = 0U; i < data->n_clocks; i++) {
			device_clk_init(dev, i);
		}


		if (device_get_state(dev) != 0U) {
			device_set_state(dev, DEV_POWER_ON_ENABLED_HOST_IDX, true);
			device_set_retention(dev, true);
		}

		if (((data->flags & DEVD_FLAG_DRV_DATA) != 0U) &&
		    ((data->flags & DEVD_FLAG_DO_INIT) != 0U)) {
			const struct drv *drvp = to_drv_data(data)->drv;

			if ((drvp != NULL) && (drvp->post_init != NULL)) {
				ret = drvp->post_init(dev);
			}
		}
	}

	return ret;
}

int32_t devices_init(void)
{
	bool done;
	bool progress;
	bool contents;
	bool errors;
	dev_idx_t idx;
	int32_t ret = SUCCESS;

	errors = false;
	contents = false;
	do {
		struct device *dev;

		done = true;
		progress = false;
		dev = soc_devices;

		for (idx = 0U; idx < soc_device_count; idx++) {
			devgrp_t devgrp;

			dev = &soc_devices[idx];
			if (dev->initialized != 0U) {
				continue;
			}

			if (soc_device_data_arr[idx] == NULL) {
				continue;
			}


			if (soc_device_data_arr[idx]->pm_devgrp == PM_DEVGRP_DMSC) {
				devgrp = DEVGRP_DMSC;
			} else {
				devgrp = (devgrp_t) BIT(soc_device_data_arr[idx]->pm_devgrp - 1U);
			}

			if (!pm_devgroup_is_enabled(devgrp)) {
				continue;
			}

			contents = true;

			ret = device_init(dev);
			if (ret == -EDEFER) {
				done = false;
			} else {
				progress = true;
				dev->initialized = 1U;
				if (ret < 0) {
					pm_trace(TRACE_PM_ACTION_FAIL | TRACE_PM_ACTION_DEV_INIT,
						 ((((uint32_t) (-ret)) &
						   TRACE_PM_VAL_DEV_INIT_ERR_MASK) <<
						  TRACE_PM_VAL_DEV_INIT_ERR_SHIFT) |
						 ((((uint16_t) idx) &
						   TRACE_PM_VAL_DEV_INIT_DEVICE_ID_MASK) <<
						  TRACE_PM_VAL_DEV_INIT_DEVICE_ID_SHIFT));
					errors = true;
				}
			}
		}
	} while (!done && progress);

	if (devices_rw) {

		clk_drop_pwr_up_en();
	}





	for (idx = 0U; idx < soc_device_count; idx++) {
        struct device *dev = &soc_devices[idx];
        if (soc_device_data_arr[idx] == NULL ||
            !pm_devgroup_is_enabled(soc_device_data_arr[idx]->pm_devgrp == PM_DEVGRP_DMSC ?
                                  DEVGRP_DMSC :
                                  BIT(soc_device_data_arr[idx]->pm_devgrp - 1U))) {
            continue;
		}












        if (dev->initialized == 0U) {
            return -EDEFER;
	}
}


    if (progress) {
        if (errors == false) {
            pm_trace(TRACE_PM_ACTION_DEV_INIT, 0x0U);
        }
        ret = SUCCESS;
    } else if (contents) {
        ret = -EDEFER;
    } else {
        ret = SUCCESS;
    }

    return ret;
}

int32_t devices_init_rw(void)
{
	int32_t ret = SUCCESS;

	if (!devices_rw) {
		uint32_t i;





		for (i = 0U; i < soc_device_count; i++) {
			struct device *dev = &soc_devices[i];

			dev->initialized = 0U;
		}

		devices_rw = true;


		ret = devices_init();
	}

	return ret;
}


int32_t devices_deinit(uint8_t pm_devgrp)
{
	int32_t ret = SUCCESS;
	uint32_t i;
	bool flag_enabled;

	for (i = 0U; i < soc_device_count; i++) {
		if (soc_device_data_arr[i] == NULL) {
			continue;
		}
		if (soc_device_data_arr[i]->pm_devgrp == pm_devgrp) {
			struct device *dev = &soc_devices[i];
			const struct dev_data *data = get_dev_data(dev);

			flag_enabled = (dev->flags & DEV_FLAG_ENABLED_MASK) != 0UL;

			dev->flags &= ~DEV_FLAG_RETENTION;

			if ((dev->initialized != 0U) && flag_enabled) {
				device_disable(dev, true);
				dev->initialized = 0;
				dev->flags &= (uint32_t) ~DEV_FLAG_ENABLED_MASK;
			}

			if ((data != NULL) && ((data->flags & DEVD_FLAG_DRV_DATA) != 0U) &&
			    ((data->flags & DEVD_FLAG_DO_INIT) != 0U)) {
				const struct drv *drvp = to_drv_data(data)->drv;

				if ((drvp != NULL) && (drvp->uninit != NULL)) {
					drvp->uninit(dev);
				}
			}
		}
	}

	return ret;
}


int32_t devices_deinit_flags(void)
{
	int32_t ret = SUCCESS;
	uint32_t i;
	struct device *dev;

	for (i = 0U; i < soc_device_count; i++) {
		if (soc_device_data_arr[i] == NULL) {
			continue;
		}
		dev = &soc_devices[i];
		dev->exclusive = 0;

		if ((dev->flags != 0U) && (dev->initialized != 0U)) {
			dev->flags = 0U;
			device_clear_flags(dev);
			dev->initialized = 0;
		}
	}
	return ret;
}

void devices_drop_power_up_ref(void)
{
	dev_idx_t idx;

	for (idx = 0U; idx < soc_device_count; idx++) {
		struct device *dev = soc_devices + idx;

		if (dev->initialized != 0U) {
			device_set_state(dev, DEV_POWER_ON_ENABLED_HOST_IDX, false);
		}
	}
}
