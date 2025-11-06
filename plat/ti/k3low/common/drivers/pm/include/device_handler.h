/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DEVICE_HANDLER_H
#define DEVICE_HANDLER_H

/**
 * Claim a device exclusively. When passed with STATE_RETENTION or STATE_ON,
 * it will claim the device exclusively. If another host already has this
 * device set to STATE_RETENTION or STATE_ON, the message will fail. Once
 * successful, other hosts attempting to set STATE_RETENTION or STATE_ON
 * will fail.
 */
#define DEVICE_EXCLUSIVE	      BIT(10)

/** Enable reset isolation for this device. */
#define DEVICE_RESET_ISO	      BIT(9)

/**
 * Used by TISCI_MSG_SET_DEVICE to turn device off when possible. This
 * must be used in conjunction with SoC dependencies to identify the
 * overall power domain state being achieved.
 */
#define DEVICE_SW_STATE_AUTO_OFF    0

/** Used by TISCI_MSG_SET_DEVICE to disable device but keep in retention. */
#define DEVICE_SW_STATE_RETENTION   1

/** Used by TISCI_MSG_SET_DEVICE to turn device on for usage. */
#define DEVICE_SW_STATE_ON	    2

/**
 * TISCI_MSG_GET_DEVICE sets this as current state to indicate device
 * is off.
 */
#define DEVICE_HW_STATE_OFF	    0

/**
 * TISCI_MSG_GET_DEVICE sets this as current state to indicate device
 * is on.
 */
#define DEVICE_HW_STATE_ON	    1

/**
 * TISCI_MSG_GET_DEVICE sets this as current state to indicate device is
 * transitioning between states. When a device stays in this state it is
 * typically due to the fact that some resource that the device is
 * dependent on (example IRQs) are pending preventing completion of
 * hardware handshake. Please refer to Technical Reference Manual for
 * additional information.
 */
#define DEVICE_HW_STATE_TRANS	    2

/**
 *  \brief  Set Device State Handler PM Function.
 *
 *  \param  dev_id   Device ID to set state for
 *  \param  enable   true to enable device, false to disable
 *  \return ret      SUCCESS if the API executed successfully.
 *                   EFAIL   if the API failed to execute.
 */
int32_t set_device_handler(uint32_t dev_id, bool enable);

/**
 *  \brief  Get Device State Handler PM Function.
 *
 *  \param  dev_id      Device ID to get state for
 *  \return is_enabled  true if enabled, false if disable
 */
bool get_device_handler(uint32_t dev_id);

#endif /* DEVICE_HANDLER_H */
