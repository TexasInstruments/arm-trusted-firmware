/*
 * Copyright (c) 2024-2026, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LPM_STB_H
#define LPM_STB_H

#include <plat/common/platform.h>

/**
 * @brief Function to copy A53 stub from DDR to WKUP SRAM
 *
 * @return 0 on success, error code otherwise
 */
int32_t k3low_lpm_stub_copy_to_sram(void);

/**
 * @brief Low power mode resume c function
 */
void k3low_lpm_resume_c(void);

/**
 * @brief Suspending the device into S2R state
 *
 * @param mode LPM mode to enter
 */
void k3low_suspend_to_ram(uint32_t mode);

/**
 * @brief Enable or disable low power mode wake up sources
 *
 * @param enable Boolean to choose between enable and disable
 */
void k3low_config_wake_sources(bool enable);

/**
 * @brief Function to jump to A53 stub in WKUP SRAM during low power mode
 *
 * @param mode LPM mode to enter
 */
void k3low_lpm_stub_entry(uint32_t mode);

/**
 * @brief Function to program the magic words in WKUP CTRL MMR
 *
 * @param mode LPM mode to enter
 */
void k3low_lpm_config_magic_words(uint32_t mode);

/**
 * @brief Function to check whether CAN IO magic words are latched
 *
 * @return true if latched, false otherwise
 */
bool k3low_lpm_check_can_io_latch(void);

/**
 * @brief Function to set/remove the IO isolation
 *
 * @param enable Boolean to set or remove isolation
 * @return 0 on success, error code otherwise
 */
int32_t k3low_lpm_set_io_isolation(bool enable);

/**
 * @brief Abort function during lpm sequence
 */
__wkupsramfunc void k3low_lpm_abort(void);

/**
 * @brief Find the pin responsible for latest system wakeup by iterating through PADCONF registers
 * and checking if WKUP_EVT flag is set
 */
void k3low_find_padconf_wakeup_pin(void);

/**
 * @brief Return the last entered low power mode
 *
 * @return last entered low power mode
 */
uint32_t k3low_get_lpm_mode(void);

/**
 * @brief Return the wakeup source register value captured on resume
 *
 * @return WKUP CTRL MMR WKUP0_SRC register value from the last wakeup
 */
uint32_t k3low_get_wakeup_src(void);

/**
 * @brief Return the padconf pin number responsible for the latest system wakeup
 *
 * @return wakeup pin number
 */
uint32_t k3low_get_wakeup_pin(void);

/**
 * @brief Put wakeup information (last low power mode, wakeup source, wakeup pin) into MAIN CTRL
 * MMR IPC_SET register.
 */
void k3low_lpm_mailbox_wakeup_info(void);

#endif /* LPM_STB_H */
