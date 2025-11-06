/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CLK_HANDLER_H
#define CLK_HANDLER_H

/**
 * The IP does not require this clock, it can be disabled, regar`dless of the
 * state of the device
 */
#define CLOCK_SW_STATE_UNREQ	    0

/**
 * Allow the system controller to automatically manage the state of this clock.
 * If the device is enabled, then the clock is enabled. If the device is set to
 * off or retention, then the clock is internally set as not being required
 * by the device. This is the default state.
 */
#define CLOCK_SW_STATE_AUTO	    1

/** Configure the clock to be enabled, regardless of the state of the device. */
#define CLOCK_SW_STATE_REQ	    2

/** Allow this clock to be modified via spread spectrum clocking.
 *  \note: The SSC feature is currently not supported in System Firmware.
 */
#define CLOCK_ALLOW_SSC		      BIT(8)

/**
 * Enable input termination, this is only applicable to clock inputs
 * on the SoC pseudo-device, BOARD0.
 */
#define CLOCK_INPUT_TERM	      BIT(10)


/**
 * Allow this clock's frequency to be changed while it is running
 * so long as it is within the min/max limits.
 */
#define CLOCK_ALLOW_FREQ_CHANGE	      BIT(9)

/**
 *  \brief  Set Clock Handler PM Function.
 *
 *  \param  dev_id   Device ID
 *  \param  clk_id   Clock ID
 *  \param  enable   Enable (true) or disable (false) clock
 *  \return ret	     SUCCESS if the API executed successfully.
 *		     EFAIL   if the API failed to execute.
 */
int32_t set_clock_handler(uint32_t dev_id, uint32_t clk_id, bool enable);

/**
 *  \brief  Get Clock Handler PM Function.
 *
 *  \param  dev_id   Device ID
 *  \param  clk_id   Clock ID
 *  \return ret	     Clock state if successful, 0 if failed.
 */
int32_t get_clock_handler(uint32_t dev_id, uint32_t clk_id);

/**
 *  \brief  Set Clock Parent Handler PM Function.
 *
 *  \param  dev_id    Device ID
 *  \param  clk_id    Clock ID
 *  \param  parent_id Parent clock ID
 *  \return ret	      SUCCESS if the API executed successfully.
 *		      EFAIL   if the API failed to execute.
 */
int32_t set_clock_parent_handler(uint32_t dev_id, uint32_t clk_id, uint32_t parent_id);

/**
 *  \brief  Get Clock Parent Handler PM Function.
 *
 *  \param  dev_id    Device ID
 *  \param  clk_id    Clock ID
 *  \param  parent_id Pointer to store parent clock ID
 *  \return ret	      SUCCESS if the API executed successfully.
 *		      EFAIL   if the API failed to execute.
 */
int32_t get_clock_parent_handler(uint32_t dev_id, uint32_t clk_id, uint32_t *parent_id);

/**
 *  \brief  Get Number of Clock Parents Handler PM Function.
 *
 *  \param  dev_id   Device ID
 *  \param  clk_id   Clock ID
 *  \return ret	     Number of parents if successful, 0 if failed.
 */
int32_t get_num_clock_parents_handler(uint32_t dev_id, uint32_t clk_id);

/**
 *  \brief  Set clock frequency Handler PM Function.
 *
 *  \param  dev_id      Device ID
 *  \param  clk_id      Clock ID
 *  \param  target_freq Target frequency in Hz
 *  \return ret	        SUCCESS if the API executed successfully.
 *		        EFAIL   if the API failed to execute.
 */
int32_t set_freq_handler(uint32_t dev_id, uint32_t clk_id, uint64_t target_freq);

/**
 *  \brief  Get clock frequency Handler PM Function.
 *
 *  \param  dev_id   Device ID
 *  \param  clk_id   Clock ID
 *  \return ret	     Frequency in Hz if successful, 0 if failed.
 */
uint64_t get_freq_handler(uint32_t dev_id, uint32_t clk_id);

#endif /* CLK_HANDLER_H */
