#
# Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
#
# SPDX-License-Identifier: BSD-3-Clause
#

# We need to enable this for robust clocking
CONFIG_CLK_PLL_16FFT_FRACF_CALIBRATION := 1
$(eval $(call add_define,CONFIG_CLK_PLL_16FFT_FRACF_CALIBRATION))

BL31_SOURCES += ${PLAT_PATH}/common/drivers/pm/clock/clk_handler.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk.c    		\
		${PLAT_PATH}/common/drivers/pm/clock/clk_dev.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk_div.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk_fixed.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk_mux.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk_pll_16fft.c    \
		${PLAT_PATH}/common/drivers/pm/clock/clk_pllctrl.c    	\
		${PLAT_PATH}/common/drivers/pm/clock/clk_pll_deskew.c   \
		${PLAT_PATH}/common/drivers/pm/clock/clk_soc_hfosc0.c   \
		${PLAT_PATH}/common/drivers/pm/clock/clk_soc_lfosc0.c   \
		${PLAT_PATH}/common/drivers/pm/clock/pll.c    		\
		drivers/delay_timer/delay_timer.c

