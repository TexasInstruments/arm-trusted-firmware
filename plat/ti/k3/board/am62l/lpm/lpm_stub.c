/*
 * Copyright (c) 2024, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <lib/mmio.h>

#include <plat/common/platform.h>
#include "lpm_stub.h"
#include "pll_16fft_raw.h"
#include "lpm_trace.h"
#include "psc_raw.h"
#include <ti_sci.h>
#include "rtc.h"
#include "ddr.h"
#include <mailbox.h>
#include <bl31/bl31.h>
#include <board_def.h>

#define WFI_STATUS							(0x400)
#define MPU_TIFS_WFI_MASK					BIT(2)
#define WKUP0_EN							(0x4030U)
#define WKUP0_SRC							(0x4040U)
#define RST_CTRL							(0x4000U)
#define PMCTRL_SYS							(0x80)
#define WKUP_CTRL_PMCTRL_IO_0				(0x84)
#define WKUP_CTRL_PMCTRL_IO_1				(0x88)
#define WKUP_CTRL_DEEPSLEEP_CTRL			(0x160)
#define CANUART_WAKE_RESUME_KEY0_STAT		(0x3100U)
#define CANUART_WAKE_OFF_MODE				(0x1310U)
#define CANUART_WAKE_OFF_MODE_STAT1			(0x130CU)
#define CANUART_WAKE_OFF_MODE_STAT1_ENABLED (0x1U)
#define PD_DDR								2U
#define LPSC_MAIN_DDR_LOCAL					21U
#define LPSC_MAIN_DDR_CFG_ISO_N             22U
#define LPSC_MAIN_DDR_DATA_ISO_N            23U
#define MAIN_PSC_BASE						(0x400000UL)
#define PLLOFFSET(idx)						(0x1000 * (idx))
#define SCTLR_EL3_M_BIT						((uint32_t)1U << 0)

#define WKUP_CTRL_PMCTRL_IO_GLB_ENABLE_IO		1
#define WKUP_CTRL_DEEPSLEEP_CTRL_ENABLE_IO		(0x101U)
#define WKUP_CTRL_DEEPSLEEP_CTRL_DISABLE_IO		0
#define WKUP_CTRL_PMCTRL_IO_GLB_DISABLE_IO		0
#define WKUP_CTRL_PMCTRL_IO_0_ISOCLK_OVRD		BIT(0)
#define WKUP_CTRL_PMCTRL_IO_0_ISOOVR_EXTEND		BIT(4)
#define WKUP_CTRL_PMCTRL_IO_0_ISO_BYPASS		BIT(6)
#define WKUP_CTRL_PMCTRL_IO_0_WUCLK_CTRL    	BIT(8)
#define WKUP_CTRL_PMCTRL_IO_0_IO_ISO_STATUS 	BIT(25)
#define WKUP_CTRL_PMCTRL_IO_0_WUCLK_STATUS_ENABLED  1U
#define WKUP_CTRL_PMCTRL_IO_0_WUCLK_STATUS_DISABLED 0U
#define WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN   	BIT(16)
#define WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL   	BIT(24)
#define WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK (WKUP_CTRL_PMCTRL_IO_0_ISOCLK_OVRD	\
					  | WKUP_CTRL_PMCTRL_IO_0_ISOOVR_EXTEND	 \
					  | WKUP_CTRL_PMCTRL_IO_0_ISO_BYPASS	 \
					  | WKUP_CTRL_PMCTRL_IO_0_WUCLK_CTRL	 \
					  | WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN	 \
					  | WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL)

#define WKUP_CTRL_MMR_SEC_5_BASE				(0x43050000UL)
#define MAIN_PLL_MMR_BASE						(0x04060000UL)

/* counts of 1us delay for 10ms */
#define TIMEOUT_10MS                    10000U

/* Main PLL to be saved and restored */
__wkupsramdata struct pll_raw_data main_pll0 = {
.base = MAIN_PLL_MMR_BASE + PLLOFFSET(0U), };

__wkupsramdata struct pll_raw_data main_pll8 = {
.base = MAIN_PLL_MMR_BASE + PLLOFFSET(8U), };

__wkupsramdata struct pll_raw_data main_pll17 = {
.base = MAIN_PLL_MMR_BASE + PLLOFFSET(17U), };

/* Base addresses of main PLL structures to be saved and restored */
__wkupsramdata struct pll_raw_data *main_plls_save_rstr[3] = {
&main_pll0, &main_pll8, &main_pll17};

__wkupsramdata int num_main_plls_save_rstr = 3;

extern uint32_t k3_lpm_switch_stack(uintptr_t jump, uintptr_t stack, uint32_t arg);
extern void plat_invalidate_icache(void);
static void k3_lpm_jump_to_stub(uint32_t mode);

void k3_config_wake_sources(bool enable)
{
	uint32_t wake_up_src;

	if (enable) {
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_EN, 0x7FFFF);
	} else {
		wake_up_src = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_SRC);
		ERROR("Wake up src 0x%lx\n", (unsigned long)wake_up_src);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_EN, 0x00);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_SRC, wake_up_src);
	}
}

void k3_lpm_config_magic_words(uint32_t mode)
{
	if (mode == 0) {
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + CANUART_WAKE_OFF_MODE, 0xD5555555U);
	} else {
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + CANUART_WAKE_OFF_MODE, 0x6D555555U);
	}
}

bool k3_lpm_check_can_io_latch(void)
{
	return (mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + CANUART_WAKE_OFF_MODE_STAT1) & CANUART_WAKE_OFF_MODE_STAT1_ENABLED);
}

/**
 * @brief Save main domain pll configuration
 *
 */
__wkupsramfunc void save_main_pll(void)
{
	int i = 0;

	for (i = 0; i < num_main_plls_save_rstr; i++) {
		pll_save(main_plls_save_rstr[i]);
	}
}

/**
 * @brief Disable main domain plls
 *
 */
__wkupsramfunc void disable_main_pll(void)
{
	int i;

	for (i = 0; i < num_main_plls_save_rstr; i++) {
		pll_disable(main_plls_save_rstr[i]);
	}
}

/**
 * @brief Disable main domain plls
 *
 */
__wkupsramfunc void bypass_main_pll(void)
{
	int i;

	for (i = 0; i < num_main_plls_save_rstr; i++) {
		pll_bypass(main_plls_save_rstr[i]);
	}
}

/**
 * @brief Disable main domain plls
 *
 */
__wkupsramfunc void unbypass_main_pll(void)
{
	int i;

	for (i = 0; i < num_main_plls_save_rstr; i++) {
		pll_unbypass(main_plls_save_rstr[i]);
	}
}

/**
 * @brief Disable DDR LPSC
 *
 */
__wkupsramfunc void disable_ddr_lpsc(void)
{
	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_DATA_ISO_N,
			       MDCTL_STATE_SWRSTDISABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);

	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_CFG_ISO_N,
			       MDCTL_STATE_SWRSTDISABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);

	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_LOCAL,
			       MDCTL_STATE_SWRSTDISABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);
}

/**
 * @brief Enable DDR LPSC
 *
 */
__wkupsramfunc void enable_ddr_lpsc(void)
{
	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_LOCAL,
			       MDCTL_STATE_ENABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);

	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_CFG_ISO_N,
			       MDCTL_STATE_ENABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);

	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_DATA_ISO_N,
			       MDCTL_STATE_ENABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);
}

/**
 * @brief Restore main domain plls
 *
 */
__wkupsramfunc void restore_main_pll(void)
{
	int i;

	for (i = 0; i < num_main_plls_save_rstr; i++) {
		pll_restore(main_plls_save_rstr[i]);
	}
}

__wkupsramfunc void lpm_abort(void)
{
	volatile int a = 0x123;

	while (a) {
	}
}

/**
 * @brief Wait for TIFS to be in WFI
 *
 */
__wkupsramfunc bool lpm_sleep_wait_for_tifs_wfi(void)
{
	uint32_t reg;

	do {
		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WFI_STATUS);
		if ((reg & MPU_TIFS_WFI_MASK) == MPU_TIFS_WFI_MASK) {
			return true;
		}
	} while (1);
	return false;
}

/**
 * @brief Wait for secondary core to power off.
 *
 */
__wkupsramfunc bool lpm_wait_for_secondary_core_down(void)
{
	uint32_t reg;

	do {
		reg = mmio_read_32(MAIN_PSC_BASE +  0x8A4);
		if ((reg & 0x1F) == 0) {
			return true;
		}
	} while (1);
	return false;
}

/**
 * @brief Entry function for a53 stub
 *
 */
__wkupsramsuspendentry void k3_lpm_stub_entry(uint32_t mode)
{
	if (mode == 6) {
		/* Wait for a53_1 to turn off */
		lpm_wait_for_secondary_core_down();
		lpm_seq_trace(0x01);

		lpm_sleep_wait_for_tifs_wfi();
		lpm_seq_trace(0x2);

		/*Place DDR into self-refresh */
		put_ddr_in_rtc_lpm();
		lpm_seq_trace(0x4);

		disable_ddr_lpsc();
		lpm_seq_trace(0x5);

		save_main_pll();
		lpm_seq_trace(0x2);

		disable_main_pll();
		lpm_seq_trace(0x3);

		/* configure the pmic input */
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + PMCTRL_SYS, 0x0U);
		lpm_seq_trace(0x4);
		dsb();
		isb();

		for (;;)
			wfi();

	} else if (mode == 0) {

		/* Wait for a53_1 to turn off  */
		lpm_wait_for_secondary_core_down();
		lpm_seq_trace(0x01);

		save_main_pll();
		lpm_seq_trace(0x2);

		save_ddr_reg_configs();
		lpm_seq_trace(0x20);

		disable_ddr_lpsc();
		lpm_seq_trace(0x21);

		disable_main_pll();
		lpm_seq_trace(0x3);

		dsb();
		isb();
		lpm_seq_trace(0x5);

		for (;;) {
			wfi();
			lpm_seq_trace(0x6);
		}
	} else  {
		for (;;) {
			lpm_seq_trace(0x88);
		}
	}
}

int32_t k3_lpm_set_io_isolation(bool enable)
{
	int32_t ret = -55;
	uint32_t reg;

	if (enable) {
		mmio_write_32((WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_DEEPSLEEP_CTRL), WKUP_CTRL_DEEPSLEEP_CTRL_ENABLE_IO);

		/* Set global wuen */
		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg | WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN;
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0, reg);

		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg | WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN;
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1, reg);

		/* Set global isoin */
		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg | WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL;
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0, reg);

		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg | WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL;
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1, reg);

		/* Wait for wu clock state to be 1 */
		do {
			ret = -52;
			reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0);
			if ((reg & WKUP_CTRL_PMCTRL_IO_0_IO_ISO_STATUS) == WKUP_CTRL_PMCTRL_IO_0_IO_ISO_STATUS) {
				ret = 0;
				break;
			}
		} while (1);
		do {
			ret = -52;
			reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1);
			if ((reg & WKUP_CTRL_PMCTRL_IO_0_IO_ISO_STATUS) == WKUP_CTRL_PMCTRL_IO_0_IO_ISO_STATUS) {
				ret = 0;
				break;
			}
		} while (1);
	} else {
		/* Clear global wuen */
		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg & (~WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0, reg);

		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg & (~WKUP_CTRL_PMCTRL_IO_0_GLOBAL_WUEN);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1, reg);

		/* Clear global isoin */
		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg & (~WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_0, reg);

		reg = mmio_read_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1);
		reg = reg & WKUP_CTRL_PMCTRL_IO_0_WRITE_MASK;
		reg = reg & (~WKUP_CTRL_PMCTRL_IO_0_IO_ISO_CTRL);
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_PMCTRL_IO_1, reg);

		mmio_write_32((WKUP_CTRL_MMR_SEC_5_BASE + WKUP_CTRL_DEEPSLEEP_CTRL), WKUP_CTRL_DEEPSLEEP_CTRL_DISABLE_IO);

		ret = 0;
	}
	return ret;
}

/**
 * @brief send core resume message to TIFS
 *
 */
__wkupsramfunc void mailbox_send_message(void)
{
	uint32_t *dst_ptr = (void *)(MAILBOX_TX_START_REGION + 0x100);

	dst_ptr[0] = 0x0;
	dst_ptr[1] = 0x000A0304;
	dst_ptr[2] = 0x0;
	dst_ptr[3] = 0x0;
	dst_ptr[4] = 0x0;
	lpm_seq_trace(0xE3);

	mmio_write_32(TIFS_MAILBOX_BASE0 + TIFS_MAILBOX_MSG, (unsigned long)(void *)dst_ptr);
	lpm_seq_trace(0xAA);
}

__wkupsramfunc void k3_lpm_resume_c(void)
{
	lpm_seq_trace(0x5);
	restore_main_pll();

	lpm_seq_trace(0x6);
	enable_ddr_lpsc();

	lpm_seq_trace(0x7);
	restore_ddr_reg_configs();

	lpm_seq_trace(0x92);
	mailbox_send_message();

	for (;;) {
		wfi();
		lpm_seq_trace(0xB);
	}
}

void k3_suspend_to_ram(uint32_t mode)
{
	rtc_suspend();
	k3_lpm_jump_to_stub(mode);
}

#ifndef __ASSEMBLER__
IMPORT_SYM(unsigned long, __wkup_sram_start__, WKUP_SRAM_START);
IMPORT_SYM(unsigned long, __wkup_sram_end__, WKUP_SRAM_END);
IMPORT_SYM(unsigned long, __WKUP_SRAM_COPY_START__, WKUP_SRAM_COPY_START);
IMPORT_SYM(unsigned long, __wkup_sram_suspend_entry__, K3_SUSPEND_ENTRY);
#endif

/**
 * @brief function to jump to stub in wkup SRAM
 *
 */
static void k3_lpm_jump_to_stub(uint32_t mode)
{
	uintptr_t jump = (uintptr_t)K3_SUSPEND_ENTRY;
	uintptr_t stack = (uintptr_t)DEVICE_WKUP_SRAM_STACK_BASE;
	uint32_t sctlr;
	/* disable MMU */
	sctlr = (uint32_t)read_sctlr_el3();
	sctlr &= (uint32_t)~SCTLR_EL3_M_BIT;
	write_sctlr_el3((uint64_t)sctlr);
	INFO("k3_lpm_jump_to_stub x%lx\n", (unsigned long)K3_SUSPEND_ENTRY);

	k3_lpm_switch_stack(jump, stack, mode);
}

int32_t k3_lpm_stub_copy_to_sram(void)
{
	int ret = 0;
	uint32_t attr;
	uintptr_t sram_base_addr = (uintptr_t)DEVICE_WKUP_SRAM_BASE;
	size_t sram_len = DEVICE_WKUP_SRAM_SIZE;
	void *a53_stub_start = (void *)WKUP_SRAM_COPY_START;
	size_t a53_stub_len = WKUP_SRAM_END - WKUP_SRAM_START;

	if (a53_stub_len > sram_len) {
		ret = -1;
	}

	if (ret == 0) {
		attr = MT_MEMORY | MT_RW | MT_SECURE | MT_EXECUTE_NEVER;
		ret = xlat_change_mem_attributes(sram_base_addr, sram_len, attr);
	}

	if (ret == 0) {		
		INFO("stub copy 0x%lx  0x%lx  0x%lx\n", (unsigned long)WKUP_SRAM_START, (unsigned long)WKUP_SRAM_COPY_START, (unsigned long)WKUP_SRAM_END);
		memcpy((void *)sram_base_addr, a53_stub_start, a53_stub_len);
		flush_dcache_range((uint64_t)sram_base_addr, a53_stub_len);
		attr = MT_MEMORY | MT_RO | MT_SECURE | MT_EXECUTE;
		ret = xlat_change_mem_attributes(sram_base_addr, sram_len, attr);
	}
	return ret;
}
