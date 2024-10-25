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

#define WFI_STATUS                  		(0x400)
#define MPU_TIFS_WFI_MASK           		BIT(2)
#define WKUP0_EN                    		(0x4030U)
#define RST_CTRL 							(0x4000U)
#define PMCTRL_SYS							(0x80)
#define CANUART_WAKE_RESUME_KEY0_STAT   	(0x3100U)
#define CANUART_WAKE_OFF_MODE				(0x1310U)
#define CANUART_WAKE_OFF_MODE_STAT1			(0x130CU)
#define CANUART_WAKE_OFF_MODE_STAT1_ENABLED (0x1U)
#define PD_DDR                                	2U
#define LPSC_MAIN_DDR_LOCAL                    	21U
#define LPSC_MAIN_DDR_CFG_ISO_N                 22U
#define LPSC_MAIN_DDR_DATA_ISO_N                23U
#define MAIN_PSC_BASE  							(0x400000UL)
#define PLLOFFSET(idx) 					(0x1000 * (idx))
#define SCTLR_EL3_M_BIT				((uint32_t)1U << 0)

/* counts of 1us delay for 10ms */
#define TIMEOUT_10MS                    10000U
typedef void (*mailbox_entrypoint_t)(void);

/* Main PLL to be saved and restored */
__wkupsramdata struct pll_raw_data main_pll0 =
{ .base = MAIN_PLL_MMR_BASE + PLLOFFSET(0U), };

__wkupsramdata struct pll_raw_data main_pll8 =
{ .base = MAIN_PLL_MMR_BASE + PLLOFFSET(8U), };

__wkupsramdata struct pll_raw_data main_pll17 =
{ .base = MAIN_PLL_MMR_BASE + PLLOFFSET(17U), };

/* Base addresses of main PLL structures to be saved and restored */
__wkupsramdata struct pll_raw_data *main_plls_save_rstr[3] =
{ &main_pll0, &main_pll8, &main_pll17};

__wkupsramdata int num_main_plls_save_rstr = 3;

extern uint32_t k3_lpm_switch_stack(uintptr_t jump, uintptr_t stack, uint32_t arg);
extern void plat_invalidate_icache(void);
static void k3_lpm_jump_to_stub(void);

void k3_config_wake_sources(bool enable)
{
	if (enable) {
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_EN, 0x7FFFF);
	} else {
		mmio_write_32(WKUP_CTRL_MMR_SEC_5_BASE + WKUP0_EN, 0x00);
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
__wkupsramfunc void Disable_DDR_LPSC(void)
{
	psc_raw_lpsc_set_state(MAIN_PSC_BASE, LPSC_MAIN_DDR_DATA_ISO_N,
			       MDCTL_STATE_SWRSTDISABLE, 0);
	psc_raw_pd_initiate(MAIN_PSC_BASE, PD_DDR);
	psc_raw_pd_wait(MAIN_PSC_BASE, PD_DDR);

	psc_raw_lpsc_set_state(MAIN_PSC_BASE,LPSC_MAIN_DDR_CFG_ISO_N ,
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
__wkupsramfunc void Enable_DDR_LPSC(void)
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


__wkupsramfunc void lpm_abort()
{
	volatile int a = 0x123;
	while(a) {
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
 * @brief Entry function for a53 stub
 * 
 */
__wkupsramsuspendentry void k3_lpm_stub_entry(uint32_t mode)
{
	if (mode == 5) {
		lpm_sleep_wait_for_tifs_wfi();
		lpm_seq_trace(0x2);

		//Place DDR into self-refresh
		put_ddr_in_rtc_lpm();
		lpm_seq_trace(0x4);

		Disable_DDR_LPSC();
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

		save_main_pll();	
		lpm_seq_trace(0x2);

		save_ddr_reg_configs();
		lpm_seq_trace(0x20);

		Disable_DDR_LPSC();
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
			

	}

}

/**
 * @brief send core resume message to TIFS
 * 
 */
__wkupsramfunc void mailbox_send_message()
{
	uint32_t * dst_ptr = (void*) (AM62L_RSVD_SRAM_BASE + 0x100);
	dst_ptr[0] = 0x0;
	dst_ptr[1] = 0x000A0304;
	dst_ptr[2] = 0x0;
	dst_ptr[3] = 0x0;
	dst_ptr[4] = 0x0;	
	lpm_seq_trace(0xE3);
	
	mmio_write_32(TIFS_MAILBOX_BASE0 + TIFS_MAILBOX_MSG, (long unsigned int)(void*)dst_ptr);
	lpm_seq_trace(0xAA);
}

__wkupsramfunc void k3_lpm_resume_c(void)
{
	lpm_seq_trace(0x5);
	restore_main_pll();

	lpm_seq_trace(0x6);
	Enable_DDR_LPSC();
	
	lpm_seq_trace(0x7);
	restore_ddr_reg_configs();

	lpm_seq_trace(0x92);
	mailbox_send_message();

	for (;;) {
		wfi();
		lpm_seq_trace(0xB);
	}
}

void k3_suspend_to_ram(void)
{
	rtc_suspend();
	k3_lpm_jump_to_stub();
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
static void k3_lpm_jump_to_stub(void)
{
	uintptr_t jump = (uintptr_t) K3_SUSPEND_ENTRY;
	uintptr_t stack = (uintptr_t) DEVICE_WKUP_SRAM_STACK_BASE;
	uint32_t sctlr;
	uint32_t mode = get_low_power_mode();
	// TODO: Clear .bss and stack section 
	/* disable MMU */
	sctlr = (uint32_t) read_sctlr_el3();
	sctlr &= (uint32_t) ~SCTLR_EL3_M_BIT;
	write_sctlr_el3((uint64_t) sctlr);
	NOTICE("k3_lpm_jump_to_stub x%lx \n",(long unsigned int)K3_SUSPEND_ENTRY);

	k3_lpm_switch_stack(jump, stack, mode);
}

int32_t k3_lpm_stub_copy_to_sram(void)
{
	int ret = 0;
	uint32_t attr;
	uintptr_t sram_base_addr = (uintptr_t) DEVICE_WKUP_SRAM_BASE;
	size_t sram_len = DEVICE_WKUP_SRAM_SIZE;
	void * a53_stub_start = (void *) WKUP_SRAM_COPY_START;
	size_t a53_stub_len = WKUP_SRAM_END - WKUP_SRAM_START;

	if (a53_stub_len > sram_len) {
		ret = -1;
	}

	if (ret == 0) {
		attr = MT_MEMORY | MT_RW | MT_SECURE | MT_EXECUTE_NEVER;
		ret = xlat_change_mem_attributes(sram_base_addr, sram_len, attr);
	}

	if (ret == 0) {
		NOTICE("doing test final memcopy 0x%lx  0x%lx  0x%lx \n",(long unsigned int)WKUP_SRAM_START, (long unsigned int)WKUP_SRAM_COPY_START, (long unsigned int)WKUP_SRAM_END);

		memcpy((void *)sram_base_addr, a53_stub_start, a53_stub_len);
		flush_dcache_range((uint64_t) sram_base_addr, a53_stub_len);

		attr = MT_MEMORY | MT_RO | MT_SECURE | MT_EXECUTE;
		ret = xlat_change_mem_attributes(sram_base_addr, sram_len, attr);
	}
	
	return ret;
}
