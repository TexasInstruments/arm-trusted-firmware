/*
 * Copyright (c) 2026, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <lib/mmio.h>
#include <board_def.h>
#include <lpm_stub.h>
#include <standby.h>

#define MAIN_PSC_BASE		0x00400000
#define MAIN_PSC_MDCTL_BASE	0x00400A00
#define MAIN_PSC_MDSTAT_BASE	0x00400800
#define MAIN_PSC_PDCTL_BASE	0x00400300
#define MAIN_PSC_PDSTAT_BASE	0x00400200
#define MAIN_PSC_PTSTAT	(MAIN_PSC_BASE + PSC_PTSTAT)
#define MAIN_PSC_PTCMD		(MAIN_PSC_BASE + PSC_PTCMD)

#define PSC_PTCMD	0x120
#define PSC_PTSTAT	0x128

#define PSC_TIMEOUT_US  100000  /* 100ms timeout */

#define MAIN_PLL_MMR_CFG_BASE	(0x04060000UL)
#define WKUP_PLL_MMR_CFG_BASE   (0x04040000UL)

#define MAIN_PLL0_HSDIVx(x)	MAIN_PLL_MMR_CFG_BASE + 0x80 + (0x4 * x)
#define MAIN_PLL8_BASE MAIN_PLL_MMR_CFG_BASE + 0x1000 * 8
#define MAIN_PLL8_CTRL MAIN_PLL8_BASE + 0x20
#define MAIN_PLL17_BASE MAIN_PLL_MMR_CFG_BASE + 0x1000 * 17
#define MAIN_PLL17_CTRL MAIN_PLL17_BASE + 0x20
#define WKUP_MAIN_PLL0_HSDIVx(x)	WKUP_PLL_MMR_CFG_BASE + 0x80 + (0x4 * x)

#define LPSC_ADDR(lpsc_id) MAIN_PSC_MDSTAT_BASE + (4 * lpsc_id)
#define PSC_ADDR(psc_id) MAIN_PSC_PDSTAT_BASE + (4 * psc_id)

#define  EMIF_CTLCFG_DENALI_CTL_168 0x0F3082A0
#define  EMIF_CTLCFG_DENALI_CTL_169 0x0F3082A4
#define  EMIF_CTLCFG_DENALI_CTL_167 0x0F30829C

#define WKUP_CTRL_MMR_CFG5_CLKGATE_CTRL0 0x43054050

#define LPSC_COUNT 4

static unsigned int lpsc_id[] = {
	1,  /* LPSC_main_gp_test */
	2,  /* LPSC_main_gp_pbist0 */
	33, /* LPSC_mainip_pbist */
	39, /* LPSC_main_mpu_clst0_pbist */
};

static unsigned int psc_id[] = {0,0,3,4};

struct am62l_pm_state {
    uint32_t pll_hsdiv_val[11];
    uint32_t lpsc_value[LPSC_COUNT];
    uint32_t ddr_reg[3];
    uint32_t auto_clk_gate;
};

static volatile struct am62l_pm_state saved_state;

/*
 * Sets the requested state of required module and power domain.
 * This function:
 * 1. Checks if the requested states are already set
 * 2. Waits for any ongoing power state transitions to complete
 * 3. Programs the PDCTL and MDCTL registers with the new states
 * 4. Initiates the power state transition
 * 5. Waits for the transition to complete if powering on
 * 6. Logs the before and after states for debugging
 *
 * @pd_id: Power domain ID (e.g., PD_MPU_CLST, PD_MPU_CLST_CORE_0)
 * @md_id: Module ID (e.g., LPSC_MAIN_MPU_CLST, LPSC_MAIN_MPU_CLST_CORE_0)
 * @pd_state: Target power domain state (PSC_PD_ON or PSC_PD_OFF)
 * @md_state: Target module state (PSC_ENABLE, PSC_DISABLE, PSC_SYNCRESETDISABLE, etc.)
 */
void set_main_psc_state(uint32_t pd_id, uint32_t md_id, uint32_t pd_state, uint32_t md_state)
{
	uintptr_t mdctrl_ptr, mdstat_ptr, pdctrl_ptr, pdstat_ptr;
	volatile uint32_t mdctrl, mdstat, pdctrl, pdstat, psc_ptstat, psc_ptcmd;
	uint64_t tick_start, timeout_ticks;
	uint32_t ticks_per_us;

	// Calculate addresses with simplified approach
	mdctrl_ptr = MAIN_PSC_MDCTL_BASE + (4 * md_id);
	mdstat_ptr = MAIN_PSC_MDSTAT_BASE + (4 * md_id);
	pdctrl_ptr = MAIN_PSC_PDCTL_BASE + (4 * pd_id);
	pdstat_ptr = MAIN_PSC_PDSTAT_BASE + (4 * pd_id);

	// Use mmio_read_32 with simplified addresses
	mdctrl = mmio_read_32(mdctrl_ptr);
	mdstat = mmio_read_32(mdstat_ptr);
	pdctrl = mmio_read_32(pdctrl_ptr);
	pdstat = mmio_read_32(pdstat_ptr);

	INFO("%s: before: md_id=%d, mdstat=0x%x, pdstat=0x%x\n", __func__, md_id, mdstat, pdstat);
	if (((pdstat & 0x1) == pd_state) && ((mdstat & 0x1f) == md_state))
		return;

	// Calculate timeout parameters
	ticks_per_us = plat_get_syscnt_freq2() / 1000000;
	tick_start = (uint32_t)read_cntpct_el0();
	timeout_ticks = PSC_TIMEOUT_US * ticks_per_us;

	// wait for GOSTAT to clear
	psc_ptstat = mmio_read_32(MAIN_PSC_PTSTAT);

	while ((psc_ptstat & (0x1 << pd_id)) != 0) {
		if (((uint32_t)read_cntpct_el0() - tick_start) > timeout_ticks) {
			ERROR("PSC timeout waiting for initial GOSTAT to clear for md_id %d and pd_id %d\n",
			      md_id ,pd_id);
			break;
		}
		psc_ptstat = mmio_read_32(MAIN_PSC_PTSTAT);
	}

	// Set PDCTL NEXT to new state
	mmio_write_32(pdctrl_ptr, (pdctrl & ~(0x1)) | pd_state);
	// Set MDCTL NEXT to new state
	mmio_write_32(mdctrl_ptr, (mdctrl & ~(0x1f)) | md_state);
	// Start power transition by setting PTCMD Go to 1
	psc_ptcmd = mmio_read_32(MAIN_PSC_PTCMD);
	psc_ptcmd |= (0x1 << pd_id);
	mmio_write_32(MAIN_PSC_PTCMD, psc_ptcmd);
	// return early in case powering off
	// This prevents the core from timing out waiting for GOSTAT to clear
	if (md_state == PSC_SYNCRESETDISABLE)
		return;

	// Reset timeout for second wait
	tick_start = (uint32_t)read_cntpct_el0();

	// Initial read
	psc_ptstat = mmio_read_32(MAIN_PSC_PTSTAT);

	// Wait loop with timeout
	while ((psc_ptstat & (0x1 << pd_id)) != 0) {
		if (((uint32_t)read_cntpct_el0() - tick_start) > timeout_ticks) {
			ERROR("PSC timeout waiting for GOSTAT to clear for md_id %d and pd_id %d\n",md_id ,pd_id);
			break;
		}
		psc_ptstat = mmio_read_32(MAIN_PSC_PTSTAT);
	}

	//check states
	mdstat = mmio_read_32(mdstat_ptr);
	pdstat = mmio_read_32(pdstat_ptr);
	INFO("%s: after: md_id=%d, mdstat=0x%x, pdstat=0x%x\n", __func__, md_id, mdstat, pdstat);
}

void am62l_save_state()
{
    // pll value save
    for(int i=0;i<10;i++){
        saved_state.pll_hsdiv_val[i] = mmio_read_32(MAIN_PLL0_HSDIVx(i));
    }
   saved_state.pll_hsdiv_val[10] = mmio_read_32(MAIN_PLL8_CTRL);

    //lpsc value save
    for(int i=0;i<LPSC_COUNT;i++){
        saved_state.lpsc_value[i] = mmio_read_32(LPSC_ADDR(lpsc_id[i])) & 0x3U;
    }

    saved_state.ddr_reg[0] = mmio_read_32(EMIF_CTLCFG_DENALI_CTL_168);
    saved_state.ddr_reg[1] = mmio_read_32(EMIF_CTLCFG_DENALI_CTL_169);
    saved_state.ddr_reg[2] = mmio_read_32(EMIF_CTLCFG_DENALI_CTL_167);

    saved_state.auto_clk_gate = mmio_read_32(WKUP_CTRL_MMR_CFG5_CLKGATE_CTRL0);
}

void am62l_restore_state()
{
    // AUTO CLOCK GATING OFF
    mmio_write_32(WKUP_CTRL_MMR_CFG5_CLKGATE_CTRL0,saved_state.auto_clk_gate);

    // LPSC
    for(int i=0;i<LPSC_COUNT;i++){
        if(saved_state.lpsc_value[lpsc_id[i]]!=0){
        	set_main_psc_state(psc_id[i],lpsc_id[i],1,saved_state.lpsc_value[lpsc_id[i]]);
        }
    }

    /* Restore PLL */
    for(int i=0;i<10;i++){
        mmio_write_32(MAIN_PLL0_HSDIVx(i), saved_state.pll_hsdiv_val[i]);
    }
    mmio_write_32(MAIN_PLL8_CTRL,saved_state.pll_hsdiv_val[10]);
}

void am62l_low_latency_standby()
{
	// MAIN_PLL0
	mmio_write_32(MAIN_PLL0_HSDIVx(0), (saved_state.pll_hsdiv_val[0] & ~(0xff)) | 0xf);
	mmio_write_32(MAIN_PLL0_HSDIVx(5), (saved_state.pll_hsdiv_val[5] & ~(0xff)) | 0x4);
	mmio_write_32(MAIN_PLL0_HSDIVx(6), (saved_state.pll_hsdiv_val[6] & ~(0xff)) | 0x3);
	mmio_write_32(MAIN_PLL0_HSDIVx(7), (saved_state.pll_hsdiv_val[7] & ~(0xff)) | 0x5);
	mmio_write_32(MAIN_PLL0_HSDIVx(8), (saved_state.pll_hsdiv_val[8] & ~(0xff)) | 0x27);
	mmio_write_32(MAIN_PLL0_HSDIVx(9), (saved_state.pll_hsdiv_val[9] & ~(0x8000)));

	// A53 running off Bypass clock
	mmio_write_32(MAIN_PLL8_CTRL, saved_state.pll_hsdiv_val[10] | 0x80000000);

	// change the LPSC values only if they are not already disabled
	for(int i=0;i<LPSC_COUNT;i++){
		if(saved_state.lpsc_value[lpsc_id[i]]!=0){
			set_main_psc_state(psc_id[i],lpsc_id[i],1,2);
		}
	}

	//DDR AUTO SELF REFRESH
	mmio_write_32(EMIF_CTLCFG_DENALI_CTL_168,0x0000ff07);
	mmio_write_32(EMIF_CTLCFG_DENALI_CTL_169,0x0F0F00FF);
	mmio_write_32(EMIF_CTLCFG_DENALI_CTL_167,0x07074007);

	// AUTO CLOCK GATING
	mmio_write_32(WKUP_CTRL_MMR_CFG5_CLKGATE_CTRL0,0);

	return;
}
