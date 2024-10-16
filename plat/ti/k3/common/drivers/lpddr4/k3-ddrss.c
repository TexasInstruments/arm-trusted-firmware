// SPDX-License-Identifier: BSD-3-Clause
/*
 * Wrapper for Cadence DDR Driver
 *
 * Copyright (C) 2018-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <platform_def.h>

#include <arch.h>
#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>

#include <k3_console.h>

#include "lpddr4_obj_if.h"
#include "lpddr4_if.h"
#include "lpddr4_structs_if.h"
#include "lpddr4_ctl_regs.h"
#include "am62l_lpddr4.h"

lpddr4_privatedata lpddr4_priv_data;

struct k3_ddrss_desc {
	void *ddrss_ss_cfg;
	void *ddrss_ctrl_mmr;
	void *ddrss_ctl_cfg;
	uint32_t ddr_freq0;
	uint32_t ddr_freq1;
	uint32_t ddr_freq2;
	uint32_t ddr_fhs_cnt;
	uint32_t dram_class;
	uint32_t instance;
	lpddr4_obj *driverdt;
	lpddr4_config config;
	lpddr4_privatedata pd;
	uint64_t ecc_reserved_space;
	bool ti_ecc_enabled;
};

static struct k3_ddrss_desc ddrss;
const uint32_t lpddr4_ctl_data[] = {
#include "am62l_ddr_ctl.h"
};

const uint32_t lpddr4_pi_data[] = {
#include "am62l_ddr_pi.h"
};

const uint32_t lpddr4_phy_data[] = {
#include "am62l_ddr_phy.h"
};

#define DDRSS_CTL_CFG 0x0f308000
#define DDRSS_CTRL_MMR_L 0x43014000
#define DDRSS_CTRL_MMR 0x43040000
#define DDRSS_SS_CFG 0x0f300000
#define PLL_CFG 0x04060000
#define DDRSS_PI_REGISTER_BLOCK__OFFS   0x2000
#define DDRSS_PI_87__SFR_OFFS   0x15C
#define DDRSS_PI_83__SFR_OFFS   0x14C
#define DDRSS_CTL_350__SFR_OFFS 0x578
#define DDRSS_CTL_342__SFR_OFFS 0x558

// #define DDRSS_PLL_FHS_CNT 6
#define DDRSS_PLL_FREQUENCY_0 25000000
// #define DDRSS_PLL_FREQUENCY_1 400000000
// #define DDRSS_PLL_FREQUENCY_2 400000000

#define DDR32SS_PMCTRL	 				(0x1000U)
#define CANUART_WAKE_OFF_MODE_STAT		(0x1318U)
#define RTC_ONLY_PLUS_DDR_MAGIC_WORD 	(0x6D555555U)

#define GP_CORE_CTL     0
#define PD_CRYPTO       1
#define PD_DDR          2
#define PD_MAIN_IP      3
#define PD_MPU_CLST     4
#define PD_MPU_CLST_CORE_0      5
#define PD_MPU_CLST_CORE_1      6
#define PD_PER                  9

#define LPSC_MAIN_GP_ALWAYSON           0
#define LPSC_MAIN_GP_TEST		1
#define LPSC_MAIN_GP_PBIST		2
#define LPSC_MAIN_GP_ISO0_N		3
#define LPSC_MAIN_GP_ISO1_N		4
#define LPSC_MAIN_GP_TIFS		5
#define LPSC_MAIN_GP_USB0		7
#define LPSC_MAIN_GP_USB0_ISO_N         8
#define LPSC_MAIN_GP_USB1		9
#define LPSC_MAIN_GP_USB1_ISO_N         10
#define LPSC_MAIN_GP_DPHY_TX            11
#define LPSC_MAIN_GP_WKPERI		18
#define LPSC_MAIN_CRYPTO		19
#define LPSC_MAIN_DDR_LOCAL		21
#define LPSC_MAIN_DDR_CFG_ISO_N         22
#define LPSC_MAIN_DDR_DATA_ISO_N        23
#define LPSC_MAIN_IP_COMMON		24
#define LPSC_MAIN_IP_DSS		25
#define LPSC_MAIN_IP_DSI		26
#define LPSC_MAIN_IP_EMMC8B		27
#define LPSC_MAIN_IP_EMMC4B0            28
#define LPSC_MAIN_IP_EMMC4B1            29
#define LPSC_MAIN_IP_CPSW		30
#define LPSC_MAIN_IP_GIC		32
#define LPSC_MAIN_IP_PBIST		33
#define LPSC_MAIN_MPU_CLST		38
#define LPSC_MAIN_MPU_CLST_PBIST        39
#define LPSC_MAIN_MPU_CLST_CORE_0       40
#define LPSC_MAIN_MPU_CLST_CORE_1       41
#define LPSC_MAIN_PER_COMMON            44
#define LPSC_MAIN_PER_MCASP0            45
#define LPSC_MAIN_PER_MCASP1            46
#define LPSC_MAIN_PER_MCASP2            47
#define LPSC_MAIN_PER_XSPI		48
#define LPSC_MAIN_PER_MCAN0		49
#define LPSC_MAIN_PER_MCAN1		50
#define LPSC_MAIN_PER_MCAN2		51
#define LPSC_MAIN_PER_GPMC		52

#define PSC_SYNCRESETDISABLE		(0x0)
#define PSC_SYNCRESET			(0x1)
#define PSC_DISABLE			(0x2)
#define PSC_ENABLE			(0x3)
#define PSC_PD_OFF			(0x0)
#define PSC_PD_ON			(0x1)

#define LPSC_DDR16SS0 21
#define LPSC_EMIF_CFG 22
#define LPSC_EMIF_DATA 23

// #define PSC_PTCMD	(0x120)
// #define PSC_PTSTAT  (0x128)
#define MAIN_PSC_BASE 0x00400000
#define MAIN_PSC_MDCTL_BASE 0x00400A00
#define MAIN_PSC_MDSTAT_BASE 0x00400800
#define MAIN_PSC_PDCTL_BASE 0x00400300
#define MAIN_PSC_PDSTAT_BASE 0x00400200
#define MAIN_PSC_PTSTAT (MAIN_PSC_BASE + PSC_PTSTAT)
#define MAIN_PSC_PTCMD (MAIN_PSC_BASE + PSC_PTCMD)

#define PSC_PTCMD               0x120
#define PSC_PTCMD_H             0x124
#define PSC_PTSTAT              0x128
#define PSC_PTSTAT_H            0x12C
#define PSC_PDSTAT              0x200
#define PSC_PDCTL               0x300
#define PSC_MDSTAT              0x800
#define PSC_MDCTL               0xa00

#define PDCTL_STATE_MASK                0x1
#define PDCTL_STATE_OFF                 0x0
#define PDCTL_STATE_ON                  0x1

#define MDSTAT_STATE_MASK               0x3f
#define MDSTAT_BUSY_MASK                0x30
#define MDSTAT_STATE_SWRSTDISABLE       0x0
#define MDSTAT_STATE_ENABLE             0x3

#define TH_MACRO_EXP(fld, str) (fld ## str)
#define TH_FLD_MASK(fld)  TH_MACRO_EXP(fld, _MASK)
#define TH_FLD_SHIFT(fld) TH_MACRO_EXP(fld, _SHIFT)
#define TH_FLD_WIDTH(fld) TH_MACRO_EXP(fld, _WIDTH)
#define TH_FLD_WOCLR(fld) TH_MACRO_EXP(fld, _WOCLR)
#define TH_FLD_WOSET(fld) TH_MACRO_EXP(fld, _WOSET)

#define str(s) #s
#define xstr(s) str(s)

#define  CTL_SHIFT 11
#define  PHY_SHIFT 11
#define  PI_SHIFT 10

#define TH_OFFSET_FROM_REG(REG, SHIFT, offset) do { \
		char *i, *pstr = xstr(REG); offset = 0; \
		for (i = &pstr[SHIFT]; *i != '\0'; ++i) { \
			offset = offset * 10 + (*i - '0'); } \
} while (0)

static void set_main_psc_state(uint32_t pd_id, uint32_t md_id, uint32_t pd_state, uint32_t md_state)
{
	uint32_t *mdctrl_ptr;
	volatile uint32_t mdctrl;
	uint32_t *mdstat_ptr;
	volatile uint32_t mdstat;
	uint32_t *pdctrl_ptr;
	volatile uint32_t pdctrl;
	uint32_t *pdstat_ptr;
	volatile uint32_t pdstat;
	volatile uint32_t psc_ptstat;
	volatile uint32_t psc_ptcmd;

	mdctrl_ptr = (uint32_t *)(uint64_t)((MAIN_PSC_MDCTL_BASE + (4 * md_id)));
	mdctrl = (uint32_t)*((uint32_t *)mdctrl_ptr);
	mdstat_ptr = (uint32_t *)(uint64_t)((MAIN_PSC_MDSTAT_BASE + (4 * md_id)));
	mdstat = (uint32_t)*((uint32_t *)mdstat_ptr);
	pdctrl_ptr = (uint32_t *)(uint64_t)((MAIN_PSC_PDCTL_BASE + (4 * pd_id)));
	pdctrl = (uint32_t)*((uint32_t *)pdctrl_ptr);
	pdstat_ptr = (uint32_t *)(uint64_t)((MAIN_PSC_PDSTAT_BASE + (4 * pd_id)));
	pdstat = (uint32_t)*((uint32_t *)pdstat_ptr);

	INFO("%s: before: md_id=%d, mdstat=0x%x, pdstat=0x%x \n", __func__, md_id, mdstat, pdstat);

	if (((pdstat & 0x1) == pd_state) && ((mdstat & 0x1f) == md_state))
		return;

	// wait for GOSTAT to clear
	// may need a timeout
	psc_ptstat = *((uint32_t *)MAIN_PSC_PTSTAT);

	while ((psc_ptstat & (0x1 << pd_id)) != 0)
		psc_ptstat = *((uint32_t *)MAIN_PSC_PTSTAT);

	// Set PDCTL NEXT to new state
	*pdctrl_ptr = (pdctrl & ~(0x1)) | pd_state;

	// Set MDCTL NEXT to new state
	*mdctrl_ptr = (mdctrl & ~(0x1f)) | md_state;

	// start power transisition by setti ng PTCMD Go to 1
	psc_ptcmd = *((uint32_t *)MAIN_PSC_PTCMD);
	psc_ptcmd |= (0x1 << pd_id);
	*((uint32_t *)MAIN_PSC_PTCMD) = psc_ptcmd;

	// wait for GOSTAT to clear
	// may need a timeout
	psc_ptstat = *((uint32_t *)MAIN_PSC_PTSTAT);
	while ((psc_ptstat & (0x1 << pd_id)) != 0)
		psc_ptstat = *((uint32_t *)MAIN_PSC_PTSTAT);

	//check states
	mdstat = (uint32_t)*((uint32_t *)mdstat_ptr);
	pdstat = (uint32_t)*((uint32_t *)pdstat_ptr);

	INFO("%s: after: md_id=%d, mdstat=0x%x, pdstat=0x%x \n", __func__, md_id, mdstat, pdstat);
}

void set_ddr_pll_div(int div)
{
	volatile uint32_t val;

	INFO("%s: setting DDR div to %d \n", __func__, div);
	// Set reset to high
	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	val |= 0x80000000;
	*((uint32_t *)(PLL_CFG + 0x88)) = val;

	// Set divider value
	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	val &= 0xFFFFFF80;
	val |= div;
	*((uint32_t *)(PLL_CFG + 0x88)) = val;

	// Set enable bit
	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	val |= 0x8000;
	*((uint32_t *)(PLL_CFG + 0x88)) = val;

	// clear eset
	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	val &= 0x7FFFFFFF;
	*((uint32_t *)(PLL_CFG + 0x88)) = val;

	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	INFO("%s - pll_cfg_div2 =0x%x \n", __func__, val);
}

void ddrss_set_pll(unsigned long freq)
{
	if (freq == DDRSS_PLL_FREQUENCY_1)
		set_ddr_pll_div(4);
	else if (freq == 25000000)
		set_ddr_pll_div(79);
}

static void  k3_lpddr4_freq_update(struct k3_ddrss_desc *ddr)
{
	volatile uint32_t val;
	// volatile uint32_t	*mmr_addr;
	uint32_t counter = 0;

	for (counter = 0; counter < DDRSS_PLL_FHS_CNT; counter++) {
		// update the PLL divisor for DDR PLL
		val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
		// val2 = (uint32_t) *((uint32_t*) (DDRSS_CTRL_MMR_L+0x80));

		INFO("%s : counter=%d val=0x%x \n", __func__, counter, val);

		val &= 0x80;

		while (val == 0) {
			val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
			val &= 0x80;
		}

		val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
		INFO("%s after : val=0x%x, \n", __func__, val);

		val &= 0x03;
		if (val == 1)
			ddrss_set_pll(DDRSS_PLL_FREQUENCY_1);
		else if (val == 2)
			ddrss_set_pll(DDRSS_PLL_FREQUENCY_2);
		else if (val == 0)
			ddrss_set_pll(DDRSS_PLL_FREQUENCY_0);
		else
			INFO("%s invalid DDR Freq request \n", __func__);

		*((uint32_t *)(DDRSS_CTRL_MMR + 0x84)) = 0x01;

		val = *((uint32_t *)(DDRSS_CTRL_MMR + 0x84));
		INFO("%s DDRSS_CTRL_MMR+84 val=0x%x, \n", __func__, val);

		val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
		val &= 0x80;

		INFO("%s Entering loop DDRSS_CTRL_MMR+80 val=0x%x, \n", __func__, val);
		while (val == 0x80) {
			val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
			val &= 0x80;
		}

		*((uint32_t *)(DDRSS_CTRL_MMR + 0x84)) = 0x00;
	}

	INFO("%s DDR Freq change complete \n", __func__);
}

static void k3_lpddr4_ack_freq_upd_req(const lpddr4_privatedata *pd)
{
	// struct k3_ddrss_desc *ddrss = (struct k3_ddrss_desc *)pd->ddr_instance;

	// debug("--->>> LPDDR4 Initialization is in progress ... <<<---\n");

	// switch (ddrss->dram_class) {
	// case DENALI_CTL_0_DRAM_CLASS_DDR4:
	//       break;
	//case DENALI_CTL_0_DRAM_CLASS_LPDDR4:
	k3_lpddr4_freq_update(&ddrss);
	//       break;
	//default:
	//       printf("unrecognized dram_class cannot update frequency!\n");
	// }
}

static void k3_lpddr4_info_handler(const lpddr4_privatedata *pd,
				   lpddr4_infotype infotype)
{
	if (infotype == LPDDR4_DRV_SOC_PLL_UPDATE)
		k3_lpddr4_ack_freq_upd_req(pd);
}

void k3_lpddr4_init(void)
{
	uint16_t configsize = 0U;
	uint32_t status = 0U;
	lpddr4_config *config = &ddrss.config;
	lpddr4_privatedata *pd = &ddrss.pd;
	lpddr4_obj *driverdt;
	// volatile int i =1;
	uint32_t ddr_size;
	volatile uint32_t val;
	uint32_t *mem_ptr;
	int loop_count = 0;
	uint32_t offset = 0;
	uint32_t regval;
	bool restore;

	INFO("lpdd4_init <-- \n");
	// while (i)
	// ;
	// Set DDR PLL to 25MHz
	// ddrss_set_pll(25000000);
	ddrss_set_pll(DDRSS_PLL_FREQUENCY_0);

#if 0
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_LOCAL, PSC_PD_ON, PSC_SYNCRESET);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_CFG_ISO_N, PSC_PD_ON, PSC_SYNCRESET);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_DATA_ISO_N, PSC_PD_ON, PSC_SYNCRESET);

	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_LOCAL, PSC_PD_ON, PSC_SYNCRESETDISABLE);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_CFG_ISO_N, PSC_PD_ON, PSC_SYNCRESETDISABLE);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_DATA_ISO_N, PSC_PD_OFF, PSC_SYNCRESETDISABLE);
#endif
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_LOCAL, PSC_PD_ON, PSC_ENABLE);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_CFG_ISO_N, PSC_PD_ON, PSC_ENABLE);
	set_main_psc_state(PD_DDR, LPSC_MAIN_DDR_DATA_ISO_N, PSC_PD_ON, PSC_ENABLE);

	ddrss.ddrss_ctl_cfg = (void *)DDRSS_CTL_CFG;
	ddrss.ddrss_ctrl_mmr = (void *)DDRSS_CTRL_MMR;
	ddrss.ddrss_ss_cfg = (void *)DDRSS_SS_CFG;

	ddrss.driverdt = lpddr4_getinstance();
	driverdt = ddrss.driverdt;
	// populate config, ctl data i.e what ever is done in ddrss-probe

	INFO("lpddr4: driverdt=%p \n", driverdt);
	ddrss.driverdt->probe(config, &configsize);
	INFO("lpddr4: probe done \n");
	// check status and config size

	// set DDR size
	ddr_size = (uint32_t)*((uint32_t *)(DDRSS_SS_CFG + 0x20));
	INFO("ddr_size - before =0x%x \n", ddr_size);
	ddr_size &= 0x3FF;
	ddr_size |= 0x1EF;
	*((uint32_t *)(DDRSS_SS_CFG + 0x20)) = ddr_size;
	val = *((uint32_t *)(DDRSS_SS_CFG + 0x20));
	INFO("ddr_size - after =0x%x \n", val);

	val = (uint32_t)*((uint32_t *)(PLL_CFG + 0x88));
	INFO("pll_cfg_div2 =0x%x \n", val);

	val = (uint32_t)*((uint32_t *)(DDRSS_CTRL_MMR + 0x80));
	INFO("DDRSS_CTRL_MMR+80 =0x%x \n", val);

	val = (uint32_t)*((uint32_t *)(DDRSS_CTL_CFG + DDRSS_PI_REGISTER_BLOCK__OFFS + DDRSS_PI_83__SFR_OFFS));
	INFO("lpddr4: pre init - PI DONE=0x%x \n", val);

	val = (uint32_t)*((uint32_t *)(DDRSS_CTL_CFG + DDRSS_CTL_342__SFR_OFFS));
	INFO("lpddr4: pre init - CTL DONE=0x%x \n", val);

	// Initialize
	config->ctlbase = ddrss.ddrss_ctl_cfg;
	config->infohandler = k3_lpddr4_info_handler;
	driverdt->init(pd, config);
	INFO("lpddr4: init done \n");
	// pd->ddr_instance = (void *)ddrss;

	// Fill ddr config and control structures
	driverdt->writectlconfigex(pd, lpddr4_ctl_data, LPDDR4_INTR_CTL_REG_COUNT);
	INFO("lpddr4: writectlconfig done \n");
	driverdt->writephyindepconfigex(pd, lpddr4_pi_data, LPDDR4_INTR_PHY_INDEP_REG_COUNT);
	INFO("lpddr4: WritePhyIndeponfig done \n");
	driverdt->writephyconfigex(pd, lpddr4_phy_data, LPDDR4_INTR_PHY_REG_COUNT);
	INFO("lpddr4: WritePhyconfig done \n");

	TH_OFFSET_FROM_REG(LPDDR4__START__REG, CTL_SHIFT, offset);
	INFO("start-status: offset =0x%x \n", offset);

	driverdt->readreg(pd, LPDDR4_CTL_REGS, offset, &regval);
	INFO("start-status reg: before =0x%x \n", regval);
	if ((regval & TH_FLD_MASK(LPDDR4__START__FLD)) != 0)
		INFO("LPDDR4 prestart failed \n");

	restore = (mmio_read_32((WKUP_CTRL_MMR_SEC_5_BASE + CANUART_WAKE_OFF_MODE_STAT)) == RTC_ONLY_PLUS_DDR_MAGIC_WORD);
	
	if(restore){
		INFO("Exiting RTC only + DDR");
		// disable auto-entry / -exit
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x0000029C / 4), &regval);
		regval = (regval & (0xF0F0FFFF));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x0000029C / 4), regval);

					
		//Set the phy_set_dfi_input_Z parameter bit corresponding to the reset signal to 1'b1. Program the controller to a state
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00005468 / 4), &regval);
		regval = (regval | (0x1));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00005468 / 4), regval);

		//Configure the DDR controller (and not the PI) to issue a PWRUP SREFRESH EXIT
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x000001A8 / 4), &regval);
		regval = (regval | (0x1));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x000001A8 / 4), regval);
		
		// PI_PWRUP_SREFRESH_EXIT = 0 
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00002218 / 4), &regval);
		regval = (regval & 0x0);
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00002218 / 4), regval);	
				
		// PI_DRAM_INIT_EN = 0
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00002228 / 4), &regval);
		regval = (regval & (0xFFFFFEFF));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00002228 / 4), regval);	
		
		// PI_DFI_PHYMSTR_STATE_SEL_R = 1
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00002018 / 4), &regval);
		regval = (regval | (0x1<<8));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00002018 / 4), regval);		
		
		// PHY_INDEP_INIT_MODE = 0
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00000054 / 4), &regval);
		regval = (regval & (0xFFFFFEFF));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00000054 / 4), regval);	
		
		// PHY_INDEP_TRAIN_MODE = 1
		driverdt->readreg(pd, LPDDR4_CTL_REGS, (0x00000050 / 4), &regval);
		regval = (regval | (0x1<<24));
		driverdt->writereg(pd, LPDDR4_CTL_REGS, (0x00000050 / 4), regval);	
	} else {
		INFO("Doing normal DDR init");
	}

	if(restore){
		// De-assert the data_retention signal.		
		mmio_write_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), 0x0);
		mmio_write_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), (0x1U << 31));
		while((mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL)) & 0x80000000) ==  0x0)
		{}
		mmio_write_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), 0x0);
	}

	// Init freq
	// check for prestart
	INFO("lpddr4: Start DDR controller \n");
	status = driverdt->start(pd);
	if (status != 0)
		INFO("lpddr4: start failed status = 0x%x\n", status);
		// hang/loop here
	else
		INFO("lpddr4: start completed successfully status=0x%x\n", status);

	// check for poststart

	driverdt->readreg(pd, LPDDR4_CTL_REGS, offset, &regval);
	INFO("start-status reg: after =0x%x \n", regval);
	if ((regval & TH_FLD_MASK(LPDDR4__START__FLD)) != 1)
		INFO("LPDDR4 poststart failed \n");
	else
		INFO("LPDDR4 start completed !! \n");

	val = (uint32_t)*((uint32_t *)(DDRSS_CTL_CFG + DDRSS_PI_REGISTER_BLOCK__OFFS + DDRSS_PI_83__SFR_OFFS));
	INFO("lpddr4: post start - PI DONE=0x%x \n", val);

	val = (uint32_t)*((uint32_t *)(DDRSS_CTL_CFG + DDRSS_CTL_342__SFR_OFFS));
	INFO("lpddr4: post start - CTL DONE=0x%x \n", val);
	mem_ptr = (uint32_t *)0x80000000;
	val = 0xdeadbeef;
	for (loop_count = 0; loop_count < 10; loop_count++)
		*mem_ptr++ = val++;
}
