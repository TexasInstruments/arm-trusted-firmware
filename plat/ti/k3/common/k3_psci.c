/*
 * Copyright (c) 2017-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stdbool.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/el3_runtime/cpu_data.h>
#include <lib/psci/psci.h>
#include <plat/common/platform.h>
#include <lib/mmio.h>

#include <ti_sci_protocol.h>
#include <k3_gicv3.h>
#include <ti_sci.h>
#include <lpm_stub.h>
#include <gtc.h>
#include <plat_scmi_def.h>
#include <device_wrapper.h>
#include <devices.h>
#include <rtc.h>

#define CORE_PWR_STATE(state) ((state)->pwr_domain_state[MPIDR_AFFLVL0])
#define CLUSTER_PWR_STATE(state) ((state)->pwr_domain_state[MPIDR_AFFLVL1])
#define SYSTEM_PWR_STATE(state) ((state)->pwr_domain_state[PLAT_MAX_PWR_LVL])

#define WKUP_CTRL_MMR0_DEVICE_MANAGEMENT_BASE	(0x43050000UL)
#define WKUP_CTRL_MMR0_DEVICE_RESET_OFFSET	(0x4000)

uintptr_t k3_sec_entrypoint;
uintptr_t k3_sec_entrypoint_glob;

/*********** PROC BOOT CODE ******************/

#include <lib/mmio.h>

/* power domain indices */
#define GP_CORE_CTL     0
#define PD_CRYPTO       1
#define PD_DDR          2
#define PD_MAIN_IP      3
#define PD_MPU_CLST     4
#define PD_MPU_CLST_CORE_0      5
#define PD_MPU_CLST_CORE_1      6
#define PD_PER                  9

/* lpsc indices */
#define LPSC_MAIN_GP_ALWAYSON           0
#define LPSC_MAIN_GP_TEST                       1
#define LPSC_MAIN_GP_PBIST                      2
#define LPSC_MAIN_GP_ISO0_N                     3
#define LPSC_MAIN_GP_ISO1_N                     4
#define LPSC_MAIN_GP_TIFS                       5
#define LPSC_MAIN_GP_USB0                       7
#define LPSC_MAIN_GP_USB0_ISO_N         8
#define LPSC_MAIN_GP_USB1                       9
#define LPSC_MAIN_GP_USB1_ISO_N         10
#define LPSC_MAIN_GP_DPHY_TX            11
#define LPSC_MAIN_GP_WKPERI                     18
#define LPSC_MAIN_CRYPTO                        19
#define LPSC_MAIN_DDR_LOCAL                     21
#define LPSC_MAIN_DDR_CFG_ISO_N         22
#define LPSC_MAIN_DDR_DATA_ISO_N        23
#define LPSC_MAIN_IP_COMMON                     24
#define LPSC_MAIN_IP_DSS                        25
#define LPSC_MAIN_IP_DSI                        26
#define LPSC_MAIN_IP_EMMC8B                     27
#define LPSC_MAIN_IP_EMMC4B0            28
#define LPSC_MAIN_IP_EMMC4B1            29
#define LPSC_MAIN_IP_CPSW                       30
#define LPSC_MAIN_IP_GIC                        32
#define LPSC_MAIN_IP_PBIST                      33
#define LPSC_MAIN_MPU_CLST                      38
#define LPSC_MAIN_MPU_CLST_PBIST        39
#define LPSC_MAIN_MPU_CLST_CORE_0       40
#define LPSC_MAIN_MPU_CLST_CORE_1       41
#define LPSC_MAIN_PER_COMMON            44
#define LPSC_MAIN_PER_MCASP0            45
#define LPSC_MAIN_PER_MCASP1            46
#define LPSC_MAIN_PER_MCASP2            47
#define LPSC_MAIN_PER_XSPI                      48
#define LPSC_MAIN_PER_MCAN0                     49
#define LPSC_MAIN_PER_MCAN1                     50
#define LPSC_MAIN_PER_MCAN2                     51
#define LPSC_MAIN_PER_GPMC                      52

#define PSC_SYNCRESETDISABLE            		(0x0)
#define PSC_SYNCRESET                           (0x1)
#define PSC_DISABLE                             (0x2)
#define PSC_ENABLE                              (0x3)
#define PSC_PD_OFF                                      (0x0)
#define PSC_PD_ON                                       (0x1)

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

#if 0
static uint32_t lpsc_read(uint32_t lpsc_idx, uint32_t reg)
{
	uint32_t val;

   	val = mmio_read_32(PSC_ADDR + (reg + 4*lpsc_idx));
	return val;
}

static void lpsc_write(uint32_t lpsc_idx, uint32_t reg, uint32_t val)
{

    mmio_write_32(PSC_ADDR + (reg + 4*lpsc_idx),val);
}

static void lpsc_transition(uint32_t lpsc_idx, uint32_t state)
{
	uint32_t mdctl;

	mdctl = lpsc_read(lpsc_idx, PSC_MDCTL);

    mdctl &= ~MDSTAT_STATE_MASK;
    mdctl |= state;

	lpsc_write(lpsc_idx, PSC_MDCTL, mdctl);

    // add code to transisiion pd as well
}

#endif

static void __unused set_main_psc_state(uint32_t pd_id, uint32_t md_id, uint32_t pd_state, uint32_t md_state)
{
	uint32_t	*mdctrl_ptr;
	volatile uint32_t	mdctrl;
	uint32_t	*mdstat_ptr;
	volatile uint32_t	mdstat;
	uint32_t	*pdctrl_ptr;
	volatile uint32_t	pdctrl;
	uint32_t	*pdstat_ptr;
	volatile uint32_t	pdstat;
	volatile uint32_t	psc_ptstat;
	volatile uint32_t	psc_ptcmd;


	mdctrl_ptr = (uint32_t*) (uint64_t) ((MAIN_PSC_MDCTL_BASE + (4*md_id)));
	mdctrl = (uint32_t) *((uint32_t*)mdctrl_ptr);
	mdstat_ptr = (uint32_t*) (uint64_t) ((MAIN_PSC_MDSTAT_BASE + (4*md_id)));
	mdstat = (uint32_t) *((uint32_t*)mdstat_ptr);
	pdctrl_ptr = (uint32_t*) (uint64_t) ((MAIN_PSC_PDCTL_BASE + (4*pd_id)));
	pdctrl = (uint32_t) *((uint32_t*)pdctrl_ptr);
	pdstat_ptr = (uint32_t*) (uint64_t) ((MAIN_PSC_PDSTAT_BASE + (4*pd_id)));
	pdstat = (uint32_t) *((uint32_t*)pdstat_ptr);

	ERROR("%s: before: md_id=%d, mdstat=0x%x, pdstat=0x%x \n",__func__,md_id,mdstat,pdstat);

	if (((pdstat & 0x1) == pd_state) && ((mdstat & 0x1f) == md_state))
		return;

	// wait for GOSTAT to clear
	// may need a timeout
	psc_ptstat = *((uint32_t*) MAIN_PSC_PTSTAT);

	while ((psc_ptstat & (0x1 << pd_id)) !=0)
		psc_ptstat = *((uint32_t*) MAIN_PSC_PTSTAT);

	// Set PDCTL NEXT to new state
	*pdctrl_ptr = (pdctrl & ~(0x1)) | pd_state;

	// Set MDCTL NEXT to new state
	*mdctrl_ptr = (mdctrl & ~(0x1f)) | md_state;

	// start power transisition by setti ng PTCMD Go to 1
	psc_ptcmd = *((uint32_t*) MAIN_PSC_PTCMD);
	psc_ptcmd |= (0x1 << pd_id);

	*((uint32_t*) MAIN_PSC_PTCMD) = psc_ptcmd;

	if (!md_state)
		return;

	// wait for GOSTAT to clear
	// may need a timeout
	psc_ptstat = *((uint32_t*) MAIN_PSC_PTSTAT);
	while ((psc_ptstat & (0x1 << pd_id)) !=0)
		psc_ptstat = *((uint32_t*) MAIN_PSC_PTSTAT);

	//check states
	mdstat = (uint32_t) *((uint32_t*)mdstat_ptr);
	pdstat = (uint32_t) *((uint32_t*)pdstat_ptr);

	ERROR("%s: after: md_id=%d, mdstat=0x%x, pdstat=0x%x \n",__func__,md_id,mdstat,pdstat);

}


/*********** PROC BOOT CODE ENDS******************/


static void k3_cpu_standby(plat_local_state_t cpu_state)
{
	u_register_t scr;

	scr = read_scr_el3();
	/* Enable the Non secure interrupt to wake the CPU */
	write_scr_el3(scr | SCR_IRQ_BIT | SCR_FIQ_BIT);
	isb();
	/* dsb is good practice before using wfi to enter low power states */
	dsb();
	/* Enter standby state */
	wfi();
	/* Restore SCR */
	write_scr_el3(scr);
}

volatile int HOLDIIT=0x1234BEEF;
volatile int cnt=3;

static int k3_pwr_domain_on(u_register_t mpidr)
{
	int core, proc_id, ret;
	HOLDIIT = 0xFEEFBEEF;

	core = plat_core_pos_by_mpidr(mpidr);
	if (core < 0) {
		ERROR("Could not get target core id: %d\n", core);
		return PSCI_E_INTERN_FAIL;
	}

	proc_id = PLAT_PROC_START_ID + core;	// should be 0x21

	INFO("proc_id = 0x%x\n", proc_id);

	ret = ti_sci_proc_request(proc_id);
	if (ret) {
		ERROR("Request for processor failed: %d\n", ret);
		return PSCI_E_INTERN_FAIL;
	}

	ret = ti_sci_proc_set_boot_cfg(proc_id, k3_sec_entrypoint, 0, 0);
	if (ret) {
		ERROR("Request to set core boot address failed: %d\n", ret);
		return PSCI_E_INTERN_FAIL;
	}

	/* sanity check these are off before starting a core */
	ret = ti_sci_proc_set_boot_ctrl(proc_id,
			0, PROC_BOOT_CTRL_FLAG_ARMV8_L2FLUSHREQ |
			   PROC_BOOT_CTRL_FLAG_ARMV8_AINACTS |
			   PROC_BOOT_CTRL_FLAG_ARMV8_ACINACTM);
	if (ret) {
		ERROR("Request to clear boot configuration failed: %d\n", ret);
		return PSCI_E_INTERN_FAIL;
	}

	// scmi_handler_device_state_set_on(AM62LX_DEV_COMPUTE_CLUSTER0_A53_0 + core);
	set_main_psc_state(PD_MPU_CLST_CORE_1, LPSC_MAIN_MPU_CLST_CORE_1, PSC_PD_ON, PSC_ENABLE);

	return PSCI_E_SUCCESS;
}


static void k3_pwr_domain_off(const psci_power_state_t *target_state)
{
}

static void __dead2 k3_pwr_domain_off_wfi(const psci_power_state_t *target_state)
{
	// int core;

	// core = plat_my_core_pos();

	/* At very least the local core should be powering down */
	assert(CORE_PWR_STATE(target_state) == PLAT_MAX_OFF_STATE);

	/* Prevent interrupts from spuriously waking up this cpu */
	k3_gic_cpuif_disable();

	/* If our cluster is not going down we stop here */
	if (CLUSTER_PWR_STATE(target_state) != PLAT_MAX_OFF_STATE) {
		// scmi_handler_device_state_set_off(AM62LX_DEV_COMPUTE_CLUSTER0_A53_0 + core);
		set_main_psc_state(PD_MPU_CLST_CORE_1, LPSC_MAIN_MPU_CLST_CORE_1, PSC_PD_OFF, PSC_SYNCRESETDISABLE);
	}
	
	while(1)
		wfi();
}

void k3_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	/* TODO: Indicate to System firmware about completion */

	k3_gic_pcpu_init();
	k3_gic_cpuif_enable();
}

static void __dead2 k3_system_off(void)
{
	int ret;

	/* Queue up the system shutdown request */
	ret = ti_sci_device_put_no_wait(PLAT_BOARD_DEVICE_ID);
	if (ret != 0) {
		ERROR("Sending system shutdown message failed (%d)\n", ret);
	}

	while (true)
		wfi();
}

static void __dead2 k3_system_reset(void)
{
	/* Send the system reset request to system firmware */
#ifdef K3_TI_SCI_MAILBOX
	INFO("TF-A: %s: AM62L: Resetting device\n", __func__);
	mmio_write_32(WKUP_CTRL_MMR0_DEVICE_MANAGEMENT_BASE + WKUP_CTRL_MMR0_DEVICE_RESET_OFFSET, 0x6);
#else
	ti_sci_core_reboot();
#endif

	while (true)
		wfi();
}

static int k3_validate_power_state(unsigned int power_state,
				   psci_power_state_t *req_state)
{
	/* TODO: perform the proper validation */

	return PSCI_E_SUCCESS;
}

static void k3_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	unsigned int core, proc_id;
	/* TODO: create a static carvout for TIFS context save and restore */
	uint64_t  context_save_addr = 0x90000000;
	uint32_t mode = 0;
	mode = get_low_power_mode();
	core = plat_my_core_pos();
	INFO("k3_pwr_domain_suspend %d\n",mode);

	proc_id = PLAT_PROC_START_ID + core;

	/* Prevent interrupts from spuriously waking up this cpu */
	k3_gic_cpuif_disable();
	k3_gic_save_context();

	if(mode == 5){

		k3_lpm_config_magic_words(mode);
		ti_sci_prepare_sleep(mode, context_save_addr, 0);
		INFO("sent prepare message\n");
		k3_config_wake_sources(true);
		ti_sci_enter_sleep(proc_id, mode, k3_sec_entrypoint);
		INFO("sent enter sleep message\n");

	} else if ( mode == 0){
		
		k3_lpm_config_magic_words(mode);
		ti_sci_prepare_sleep(mode, context_save_addr, 0);
		INFO("sent prepare message\n");
		k3_config_wake_sources(true);
		ti_sci_enter_sleep(proc_id, mode, k3_sec_entrypoint);
		INFO("sent enter sleep message\n");
	}

	k3_suspend_to_ram();

}

static void k3_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	k3_config_wake_sources(false);
	k3_gic_restore_context();
	k3_gic_cpuif_enable();
	ti_init_scmi_channel();
	k3_lpm_stub_copy_to_sram();
	rtc_resume();
}

static void k3_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	unsigned int i;

	/* CPU & cluster off, system in retention */
	for (i = MPIDR_AFFLVL0; i <= PLAT_MAX_PWR_LVL; i++) {
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
	}
}

static plat_psci_ops_t k3_plat_psci_ops = {
	.cpu_standby = k3_cpu_standby,
	.pwr_domain_on = k3_pwr_domain_on,
	.pwr_domain_off = k3_pwr_domain_off,
	.pwr_domain_pwr_down_wfi = k3_pwr_domain_off_wfi,
	.pwr_domain_on_finish = k3_pwr_domain_on_finish,
	.pwr_domain_suspend = k3_pwr_domain_suspend,
	.pwr_domain_suspend_finish = k3_pwr_domain_suspend_finish,
	.get_sys_suspend_power_state = k3_get_sys_suspend_power_state,
	.system_off = k3_system_off,
	.system_reset = k3_system_reset,
	.validate_power_state = k3_validate_power_state,
};


void  __attribute__((aligned(16))) jump_to_atf_func() {
	void (*bl31_loc_warm_entry)(void) = (void*)k3_sec_entrypoint_glob; // bl31_warm_entrypoint
	bl31_loc_warm_entry();
}

int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
#ifdef TI_AM62L_LPM

	k3_sec_entrypoint_glob = sec_entrypoint;
	k3_sec_entrypoint = (long unsigned int)(void*)&jump_to_atf_func;
	ERROR("k3_sec_entrypoint = 0x%lx\n", k3_sec_entrypoint);

	*psci_ops = &k3_plat_psci_ops;

	return 0;
#else

	uint64_t fw_caps = 0;
	int ret;

	k3_sec_entrypoint_glob = sec_entrypoint;
	k3_sec_entrypoint = (long unsigned int)(void*)&jump_to_atf_func;

	ret = ti_sci_query_fw_caps(&fw_caps);
	if (ret) {
		ERROR("Unable to query firmware capabilities (%d)\n", ret);
	}

	/* If firmware does not support any known suspend mode */
	if (!(fw_caps & (MSG_FLAG_CAPS_LPM_DEEP_SLEEP |
			 MSG_FLAG_CAPS_LPM_MCU_ONLY |
			 MSG_FLAG_CAPS_LPM_STANDBY |
			 MSG_FLAG_CAPS_LPM_PARTIAL_IO))) {
		/* Disable PSCI suspend support */
		k3_plat_psci_ops.pwr_domain_suspend = NULL;
		k3_plat_psci_ops.pwr_domain_suspend_finish = NULL;
		k3_plat_psci_ops.get_sys_suspend_power_state = NULL;
	}
	ERROR("k3_sec_entrypoint = 0x%lx\n", k3_sec_entrypoint);

	*psci_ops = &k3_plat_psci_ops;

	return 0;
#endif /* TI_AM62L_LPM */
}
