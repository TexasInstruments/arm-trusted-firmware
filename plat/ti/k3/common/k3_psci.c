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

#include <ti_sci_protocol.h>
#include <k3_gicv3.h>
#include <ti_sci.h>

#include <device_wrapper.h>
#include <devices.h>

#define CORE_PWR_STATE(state) ((state)->pwr_domain_state[MPIDR_AFFLVL0])
#define CLUSTER_PWR_STATE(state) ((state)->pwr_domain_state[MPIDR_AFFLVL1])
#define SYSTEM_PWR_STATE(state) ((state)->pwr_domain_state[PLAT_MAX_PWR_LVL])

uintptr_t k3_sec_entrypoint;
uintptr_t k3_sec_entrypoint_glob;

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

static int k3_pwr_domain_on(u_register_t mpidr)
{
	int core, proc_id, ret;

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

	scmi_handler_device_state_set_on(AM62LX_DEV_COMPUTE_CLUSTER0_A53_0 + core);

	return PSCI_E_SUCCESS;
}


static void k3_pwr_domain_off(const psci_power_state_t *target_state)
{
}

static void __dead2 k3_pwr_domain_off_wfi(const psci_power_state_t *target_state)
{
	int core;

	core = plat_my_core_pos();

	/* At very least the local core should be powering down */
	assert(CORE_PWR_STATE(target_state) == PLAT_MAX_OFF_STATE);

	/* Prevent interrupts from spuriously waking up this cpu */
	k3_gic_cpuif_disable();

	/* If our cluster is not going down we stop here */
	if (CLUSTER_PWR_STATE(target_state) != PLAT_MAX_OFF_STATE) {
		scmi_handler_device_state_set_off(AM62LX_DEV_COMPUTE_CLUSTER0_A53_0 + core);
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
	ti_sci_core_reboot();

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

	core = plat_my_core_pos();
	proc_id = PLAT_PROC_START_ID + core;

	/* Prevent interrupts from spuriously waking up this cpu */
	k3_gic_cpuif_disable();
	k3_gic_save_context();

	k3_pwr_domain_off(target_state);

	ti_sci_enter_sleep(proc_id, 0, k3_sec_entrypoint);
}

static void k3_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	k3_gic_restore_context();
	k3_gic_cpuif_enable();
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
}
