/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <assert.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/interrupt_props.h>
#include <drivers/arm/gicv3.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>

#include <k3_gicv3.h>

/* The GICv3 driver only needs to be initialized in EL3 */
uintptr_t rdistif_base_addrs[PLATFORM_CORE_COUNT];

static gicv3_redist_ctx_t rdist_ctx[PLATFORM_CORE_COUNT];
static gicv3_dist_ctx_t dist_ctx;
static gicv3_its_ctx_t its_ctx;
static int its_ctx_saved;

static const interrupt_prop_t k3_interrupt_props[] = {
	PLAT_ARM_G1S_IRQ_PROPS(INTR_GROUP1S),
	PLAT_ARM_G0_IRQ_PROPS(INTR_GROUP0)
};

static unsigned int k3_mpidr_to_core_pos(unsigned long mpidr)
{
	return (unsigned int)plat_core_pos_by_mpidr(mpidr);
}

gicv3_driver_data_t k3_gic_data = {
	.rdistif_num = PLATFORM_CORE_COUNT,
	.rdistif_base_addrs = rdistif_base_addrs,
	.interrupt_props = k3_interrupt_props,
	.interrupt_props_num = ARRAY_SIZE(k3_interrupt_props),
	.mpidr_to_core_pos = k3_mpidr_to_core_pos,
};

void k3_gic_driver_init(uintptr_t gic_base)
{
	/* GIC Distributor is always at the base of the IP */
	uintptr_t gicd_base = gic_base;
	/* GIC Redistributor base is run-time detected */
	uintptr_t gicr_base = 0;

	for (unsigned int gicr_shift = 18; gicr_shift < 21; gicr_shift++) {
		uintptr_t gicr_check = gic_base + BIT(gicr_shift);
		uint32_t iidr = mmio_read_32(gicr_check + GICR_IIDR);
		if (iidr != 0) {
			/* Found the GICR base */
			gicr_base = gicr_check;
			break;
		}
	}
	/* Assert if we have not found the GICR base */
	assert(gicr_base != 0);

	/*
	 * The GICv3 driver is initialized in EL3 and does not need
	 * to be initialized again in SEL1. This is because the S-EL1
	 * can use GIC system registers to manage interrupts and does
	 * not need GIC interface base addresses to be configured.
	 */
	k3_gic_data.gicd_base = gicd_base;
	k3_gic_data.gicr_base = gicr_base;
	gicv3_driver_init(&k3_gic_data);
}

void k3_gic_init(void)
{
	gicv3_distif_init();
	gicv3_rdistif_init(plat_my_core_pos());
	gicv3_cpuif_enable(plat_my_core_pos());
}

void k3_gic_cpuif_enable(void)
{
	gicv3_cpuif_enable(plat_my_core_pos());
}

void k3_gic_cpuif_disable(void)
{
	gicv3_cpuif_disable(plat_my_core_pos());
}

void k3_gic_pcpu_init(void)
{
	gicv3_rdistif_init(plat_my_core_pos());
}

void k3_gic_save_context(void)
{
	for (unsigned int i = 0U; i < PLATFORM_CORE_COUNT; i++) {
		gicv3_rdistif_save(i, &rdist_ctx[i]);
	}
	gicv3_distif_save(&dist_ctx);
}

void k3_gic_restore_context(void)
{
	gicv3_distif_init_restore(&dist_ctx);
	for (unsigned int i = 0U; i < PLATFORM_CORE_COUNT; i++) {
		gicv3_rdistif_init_restore(i, &rdist_ctx[i]);
	}
}

void k3_gic_its_save(void)
{
	uint32_t ctlr;
	uint32_t count = 1000000U;
	unsigned int i;

	its_ctx_saved = 0;

	ctlr = mmio_read_32(K3_GITS_BASE + GITS_CTLR);
	its_ctx.gits_ctlr = ctlr;

	/* Skip quiesce if already disabled and quiescent */
	if ((ctlr & GITS_CTLR_QUIESCENT_BIT) != 0U &&
	    (ctlr & GITS_CTLR_ENABLED_BIT) == 0U) {
		its_ctx.gits_cbaser = mmio_read_64(K3_GITS_BASE + GITS_CBASER);
		for (i = 0U; i < ARRAY_SIZE(its_ctx.gits_baser); i++) {
			its_ctx.gits_baser[i] = mmio_read_64(K3_GITS_BASE +
							     GITS_BASER +
							     (8U * i));
		}
		its_ctx_saved = 1;
		return;
	}

	/* Clear Enable and ImDe, then wait for the ITS to drain */
	ctlr &= ~(GITS_CTLR_ENABLED_BIT | BIT_32(1));
	mmio_write_32(K3_GITS_BASE + GITS_CTLR, ctlr);

	while (count != 0U) {
		if ((mmio_read_32(K3_GITS_BASE + GITS_CTLR) &
		     GITS_CTLR_QUIESCENT_BIT) != 0U) {
			break;
		}
		count--;
		udelay(1);
	}

	if (count == 0U) {
		ERROR("k3_gic_its_save: ITS failed to quiesce\n");
		mmio_write_32(K3_GITS_BASE + GITS_CTLR, its_ctx.gits_ctlr);
		return;
	}

	its_ctx.gits_cbaser = mmio_read_64(K3_GITS_BASE + GITS_CBASER);
	for (i = 0U; i < ARRAY_SIZE(its_ctx.gits_baser); i++) {
		its_ctx.gits_baser[i] = mmio_read_64(K3_GITS_BASE +
						     GITS_BASER + (8U * i));
	}
	its_ctx_saved = 1;
}

void k3_gic_its_restore(void)
{
	uint32_t ctlr;
	uint32_t count = 1000000U;
	unsigned int i;

	/* ITS must be disabled before we touch CBASER/BASER */
	assert((mmio_read_32(K3_GITS_BASE + GITS_CTLR) &
		GITS_CTLR_ENABLED_BIT) == 0U);

	/* Ensure quiescent defensively before writing registers */
	ctlr = mmio_read_32(K3_GITS_BASE + GITS_CTLR);
	if ((ctlr & GITS_CTLR_QUIESCENT_BIT) == 0U) {
		ctlr &= ~(GITS_CTLR_ENABLED_BIT | BIT_32(1));
		mmio_write_32(K3_GITS_BASE + GITS_CTLR, ctlr);

		while (count != 0U) {
			if ((mmio_read_32(K3_GITS_BASE + GITS_CTLR) &
			     GITS_CTLR_QUIESCENT_BIT) != 0U) {
				break;
			}
			count--;
			udelay(1);
		}

		if (count == 0U) {
			ERROR("k3_gic_its_restore: ITS failed to quiesce on resume\n");
			return;
		}
	}

	if (its_ctx_saved == 0) {
		WARN("k3_gic_its_restore: skipping CBASER/BASERn restore (save incomplete)\n");
	} else {
		/* Writing CBASER resets CREADR to 0; reset CWRITER to match */
		mmio_write_64(K3_GITS_BASE + GITS_CBASER, its_ctx.gits_cbaser);
		mmio_write_64(K3_GITS_BASE + GITS_CWRITER, 0ULL);

		/* Restore only valid BASERn entries (VALID = bit 63) */
		for (i = 0U; i < ARRAY_SIZE(its_ctx.gits_baser); i++) {
			if ((its_ctx.gits_baser[i] & BIT_64(63)) == 0ULL) {
				continue;
			}
			mmio_write_64(K3_GITS_BASE + GITS_BASER + (8U * i),
				      its_ctx.gits_baser[i]);
		}
	}

	/*
	 * Restore CTLR as-is (ENABLE=1). For deep sleep the kernel's
	 * syscore resume path re-enables the ITS anyway. For s2idle the
	 * syscore ops are never called, so TF-A must leave the ITS enabled
	 * here or subsequent ITS commands issued during CPU hotplug will
	 * time out waiting for the queue to drain.
	 */
	mmio_write_32(K3_GITS_BASE + GITS_CTLR, its_ctx.gits_ctlr);
}
