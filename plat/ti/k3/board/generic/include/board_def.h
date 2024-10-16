/*
 * Copyright (c) 2017-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BOARD_DEF_H
#define BOARD_DEF_H

#include <lib/utils_def.h>

/* The ports must be in order and contiguous */
#define K3_CLUSTER0_CORE_COUNT		U(2)
#define K3_CLUSTER1_CORE_COUNT		U(2)
#define K3_CLUSTER2_CORE_COUNT		U(0)
#define K3_CLUSTER3_CORE_COUNT		U(0)

#define PLAT_PROC_START_ID		U(32)
#define PLAT_PROC_DEVICE_START_ID	U(202)
#define PLAT_CLUSTER_DEVICE_START_ID	U(198)
#define PLAT_BOARD_DEVICE_ID		U(157)

/*
 * Defines the maximum number of translation tables that are allocated by the
 * translation table library code. To minimize the amount of runtime memory
 * used, choose the smallest value needed to map the required virtual addresses
 * for each BL stage.
 */
#if USE_COHERENT_MEM
#define MAX_XLAT_TABLES		10
#elif IMAGE_BL31
#define MAX_XLAT_TABLES		10
#else
#define MAX_XLAT_TABLES		2
#endif

/*
 * Defines the maximum number of regions that are allocated by the translation
 * table library code. A region consists of physical base address, virtual base
 * address, size and attributes (Device/Memory, RO/RW, Secure/Non-Secure), as
 * defined in the `mmap_region_t` structure. The platform defines the regions
 * that should be mapped. Then, the translation table library will create the
 * corresponding tables and descriptors at runtime. To minimize the amount of
 * runtime memory used, choose the smallest value needed to register the
 * required regions for each BL stage.
 */
#define MAX_MMAP_REGIONS	10

#endif /* BOARD_DEF_H */
