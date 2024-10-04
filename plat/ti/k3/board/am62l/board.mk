#
# Copyright (c) 2024, Texas Instruments Inc. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

include ${PLAT_PATH}/am62l/scmi/scmi.mk
include ${PLAT_PATH}/am62l/scmi/drivers/clock/clock.mk
include ${PLAT_PATH}/am62l/scmi/drivers/device/device.mk
include ${PLAT_PATH}/am62l/scmi/drivers/psc/psc.mk
include ${PLAT_PATH}/am62l/scmi/drivers/soc/am62lx/soc.mk

# We dont have system level coherency capability
USE_COHERENT_MEM	:=	0
K3_TI_SCI_MAILBOX	:=	1
$(eval $(call add_define,K3_TI_SCI_MAILBOX))

TI_USE_SCMI	:=	1
$(eval $(call add_define,TI_USE_SCMI))

ifeq (${IMAGE_BL1}, 1)
override ENABLE_PIE := 0
endif

PLAT_INCLUDES		+=	\
				-I${PLAT_PATH}/common/drivers/lpddr4	\
				-I${PLAT_PATH}/common/drivers/lpddr4/common	\
				-I${PLAT_PATH}/common/drivers/lpddr4/16bit	\

K3_LPDDR4_SOURCES	+= 	\
				${PLAT_PATH}/common/drivers/lpddr4/k3-ddrss.c \
				${PLAT_PATH}/common/drivers/lpddr4/lpddr4_obj_if.c \
				${PLAT_PATH}/common/drivers/lpddr4/lpddr4.c \
				${PLAT_PATH}/common/drivers/lpddr4/lpddr4_16bit_ctl_regs_rw_masks.c \
				${PLAT_PATH}/common/drivers/lpddr4/lpddr4_16bit.c \

BL1_SOURCES		+=	\
				${PLAT_PATH}/common/k3_bl1_setup.c	\
				${PLAT_PATH}/common/k3_helpers.S	\
				${PLAT_PATH}/common/k3_topology.c	\
				drivers/io/io_storage.c \
				${K3_LPDDR4_SOURCES}			\
				${K3_TI_SCI_TRANSPORT}	\

K3_TI_SCI_TRANSPORT	=	${PLAT_PATH}/common/drivers/mailbox/mailbox.c

BL32_BASE ?= 0x80200000
$(eval $(call add_define,BL32_BASE))

PRELOADED_BL33_BASE ?= 0x82000000
$(eval $(call add_define,PRELOADED_BL33_BASE))

K3_HW_CONFIG_BASE ?= 0x88000000
$(eval $(call add_define,K3_HW_CONFIG_BASE))
