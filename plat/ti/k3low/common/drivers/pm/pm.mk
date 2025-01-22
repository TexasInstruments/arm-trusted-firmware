#
# Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
#
# SPDX-License-Identifier: BSD-3-Clause
#

PLAT_INCLUDES += -I${PLAT_PATH}/common/drivers/pm/include/

include ${PLAT_PATH}/common/drivers/pm/clock/clock.mk
include ${PLAT_PATH}/common/drivers/pm/device/device.mk

