#
# Copyright (c) 2023, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

# Define sec_proxy usage as the full prioritized communication scheme
K3_SEC_PROXY_LITE	:=	0
$(eval $(call add_define,K3_SEC_PROXY_LITE))

# System coherency is managed in hardware
USE_COHERENT_MEM	:=	1

# Address to save BL31 for suspend to RAM
# It belongs to a reserved memory region
# If K3_LPM_DDR_SAVE_ADDRESS is set to 0x00000000, BL31 will not save itself
# during suspend
K3_LPM_DDR_SAVE_ADDRESS	?=	0x00000000
$(eval $(call add_define,K3_LPM_DDR_SAVE_ADDRESS))
K3_LPM_DDR_MAX_SAVE_SZ	?=	0x30000
$(eval $(call add_define,K3_LPM_DDR_MAX_SAVE_SZ))
