// SPDX-License-Identifier: GPL-2.0
/*
 * AM62Lite mailbox driver
 *
 * Copyright (C) 2024 Texas Instruments Incorporated - https://www.ti.com
 *
 */

#include <errno.h>
#include <stdlib.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/utils_def.h>

#include "mailbox.h"

void init_mbox(void) {
	uint32_t i = 0;
	uint64_t rcv_addr;

	// timeout mechanism
	uint32_t tick_start = (uint32_t)read_cntpct_el0();
	uint32_t ticks_per_us = SYS_COUNTER_FREQ_IN_TICKS / 1000000;

	while (!i){
		i = mmio_read_32(TIFS_MAILBOX_BASE1 + TIFS_MAILBOX_MSG_STATUS);
		if (((uint32_t)read_cntpct_el0() - tick_start) >
		    (SEC_PROXY_TIMEOUT_US * ticks_per_us)) {
			ERROR("Timeout waiting for boot notification \n");
			break;
		}
	}

	/* consume boot notification, but do nothing about it for now */
	rcv_addr = mmio_read_32(TIFS_MAILBOX_BASE1 + TIFS_MAILBOX_MSG);
	INFO("%s: boot notification recevied from TIFS: 0x%lx\n",__func__, rcv_addr);
}

int k3_sec_proxy_clear_rx_thread(enum k3_sec_proxy_chan_id id)
{
	/* Dummy function to maintain API backward compatiblity */
	/* mmio_write_32(TIFS_MAILBOX_BASE1 + 0x10, 0x1); /1* MESSAGES[a]_MAILBOX_MESSAGE  *1/ */

	return 0;

}

int k3_sec_proxy_send(enum k3_sec_proxy_chan_id id, const struct k3_sec_proxy_msg *msg)
{
	int num_bytes;

	void * dst_ptr = (void*)AM62L_RSVD_SRAM_BASE;
	num_bytes = msg->len / sizeof(uint8_t);

	/* move the buffer contents into the SRAM to be accessed by TIFS */
	memmove(dst_ptr, msg->buf, num_bytes);

	mmio_write_32(TIFS_MAILBOX_BASE0 + TIFS_MAILBOX_MSG, (long unsigned int)(void*)dst_ptr);

	return 0;
}

int k3_sec_proxy_recv(enum k3_sec_proxy_chan_id id, struct k3_sec_proxy_msg *msg)
{
	int num_bytes, i = 0;
	uint64_t rcv_addr;

	// timeout mechanism
	uint32_t tick_start = (uint32_t)read_cntpct_el0();
	uint32_t ticks_per_us = SYS_COUNTER_FREQ_IN_TICKS / 1000000;

	while (!i){
		i = mmio_read_32(TIFS_MAILBOX_BASE1 + TIFS_MAILBOX_MSG_STATUS);
		if (((uint32_t)read_cntpct_el0() - tick_start) >
		    (SEC_PROXY_TIMEOUT_US * ticks_per_us)) {
			ERROR("Timeout waiting for recieve \n");
			return -ETIMEDOUT;
		}
	}

	rcv_addr = mmio_read_32(TIFS_MAILBOX_BASE1 + TIFS_MAILBOX_MSG);
	num_bytes = msg->len / sizeof(uint8_t);

	/* Only Read whole words supported */
	for (i = 0; i < num_bytes; i++) {
		((uint8_t *)msg->buf)[i] = *(uint8_t*)(rcv_addr);
		rcv_addr += sizeof(uint8_t);
	}

	return 0;
}
