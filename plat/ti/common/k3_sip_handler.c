/*
 * Copyright (c) 2025-2026, Texas Instruments. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include <ti_platform_defs.h>
#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/smccc.h>
#include <k3_sip_svc.h>
#include <ti_sci.h>
#include <ti_sci_protocol.h>

/* K3 FUSE BUFFER STRUCT */
struct k3_fuse_writebuff {
	uint32_t ver_info;
	uint32_t tisci_id;
	void *payload;
} __packed;

/**
 * ti_fuse_writebuiff_handler - Handler for fuse writebuff SMCCC.
 *
 * @x1: The starting memory address of the buffer
 *
 * Return: 0 if all goes well, else appropriate error message
 */
int ti_fuse_writebuff_handler(u_register_t x1)
{
	int ret;
	const struct k3_fuse_writebuff *k3_fuse_buff;

	if (x1 == 0U) {
		ERROR("x1 param is NULL\n");
		return SMC_UNK;
	}
	k3_fuse_buff = (const struct k3_fuse_writebuff *)x1;
	NOTICE("Writebuff version info: 0x%x\n", k3_fuse_buff->ver_info);
	if (k3_fuse_buff->tisci_id == TISCI_MSG_KEY_WRITER_LITE) {

		/*
		 * Buffer needs to be visible to the TISCI co-processor, since
		 * the size is unknown, flush the entire page
		 */
		flush_dcache_range((uintptr_t)k3_fuse_buff, PAGE_SIZE);

		ret = ti_sci_keywriter_lite((unsigned long)&k3_fuse_buff->payload);
		if (ret) {
			ERROR("Keywriter Lite Failed: (%d)\n", ret);
			return SMC_UNK;
		}
	} else if (k3_fuse_buff->tisci_id == TISCI_MSG_KEY_WRITER) {

		/*
		 * Buffer needs to be visible to the TISCI co-processor, since
		 * the size can be maximum 12KB, flush 12KB from the start of the address
		 * Since PAGE_SIZE is 4KB, flush 3 pages (3U * 4KB = 12KB)
		 */
		flush_dcache_range((uintptr_t)k3_fuse_buff, (3U * PAGE_SIZE));

		ret = ti_sci_keywriter((unsigned long)&k3_fuse_buff->payload);
		if (ret) {
			ERROR("Keywriter Failed: (%d)\n", ret);
			return SMC_UNK;
		}
	} else {
		ERROR("Invalid TISCI ID (0x%x)\n", k3_fuse_buff->tisci_id);
		return SMC_UNK;
	}

	return 0;
}

/*
 * This is a list of TI_SCI messages which are exposed to non-secure users
 * through this SiP service. The SiP call with any other message type will
 * return -EPERM.
 * Expand the list as necessary.
 */
static const uint16_t allowed_sip_ti_sci_msg_types[] = {
	/* General */
	TI_SCI_MSG_VERSION,

	/* Security - Processor Boot */
	TISCI_MSG_PROC_AUTH_BOOT_IMAGE,

	/* Security - Runtime Debug */
	TISCI_MSG_GET_SOC_UID,
};

static bool ti_sip_is_ti_sci_call_allowed(uint16_t msg_type)
{
	for (int i = 0; i < ARRAY_SIZE(allowed_sip_ti_sci_msg_types); ++i) {
		if (msg_type == allowed_sip_ti_sci_msg_types[i])
			return true;
	}
	return false;
}

/**
 * ti_sci_xfer_sip_handler - Forward a TISCI message from non-secure world via SiP SMC.
 *
 * @tx_size:     Size of the transmit message in bytes. Must be in the range
 *               [sizeof(TISCI header), TI_SCI_MAX_MESSAGE_SIZE].
 * @rx_size:     Size of the receive message in bytes. Must be in the range
 *               [sizeof(TISCI header), TI_SCI_MAX_MESSAGE_SIZE].
 * @msg:         The message itself, serialized from the 64-bit SMC registers.
 *               The caller needs to ensure that the length of this buffer is at
 *               least tx_size in length.
 * @res:         Buffer to store the response. Response is passed on to the caller
 *               through this buffer. Hence it is the responsibility of the caller
 *               to ensure the size of this buffer is at least rx_size.
 *
 * TF-A copies the message to a local stack buffer, does the TISCI xfer, then
 * copies the response back to the caller's buffer.
 *
 * Return: 0 on success, negative errno on error.
 */
int ti_sci_xfer_sip_handler(size_t tx_size, size_t rx_size,
			    uint64_t *msg, uint8_t *res)
{
	uint8_t req[TI_SCI_MAX_MESSAGE_SIZE];
	struct ti_sci_msg_hdr *hdr;
	int ret;

	/*
	 * The message header on non-secure host does not contain
	 * secure header. This is prepended in the TF-A. Add its
	 * size to msg size.
	 */
	size_t secure_tx_size = tx_size + sizeof(struct ti_sci_secure_msg_hdr);
	size_t secure_rx_size = rx_size + sizeof(struct ti_sci_secure_msg_hdr);

	if (secure_tx_size > TI_SCI_MAX_MESSAGE_SIZE ||
	    secure_tx_size < sizeof(struct ti_sci_msg_hdr) ||
	    secure_rx_size > TI_SCI_MAX_MESSAGE_SIZE ||
	    secure_rx_size < sizeof(struct ti_sci_msg_hdr)) {
		ERROR("ERROR: Invalid TISCI message size\n");
		return -ERANGE;
	}

	/* Leave space for secure header at the beginning. */
	memcpy(req + sizeof(struct ti_sci_secure_msg_hdr), msg, tx_size);

	hdr = (struct ti_sci_msg_hdr *)req;

	if (!ti_sip_is_ti_sci_call_allowed(hdr->type)) {
		ERROR("TISCI Message %u is not allowed/supported\n",
		      hdr->type);
		return -EPERM;
	}

	ret = ti_sci_xfer_sip(req, secure_tx_size, secure_rx_size);
	if (ret != 0) {
		ERROR("Error: TISCI transfer failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Write response back to the caller's buffer. Remove secure header as
	 * non-secure hosts do not expect it.
	 */
	memcpy(res, req + sizeof(struct ti_sci_secure_msg_hdr), rx_size);

	return 0;
}
