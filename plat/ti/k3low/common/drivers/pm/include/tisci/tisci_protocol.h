/*
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef TISCI_PROTOCOL_H
#define TISCI_PROTOCOL_H

#include <cdefs.h>
#include <stdint.h>

/**
 * This flag is reserved and not to be used.
 */
#define TISCI_MSG_FLAG_RESERVED0    BIT(0)

/**
 * ACK on Processed: Send a response to a message after it has been processed
 * with TISCI_MSG_FLAG_ACK set if the processing succeeded, or a NAK otherwise.
 * This response contains the complete response to the message with the result
 * of the actual action that was requested.
 */
#define TISCI_MSG_FLAG_AOP    BIT(1)

/** Indicate that this message is marked secure */
#define TISCI_MSG_FLAG_SEC    BIT(2)

/**
 * Response flag for a message that indicates success. If this flag is NOT
 * set then that is to be interpreted as a NAK.
 */
#define TISCI_MSG_FLAG_ACK    BIT(1)

/**
 * \brief Header that prefixes all TISCI messages.
 *
 * \param type of message identified by a TISCI_MSG_* ID
 * \param host of the message.
 * \param seq Message identifier indicating a transfer sequence.
 * \param flags TISCI_MSG_FLAG_* for the message.
 * \param payload Placeholder pointer that can be used to access the raw message body.
 */
struct tisci_header {
	uint16_t type;
	uint8_t host;
	uint8_t seq;
	uint32_t flags;
	uint8_t payload[];
};

/*
 * Messages sent via secure transport need an additional field for integrity
 * check in addition to the fields of a TISCI message. The below data structure
 * is used to enhance the standard TISCI message header with a field for
 * integrity check and a reserved field for future use.
 *
 * The below defined header is placed before the \ref tisci_header when
 * transmitting a message via a secure transport.
 *
 * For more information, please refer to the TISCI documentation.
 */

/**
 * \brief Header that prefixes all TISCI messages sent via secure transport.
 *
 * \param integ_check This field should always be initialized to zero on GP
 * devices. The mechanism to calculate the integrity check for HS devices
 * is under definition.
 *
 * \param rsvd Reserved field for future use. Initialize to zero.
 *
 */
struct tisci_sec_header {
	uint16_t integ_check;
	uint16_t rsvd;
};

/* TISCI Message IDs */
#define TISCI_MSG_VERSION			(0x0002U)
#define TISCI_MSG_BOOT_NOTIFICATION		(0x000AU)
#define TISCI_MSG_BOARD_CONFIG			(0x000BU)
#define TISCI_MSG_BOARD_CONFIG_RM		(0x000CU)
#define TISCI_MSG_BOARD_CONFIG_SECURITY		(0x000DU)
#define TISCI_MSG_BOARD_CONFIG_PM		(0x000EU)

#define TISCI_MSG_ENABLE_WDT			(0x0000U)
#define TISCI_MSG_WAKE_RESET			(0x0001U)
#define TISCI_MSG_WAKE_REASON			(0x0003U)
#define TISCI_MSG_GOODBYE			(0x0004U)
#define TISCI_MSG_SYS_RESET			(0x0005U)

#define TISCI_MSG_QUERY_MSMC			(0x0020U)
#define TISCI_MSG_GET_TRACE_CONFIG		(0x0021U)
#define TISCI_MSG_QUERY_FW_CAPS			(0x0022U)

#define TISCI_MSG_SET_CLOCK			(0x0100U)
#define TISCI_MSG_GET_CLOCK			(0x0101U)
#define TISCI_MSG_SET_CLOCK_PARENT		(0x0102U)
#define TISCI_MSG_GET_CLOCK_PARENT		(0x0103U)
#define TISCI_MSG_GET_NUM_CLOCK_PARENTS		(0x0104U)
#define TISCI_MSG_SET_FREQ			(0x010cU)
#define TISCI_MSG_QUERY_FREQ			(0x010dU)
#define TISCI_MSG_GET_FREQ			(0x010eU)

#define TISCI_MSG_SET_DEVICE			(0x0200U)
#define TISCI_MSG_GET_DEVICE			(0x0201U)
#define TISCI_MSG_SET_DEVICE_RESETS		(0x0202U)
#define TISCI_MSG_DEVICE_DROP_POWERUP_REF	(0x0203U)

#define TISCI_MSG_PREPARE_SLEEP			(0x0300U)
#define TISCI_MSG_ENTER_SLEEP			(0x0301U)

#endif /* TISCI_PROTOCOL_H */
