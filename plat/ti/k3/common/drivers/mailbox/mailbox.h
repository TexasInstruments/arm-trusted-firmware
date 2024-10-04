/* SPDX-License-Identifier: GPL-2.0-only
 * Following data taken from confluence
 * https://confluence.itg.ti.com/pages/viewpage.action?pageId=814781225#MailboxconfigurationDiscussionforAM62L/AM61-RecommendedmailboxaddressinTIFS_CBASS
 */

#include <sec_proxy.h>

#define TIFS_MAILBOX_BASE0		UL(0x44240000) /* TFA sending IPC messages to TIFS */
#define TIFS_MAILBOX_BASE1		UL(0x44250000) /* TIFS sending IPC messages to A53 */
#define TIFS_MAILBOX_SYSC		UL(0x10)

#define TIFS_MAILBOX_MSG		UL(0x40)
#define TIFS_MAILBOX_MSG_STATUS		UL(0xc0)

#define AM62L_RSVD_SRAM_BASE		UL(0x70814000)
#define AM62L_SRAM_SIZE			UL(0x2000)

extern void k3_sysctrler_boot_notification_response(void);

