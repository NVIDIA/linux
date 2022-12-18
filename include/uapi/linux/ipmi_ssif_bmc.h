/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note*/
/*
 * Copyright (c) 2022, Ampere Computing LLC.
 */

#ifndef _UAPI_LINUX_IPMI_SSIF_BMC_H
#define _UAPI_LINUX_IPMI_SSIF_BMC_H

#include <linux/types.h>

/* Max length of ipmi ssif message included netfn and cmd field */
#define IPMI_SSIF_PAYLOAD_MAX         254
struct ipmi_ssif_msg_header {
	unsigned int len;
	u8 msg_num;
} __attribute((packed));

struct ipmi_ssif_msg {
	struct ipmi_ssif_msg_header header;
	__u8    payload[IPMI_SSIF_PAYLOAD_MAX];
};

#endif /* _UAPI_LINUX_IPMI_SSIF_BMC_H */
