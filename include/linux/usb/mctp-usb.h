/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * mctp-usb.h - MCTP USB transport binding: common definitions.
 *
 * These are shared between the host and gadget drivers.
 *
 * Copyright (C) 2024 Code Construct Pty Ltd
 */

#ifndef __LINUX_USB_MCTP_USB_H
#define __LINUX_USB_MCTP_USB_H

#include <linux/types.h>

struct mctp_usb_hdr {
	__le16	id;
	__u8	rsvd;
	__u8	len;
} __packed;

#define MCTP_USB_XFER_SIZE	512
#define MCTP_USB_BTU		68
#define MCTP_USB_MTU_MIN	MCTP_USB_BTU
#define MCTP_USB_MTU_MAX	(U8_MAX - sizeof(struct mctp_usb_hdr))
#define MCTP_USB_DMTF_ID	0x1ab4

#endif /*  __LINUX_USB_MCTP_USB_H */
