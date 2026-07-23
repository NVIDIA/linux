/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * mctp-usb-error-inject.h - Error injection infrastructure for MCTP USB
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Generic plumbing lives in mctp-error-inject-common.c.  USB keeps its
 * transport-specific fields (separate sync/async TX and RX URB error codes)
 * and the sync/async/RX/fragment hook signatures here.
 */

#ifndef _MCTP_USB_ERROR_INJECT_H
#define _MCTP_USB_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>

#include <net/mctp-error-inject-common.h>

struct mctp_usb;
struct urb;

/* Per-interface error injection state - common state plus USB-specific fields. */
struct mctp_error_inject {
	struct mctp_ei_common common;

	/* TX injection - separate sync and async */
	int urb_tx_sync_error_code;   /* Sync error (URB submission) */
	int urb_tx_async_error_code;  /* Async error (URB completion) */
	u32 urb_tx_error_rate;        /* Percentage (0-100) or count */
	u32 urb_tx_sync_inject_count; /* Counter for sync count mode */
	u32 urb_tx_async_inject_count;/* Counter for async count mode */
	u32 urb_tx_sync_errors_injected;
	u32 urb_tx_async_errors_injected;

	/* RX injection */
	int urb_rx_error_code;
	u32 urb_rx_error_rate;
	u32 urb_rx_inject_count;
	u32 urb_rx_errors_injected;

	/* debugfs descriptors for the USB-specific scalar files */
	struct mctp_ei_scalar tx_sync_error_code_desc;
	struct mctp_ei_scalar tx_async_error_code_desc;
	struct mctp_ei_scalar tx_error_rate_desc;
	struct mctp_ei_scalar rx_error_code_desc;
	struct mctp_ei_scalar rx_error_rate_desc;
};

/* Module init/exit functions */
int mctp_usb_error_inject_module_init(void);
void mctp_usb_error_inject_module_exit(void);

/* Public API for main driver */
void mctp_usb_error_inject_init(struct mctp_usb *mctp_usb);
void mctp_usb_error_inject_cleanup(struct mctp_usb *mctp_usb);

int mctp_usb_error_inject_tx_sync(struct mctp_usb *mctp_usb, struct sk_buff *skb);
int mctp_usb_error_inject_tx_async(struct mctp_usb *mctp_usb, struct urb *urb);
int mctp_usb_error_inject_rx(struct mctp_usb *mctp_usb, int original_status);
int mctp_usb_error_inject_fragment(struct mctp_usb *mctp_usb, struct sk_buff *skb);

#endif /* _MCTP_USB_ERROR_INJECT_H */
