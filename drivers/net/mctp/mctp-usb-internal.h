/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP USB Internal Definitions
 * Shared between mctp-usb.c and mctp-usb-error-inject.c
 */

#ifndef __MCTP_USB_INTERNAL_H
#define __MCTP_USB_INTERNAL_H

#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>

#include "mctp-usb-error-inject.h"

/* Number of IN/OUT URBs to queue */
#define MCTP_USB_N_RX_QUEUE 8
#define MCTP_USB_N_TX_QUEUE 8

/* Main device structure - one per USB interface */
struct mctp_usb {
	struct usb_device *usbdev;
	struct usb_interface *intf;
	bool stopped;

	struct net_device *netdev;

	u8 ep_in;
	u8 ep_out;

	struct usb_anchor rx_anchor;
	struct usb_anchor tx_anchor;
	/* number of urbs currently queued */
	atomic_t rx_qlen, tx_qlen;

	struct delayed_work rx_retry_work;

	/* TX batching support - controlled via sysfs */
	bool tx_batching_enabled;
	
	/* Error injection support */
	struct mctp_error_inject error_inject;
	struct dentry *debugfs_dir;
	
	/* Per-EID statistics tracking - SINGLE source of truth
	 *
	 * All statistics are tracked per-endpoint-ID (EID). Two special EIDs:
	 * - EID 0: "null endpoint" - valid packets with EID=0 (unallocated endpoint)
	 * - EID 256 (MCTP_EID_UNKNOWN): errors where EID could not be determined
	 *   (URB completion callbacks, allocation failures, pre-parse errors)
	 */
	struct {
		DECLARE_BITMAP(active, 257);  /* Which EIDs have activity */
		struct mctp_usb_eid_stats {
			/* RX stats */
			u64 rx_drop_no_memory;
			u64 rx_drop_urb_error;
			u64 rx_drop_invalid_len;
			u64 rx_drop_parse_error;
			u64 rx_drop_fragment_error;
			/* TX stats - Valid USB error codes only */
			u64 tx_drop_no_memory;        /* -ENOMEM */
			u64 tx_drop_urb_error;        /* Generic/unknown errors */
			u64 tx_drop_ebusy;            /* -EBUSY: URB already active */
			u64 tx_drop_enodev;           /* -ENODEV: Device/bus doesn't exist */
			u64 tx_drop_enoent;           /* -ENOENT: Interface/endpoint doesn't exist */
			u64 tx_drop_enxio;            /* -ENXIO: Host controller doesn't support URB type */
			u64 tx_drop_einval;           /* -EINVAL: Invalid transfer parameters */
			u64 tx_drop_exdev;            /* -EXDEV: ISO frames expired */
			u64 tx_drop_efbig;            /* -EFBIG: Too many ISO frames */
			u64 tx_drop_epipe;            /* -EPIPE: Pipe type mismatch */
			u64 tx_drop_emsgsize;         /* -EMSGSIZE: Transfer length invalid */
			u64 tx_drop_enospc;           /* -ENOSPC: Bandwidth overcommit */
			u64 tx_drop_eshutdown;        /* -ESHUTDOWN: Device/HC disabled */
			u64 tx_drop_eperm;            /* -EPERM: URB rejected */
			u64 tx_drop_ehostunreach;     /* -EHOSTUNREACH: Device suspended */
			u64 tx_drop_enoexec;          /* -ENOEXEC: Control URB missing Setup packet */
			u64 tx_drop_queue_full;       /* Queue full condition */
			u64 tx_requeued;
			u64 rx_requeued;
			u64 rx_urb_submitted;         /* RX URBs submitted */
			u64 tx_urb_submitted;         /* TX URBs submitted */
		} eid[257];
	} eid_stats;	
};

/* Structure to track batched packets in URB context */
struct mctp_usb_batch_ctx {
	struct net_device *netdev;
	struct sk_buff_head skbs;
	unsigned int num_packets;
};

#endif /* __MCTP_USB_INTERNAL_H */


