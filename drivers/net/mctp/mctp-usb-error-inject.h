/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * mctp-usb-error-inject.h - Error injection infrastructure for MCTP USB
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef _MCTP_USB_ERROR_INJECT_H
#define _MCTP_USB_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>

struct mctp_usb;

/* Error injection modes */
enum mctp_error_inject_mode {
	MCTP_ERR_MODE_ALWAYS,
	MCTP_ERR_MODE_RANDOM,
	MCTP_ERR_MODE_COUNT
};

/* Per-interface error injection state */
struct mctp_error_inject {
	bool enable_tx;  /* Separate enable for TX path */
	bool enable_rx;  /* Separate enable for RX path */
	enum mctp_error_inject_mode mode;
	
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
	
	/* Fragment injection - drop, corrupt sequence, clear SOM */
	bool enable_fragment_drop;      /* Drop 2nd+ fragments */
	bool enable_seq_corrupt;        /* Corrupt sequence number in middle/end fragments */
	bool enable_som_clear;          /* Clear SOM bit in first fragment */
	u32 fragments_dropped;
	u32 seq_corruptions;
	u32 som_clears;
	
	/* Delay injection */
	u32 delay_ms;
	
	/* EID filtering */
	struct {
		bool enabled;
		u8 src_eid;      /* 0 = any */
		u8 dest_eid;     /* 0 = any */
		u8 msg_type;     /* 0 = any */
	} eid_filter;
	
	/* Statistics */
	u64 total_packets_processed;
	u64 total_errors_injected;
	
	/* RNG state */
	struct rnd_state rng;
	spinlock_t lock;
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

