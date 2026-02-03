/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP I2C Internal Definitions
 * Shared between mctp-i2c.c and mctp-i2c-error-inject.c
 */

#ifndef __MCTP_I2C_INTERNAL_H
#define __MCTP_I2C_INTERNAL_H

#include <linux/i2c.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/skbuff.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>

#include "mctp-i2c-error-inject.h"

/* Constants from mctp-i2c.c */
#define MCTP_I2C_MAXBLOCK 255
#define MCTP_I2C_BUFSZ (3 + MCTP_I2C_MAXBLOCK + 1)

/* Forward declaration */
struct mctp_i2c_client;

/* The netdev structure. One of these per I2C adapter. */
struct mctp_i2c_dev {
	struct net_device *ndev;
	struct i2c_adapter *adapter;
	struct mctp_i2c_client *client;
	struct list_head list; /* For mctp_i2c_client.devs */

	size_t rx_pos;
	u8 rx_buffer[MCTP_I2C_BUFSZ];
	struct completion rx_done;

	struct task_struct *tx_thread;
	wait_queue_head_t tx_wq;
	struct sk_buff_head tx_queue;
	u8 tx_scratch[MCTP_I2C_BUFSZ];

	/* A fake entry in our tx queue to perform an unlock operation */
	struct sk_buff unlock_marker;

	/* NACK retry parameters */
	u32 nack_retries;
	u32 nack_retry_delay_us;
	u32 nack_retry_addr;
	u32 nack_total_retries;
	u32 nack_recovered;
	u32 nack_failed;
	u32 nack_max_depth;

	/* Spinlock protects i2c_lock_count, release_count, allow_rx */
	spinlock_t lock;
	int i2c_lock_count;
	int release_count;
	/* Indicates that the netif is ready to receive incoming packets */
	bool allow_rx;
	bool flows_enabled; /* Runtime flow control */

	/* Error injection support */
	struct mctp_i2c_error_inject error_inject;
	struct dentry *debugfs_dir;
	
	/* Per-EID statistics tracking - SINGLE source of truth
	 *
	 * All statistics are tracked per-endpoint-ID (EID). Two special EIDs:
	 * - EID 0: "null endpoint" - valid packets with EID=0 (unallocated endpoint)
	 * - EID 256 (MCTP_EID_UNKNOWN): errors where EID could not be determined
	 *   (pre-parse errors, allocation failures, etc.)
	 */
	struct {
		DECLARE_BITMAP(active, 257);  /* Which EIDs have activity */
		struct mctp_i2c_eid_stats {
			/* RX stats */
			u64 rx_drop_invalid_cmd;      /* Invalid I2C command (tracked as UNKNOWN) */
			u64 rx_drop_no_memory;
			u64 rx_drop_invalid_pec;      /* PEC check failed (tracked as UNKNOWN) */
			u64 rx_drop_fragment_error;
			u64 rx_drop_not_ready;
			u64 rx_early_exit_no_rx;      /* RX not allowed (tracked as UNKNOWN) */
			
		/* TX stats */
		u64 tx_drop_no_memory;             /* -ENOMEM: Memory allocation failure */
		u64 tx_drop_busy;                  /* -EBUSY: Bus busy too long */
		u64 tx_drop_no_device;             /* -ENXIO: No device at address (no ACK) */
		u64 tx_drop_enodev;                /* -ENODEV: Device not found */
		u64 tx_drop_arbitration_lost;      /* -EAGAIN: Lost arbitration */
		u64 tx_drop_proto_error;           /* -EPROTO: Protocol violation */
		u64 tx_drop_timeout;               /* -ETIMEDOUT: Transfer timeout */
		u64 tx_drop_eopnotsupp;            /* -EOPNOTSUPP: Operation not supported */
		u64 tx_drop_einval;                /* -EINVAL: Invalid parameter */
		u64 tx_drop_eafnosupport;          /* -EAFNOSUPPORT: 10-bit address not supported */
		u64 tx_drop_ebadmsg;               /* -EBADMSG: Invalid SMBus PEC */
		u64 tx_drop_eshutdown;             /* -ESHUTDOWN: Adapter suspended */
		u64 tx_drop_eio;                   /* -EIO: Generic I/O error */
		u64 tx_drop_invalid_len;           /* Invalid packet length */
		u64 tx_drop_queue_full;            /* TX queue full */
		u64 tx_drop_flow_invalid;          /* Invalid flow state */
		u64 tx_early_exit_stopped;         /* TX thread stopped (tracked as UNKNOWN) */
			
			/* Retry/requeue */
			u64 tx_retries_attempted;
			u64 tx_retry_success;
			u64 tx_retry_exhausted;
			u64 tx_requeued;
			/* NACK retry stats */
			u64 tx_nack;
			u64 tx_nack_retries;
			u64 tx_nack_retry_depth;
		} eid[257];
	} eid_stats;	
};

#endif /* __MCTP_I2C_INTERNAL_H */

