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

	/* Spinlock protects i2c_lock_count, release_count, allow_rx */
	spinlock_t lock;
	int i2c_lock_count;
	int release_count;
	/* Indicates that the netif is ready to receive incoming packets */
	bool allow_rx;

	/* Error injection support */
	struct mctp_i2c_error_inject error_inject;
	struct dentry *debugfs_dir;
};

#endif /* __MCTP_I2C_INTERNAL_H */

