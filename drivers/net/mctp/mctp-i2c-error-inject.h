/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP I2C Error Injection Infrastructure
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over I2C binding.
 */

#ifndef __MCTP_I2C_ERROR_INJECT_H
#define __MCTP_I2C_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>

/* Forward declarations */
struct dentry;
struct mctp_i2c_client;

/* Error injection modes - unified with USB */
enum mctp_error_inject_mode {
	MCTP_ERR_MODE_ALWAYS,
	MCTP_ERR_MODE_RANDOM,
	MCTP_ERR_MODE_COUNT
};

/* Error injection control structure - unified with USB where possible */
struct mctp_i2c_error_inject {
	bool enable_tx;  /* TX error injection enable */
	bool enable_rx;  /* RX/fragment error injection enable */
	enum mctp_error_inject_mode mode;
	
	/* TX injection - I2C only has one synchronous path */
	int i2c_tx_error_code;   /* Error code to inject (ENXIO, EAGAIN, EBUSY, etc.) */
	u32 i2c_tx_error_rate;   /* Percentage (0-100) or count */
	u32 i2c_tx_inject_count; /* Counter for count mode */
	u32 i2c_tx_errors_injected;
	
	/* Fragment injection - same as USB */
	bool enable_fragment_drop;      /* Drop 2nd+ fragments */
	bool enable_seq_corrupt;        /* Corrupt sequence number in middle/end fragments */
	bool enable_som_clear;          /* Clear SOM bit in first fragment */
	u32 fragments_dropped;
	u32 seq_corruptions;
	u32 som_clears;
	
	/* Delay injection */
	u32 delay_ms;
	
	/* EID filtering - unified with USB */
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

/* Forward declaration - full definition in mctp-i2c.c */
struct mctp_i2c_dev;

/* Module init/exit functions */
int mctp_i2c_error_inject_module_init(void);
void mctp_i2c_error_inject_module_exit(void);

/* Public API for main driver */
void mctp_i2c_error_inject_init(struct mctp_i2c_dev *midev);
void mctp_i2c_error_inject_cleanup(struct mctp_i2c_dev *midev);

int mctp_i2c_error_inject_tx(struct mctp_i2c_dev *midev, struct sk_buff *skb);
int mctp_i2c_error_inject_fragment(struct mctp_i2c_dev *midev, struct sk_buff *skb);

#endif /* __MCTP_I2C_ERROR_INJECT_H */


