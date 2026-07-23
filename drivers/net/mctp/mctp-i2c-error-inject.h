/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP I2C Error Injection Infrastructure
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over I2C binding.
 *
 * Generic plumbing (mode parsing, injection decision, EID filtering, fragment
 * mutation and the generic debugfs files) lives in mctp-error-inject-common.c.
 * Only I2C-specific error codes and hook signatures are kept here.
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

#include <net/mctp-error-inject-common.h>

/* Forward declarations */
struct dentry;
struct mctp_i2c_client;

/* Error injection control structure - common state plus I2C-specific fields. */
struct mctp_i2c_error_inject {
	struct mctp_ei_common common;

	/* TX injection - I2C only has one synchronous path */
	int i2c_tx_error_code;   /* Error code to inject (ENXIO, EAGAIN, EBUSY, etc.) */
	u32 i2c_tx_error_rate;   /* Percentage (0-100) or count */
	u32 i2c_tx_inject_count; /* Counter for count mode */
	u32 i2c_tx_errors_injected;

	/* debugfs descriptors for the I2C-specific scalar files */
	struct mctp_ei_scalar tx_error_code_desc;
	struct mctp_ei_scalar tx_error_rate_desc;
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
