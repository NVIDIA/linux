/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Aspeed MCTP Controller Error Injection Infrastructure
 *
 * Copyright (c) 2026 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over the Aspeed PCIe VDM controller. Mirrors the
 * drivers/net/mctp/mctp-i2c-error-inject.{c,h} interface.
 */

#ifndef __ASPEED_MCTP_CTRL_ERROR_INJECT_H
#define __ASPEED_MCTP_CTRL_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/list.h>

struct dentry;
struct aspeed_mctp_ctrl;

enum aspeed_mctp_error_inject_mode {
	ASPEED_MCTP_ERR_MODE_ALWAYS,
	ASPEED_MCTP_ERR_MODE_RANDOM,
	ASPEED_MCTP_ERR_MODE_COUNT,
};

struct aspeed_mctp_error_inject {
	bool enable_tx;
	bool enable_rx;
	enum aspeed_mctp_error_inject_mode mode;

	/* TX injection */
	int tx_error_code;
	u32 tx_error_rate;
	u32 tx_inject_count;
	u32 tx_errors_injected;

	/* Fragment injection */
	bool enable_fragment_drop;
	bool enable_seq_corrupt;
	bool enable_som_clear;
	u32 fragments_dropped;
	u32 seq_corruptions;
	u32 som_clears;

	u32 delay_ms;

	struct {
		bool enabled;
		u8 src_eid;
		u8 dest_eid;
		u8 msg_type;
	} eid_filter;

	u64 total_packets_processed;
	u64 total_errors_injected;

	struct rnd_state rng;
	spinlock_t lock;
};

int aspeed_mctp_error_inject_module_init(void);
void aspeed_mctp_error_inject_module_exit(void);

void aspeed_mctp_error_inject_init(struct aspeed_mctp_ctrl *priv);
void aspeed_mctp_error_inject_cleanup(struct aspeed_mctp_ctrl *priv);

int aspeed_mctp_error_inject_tx(struct aspeed_mctp_ctrl *priv,
				struct sk_buff *skb);
int aspeed_mctp_error_inject_fragment(struct aspeed_mctp_ctrl *priv,
				      struct sk_buff *skb);
void aspeed_mctp_error_inject_filter_list(struct aspeed_mctp_ctrl *priv,
					  struct list_head *skb_list);

#endif /* __ASPEED_MCTP_CTRL_ERROR_INJECT_H */
