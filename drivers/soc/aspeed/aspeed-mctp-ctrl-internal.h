/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Aspeed MCTP Controller internal definitions
 * Shared between aspeed-mctp-ctrl.c and aspeed-mctp-ctrl-error-inject.c
 */

#ifndef __ASPEED_MCTP_CTRL_INTERNAL_H
#define __ASPEED_MCTP_CTRL_INTERNAL_H

#include <linux/netdevice.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "aspeed-mctp-ctrl-error-inject.h"

struct dentry;
struct regmap;
struct reset_control;

enum rx_mode {
	/* RX fast path: when we can trust hardware pointers.
	 *
	 * This path only works if hardware pointers are consistent
	 * with sofware pointers and we can rely on the pointers to
	 * detect RX packets presense.
	 */
	RX_MODE_FAST,

	/* RX warmup path: peeks at packet headers to detect packet arrival.
	 *
	 * We have to do this in some cases where hardware pointers are unreliable
	 * after reset. This is suboptimal so we should switch to fast path
	 * when hardware pointers stablize after a few loops.
	 */
	RX_MODE_WARMUP,
};

struct tx_ring {
	u64 *cmd;
	dma_addr_t cmd_paddr;
	u8 *pkt;
	dma_addr_t pkt_paddr;
	unsigned int pkt_size;
	unsigned int pkt_count;

	u8 next_xmit;
	u8 next_reclaim;
};

struct rx_ring {
	u32 *cmd;
	dma_addr_t cmd_paddr;
	u8 *pkt;
	dma_addr_t pkt_paddr;
	unsigned int pkt_size;
	unsigned int pkt_count;

	u8 next_read;
	u8 next_refill;
	u8 mode;

	u64 scan_counter;
	u8 scan_hw_offset;
};

struct aspeed_mctp_ctrl_match_data {
	u8 starting_mode;
	u32 tx_max_payload_size_regval;
	u32 rx_max_payload_size_regval;
	u32 max_payload_size;
};

struct aspeed_mctp_ctrl {
	struct net_device *ndev;
	struct regmap *map;
	struct regmap *map_pcie;
	struct reset_control *reset;
	struct reset_control *reset_dma;
	const struct aspeed_mctp_ctrl_match_data *match_data;

	struct tx_ring tx;
	struct rx_ring rx;

	bool rc_f;
	int irq_mctp;
	int irq_perst_lo;
	int irq_perst_hi;
	u32 rx_poll_interval_jiffies;

	struct napi_struct napi;
	struct delayed_work bdf_work;
	struct work_struct rst_work;
	struct timer_list rx_poll_timer;

	struct aspeed_mctp_error_inject error_inject;
	struct dentry *debugfs_dir;
};

#endif /* __ASPEED_MCTP_CTRL_INTERNAL_H */
