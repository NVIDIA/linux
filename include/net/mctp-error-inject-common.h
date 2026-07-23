/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP Error Injection - common downstream helper
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * This header carries the diagnostic error-injection plumbing that is shared
 * by every MCTP transport binding (I2C, SPI, USB, ...).  Each binding embeds a
 * struct mctp_ei_common and keeps only its transport-specific error codes and
 * hook signatures local.  The goal is to keep upstream-owned binding files thin
 * so future LTS/OpenBMC syncs reattach small hooks instead of re-merging
 * thousands of lines of duplicated debugfs/mode/filter/stats code.
 *
 * This file is downstream-only.  See MCTP_DOWNSTREAM_API.md.
 */

#ifndef __MCTP_ERROR_INJECT_COMMON_H
#define __MCTP_ERROR_INJECT_COMMON_H

#include <linux/types.h>
#include <linux/bits.h>
#include <linux/random.h>
#include <linux/prandom.h>
#include <linux/spinlock.h>

/* Forward declarations to keep this header light. */
struct dentry;
struct sk_buff;
struct net_device;
struct mctp_hdr;

/* Error injection modes - shared across all bindings. */
enum mctp_error_inject_mode {
	MCTP_ERR_MODE_ALWAYS,
	MCTP_ERR_MODE_RANDOM,
	MCTP_ERR_MODE_COUNT
};

/*
 * Common error-injection state shared by all transport bindings.
 *
 * A binding embeds this as its first member and adds transport-specific error
 * codes/rates alongside it, e.g.:
 *
 *   struct mctp_i2c_error_inject {
 *           struct mctp_ei_common common;
 *           int i2c_tx_error_code;
 *           ...
 *   };
 *
 * All fields are protected by @lock except @rng which is only touched under it.
 */
struct mctp_ei_common {
	bool enable_tx;			/* TX error injection enable */
	bool enable_rx;			/* RX/fragment error injection enable */
	enum mctp_error_inject_mode mode;

	/* Delay injection (milliseconds) applied on the TX hook path. */
	u32 delay_ms;

	/* EID filter - src/dest of 0 means "any". */
	struct {
		bool enabled;
		u8 src_eid;
		u8 dest_eid;
		u8 msg_type;
	} eid_filter;

	/* Fragment injection (not used by TX-only bindings such as SPI). */
	bool enable_fragment_drop;	/* Drop 2nd+ fragments */
	bool enable_seq_corrupt;	/* Corrupt sequence in middle/end fragments */
	bool enable_som_clear;		/* Clear SOM bit in first fragment */
	u32 fragments_dropped;
	u32 seq_corruptions;
	u32 som_clears;

	/* Aggregate statistics. */
	u64 total_packets_processed;
	u64 total_errors_injected;

	struct rnd_state rng;
	spinlock_t lock;
};

/*
 * Capability flags selecting which generic debugfs files a binding wants.
 * enable_tx and mode are always created.
 */
#define MCTP_EI_CAP_RX_ENABLE	BIT(0)	/* enable_rx */
#define MCTP_EI_CAP_FRAGMENT	BIT(1)	/* enable_fragment_drop/seq_corrupt/som_clear */
#define MCTP_EI_CAP_DELAY	BIT(2)	/* delay_ms */
#define MCTP_EI_CAP_EID_FILTER	BIT(3)	/* eid_filter/ subdirectory */

/* Generic scalar debugfs file descriptor (for transport-specific error codes). */
enum mctp_ei_scalar_type {
	MCTP_EI_SCALAR_INT,	/* signed, printed/parsed as %d */
	MCTP_EI_SCALAR_U32,	/* unsigned, printed/parsed as %u */
};

/*
 * Descriptor backing one transport-specific scalar debugfs file.  The binding
 * must keep one of these per file alive for the lifetime of the debugfs node
 * (typically as a member of the per-device error-inject struct).
 */
struct mctp_ei_scalar {
	enum mctp_ei_scalar_type type;
	void *ptr;		/* points at the u32/int field */
	spinlock_t *lock;	/* usually &common.lock */
};

/* Initialise common state (seed RNG, init lock, default mode). */
void mctp_ei_common_init(struct mctp_ei_common *c);

/* Human-readable mode string ("always"/"random"/"count"/"unknown"). */
const char *mctp_ei_mode_str(enum mctp_error_inject_mode mode);

/*
 * Decide whether to inject based on mode and @rate; @count is decremented in
 * COUNT mode.  Takes @c->lock internally.
 */
bool mctp_ei_should_inject(struct mctp_ei_common *c, u32 rate, u32 *count);

/*
 * EID filter match on src/dest.  Returns true (inject-eligible) when the filter
 * is disabled.  When enabled, a NULL header does not match.
 */
bool mctp_ei_match_filter(struct mctp_ei_common *c, const struct mctp_hdr *mh);

/*
 * Apply fragment mutations (SOM clear / sequence corrupt / drop) to @skb.
 * Returns 0 to pass the packet through, 1 to drop it.  @ndev is used for debug
 * logging only.
 */
int mctp_ei_fragment(struct mctp_ei_common *c, struct sk_buff *skb,
		     struct net_device *ndev);

/* Reset all common state and counters.  Caller MUST hold @c->lock. */
void mctp_ei_common_reset(struct mctp_ei_common *c);

/* Create the generic debugfs files for @c under @dir per @caps. */
void mctp_ei_common_debugfs_create(struct dentry *dir, struct mctp_ei_common *c,
				   u32 caps);

/*
 * Create one transport-specific scalar debugfs file.  @desc is filled in and
 * used as the file's private data, so it must outlive the file.
 */
void mctp_ei_create_scalar_file(struct dentry *dir, const char *name,
				umode_t mode, struct mctp_ei_scalar *desc,
				enum mctp_ei_scalar_type type, void *ptr,
				spinlock_t *lock);

#endif /* __MCTP_ERROR_INJECT_COMMON_H */
