/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP per-net structures
 */

#ifndef __NETNS_MCTP_H__
#define __NETNS_MCTP_H__

#include <linux/mutex.h>
#include <linux/types.h>

struct netns_mctp {
	/* Only updated under RTNL, entries freed via RCU */
	struct list_head routes;

	/* Bound sockets: list of sockets bound by type.
	 * This list is updated from non-atomic contexts (under bind_lock),
	 * and read (under rcu) in packet rx
	 */
	struct mutex bind_lock;
	struct hlist_head binds;

	/* tag allocations. This list is read and updated from atomic contexts,
	 * but elements are free()ed after a RCU grace-period
	 */
	spinlock_t keys_lock;
	struct hlist_head keys;

	/* Persistent "next tag" hint per (net, peer) for incremental allocation */
	struct hlist_head tag_hints;

	/* MCTP network */
	unsigned int default_net;

	/* neighbour table */
	struct mutex neigh_lock;
	struct list_head neighbours;

	/* Global socket statistics (atomic for lockless updates) */
	atomic_t num_sockets;
	atomic_t num_bound_sockets;

	atomic64_t tx_bytes;
	atomic64_t tx_packets;
	atomic64_t tx_messages;
	atomic64_t tx_errors;
	atomic64_t tx_drops;

	atomic64_t rx_bytes;
	atomic64_t rx_packets;
	atomic64_t rx_messages;
	atomic64_t rx_errors;
	atomic64_t rx_drops;

	/* Detailed drop reasons - RX */
	atomic64_t rx_dropped_no_route;
	atomic64_t rx_dropped_no_memory;
	atomic64_t rx_dropped_seq_mismatch;
	atomic64_t rx_dropped_tag_mismatch;
	atomic64_t rx_dropped_queue_full;
	atomic64_t rx_dropped_invalid_header;
	atomic64_t rx_dropped_permission;
	atomic64_t rx_dropped_timeout;

	/* Detailed drop reasons - TX */
	atomic64_t tx_dropped_no_route;
	atomic64_t tx_dropped_mtu_exceeded;
	atomic64_t tx_dropped_no_memory;
	atomic64_t tx_dropped_queue_full;
	atomic64_t tx_dropped_device_down;
	atomic64_t tx_dropped_tag_exhaustion;
	atomic64_t tx_dropped_permission;
	atomic64_t tx_dropped_bad_addrlen;
};

#endif /* __NETNS_MCTP_H__ */
