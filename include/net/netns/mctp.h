/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP per-net structures
 */

#ifndef __NETNS_MCTP_H__
#define __NETNS_MCTP_H__

#include <linux/hash.h>
#include <linux/hashtable.h>
#include <linux/mutex.h>
#include <linux/types.h>

#define MCTP_BINDS_BITS 7

struct netns_mctp {
	/* Only updated under RTNL, entries freed via RCU */
	struct list_head routes;

	/* Bound sockets: hash table of sockets, keyed by
	 * (type, src_eid, dest_eid).
	 * Specific src_eid/dest_eid entries also have an entry for
	 * MCTP_ADDR_ANY. This list is updated from non-atomic contexts
	 * (under bind_lock), and read (under rcu) in packet rx.
	 */
	struct mutex bind_lock;
	DECLARE_HASHTABLE(binds, MCTP_BINDS_BITS);

	/* tag allocations. This list is read and updated from atomic contexts,
	 * but elements are free()ed after a RCU grace-period
	 */
	spinlock_t keys_lock;
	struct hlist_head keys;

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

static inline u32 mctp_bind_hash(u8 type, u8 local_addr, u8 peer_addr)
{
	return hash_32(type | (u32)local_addr << 8 | (u32)peer_addr << 16,
		       MCTP_BINDS_BITS);
}

#endif /* __NETNS_MCTP_H__ */
