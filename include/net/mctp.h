/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Management Component Transport Protocol (MCTP)
 *
 * Copyright (c) 2021 Code Construct
 * Copyright (c) 2021 Google
 */

#ifndef __NET_MCTP_H
#define __NET_MCTP_H

#include <linux/bits.h>
#include <linux/mctp.h>
#include <linux/netdevice.h>
#include <linux/timekeeping.h>
#include <net/net_namespace.h>
#include <net/sock.h>

/* MCTP packet definitions */
struct mctp_hdr {
	u8	ver;
	u8	dest;
	u8	src;
	u8	flags_seq_tag;
};

#define MCTP_VER_MIN	1
#define MCTP_VER_MAX	1

/* Definitions for ver field */
#define MCTP_HDR_VER_MASK	GENMASK(3, 0)

/* Definitions for flags_seq_tag field */
#define MCTP_HDR_FLAG_SOM	BIT(7)
#define MCTP_HDR_FLAG_EOM	BIT(6)
#define MCTP_HDR_FLAG_TO	BIT(3)
#define MCTP_HDR_FLAGS		GENMASK(5, 3)
#define MCTP_HDR_SEQ_SHIFT	4
#define MCTP_HDR_SEQ_MASK	GENMASK(1, 0)
#define MCTP_HDR_TAG_SHIFT	0
#define MCTP_HDR_TAG_MASK	GENMASK(2, 0)

#define MCTP_INITIAL_DEFAULT_NET	1
#define MCTP_DEFAULT_LIFETIME		(6 * CONFIG_HZ)

static inline bool mctp_address_unicast(mctp_eid_t eid)
{
	return eid >= 8 && eid < 255;
}

static inline bool mctp_address_broadcast(mctp_eid_t eid)
{
	return eid == 255;
}

static inline bool mctp_address_null(mctp_eid_t eid)
{
	return eid == 0;
}

static inline bool mctp_address_matches(mctp_eid_t match, mctp_eid_t eid)
{
	return match == eid || match == MCTP_ADDR_ANY;
}

static inline struct mctp_hdr *mctp_hdr(struct sk_buff *skb)
{
	return (struct mctp_hdr *)skb_network_header(skb);
}

/* socket implementation */
struct mctp_sock {
	struct sock	sk;

	/* bind() params */
	unsigned int	bind_net;
	mctp_eid_t	bind_local_addr;
	mctp_eid_t	bind_peer_addr;
	unsigned int	bind_peer_net;
	bool		bind_peer_set;
	__u8		bind_type;

	/* sendmsg()/recvmsg() uses struct sockaddr_mctp_ext */
	bool		addr_ext;

	/* list of mctp_sk_key, for incoming tag lookup. updates protected
	 * by sk->net->keys_lock
	 */
	struct hlist_head keys;

	/* mechanism for expiring allocated keys; will release an allocated
	 * tag, and any netdev state for a request/response pairing
	 */
	struct timer_list key_expiry;

	/* Error queue control */
	bool		enable_errqueue;

	/* Deferred error reporting (to avoid deadlock in timer context) */
	struct work_struct error_report_work;
	struct list_head pending_errors;
	spinlock_t error_queue_lock;

	/* Per-socket statistics */
	struct {
		u64 tx_bytes;
		u64 tx_packets;
		u64 tx_messages;
		u64 tx_errors;
		u64 tx_drops;

		u64 rx_bytes;
		u64 rx_packets;
		u64 rx_messages;
		u64 rx_errors;
		u64 rx_drops;

		/* Detailed drop reasons - RX */
		u64 rx_dropped_no_route;
		u64 rx_dropped_no_memory;
		u64 rx_dropped_seq_mismatch;
		u64 rx_dropped_tag_mismatch;
		u64 rx_dropped_queue_full;
		u64 rx_dropped_invalid_header;
		u64 rx_dropped_permission;
		u64 rx_dropped_timeout;

		/* Detailed drop reasons - TX */
		u64 tx_dropped_no_route;
		u64 tx_dropped_mtu_exceeded;
		u64 tx_dropped_no_memory;
		u64 tx_dropped_queue_full;
		u64 tx_dropped_device_down;
		u64 tx_dropped_tag_exhaustion;
		u64 tx_dropped_permission;
		u64 tx_dropped_bad_addrlen;

		/* Timestamps */
		u64 last_tx_time;
		u64 last_rx_time;
	} stats;

	spinlock_t stats_lock;  /* Protects stats */
	pid_t pid;		/* PID of the creating process */
};

struct mctp_pending_error {
	struct list_head list;
	struct sk_buff *skb;
	struct sock *sk;
	int error_code;
	struct net_device *dev;
	u8 direction;
	u8 binding;
	u8 orig_msg_type;
	u16 orig_payload_len;
	u8 orig_payload[32];
};

/* Key for matching incoming packets to sockets or reassembly contexts.
 * Packets are matched on (peer EID, local EID, tag).
 *
 * Lifetime / locking requirements:
 *
 *  - individual key data (ie, the struct itself) is protected by key->lock;
 *    changes must be made with that lock held.
 *
 *  - the lookup fields: peer_addr, local_addr and tag are set before the
 *    key is added to lookup lists, and never updated.
 *
 *  - A ref to the key must be held (throuh key->refs) if a pointer to the
 *    key is to be accessed after key->lock is released.
 *
 *  - a mctp_sk_key contains a reference to a struct sock; this is valid
 *    for the life of the key. On sock destruction (through unhash), the key is
 *    removed from lists (see below), and marked invalid.
 *
 * - these mctp_sk_keys appear on two lists:
 *     1) the struct mctp_sock->keys list
 *     2) the struct netns_mctp->keys list
 *
 *   presences on these lists requires a (single) refcount to be held; both
 *   lists are updated as a single operation.
 *
 *   Updates and lookups in either list are performed under the
 *   netns_mctp->keys lock. Lookup functions will need to lock the key and
 *   take a reference before unlocking the keys_lock. Consequently, the list's
 *   keys_lock *cannot* be acquired with the individual key->lock held.
 *
 * - a key may have a sk_buff attached as part of an in-progress message
 *   reassembly (->reasm_head). The reasm data is protected by the individual
 *   key->lock.
 *
 * - there are two destruction paths for a mctp_sk_key:
 *
 *    - through socket unhash (see mctp_sk_unhash). This performs the list
 *      removal under keys_lock.
 *
 *    - where a key is established to receive a reply message: after receiving
 *      the (complete) reply, or during reassembly errors. Here, we clean up
 *      the reassembly context (marking reasm_dead, to prevent another from
 *      starting), and remove the socket from the netns & socket lists.
 *
 *    - through an expiry timeout, on a per-socket timer
 */
struct mctp_sk_key {
	unsigned int	net;
	mctp_eid_t	peer_addr;
	mctp_eid_t	local_addr; /* MCTP_ADDR_ANY for local owned tags */
	__u8		tag; /* incoming tag match; invert TO for local */

	/* we hold a ref to sk when set */
	struct sock	*sk;

	/* routing lookup list */
	struct hlist_node hlist;

	/* per-socket list */
	struct hlist_node sklist;

	/* lock protects against concurrent updates to the reassembly and
	 * expiry data below.
	 */
	spinlock_t	lock;

	/* Keys are referenced during the output path, which may sleep */
	refcount_t	refs;

	/* incoming fragment reassembly context */
	struct sk_buff	*reasm_head;
	struct sk_buff	**reasm_tailp;
	bool		reasm_dead;
	u8		last_seq;

	/* key validity */
	bool		valid;

	/* expiry timeout; valid (above) cleared on expiry */
	unsigned long	expiry;

	/* free to use for device flow state tracking. Initialised to
	 * zero on initial key creation
	 */
	unsigned long	dev_flow_state;
	struct mctp_dev	*dev;

	/* a tag allocated with SIOCMCTPALLOCTAG ioctl will not expire
	 * automatically on timeout or response, instead SIOCMCTPDROPTAG
	 * is used.
	 */
	bool		manual_alloc;

	/* Original message header for error reporting on fragmented messages */
	u8		orig_msg_type;
	u16		orig_payload_len;
	u8		orig_payload[32];
};

struct mctp_skb_cb {
	unsigned int	magic;
	unsigned int	net;
	/* fields below provide extended addressing for ingress to recvmsg() */
	int		ifindex;
	unsigned char	halen;
	unsigned char	haddr[MAX_ADDR_LEN];
};

/* skb control-block accessors with a little extra debugging for initial
 * development.
 *
 * TODO: remove checks & mctp_skb_cb->magic; replace callers of __mctp_cb
 * with mctp_cb().
 *
 * __mctp_cb() is only for the initial ingress code; we should see ->magic set
 * at all times after this.
 */
static inline struct mctp_skb_cb *__mctp_cb(struct sk_buff *skb)
{
	struct mctp_skb_cb *cb = (void *)skb->cb;

	cb->magic = 0x4d435450;
	return cb;
}

static inline struct mctp_skb_cb *mctp_cb(struct sk_buff *skb)
{
	struct mctp_skb_cb *cb = (void *)skb->cb;

	BUILD_BUG_ON(sizeof(struct mctp_skb_cb) > sizeof(skb->cb));
	WARN_ON(cb->magic != 0x4d435450);
	return cb;
}

/* If CONFIG_MCTP_FLOWS, we may add one of these as a SKB extension,
 * indicating the flow to the device driver.
 */
struct mctp_flow {
	struct mctp_sk_key *key;
};

struct mctp_dst;

/* Route definition.
 *
 * These are held in the pernet->mctp.routes list, with RCU protection for
 * removed routes. We hold a reference to the netdev; routes need to be
 * dropped on NETDEV_UNREGISTER events.
 *
 * Updates to the route table are performed under rtnl; all reads under RCU,
 * so routes cannot be referenced over a RCU grace period.
 */
struct mctp_route {
	mctp_eid_t		min, max;

	unsigned char		type;

	unsigned int		mtu;

	enum {
		MCTP_ROUTE_DIRECT,
		MCTP_ROUTE_GATEWAY,
	} dst_type;
	union {
		struct mctp_dev	*dev;
		struct mctp_fq_addr gateway;
	};

	int			(*output)(struct mctp_dst *dst,
					  struct sk_buff *skb);

	struct list_head	list;
	refcount_t		refs;
	struct rcu_head		rcu;
};

/* Route lookup result: dst. Represents the results of a routing decision,
 * but is only held over the individual routing operation.
 *
 * Will typically be stored on the caller stack, and must be released after
 * usage.
 */
struct mctp_dst {
	struct mctp_dev *dev;
	unsigned int mtu;
	mctp_eid_t nexthop;

	/* set for direct addressing */
	unsigned char halen;
	unsigned char haddr[MAX_ADDR_LEN];

	int (*output)(struct mctp_dst *dst, struct sk_buff *skb);
};

int mctp_dst_from_extaddr(struct mctp_dst *dst, struct net *net, int ifindex,
			  unsigned char halen, const unsigned char *haddr);

/* route interfaces */
int mctp_route_lookup(struct net *net, unsigned int dnet,
		      mctp_eid_t daddr, struct mctp_dst *dst);

void mctp_dst_release(struct mctp_dst *dst);

/* always takes ownership of skb */
int mctp_local_output(struct sock *sk, struct mctp_dst *dst,
		      struct sk_buff *skb, mctp_eid_t daddr, u8 req_tag);

void mctp_key_unref(struct mctp_sk_key *key);
struct mctp_sk_key *mctp_alloc_local_tag(struct mctp_sock *msk,
					 unsigned int netid,
					 mctp_eid_t local, mctp_eid_t peer,
					 bool manual, u8 *tagp,
					 unsigned long lifetime);

/* routing <--> device interface */
unsigned int mctp_default_net(struct net *net);
int mctp_default_net_set(struct net *net, unsigned int index);
int mctp_route_add_local(struct mctp_dev *mdev, mctp_eid_t addr);
int mctp_route_remove_local(struct mctp_dev *mdev, mctp_eid_t addr);
void mctp_route_remove_dev(struct mctp_dev *mdev);
void mctp_key_remove_dev(struct mctp_dev *mdev);

/* neighbour definitions */
enum mctp_neigh_source {
	MCTP_NEIGH_STATIC,
	MCTP_NEIGH_DISCOVER,
};

struct mctp_neigh {
	struct mctp_dev		*dev;
	mctp_eid_t		eid;
	enum mctp_neigh_source	source;

	unsigned char		ha[MAX_ADDR_LEN];

	struct list_head	list;
	struct rcu_head		rcu;
};

int mctp_neigh_init(void);
void mctp_neigh_exit(void);

// ret_hwaddr may be NULL, otherwise must have space for MAX_ADDR_LEN
int mctp_neigh_lookup(struct mctp_dev *dev, mctp_eid_t eid,
		      void *ret_hwaddr);
void mctp_neigh_remove_dev(struct mctp_dev *mdev);

int mctp_routes_init(void);
void mctp_routes_exit(void);

int mctp_device_init(void);
void mctp_device_exit(void);

/* Per-socket statistics support - see net/mctp/stats.c */
int mctp_stats_init(void);
void mctp_stats_exit(void);
void mctp_stats_aggregate_closed_sk(struct sock *sk);

/* Error queue support - see net/mctp/route.c */
void mctp_queue_error(struct sock *sk, struct sk_buff *skb,
		      int error_code, struct net_device *dev, u8 direction,
		      u8 binding, struct mctp_sk_key *key);
struct sock *mctp_lookup_sock_by_key(struct sk_buff *skb, struct net_device *dev,
				     struct mctp_sk_key **found_key);
struct sock *mctp_lookup_sock_for_error(struct sk_buff *skb,
					struct net_device *dev,
					struct mctp_sk_key *key,
					struct mctp_sk_key **found_key);

/* MCTP IDs and Codes from DMTF specification
 * "DSP0239 Management Component Transport Protocol (MCTP) IDs and Codes"
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0239_1.11.1.pdf
 */
enum mctp_phys_binding {
	MCTP_PHYS_BINDING_UNSPEC	= 0x00,
	MCTP_PHYS_BINDING_SMBUS		= 0x01,
	MCTP_PHYS_BINDING_PCIE_VDM	= 0x02,
	MCTP_PHYS_BINDING_USB		= 0x03,
	MCTP_PHYS_BINDING_KCS		= 0x04,
	MCTP_PHYS_BINDING_SERIAL	= 0x05,
	MCTP_PHYS_BINDING_I3C		= 0x06,
	MCTP_PHYS_BINDING_MMBI		= 0x07,
	MCTP_PHYS_BINDING_PCC		= 0x08,
	MCTP_PHYS_BINDING_UCIE		= 0x09,
	MCTP_PHYS_BINDING_VENDOR	= 0xFF,
};

/* Statistics helper macro
 *
 * MCTP_SOCK_STAT_INC bumps a single counter on both the per-namespace
 * aggregate (atomic64) and, when a socket is supplied, the per-socket stats
 * (under stats_lock). Use it for events that touch exactly one counter.
 *
 * The mctp_sock_stat_*() helpers below cover the multi-counter logical events
 * (a successful tx/rx, or a drop that bumps both the generic drop counter and
 * a specific reason) so that route.c / af_mctp.c carry a single hook call per
 * event instead of an open-coded block of counter manipulation.
 */
#define MCTP_SOCK_STAT_INC(_sk, _net, _field) do { \
	struct netns_mctp *_ns = &(_net)->mctp; \
	atomic64_inc(&_ns->_field); \
	if (_sk) { \
		struct mctp_sock *_msk = container_of(_sk, struct mctp_sock, sk); \
		spin_lock_bh(&_msk->stats_lock); \
		_msk->stats._field++; \
		spin_unlock_bh(&_msk->stats_lock); \
	} \
} while (0)

/* TX drop reason selector for mctp_sock_stat_tx_drop()/_tx_error().
 * MCTP_TX_DROP_NONE bumps only the generic tx_drops counter.
 */
enum mctp_tx_drop_reason {
	MCTP_TX_DROP_NONE = 0,
	MCTP_TX_DROP_NO_ROUTE,
	MCTP_TX_DROP_MTU_EXCEEDED,
	MCTP_TX_DROP_NO_MEMORY,
	MCTP_TX_DROP_QUEUE_FULL,
	MCTP_TX_DROP_DEVICE_DOWN,
	MCTP_TX_DROP_TAG_EXHAUSTION,
	MCTP_TX_DROP_PERMISSION,
	MCTP_TX_DROP_BAD_ADDRLEN,
};

/* Account a successful transmitted message: tx_packets, tx_bytes, tx_messages
 * (and the per-socket last_tx_time), on both per-socket and per-namespace.
 */
static inline void mctp_sock_stat_tx(struct sock *sk, struct net *net,
				     unsigned int pkt_len)
{
	struct netns_mctp *ns = &net->mctp;

	if (sk) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
		u64 now = ktime_get_ns();

		spin_lock_bh(&msk->stats_lock);
		msk->stats.tx_packets++;
		msk->stats.tx_bytes += pkt_len;
		msk->stats.tx_messages++;
		msk->stats.last_tx_time = now;
		spin_unlock_bh(&msk->stats_lock);
	}

	atomic64_inc(&ns->tx_packets);
	atomic64_add(pkt_len, &ns->tx_bytes);
	atomic64_inc(&ns->tx_messages);
}

/* Account a successful received message (symmetric with mctp_sock_stat_tx). */
static inline void mctp_sock_stat_rx(struct sock *sk, struct net *net,
				     unsigned int pkt_len)
{
	struct netns_mctp *ns = &net->mctp;

	if (sk) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
		u64 now = ktime_get_ns();

		spin_lock_bh(&msk->stats_lock);
		msk->stats.rx_packets++;
		msk->stats.rx_bytes += pkt_len;
		msk->stats.rx_messages++;
		msk->stats.last_rx_time = now;
		spin_unlock_bh(&msk->stats_lock);
	}

	atomic64_inc(&ns->rx_packets);
	atomic64_add(pkt_len, &ns->rx_bytes);
	atomic64_inc(&ns->rx_messages);
}

/* Shared core for tx-drop accounting. Bumps tx_drops (and tx_errors when
 * @is_error) plus the per-reason counter, on per-socket (under stats_lock,
 * when @msk) and per-namespace.
 */
static inline void __mctp_sock_tx_drop(struct mctp_sock *msk,
				       struct netns_mctp *ns,
				       enum mctp_tx_drop_reason reason,
				       bool is_error)
{
	if (msk) {
		spin_lock_bh(&msk->stats_lock);
		if (is_error)
			msk->stats.tx_errors++;
		msk->stats.tx_drops++;
		switch (reason) {
		case MCTP_TX_DROP_NO_ROUTE:	  msk->stats.tx_dropped_no_route++; break;
		case MCTP_TX_DROP_MTU_EXCEEDED:	  msk->stats.tx_dropped_mtu_exceeded++; break;
		case MCTP_TX_DROP_NO_MEMORY:	  msk->stats.tx_dropped_no_memory++; break;
		case MCTP_TX_DROP_QUEUE_FULL:	  msk->stats.tx_dropped_queue_full++; break;
		case MCTP_TX_DROP_DEVICE_DOWN:	  msk->stats.tx_dropped_device_down++; break;
		case MCTP_TX_DROP_TAG_EXHAUSTION: msk->stats.tx_dropped_tag_exhaustion++; break;
		case MCTP_TX_DROP_PERMISSION:	  msk->stats.tx_dropped_permission++; break;
		case MCTP_TX_DROP_BAD_ADDRLEN:	  msk->stats.tx_dropped_bad_addrlen++; break;
		case MCTP_TX_DROP_NONE:		  break;
		}
		spin_unlock_bh(&msk->stats_lock);
	}

	if (is_error)
		atomic64_inc(&ns->tx_errors);
	atomic64_inc(&ns->tx_drops);
	switch (reason) {
	case MCTP_TX_DROP_NO_ROUTE:	  atomic64_inc(&ns->tx_dropped_no_route); break;
	case MCTP_TX_DROP_MTU_EXCEEDED:	  atomic64_inc(&ns->tx_dropped_mtu_exceeded); break;
	case MCTP_TX_DROP_NO_MEMORY:	  atomic64_inc(&ns->tx_dropped_no_memory); break;
	case MCTP_TX_DROP_QUEUE_FULL:	  atomic64_inc(&ns->tx_dropped_queue_full); break;
	case MCTP_TX_DROP_DEVICE_DOWN:	  atomic64_inc(&ns->tx_dropped_device_down); break;
	case MCTP_TX_DROP_TAG_EXHAUSTION: atomic64_inc(&ns->tx_dropped_tag_exhaustion); break;
	case MCTP_TX_DROP_PERMISSION:	  atomic64_inc(&ns->tx_dropped_permission); break;
	case MCTP_TX_DROP_BAD_ADDRLEN:	  atomic64_inc(&ns->tx_dropped_bad_addrlen); break;
	case MCTP_TX_DROP_NONE:		  break;
	}
}

/* Account a tx drop: bumps tx_drops plus @reason (MCTP_TX_DROP_NONE = generic
 * only). Replaces the "tx_drops + one reason" open-coded pairs.
 */
static inline void mctp_sock_stat_tx_drop(struct sock *sk, struct net *net,
					  enum mctp_tx_drop_reason reason)
{
	__mctp_sock_tx_drop(sk ? container_of(sk, struct mctp_sock, sk) : NULL,
			    &net->mctp, reason, false);
}

/* As mctp_sock_stat_tx_drop(), but also bumps tx_errors (transmit-path
 * failures that are both an error and a drop).
 */
static inline void mctp_sock_stat_tx_error(struct sock *sk, struct net *net,
					   enum mctp_tx_drop_reason reason)
{
	__mctp_sock_tx_drop(sk ? container_of(sk, struct mctp_sock, sk) : NULL,
			    &net->mctp, reason, true);
}

/* Snapshot the per-socket counters into the UAPI struct, under stats_lock.
 * Only the statistics fields are filled; the caller sets the connection-info
 * fields (num_active_keys, bind_*, reserved) outside the lock.
 */
static inline void mctp_sock_stats_snapshot(struct mctp_sock *msk,
					    struct mctp_sock_stats_info *out)
{
	spin_lock_bh(&msk->stats_lock);
	out->tx_bytes = msk->stats.tx_bytes;
	out->tx_packets = msk->stats.tx_packets;
	out->tx_messages = msk->stats.tx_messages;
	out->tx_errors = msk->stats.tx_errors;
	out->tx_drops = msk->stats.tx_drops;

	out->rx_bytes = msk->stats.rx_bytes;
	out->rx_packets = msk->stats.rx_packets;
	out->rx_messages = msk->stats.rx_messages;
	out->rx_errors = msk->stats.rx_errors;
	out->rx_drops = msk->stats.rx_drops;

	out->rx_dropped_no_route = msk->stats.rx_dropped_no_route;
	out->rx_dropped_no_memory = msk->stats.rx_dropped_no_memory;
	out->rx_dropped_seq_mismatch = msk->stats.rx_dropped_seq_mismatch;
	out->rx_dropped_tag_mismatch = msk->stats.rx_dropped_tag_mismatch;
	out->rx_dropped_queue_full = msk->stats.rx_dropped_queue_full;
	out->rx_dropped_invalid_header = msk->stats.rx_dropped_invalid_header;
	out->rx_dropped_permission = msk->stats.rx_dropped_permission;
	out->rx_dropped_timeout = msk->stats.rx_dropped_timeout;

	out->tx_dropped_no_route = msk->stats.tx_dropped_no_route;
	out->tx_dropped_mtu_exceeded = msk->stats.tx_dropped_mtu_exceeded;
	out->tx_dropped_no_memory = msk->stats.tx_dropped_no_memory;
	out->tx_dropped_queue_full = msk->stats.tx_dropped_queue_full;
	out->tx_dropped_device_down = msk->stats.tx_dropped_device_down;
	out->tx_dropped_tag_exhaustion = msk->stats.tx_dropped_tag_exhaustion;
	out->tx_dropped_permission = msk->stats.tx_dropped_permission;
	out->tx_dropped_bad_addrlen = msk->stats.tx_dropped_bad_addrlen;

	out->last_tx_time = msk->stats.last_tx_time;
	out->last_rx_time = msk->stats.last_rx_time;
	spin_unlock_bh(&msk->stats_lock);
}

#endif /* __NET_MCTP_H */
