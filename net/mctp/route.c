// SPDX-License-Identifier: GPL-2.0
/*
 * Management Component Transport Protocol (MCTP) - routing
 * implementation.
 *
 * This is currently based on a simple routing table, with no dst cache. The
 * number of routes should stay fairly small, so the lookup cost is small.
 *
 * Copyright (c) 2021 Code Construct
 * Copyright (c) 2021 Google
 */

#include <linux/idr.h>
#include <linux/kconfig.h>
#include <linux/mctp.h>
#include <linux/netdevice.h>
#include <linux/netfilter_netdev.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/jhash.h>

#include <kunit/static_stub.h>

#include <uapi/linux/if_arp.h>

#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <net/netlink.h>
#include <net/sock.h>

#include <trace/events/mctp.h>

static const unsigned int mctp_message_maxlen = 64 * 1024;
static const unsigned long mctp_nf_track_timeout = 2 * CONFIG_HZ;

/* Helper to determine binding type from network device
 * Returns the physical binding type from mctp_dev, which is set during
 * device registration and remains constant regardless of device renaming.
 */
u8 mctp_get_binding_type(struct net_device *dev)
{
	struct mctp_dev *mdev;
	u8 binding = 0;
	
	if (!dev)
		return 0;  /* Unknown */
	
	rcu_read_lock();
	mdev = __mctp_dev_get(dev);
	if (mdev) {
		binding = mdev->binding;
		mctp_dev_put(mdev);
	}
	rcu_read_unlock();
	
	return binding;
}


static void mctp_flow_prepare_output(struct sk_buff *skb, struct mctp_dev *dev);

enum mctp_nf_track_verdict {
	MCTP_NF_VERDICT_ACCEPT = 0,
	MCTP_NF_VERDICT_DROP = 1,
};

static u32 mctp_nf_track_hash(mctp_eid_t src, mctp_eid_t dst, u8 tag)
{
	return jhash_3words(src, dst, tag, 0) & (MCTP_NF_TRACK_BUCKETS - 1);
}

static bool mctp_nf_track_lookup(struct mctp_dev *mdev, mctp_eid_t src,
				 mctp_eid_t dst, u8 tag, u8 *verdict)
{
	struct mctp_nf_track_entry *entry;
	struct hlist_node *tmp;
	unsigned long now = jiffies;
	u32 idx = mctp_nf_track_hash(src, dst, tag);
	bool found = false;

	spin_lock_bh(&mdev->nf_track.lock);
	hlist_for_each_entry_safe(entry, tmp, &mdev->nf_track.buckets[idx],
				  node) {
		if (entry->src != src || entry->dst != dst || entry->tag != tag)
			continue;

		if (time_after(now, entry->expires)) {
			netdev_dbg(mdev->dev,
				   "MCTP nf track: expiry src=%u dst=%u tag=0x%x\n",
				   src, dst, tag);
			hlist_del(&entry->node);
			kfree(entry);
			mdev->nf_track.count--;
			break;
		}

		*verdict = entry->verdict;
		found = true;
		break;
	}
	spin_unlock_bh(&mdev->nf_track.lock);

	if (found)
		netdev_dbg(mdev->dev,
			   "MCTP nf track lookup: src=%u dst=%u tag=0x%x verdict=%s\n",
			   src, dst, tag, *verdict == MCTP_NF_VERDICT_ACCEPT ? "accept" : "drop");

	return found;
}

static void mctp_nf_track_delete(struct mctp_dev *mdev, mctp_eid_t src,
				 mctp_eid_t dst, u8 tag)
{
	struct mctp_nf_track_entry *entry;
	struct hlist_node *tmp;
	u32 idx = mctp_nf_track_hash(src, dst, tag);

	spin_lock_bh(&mdev->nf_track.lock);
	hlist_for_each_entry_safe(entry, tmp, &mdev->nf_track.buckets[idx],
				  node) {
		if (entry->src != src || entry->dst != dst || entry->tag != tag)
			continue;
		netdev_dbg(mdev->dev,
			   "MCTP nf track delete: src=%u dst=%u tag=0x%x\n",
			   src, dst, tag);
		hlist_del(&entry->node);
		kfree(entry);
		mdev->nf_track.count--;
		break;
	}
	spin_unlock_bh(&mdev->nf_track.lock);
}

static void mctp_nf_track_store(struct mctp_dev *mdev, mctp_eid_t src,
				mctp_eid_t dst, u8 tag, u8 verdict)
{
	struct mctp_nf_track_entry *entry, *new_entry;
	u32 idx = mctp_nf_track_hash(src, dst, tag);

	new_entry = kzalloc(sizeof(*new_entry), GFP_ATOMIC);
	if (!new_entry)
		return;

	new_entry->src = src;
	new_entry->dst = dst;
	new_entry->tag = tag;
	new_entry->verdict = verdict;
	new_entry->expires = jiffies + mctp_nf_track_timeout;

	spin_lock_bh(&mdev->nf_track.lock);
	hlist_for_each_entry(entry, &mdev->nf_track.buckets[idx], node) {
		if (entry->src != src || entry->dst != dst || entry->tag != tag)
			continue;
		entry->verdict = verdict;
		entry->expires = jiffies + mctp_nf_track_timeout;
		spin_unlock_bh(&mdev->nf_track.lock);
		netdev_dbg(mdev->dev,
			   "MCTP nf track store (update): src=%u dst=%u tag=0x%x verdict=%s\n",
			   src, dst, tag,
			   verdict == MCTP_NF_VERDICT_ACCEPT ? "accept" : "drop");
		kfree(new_entry);
		return;
	}

	if (mdev->nf_track.count < MCTP_NF_TRACK_MAX) {
		hlist_add_head(&new_entry->node, &mdev->nf_track.buckets[idx]);
		mdev->nf_track.count++;
		netdev_dbg(mdev->dev,
			   "MCTP nf track store: src=%u dst=%u tag=0x%x verdict=%s count=%u\n",
			   src, dst, tag,
			   verdict == MCTP_NF_VERDICT_ACCEPT ? "accept" : "drop",
			   mdev->nf_track.count);
		new_entry = NULL;
	} else {
		netdev_dbg(mdev->dev,
			   "MCTP nf track store: table full, drop entry src=%u dst=%u tag=0x%x\n",
			   src, dst, tag);
	}
	spin_unlock_bh(&mdev->nf_track.lock);

	kfree(new_entry);
}

static bool mctp_nf_ingress_check(struct sk_buff *skb, struct mctp_dev *mdev,
				  const struct mctp_hdr *mh)
{
	u8 flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	u8 tag = mh->flags_seq_tag & (MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);
	u8 verdict;
	u8 msg_type = 0;
	int ret;
	bool som = !!(flags & MCTP_HDR_FLAG_SOM);
	bool eom = !!(flags & MCTP_HDR_FLAG_EOM);
	unsigned int type_off = skb_network_offset(skb) + sizeof(struct mctp_hdr);

	if (skb->len >= type_off + 1 && skb_copy_bits(skb, type_off, &msg_type, 1) == 0)
		msg_type &= 0x7f;

	netdev_dbg(mdev->dev,
		   "MCTP nf ingress: src=%u dst=%u tag=0x%x SOM=%d EOM=%d type=%u\n",
		   mh->src, mh->dest, tag, som, eom, msg_type);

	if (!(flags & MCTP_HDR_FLAG_SOM)) {
		if (mctp_nf_track_lookup(mdev, mh->src, mh->dest, tag,
					 &verdict)) {
			if (flags & MCTP_HDR_FLAG_EOM)
				mctp_nf_track_delete(mdev, mh->src, mh->dest, tag);
			netdev_dbg(mdev->dev,
				   "MCTP nf ingress: fragment verdict=%s\n",
				   verdict == MCTP_NF_VERDICT_ACCEPT ? "accept" : "drop");
			return verdict == MCTP_NF_VERDICT_ACCEPT;
		}
		netdev_dbg(mdev->dev,
			   "MCTP nf ingress: fragment no track entry, pass to hook\n");
	}

	if (!nf_hook_ingress_active(skb)) {
		netdev_dbg(mdev->dev, "MCTP nf ingress: no hooks, accept\n");
		return true;
	}

	rcu_read_lock();
	ret = nf_hook_ingress(skb);
	rcu_read_unlock();

	if (ret < 0) {
		netdev_dbg(mdev->dev,
			   "MCTP nf ingress: hook dropped (ret=%d)\n", ret);
		if (!(flags & MCTP_HDR_FLAG_EOM))
			mctp_nf_track_store(mdev, mh->src, mh->dest, tag,
					    MCTP_NF_VERDICT_DROP);
		return false;
	}

	if (!(flags & MCTP_HDR_FLAG_EOM))
		mctp_nf_track_store(mdev, mh->src, mh->dest, tag,
				    MCTP_NF_VERDICT_ACCEPT);

	netdev_dbg(mdev->dev, "MCTP nf ingress: hook accept\n");
	return true;
}

static int mctp_nf_egress_check(struct sk_buff **pskb, struct mctp_dev *mdev,
				const struct mctp_hdr *mh)
{
	struct sk_buff *skb = *pskb;
	u8 flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	u8 tag = mh->flags_seq_tag & (MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);
	u8 verdict;
	u8 msg_type = 0;
	int rc = NET_XMIT_SUCCESS;
	bool som = !!(flags & MCTP_HDR_FLAG_SOM);
	bool eom = !!(flags & MCTP_HDR_FLAG_EOM);
	unsigned int type_off = skb_network_offset(skb) + sizeof(struct mctp_hdr);

	if (skb->len >= type_off + 1 && skb_copy_bits(skb, type_off, &msg_type, 1) == 0)
		msg_type &= 0x7f;

	netdev_dbg(mdev->dev,
		   "MCTP nf egress: src=%u dst=%u tag=0x%x SOM=%d EOM=%d type=%u\n",
		   mh->src, mh->dest, tag, som, eom, msg_type);

	if (!(flags & MCTP_HDR_FLAG_SOM)) {
		if (mctp_nf_track_lookup(mdev, mh->src, mh->dest, tag,
					 &verdict)) {
			if (flags & MCTP_HDR_FLAG_EOM)
				mctp_nf_track_delete(mdev, mh->src, mh->dest, tag);
			if (verdict == MCTP_NF_VERDICT_DROP) {
				netdev_dbg(mdev->dev,
					   "MCTP nf egress: fragment verdict=drop\n");
				kfree_skb(skb);
				*pskb = NULL;
				return -EPERM;
			}
			netdev_dbg(mdev->dev,
				   "MCTP nf egress: fragment verdict=accept\n");
			return 0;
		}
		netdev_dbg(mdev->dev,
			   "MCTP nf egress: fragment no track entry, pass to hook\n");
	}

	if (!nf_hook_egress_active()) {
		netdev_dbg(mdev->dev, "MCTP nf egress: no hooks, accept\n");
		return 0;
	}

	rcu_read_lock_bh();
	skb = nf_hook_egress(skb, &rc, mdev->dev);
	rcu_read_unlock_bh();

	if (!skb) {
		netdev_dbg(mdev->dev,
			   "MCTP nf egress: hook dropped (rc=%d)\n", rc);
		*pskb = NULL;
		return net_xmit_errno(rc);
	}

	*pskb = skb;

	if (!(flags & MCTP_HDR_FLAG_EOM))
		mctp_nf_track_store(mdev, mh->src, mh->dest, tag,
				    MCTP_NF_VERDICT_ACCEPT);

	netdev_dbg(mdev->dev, "MCTP nf egress: hook accept\n");
	return 0;
}

/* route output callbacks */
static int mctp_dst_discard(struct mctp_dst *dst, struct sk_buff *skb)
{
	kfree_skb(skb);
	return 0;
}

static struct mctp_sock *mctp_lookup_bind(struct net *net, struct sk_buff *skb)
{
	struct mctp_skb_cb *cb = mctp_cb(skb);
	struct mctp_hdr *mh;
	struct sock *sk;
	u8 type;

	WARN_ON(!rcu_read_lock_held());

	/* TODO: look up in skb->cb? */
	mh = mctp_hdr(skb);

	if (!skb_headlen(skb))
		return NULL;

	type = (*(u8 *)skb->data) & 0x7f;

	sk_for_each_rcu(sk, &net->mctp.binds) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
		int bound_dev_if;

		if (msk->bind_net != MCTP_NET_ANY && msk->bind_net != cb->net)
			continue;

		if (msk->bind_type != type)
			continue;

		if (!mctp_address_matches(msk->bind_addr, mh->dest))
			continue;

		/* Check SO_BINDTODEVICE constraint */
		bound_dev_if = READ_ONCE(sk->sk_bound_dev_if);
		if (bound_dev_if && skb->dev &&
		    bound_dev_if != skb->dev->ifindex)
			continue;

		return msk;
	}

	return NULL;
}

/* A note on the key allocations.
 *
 * struct net->mctp.keys contains our set of currently-allocated keys for
 * MCTP tag management. The lookup tuple for these is the peer EID,
 * local EID and MCTP tag.
 *
 * In some cases, the peer EID may be MCTP_EID_ANY: for example, when a
 * broadcast message is sent, we may receive responses from any peer EID.
 * Because the broadcast dest address is equivalent to ANY, we create
 * a key with (local = local-eid, peer = ANY). This allows a match on the
 * incoming broadcast responses from any peer.
 *
 * We perform lookups when packets are received, and when tags are allocated
 * in two scenarios:
 *
 *  - when a packet is sent, with a locally-owned tag: we need to find an
 *    unused tag value for the (local, peer) EID pair.
 *
 *  - when a tag is manually allocated: we need to find an unused tag value
 *    for the peer EID, but don't have a specific local EID at that stage.
 *
 * in the latter case, on successful allocation, we end up with a tag with
 * (local = ANY, peer = peer-eid).
 *
 * So, the key set allows both a local EID of ANY, as well as a peer EID of
 * ANY in the lookup tuple. Both may be ANY if we prealloc for a broadcast.
 * The matching (in mctp_key_match()) during lookup allows the match value to
 * be ANY in either the dest or source addresses.
 *
 * When allocating (+ inserting) a tag, we need to check for conflicts amongst
 * the existing tag set. This requires macthing either exactly on the local
 * and peer addresses, or either being ANY.
 */

static bool mctp_key_match(struct mctp_sk_key *key, unsigned int net,
			   mctp_eid_t local, mctp_eid_t peer, u8 tag)
{
	if (key->net != net)
		return false;

	if (!mctp_address_matches(key->local_addr, local))
		return false;

	if (!mctp_address_matches(key->peer_addr, peer))
		return false;

	if (key->tag != tag)
		return false;

	return true;
}

/* returns a key (with key->lock held, and refcounted), or NULL if no such
 * key exists.
 */
static struct mctp_sk_key *mctp_lookup_key(struct net *net, struct sk_buff *skb,
					   unsigned int netid, mctp_eid_t peer,
					   unsigned long *irqflags)
	__acquires(&key->lock)
{
	struct mctp_sk_key *key, *ret;
	unsigned long flags;
	struct mctp_hdr *mh;
	u8 tag;

	mh = mctp_hdr(skb);
	tag = mh->flags_seq_tag & (MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);

	ret = NULL;
	spin_lock_irqsave(&net->mctp.keys_lock, flags);

	hlist_for_each_entry(key, &net->mctp.keys, hlist) {
		if (!mctp_key_match(key, netid, mh->dest, peer, tag))
			continue;

		spin_lock(&key->lock);
		if (key->valid) {
			refcount_inc(&key->refs);
			ret = key;
			break;
		}
		spin_unlock(&key->lock);
	}

	if (ret) {
		spin_unlock(&net->mctp.keys_lock);
		*irqflags = flags;
	} else {
		spin_unlock_irqrestore(&net->mctp.keys_lock, flags);
	}

	return ret;
}

static struct mctp_sk_key *mctp_key_alloc(struct mctp_sock *msk,
					  unsigned int net,
					  mctp_eid_t local, mctp_eid_t peer,
					  u8 tag, gfp_t gfp)
{
	struct mctp_sk_key *key;

	key = kzalloc(sizeof(*key), gfp);
	if (!key)
		return NULL;

	key->net = net;
	key->peer_addr = peer;
	key->local_addr = local;
	key->tag = tag;
	key->sk = &msk->sk;
	key->valid = true;
	spin_lock_init(&key->lock);
	refcount_set(&key->refs, 1);
	sock_hold(key->sk);

	return key;
}

void mctp_key_unref(struct mctp_sk_key *key)
{
	unsigned long flags;

	if (!refcount_dec_and_test(&key->refs))
		return;

	/* even though no refs exist here, the lock allows us to stay
	 * consistent with the locking requirement of mctp_dev_release_key
	 */
	spin_lock_irqsave(&key->lock, flags);
	mctp_dev_release_key(key->dev, key);
	spin_unlock_irqrestore(&key->lock, flags);

	sock_put(key->sk);
	kfree(key);
}

static int mctp_key_add(struct mctp_sk_key *key, struct mctp_sock *msk,
			unsigned long lifetime)
{
	struct net *net = sock_net(&msk->sk);
	struct mctp_sk_key *tmp;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);

	if (sock_flag(&msk->sk, SOCK_DEAD)) {
		rc = -EINVAL;
		goto out_unlock;
	}

	hlist_for_each_entry(tmp, &net->mctp.keys, hlist) {
		if (mctp_key_match(tmp, key->net, key->local_addr,
				   key->peer_addr, key->tag)) {
			spin_lock(&tmp->lock);
			if (tmp->valid)
				rc = -EEXIST;
			spin_unlock(&tmp->lock);
			if (rc)
				break;
		}
	}

	if (!rc) {
		refcount_inc(&key->refs);
		key->expiry = jiffies + lifetime;
		timer_reduce(&msk->key_expiry, key->expiry);

		hlist_add_head(&key->hlist, &net->mctp.keys);
		hlist_add_head(&key->sklist, &msk->keys);
	}

out_unlock:
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

	return rc;
}

/* Helper for mctp_route_input().
 * We're done with the key; unlock and unref the key.
 * For the usual case of automatic expiry we remove the key from lists.
 * In the case that manual allocation is set on a key we release the
 * local ref and reset reassembly, but don't remove the key from lists.
 * We also notify the transport that the current request/response flow
 * has finished (via dev->ops->release_flow), so transport-specific
 * per-flow state -- e.g. the mctp-i2c bus segment lock -- can be
 * dropped even though the key itself persists for tag reuse.
 */
static void __mctp_key_done_in(struct mctp_sk_key *key, struct net *net,
			       unsigned long flags, unsigned long reason)
__releases(&key->lock)
{
	struct sk_buff *skb;

	trace_mctp_key_release(key, reason);
	skb = key->reasm_head;
	key->reasm_head = NULL;

	if (!key->manual_alloc) {
		key->reasm_dead = true;
		key->valid = false;
		mctp_dev_release_key(key->dev, key);
	} else if (key->dev && key->dev->ops &&
		   key->dev->ops->release_flow) {
		/* Manual-alloc keys live on for tag reuse, but the current
		 * transport flow is finished; release per-flow state (e.g.
		 * the mctp-i2c bus segment lock) so other transport users
		 * can proceed.
		 */
		key->dev->ops->release_flow(key->dev, key);
	}
	spin_unlock_irqrestore(&key->lock, flags);

	if (!key->manual_alloc) {
		spin_lock_irqsave(&net->mctp.keys_lock, flags);
		if (!hlist_unhashed(&key->hlist)) {
			hlist_del_init(&key->hlist);
			hlist_del_init(&key->sklist);
			mctp_key_unref(key);
		}
		spin_unlock_irqrestore(&net->mctp.keys_lock, flags);
	}

	/* and one for the local reference */
	mctp_key_unref(key);

	kfree_skb(skb);
}

#ifdef CONFIG_MCTP_FLOWS
static void mctp_skb_set_flow(struct sk_buff *skb, struct mctp_sk_key *key)
{
	struct mctp_flow *flow;

	flow = skb_ext_add(skb, SKB_EXT_MCTP);
	if (!flow)
		return;

	refcount_inc(&key->refs);
	flow->key = key;
}

static void mctp_flow_prepare_output(struct sk_buff *skb, struct mctp_dev *dev)
{
	struct mctp_sk_key *key;
	struct mctp_flow *flow;
	unsigned long flags;

	flow = skb_ext_find(skb, SKB_EXT_MCTP);
	if (!flow)
		return;

	key = flow->key;

	spin_lock_irqsave(&key->lock, flags);

	if (!key->dev)
		mctp_dev_set_key(dev, key);
	else
		WARN_ON(key->dev != dev);

	spin_unlock_irqrestore(&key->lock, flags);
}
#else
static void mctp_skb_set_flow(struct sk_buff *skb, struct mctp_sk_key *key) {}
static void mctp_flow_prepare_output(struct sk_buff *skb, struct mctp_dev *dev) {}
#endif

/* takes ownership of skb, both in success and failure cases */
static int mctp_frag_queue(struct mctp_sk_key *key, struct sk_buff *skb)
{
	struct mctp_hdr *hdr = mctp_hdr(skb);
	u8 exp_seq, this_seq;
	int rc = 0;

	this_seq = (hdr->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT)
		& MCTP_HDR_SEQ_MASK;

	if (!key->reasm_head) {
		/* Since we're manipulating the shared frag_list, ensure it
		 * isn't shared with any other SKBs. In the cloned case,
		 * this will free the skb; callers can no longer access it
		 * safely.
		 */
		key->reasm_head = skb_unshare(skb, GFP_ATOMIC);
		if (!key->reasm_head) {
			MCTP_SOCK_STAT_INC(key->sk, sock_net(key->sk), rx_dropped_no_memory);
			return -ENOMEM;
		}

		key->reasm_tailp = &(skb_shinfo(key->reasm_head)->frag_list);
		key->last_seq = this_seq;
		trace_mctp_reassemble_start(hdr->src, hdr->dest, this_seq);
		return 0;
	}

	exp_seq = (key->last_seq + 1) & MCTP_HDR_SEQ_MASK;

	if (this_seq != exp_seq) {
		MCTP_SOCK_STAT_INC(key->sk, sock_net(key->sk), rx_dropped_seq_mismatch);
		rc = -EINVAL;
		goto err_free;
	}

	if (key->reasm_head->len + skb->len > mctp_message_maxlen) {
		MCTP_SOCK_STAT_INC(key->sk, sock_net(key->sk), rx_dropped_queue_full);
		rc = -EMSGSIZE;
		goto err_free;
	}

	skb->next = NULL;
	skb->sk = NULL;
	*key->reasm_tailp = skb;
	key->reasm_tailp = &skb->next;

	key->last_seq = this_seq;

	key->reasm_head->data_len += skb->len;
	key->reasm_head->len += skb->len;
	key->reasm_head->truesize += skb->truesize;

	return rc;

err_free:
	kfree_skb(skb);
	return rc;
}

/*
 * mctp_report_rx_missing_som - Report error for middle/end fragment without SOM
 * @key: RX MCTP key for this transaction (provides addressing for TX key lookup)
 * @skb: The failed fragment
 * @mh: MCTP header
 * @tag: Tag value
 * @flags: Lock flags (will be used to release/reacquire key->lock)
 *
 * This function must be called with key->lock held. It will temporarily release
 * the lock to report the error (to avoid deadlock), then reacquire it.
 *
 * NEW BEHAVIOR: Looks up TX key and only reports if found. This ensures we only
 * report errors for responses to OUR requests.
 */
static void mctp_report_rx_missing_som(struct mctp_sk_key *key,
				       struct sk_buff *skb,
				       struct mctp_hdr *mh,
				       unsigned long tag,
				       unsigned long *flags)
{
	struct sock *sk;

	netdev_err(skb->dev,
		   "MCTP RX: Middle/End fragment without SOM (src=%u, dest=%u, tag=%lu)\n",
		   mh->src, mh->dest, tag & MCTP_HDR_TAG_MASK);

	/* CRITICAL: Release key->lock BEFORE error reporting to avoid deadlock.
	 * mctp_queue_error() needs to acquire keys_lock for TX key lookup, but
	 * correct lock order is keys_lock → key->lock. We currently hold key->lock,
	 * so we must release it first.
	 */
	spin_unlock_irqrestore(&key->lock, *flags);

	/* Look up socket and report error.
	 * mctp_queue_error() will look up TX key internally and only report
	 * if TX key is found (our transaction).
	 */
	sk = mctp_lookup_sock_for_error(skb, skb->dev, key, NULL);
	if (sk) {
		/* Pass RX key - mctp_queue_error will use it to find TX key */
		mctp_queue_error(sk, skb, EPROTO, skb->dev, MCTP_DIR_RX,
				 mctp_get_binding_type(skb->dev), key);
		sock_put(sk);
	}

	/* Re-acquire key->lock for caller */
	spin_lock_irqsave(&key->lock, *flags);
}

/*
 * mctp_report_rx_sequence_error - Report RX sequence/size error
 * @key: RX MCTP key for this transaction (provides addressing for TX key lookup)
 * @skb: The failed fragment
 * @mh: MCTP header
 * @tag: Tag value
 * @flags: Lock flags (will be used to release/reacquire key->lock)
 * @err_code: Error code from mctp_frag_queue (-EINVAL or -EMSGSIZE)
 *
 * This function must be called with key->lock held. It will temporarily release
 * the lock to report the error (to avoid deadlock), then reacquire it.
 *
 * NEW BEHAVIOR: Looks up TX key and only reports if found. This ensures we only
 * report errors for responses to OUR requests.
 */
static void mctp_report_rx_sequence_error(struct mctp_sk_key *key,
					  struct sk_buff *skb,
					  struct mctp_hdr *mh,
					  unsigned long tag,
					  unsigned long *flags,
					  int err_code)
{
	struct sock *sk;
	struct sk_buff *report_skb;
	u8 exp_seq = (key->last_seq + 1) & MCTP_HDR_SEQ_MASK;
	u8 this_seq = (mh->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) & MCTP_HDR_SEQ_MASK;
	int report_errno;

	/* Determine error type and error code to report */
	if (err_code == -EMSGSIZE) {
		netdev_err(skb->dev,
			   "MCTP RX: Message too large - exceeds 64KB limit (src=%u, dest=%u, tag=%lu)\n",
			   mh->src, mh->dest, tag & MCTP_HDR_TAG_MASK);
		report_errno = EMSGSIZE;
	} else {
		/* err_code == -EINVAL - sequence error */
		netdev_err(skb->dev,
			   "MCTP RX: Sequence error - expected %u, got %u (src=%u, dest=%u, tag=%lu)\n",
			   exp_seq, this_seq, mh->src, mh->dest,
			   tag & MCTP_HDR_TAG_MASK);
		report_errno = EPROTO;
	}

	/* Use first response fragment (reasm_head) for addressing if available.
	 * Fall back to failed fragment only if reasm_head is NULL.
	 */
	report_skb = key->reasm_head ? key->reasm_head : skb;

	/* CRITICAL: Release key->lock BEFORE error reporting to avoid deadlock.
	 * mctp_queue_error() needs to acquire keys_lock for TX key lookup, but
	 * correct lock order is keys_lock → key->lock. We currently hold key->lock,
	 * so we must release it first.
	 */
	spin_unlock_irqrestore(&key->lock, *flags);

	/* Look up socket and report error.
	 * mctp_queue_error() will look up TX key internally and only report
	 * if TX key is found (our transaction).
	 */
	sk = mctp_lookup_sock_for_error(report_skb, skb->dev, key, NULL);
	if (sk) {
		/* Pass RX key - mctp_queue_error will use it to find TX key */
		mctp_queue_error(sk, report_skb, report_errno, skb->dev, MCTP_DIR_RX,
				 mctp_get_binding_type(skb->dev), key);
		sock_put(sk);
	}

	/* Re-acquire key->lock for caller */
	spin_lock_irqsave(&key->lock, *flags);
}

static int mctp_dst_input(struct mctp_dst *dst, struct sk_buff *skb)
{
	struct mctp_sk_key *key, *any_key = NULL;
	struct net *net = dev_net(skb->dev);
	struct mctp_sock *msk;
	struct mctp_hdr *mh;
	unsigned int netid;
	unsigned long f;
	u8 tag, flags;
	int rc;
	u8 ver;

	msk = NULL;
	rc = -EINVAL;

	/* We may be receiving a locally-routed packet; drop source sk
	 * accounting.
	 *
	 * From here, we will either queue the skb - either to a frag_queue, or
	 * to a receiving socket. When that succeeds, we clear the skb pointer;
	 * a non-NULL skb on exit will be otherwise unowned, and hence
	 * kfree_skb()-ed.
	 */
	skb_orphan(skb);

	if (skb->pkt_type == PACKET_OUTGOING)
		skb->pkt_type = PACKET_LOOPBACK;

	/* ensure we have enough data for a header and a type */
	if (skb->len < sizeof(struct mctp_hdr) + 1) {
		trace_mctp_drop_packet(skb, "packet_too_short");
		MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_invalid_header);
		goto out;
	}

	/* grab header, advance data ptr */
	mh = mctp_hdr(skb);
	netid = mctp_cb(skb)->net;
	skb_pull(skb, sizeof(struct mctp_hdr));

	ver = mh->ver & MCTP_HDR_VER_MASK;
	if (ver < MCTP_VER_MIN || ver > MCTP_VER_MAX) {
		MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_invalid_header);
		goto out;
	}

	flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	tag = mh->flags_seq_tag & (MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);

	rcu_read_lock();

	/* lookup socket / reasm context, exactly matching (src,dest,tag).
	 * we hold a ref on the key, and key->lock held.
	 */
	key = mctp_lookup_key(net, skb, netid, mh->src, &f);

	if (flags & MCTP_HDR_FLAG_SOM) {
		if (key) {
			msk = container_of(key->sk, struct mctp_sock, sk);
		} else {
			/* first response to a broadcast? do a more general
			 * key lookup to find the socket, but don't use this
			 * key for reassembly - we'll create a more specific
			 * one for future packets if required (ie, !EOM).
			 *
			 * this lookup requires key->peer to be MCTP_ADDR_ANY,
			 * it doesn't match just any key->peer.
			 */
			any_key = mctp_lookup_key(net, skb, netid,
						  MCTP_ADDR_ANY, &f);
			if (any_key) {
				msk = container_of(any_key->sk,
						   struct mctp_sock, sk);
				spin_unlock_irqrestore(&any_key->lock, f);
			}
		}

		if (!key && !msk && (tag & MCTP_HDR_FLAG_TO))
			msk = mctp_lookup_bind(net, skb);

		if (!msk) {
			trace_mctp_drop_packet(skb, "no_socket_bound");
			MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_no_route);
			rc = -ENOENT;
			goto out_unlock;
		}

		/* single-packet message? deliver to socket, clean up any
		 * pending key.
		 */
		if (flags & MCTP_HDR_FLAG_EOM) {
			trace_mctp_rx_packet(skb);
			rc = sock_queue_rcv_skb(&msk->sk, skb);
			trace_mctp_rx_socket(skb, rc);
			if (!rc) {
				u64 rx_time = ktime_get_ns();
				unsigned int pkt_len = skb->len + sizeof(struct mctp_hdr);

				spin_lock_bh(&msk->stats_lock);
				msk->stats.rx_packets++;
				msk->stats.rx_bytes += pkt_len;
				msk->stats.rx_messages++;
				msk->stats.last_rx_time = rx_time;
				spin_unlock_bh(&msk->stats_lock);

				atomic64_inc(&net->mctp.rx_packets);
				atomic64_add(pkt_len, &net->mctp.rx_bytes);
				atomic64_inc(&net->mctp.rx_messages);

				skb = NULL;
			} else {
				trace_mctp_drop_packet(skb, "sock_queue_failed");
				if (rc == -ENOBUFS || rc == -ENOMEM)
					MCTP_SOCK_STAT_INC(&msk->sk, net, rx_dropped_queue_full);
				else if (rc == -EPERM || rc == -EACCES)
					MCTP_SOCK_STAT_INC(&msk->sk, net, rx_dropped_permission);
				else
					MCTP_SOCK_STAT_INC(&msk->sk, net, rx_drops);
			}
			if (key) {
				/* we've hit a pending reassembly; not much we
				 * can do but drop it
				 */
				__mctp_key_done_in(key, net, f,
						   MCTP_TRACE_KEY_REPLIED);
				key = NULL;
			}
			goto out_unlock;
		}

		/* broadcast response or a bind() - create a key for further
		 * packets for this message
		 */
		if (!key) {
			struct mctp_dev *mdev;
			unsigned long lifetime;

			mdev = __mctp_dev_get(skb->dev);
			lifetime = mctp_effective_key_lifetime(msk, mdev, 0);
			if (mdev)
				mctp_dev_put(mdev);

			key = mctp_key_alloc(msk, netid, mh->dest, mh->src,
					     tag, GFP_ATOMIC);
			if (!key) {
				rc = -ENOMEM;
				MCTP_SOCK_STAT_INC(&msk->sk, net, rx_dropped_no_memory);
				goto out_unlock;
			}

			/* we can queue without the key lock here, as the
			* key isn't observable yet
			*/
			mctp_frag_queue(key, skb);
			
			/* Cache message type and payload from first fragment (SOM) for error reporting.
			* If middle/end fragments are missing (timeout), we can still report
			* the correct message headers to the application.
			* Use skb_network_header() not skb->data for consistency.
			*/
			if (skb && skb->len > sizeof(struct mctp_hdr)) {
				u8 *msg_start = (u8 *)skb_network_header(skb);
				size_t payload_offset, available, capture_len;
				
				/* Capture message type (first byte after MCTP header) */
				key->orig_msg_type = *(msg_start + sizeof(struct mctp_hdr));
				
				/* Capture first 32 bytes of payload (after message type) for RX timeout errors */
				payload_offset = sizeof(struct mctp_hdr) + 1;
				if (skb->len > payload_offset) {
					available = skb->len - payload_offset;
					capture_len = min_t(size_t, available, sizeof(key->orig_payload));
					memcpy(key->orig_payload, msg_start + payload_offset, capture_len);
					key->orig_payload_len = capture_len;
				}
			}

			skb = NULL;

			/* if the key_add fails, we've raced with another
			 * SOM packet with the same src, dest and tag. There's
			 * no way to distinguish future packets, so all we
			 * can do is drop.
			 */
			rc = mctp_key_add(key, msk, lifetime);
			if (!rc)
				trace_mctp_key_acquire(key);
			else
				MCTP_SOCK_STAT_INC(&msk->sk, net, rx_dropped_seq_mismatch);

			/* we don't need to release key->lock on exit, so
			 * clean up here and suppress the unlock via
			 * setting to NULL
			 */
			mctp_key_unref(key);
			key = NULL;

		} else {
			if (key->reasm_head) {
				kfree_skb(key->reasm_head);
				key->reasm_head = NULL;
				key->reasm_tailp = NULL;
				rc = mctp_frag_queue(key, skb);
				skb = NULL;
				MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_seq_mismatch);
			} else if (key->reasm_dead) {
				/* duplicate start? drop everything */
				__mctp_key_done_in(key, net, f,
						   MCTP_TRACE_KEY_INVALIDATED);
				rc = -EEXIST;
				key = NULL;
				MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_seq_mismatch);
			} else {
				rc = mctp_frag_queue(key, skb);
				skb = NULL;
			}
		}

	} else if (key) {
		/* this packet continues a previous message; reassemble
		 * using the message-specific key
		 */

		/* we need to be continuing an existing reassembly... */
		if (!key->reasm_head) {
			rc = -EINVAL;
			mctp_report_rx_missing_som(key, skb, mh, tag, &f);
			MCTP_SOCK_STAT_INC(key->sk, net, rx_dropped_seq_mismatch);
		} else {
			rc = mctp_frag_queue(key, skb);
			
			if (rc == -EINVAL || rc == -EMSGSIZE) {
				/* Reassembly failure: sequence error (-EINVAL) or message too large (-EMSGSIZE) */
				mctp_report_rx_sequence_error(key, skb, mh, tag, &f, rc);
			}

			skb = NULL;
		}

		if (rc)
			goto out_unlock;

		/* end of message? deliver to socket, and we're done with
		 * the reassembly/response key
		 */
		if (flags & MCTP_HDR_FLAG_EOM) {
			if (key->reasm_head) {
				trace_mctp_reassemble_finish(mh->src, mh->dest, key->reasm_head->len);
				trace_mctp_rx_packet(key->reasm_head);
			}
			rc = sock_queue_rcv_skb(key->sk, key->reasm_head);
			if (key->reasm_head) {
				trace_mctp_rx_socket(key->reasm_head, rc);
			}
			if (!rc) {
				struct mctp_sock *msk = container_of(key->sk, struct mctp_sock, sk);
				u64 rx_time = ktime_get_ns();
				unsigned int pkt_len = key->reasm_head->len + sizeof(struct mctp_hdr);

				spin_lock_bh(&msk->stats_lock);
				msk->stats.rx_packets++;
				msk->stats.rx_bytes += pkt_len;
				msk->stats.rx_messages++;
				msk->stats.last_rx_time = rx_time;
				spin_unlock_bh(&msk->stats_lock);

				atomic64_inc(&net->mctp.rx_packets);
				atomic64_add(pkt_len, &net->mctp.rx_bytes);
				atomic64_inc(&net->mctp.rx_messages);

				key->reasm_head = NULL;
			} else {
				if (rc == -ENOBUFS || rc == -ENOMEM)
					MCTP_SOCK_STAT_INC(key->sk, net, rx_dropped_queue_full);
				else if (rc == -EPERM || rc == -EACCES)
					MCTP_SOCK_STAT_INC(key->sk, net, rx_dropped_permission);
				else
					MCTP_SOCK_STAT_INC(key->sk, net, rx_drops);
			}
			__mctp_key_done_in(key, net, f,
					   MCTP_TRACE_KEY_REPLIED);
			key = NULL;
		}

	} else {
		/* not a start, no matching key */
		rc = -ENOENT;
		MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_seq_mismatch); /* Implicitly unsolicited/missing SOM */
	}

out_unlock:
	rcu_read_unlock();
	if (key) {
		spin_unlock_irqrestore(&key->lock, f);
		mctp_key_unref(key);
	}
	if (any_key)
		mctp_key_unref(any_key);
out:
	kfree_skb(skb);
	return rc;
}

static int mctp_dst_output(struct mctp_dst *dst, struct sk_buff *skb)
{
	struct mctp_hdr *hdr = mctp_hdr(skb);
	char daddr_buf[MAX_ADDR_LEN];
	char *daddr = NULL;
	int rc;

	/* Tunnel: packet egress from one net dev to another (e.g. USB -> I2C).
	 * Must do neighbour lookup for egress device; stashed hwaddr is for ingress.
	 */
	bool is_tunnel = (skb->dev != dst->dev->dev);

	/* Update skb->dev to the outgoing device (needed for forwarding). */
	skb->dev = dst->dev->dev;
	skb->pkt_type = PACKET_OUTGOING;

	/* Check if this is a batched SKB (marked by protocol field with high bit
	 * set). Batched SKBs are intentionally larger than MTU as they contain
	 * multiple MCTP packets packed together. The driver will clear this marker.
	 */
	bool is_batched = (skb->protocol == htons(ETH_P_MCTP | 0x8000));

	/* Batched SKBs (protocol with high bit set) skip MTU check */
	if (!is_batched) {
		skb->protocol = htons(ETH_P_MCTP);
		if (skb->len > dst->mtu) {
			MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_drops);
			MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_dropped_mtu_exceeded);
			kfree_skb(skb);
			return -EMSGSIZE;
		}
	}

	/* direct route; use the hwaddr we stashed in sendmsg (unless tunnel) */
	if (dst->halen && !is_tunnel) {
		if (dst->halen != skb->dev->addr_len) {
			/* sanity check, sendmsg should have already caught this */
			MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_drops);
			MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_dropped_bad_addrlen);
			kfree_skb(skb);
			return -EMSGSIZE;
		}
		daddr = dst->haddr;
	} else {
		/* If lookup fails let the device handle daddr==NULL */
		if (mctp_neigh_lookup(dst->dev, dst->nexthop, daddr_buf) == 0)
			daddr = daddr_buf;
	}

	rc = dev_hard_header(skb, skb->dev, ntohs(skb->protocol),
			     daddr, skb->dev->dev_addr, skb->len);
	if (rc < 0) {
		MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_drops);
		MCTP_SOCK_STAT_INC(skb->sk, dev_net(skb->dev), tx_dropped_no_route);
		kfree_skb(skb);
		return -EHOSTUNREACH;
	}

	mctp_flow_prepare_output(skb, dst->dev);

	if (!is_batched) {
		rc = mctp_nf_egress_check(&skb, dst->dev, hdr);
		if (!skb) {
			netdev_dbg(
				dst->dev->dev,
				"MCTP nf: egress filter dropped packet src=%u dst=%u rc=%d\n",
				hdr->src, hdr->dest, rc);
			return rc;
		}
		if (rc)
			return rc;
	}

	trace_mctp_route_output(skb, skb->dev);
	rc = dev_queue_xmit(skb);
	if (rc)
		rc = net_xmit_errno(rc);

	return rc;
}

/* route alloc/release */
static void mctp_route_release(struct mctp_route *rt)
{
	if (refcount_dec_and_test(&rt->refs)) {
		if (rt->dst_type == MCTP_ROUTE_DIRECT)
			mctp_dev_put(rt->dev);
		kfree_rcu(rt, rcu);
	}
}

/* returns a route with the refcount at 1 */
static struct mctp_route *mctp_route_alloc(void)
{
	struct mctp_route *rt;

	rt = kzalloc(sizeof(*rt), GFP_KERNEL);
	if (!rt)
		return NULL;

	INIT_LIST_HEAD(&rt->list);
	refcount_set(&rt->refs, 1);
	rt->output = mctp_dst_discard;

	return rt;
}

unsigned int mctp_default_net(struct net *net)
{
	return READ_ONCE(net->mctp.default_net);
}

int mctp_default_net_set(struct net *net, unsigned int index)
{
	if (index == 0)
		return -EINVAL;
	WRITE_ONCE(net->mctp.default_net, index);
	return 0;
}

/* tag management */
static void mctp_reserve_tag(struct net *net, struct mctp_sk_key *key,
			     struct mctp_sock *msk, unsigned long lifetime)
{
	struct netns_mctp *mns = &net->mctp;

	lockdep_assert_held(&mns->keys_lock);

	key->expiry = jiffies + lifetime;
	timer_reduce(&msk->key_expiry, key->expiry);

	/* we hold the net->key_lock here, allowing updates to both
	 * then net and sk
	 */
	hlist_add_head_rcu(&key->hlist, &mns->keys);
	hlist_add_head_rcu(&key->sklist, &msk->keys);
	refcount_inc(&key->refs);
}

/* Persistent hint for incremental tag allocation per (net, peer).
 * Survives key release so req-resp pairs get tag 0, 1, 2, ... over time.
 *
 * The hint is deliberately keyed on (net, peer) only, not on the local EID.
 * The two callers of mctp_alloc_local_tag() disagree on 'local' for the same
 * peer: mctp_local_output() passes a concrete source EID (saddr), while
 * mctp_ioctl_alloctag() passes MCTP_ADDR_ANY. Tags for a given peer are drawn
 * from a single shared 8-tag space on the wire, so keying on 'local' would
 * fork one peer into several independent next_tag sequences. The hint only
 * selects a starting point; the conflict walk above still guarantees the
 * chosen tag is actually free, so sharing a sequence across local EIDs is safe.
 */
struct mctp_tag_hint {
	unsigned int net;
	mctp_eid_t peer;
	u8 next_tag;
	struct hlist_node hlist;
};

static struct mctp_tag_hint *mctp_tag_hint_find(struct netns_mctp *mns,
					       unsigned int netid,
					       mctp_eid_t peer)
{
	struct mctp_tag_hint *h;

	hlist_for_each_entry(h, &mns->tag_hints, hlist) {
		if (h->net == netid && h->peer == peer)
			return h;
	}
	return NULL;
}

/* Allocate a locally-owned tag value for (local, peer), and reserve
 * it for the socket msk
 */
struct mctp_sk_key *mctp_alloc_local_tag(struct mctp_sock *msk,
					 unsigned int netid,
					 mctp_eid_t local, mctp_eid_t peer,
					 bool manual, u8 *tagp,
					 unsigned long lifetime)
{
	struct net *net = sock_net(&msk->sk);
	struct netns_mctp *mns = &net->mctp;
	struct mctp_sk_key *key, *tmp;
	unsigned long flags;
	pid_t holder_pids[8];
	u8 holder_counts[8];
	int nholders, i;
	u8 tagbits;

	/* for NULL destination EIDs, we may get a response from any peer */
	if (peer == MCTP_ADDR_NULL)
		peer = MCTP_ADDR_ANY;

	/* be optimistic, alloc now */
	key = mctp_key_alloc(msk, netid, local, peer, 0, GFP_KERNEL);
	if (!key) {
		MCTP_SOCK_STAT_INC(&msk->sk, net, tx_dropped_no_memory);
		return ERR_PTR(-ENOMEM);
	}

	/* 8 possible tag values */
	tagbits = 0xff;

	spin_lock_irqsave(&mns->keys_lock, flags);

	/* Walk through the existing keys, looking for potential conflicting
	 * tags. If we find a conflict, clear that bit from tagbits.
	 * This includes dynamically allocated and preallocated/reserved tags
	 * (all are keys in the list); we must not allocate a tag already in use.
	 */
	hlist_for_each_entry(tmp, &mns->keys, hlist) {
		/* We can check the lookup fields (*_addr, tag) without the
		 * lock held, they don't change over the lifetime of the key.
		 */

		/* tags are net-specific */
		if (tmp->net != netid)
			continue;

		/* if we don't own the tag, it can't conflict */
		if (tmp->tag & MCTP_HDR_FLAG_TO)
			continue;

		/* Since we're avoiding conflicting entries, match peer and
		 * local addresses, including with a wildcard on ANY. See
		 * 'A note on key allocations' for background.
		 */
		if (peer != MCTP_ADDR_ANY &&
		    !mctp_address_matches(tmp->peer_addr, peer))
			continue;

		if (local != MCTP_ADDR_ANY &&
		    !mctp_address_matches(tmp->local_addr, local))
			continue;

		spin_lock(&tmp->lock);
		/* key must still be valid. If we find a match, clear the
		 * potential tag value
		 */
		if (tmp->valid)
			tagbits &= ~(1 << tmp->tag);
		spin_unlock(&tmp->lock);

		if (!tagbits)
			break;
	}

	if (tagbits) {
		struct mctp_tag_hint *hint;
		u8 start;
		u8 tag;
		int i;

		/* Use persistent hint so req-resp pairs get 0, 1, 2, ... even after key release */
		hint = mctp_tag_hint_find(mns, netid, peer);
		start = hint ? hint->next_tag : 0;

		/* Pick first free tag starting from hint (incremental per destination) */
		for (i = 0; i < 8; i++) {
			tag = (start + i) % 8;
			if (tagbits & (1 << tag))
				break;
		}
		key->tag = tag;

		/* Update hint for next allocation */
		if (hint) {
			hint->next_tag = (tag + 1) % 8;
		} else {
			hint = kzalloc(sizeof(*hint), GFP_ATOMIC);
			if (hint) {
				hint->net = netid;
				hint->peer = peer;
				hint->next_tag = (tag + 1) % 8;
				hlist_add_head(&hint->hlist, &mns->tag_hints);
			}
		}

		mctp_reserve_tag(net, key, msk, lifetime);
		trace_mctp_key_acquire(key);

		key->manual_alloc = manual;
		*tagp = key->tag;
	}

	/* Collect per-PID tag holder info while still under keys_lock, so we
	 * can emit a useful diagnostic without needing to re-acquire the lock.
	 */
	nholders = 0;
	if (!tagbits) {
		hlist_for_each_entry(tmp, &mns->keys, hlist) {
			struct mctp_sock *tmp_msk;
			bool valid;

			if (tmp->net != netid)
				continue;
			/* skip TO-owned tags — they can't conflict with us */
			if (tmp->tag & MCTP_HDR_FLAG_TO)
				continue;
			if (peer != MCTP_ADDR_ANY &&
			    !mctp_address_matches(tmp->peer_addr, peer))
				continue;
			if (local != MCTP_ADDR_ANY &&
			    !mctp_address_matches(tmp->local_addr, local))
				continue;

			spin_lock(&tmp->lock);
			valid = tmp->valid;
			spin_unlock(&tmp->lock);
			if (!valid)
				continue;

			tmp_msk = container_of(tmp->sk, struct mctp_sock, sk);
			for (i = 0; i < nholders; i++) {
				if (holder_pids[i] == tmp_msk->pid) {
					holder_counts[i]++;
					break;
				}
			}
			if (i == nholders && nholders < ARRAY_SIZE(holder_pids)) {
				holder_pids[nholders] = tmp_msk->pid;
				holder_counts[nholders] = 1;
				nholders++;
			}
		}
	}

	spin_unlock_irqrestore(&mns->keys_lock, flags);

	if (!tagbits) {
		pr_warn_ratelimited("mctp: tag exhaustion net %u local %u peer %u\n",
				    netid, local, peer);
		for (i = 0; i < nholders; i++)
			pr_warn_ratelimited("mctp:   pid %d holds %u tag(s)\n",
					    holder_pids[i], holder_counts[i]);

		mctp_key_unref(key);
		MCTP_SOCK_STAT_INC(&msk->sk, net, tx_drops);
		MCTP_SOCK_STAT_INC(&msk->sk, net, tx_dropped_tag_exhaustion);
		return ERR_PTR(-EBUSY);
	}

	return key;
}

static struct mctp_sk_key *mctp_lookup_prealloc_tag(struct mctp_sock *msk,
						    unsigned int netid,
						    mctp_eid_t daddr,
						    u8 req_tag, u8 *tagp)
{
	struct net *net = sock_net(&msk->sk);
	struct netns_mctp *mns = &net->mctp;
	struct mctp_sk_key *key, *tmp;
	unsigned long flags;

	req_tag &= ~(MCTP_TAG_PREALLOC | MCTP_TAG_OWNER);
	key = NULL;

	spin_lock_irqsave(&mns->keys_lock, flags);

	hlist_for_each_entry(tmp, &mns->keys, hlist) {
		if (tmp->net != netid)
			continue;

		if (tmp->tag != req_tag)
			continue;

		if (!mctp_address_matches(tmp->peer_addr, daddr))
			continue;

		if (!tmp->manual_alloc)
			continue;

		spin_lock(&tmp->lock);
		if (tmp->valid) {
			key = tmp;
			refcount_inc(&key->refs);
			spin_unlock(&tmp->lock);
			break;
		}
		spin_unlock(&tmp->lock);
	}
	spin_unlock_irqrestore(&mns->keys_lock, flags);

	if (!key) {
		MCTP_SOCK_STAT_INC(&msk->sk, net, tx_drops);
		MCTP_SOCK_STAT_INC(&msk->sk, net, tx_dropped_tag_exhaustion);
		return ERR_PTR(-ENOENT);
	}

	if (tagp)
		*tagp = key->tag;

	return key;
}

/* routing lookups */
static unsigned int mctp_route_netid(struct mctp_route *rt)
{
	return rt->dst_type == MCTP_ROUTE_DIRECT ?
		READ_ONCE(rt->dev->net) : rt->gateway.net;
}

static bool mctp_rt_match_eid(struct mctp_route *rt,
			      unsigned int net, mctp_eid_t eid)
{
	return mctp_route_netid(rt) == net &&
		rt->min <= eid && rt->max >= eid;
}

/* compares match, used for duplicate prevention */
static bool mctp_rt_compare_exact(struct mctp_route *rt1,
				  struct mctp_route *rt2)
{
	ASSERT_RTNL();

	if (mctp_route_netid(rt1) != mctp_route_netid(rt2))
		return false;

	if (rt1->max < rt2->min || rt1->min > rt2->max)
		return false;

	/* Allow multiple local routes for same EID on different interfaces */
	if (rt1->type == RTN_LOCAL && rt2->type == RTN_LOCAL &&
	    rt1->dev != rt2->dev)
		return false;

	return true;
}

/* must only be called on a direct route, as the final output hop */
static void mctp_dst_from_route(struct mctp_dst *dst, mctp_eid_t eid,
				unsigned int mtu, struct mctp_route *route)
{
	mctp_dev_hold(route->dev);
	dst->nexthop = eid;
	dst->dev = route->dev;
	dst->mtu = READ_ONCE(dst->dev->dev->mtu);
	if (mtu)
		dst->mtu = min(dst->mtu, mtu);
	dst->halen = 0;
	dst->output = route->output;
}

int mctp_dst_from_extaddr(struct mctp_dst *dst, struct net *net, int ifindex,
			  unsigned char halen, const unsigned char *haddr)
{
	struct net_device *netdev;
	struct mctp_dev *dev;
	int rc = -ENOENT;

	if (halen > sizeof(dst->haddr))
		return -EINVAL;

	rcu_read_lock();

	netdev = dev_get_by_index_rcu(net, ifindex);
	if (!netdev)
		goto out_unlock;

	if (netdev->addr_len != halen) {
		rc = -EINVAL;
		goto out_unlock;
	}

	dev = __mctp_dev_get(netdev);
	if (!dev)
		goto out_unlock;

	dst->dev = dev;
	dst->mtu = READ_ONCE(netdev->mtu);
	dst->halen = halen;
	dst->output = mctp_dst_output;
	dst->nexthop = 0;
	memcpy(dst->haddr, haddr, halen);

	rc = 0;

out_unlock:
	rcu_read_unlock();
	return rc;
}

void mctp_dst_release(struct mctp_dst *dst)
{
	mctp_dev_put(dst->dev);
}

static struct mctp_route *mctp_route_lookup_single(struct net *net,
						   unsigned int dnet,
						   mctp_eid_t daddr)
{
	struct mctp_route *rt;

	list_for_each_entry_rcu(rt, &net->mctp.routes, list) {
		if (mctp_rt_match_eid(rt, dnet, daddr))
			return rt;
	}

	return NULL;
}

/* populates *dst on successful lookup, if set */
int mctp_route_lookup(struct net *net, unsigned int dnet,
		      mctp_eid_t daddr, struct mctp_dst *dst)
{
	const unsigned int max_depth = 32;
	unsigned int depth, mtu = 0;
	int rc = -EHOSTUNREACH;

	rcu_read_lock();

	for (depth = 0; depth < max_depth; depth++) {
		struct mctp_route *rt;

		rt = mctp_route_lookup_single(net, dnet, daddr);
		if (!rt)
			break;

		/* clamp mtu to the smallest in the path, allowing 0
		 * to specify no restrictions
		 */
		if (mtu && rt->mtu)
			mtu = min(mtu, rt->mtu);
		else
			mtu = mtu ?: rt->mtu;

		if (rt->dst_type == MCTP_ROUTE_DIRECT) {
			if (dst)
				mctp_dst_from_route(dst, daddr, mtu, rt);
			rc = 0;
			break;

		} else if (rt->dst_type == MCTP_ROUTE_GATEWAY) {
			daddr = rt->gateway.eid;
		}
	}

	rcu_read_unlock();

	return rc;
}

static int mctp_route_lookup_null(struct net *net, struct net_device *dev,
				  struct mctp_dst *dst)
{
	int rc = -EHOSTUNREACH;
	struct mctp_route *rt;

	rcu_read_lock();

	list_for_each_entry_rcu(rt, &net->mctp.routes, list) {
		if (rt->dst_type != MCTP_ROUTE_DIRECT || rt->type != RTN_LOCAL)
			continue;

		if (rt->dev->dev != dev)
			continue;

		mctp_dst_from_route(dst, 0, 0, rt);
		rc = 0;
		break;
	}

	rcu_read_unlock();

	return rc;
}

/* Fragment and batch: pack multiple fragments into a single SKB with space
 * for transport headers. The transport driver will fill in headers and send.
 * This function may send multiple batches if the message is large.
 */
static int mctp_do_fragment_route_batch(struct mctp_dst *dst, struct sk_buff *skb,
					unsigned int mtu, u8 tag,
					unsigned int batch_hdr_len,
					unsigned int batch_max_xfer)
{
	const unsigned int hlen = sizeof(struct mctp_hdr);
	struct mctp_hdr *hdr, *hdr2;
	struct mctp_skb_cb *cb;
	struct sk_buff *batch_skb;
	unsigned int pos, size, headroom;
	unsigned int total_len, num_frags;
	unsigned int skb_pos;
	u8 *batch_data;
	u8 seq;
	int rc;

	hdr = mctp_hdr(skb);
	seq = 0;
	headroom = skb_headroom(skb);

	/* we've got the header */
	skb_pull(skb, hlen);

	skb_pos = 0;
	rc = 0;

	while (skb_pos < skb->len) {
		total_len = 0;
		num_frags = 0;
		for (pos = skb_pos; pos < skb->len;) {
			size = min(mtu - hlen, skb->len - pos);
			total_len += batch_hdr_len + hlen + size;
			num_frags++;
			pos += size;
			if (total_len + batch_hdr_len + hlen + 1 > batch_max_xfer)
				break;
		}

		pr_debug(
			"mctp: Batching %u fragments (pos=%u/%u), total_len=%u, batch_max_xfer=%u\n",
			num_frags, skb_pos, skb->len, total_len,
			batch_max_xfer);

		/* Allocate a single large SKB to hold all fragments */
		struct sock *sk_save = skb->sk;

		/* Hold a ref so sk_save stays valid across kfree_skb — the skb
		 * destructor (sock_wfree) may drop the socket's last ref otherwise.
		 */
		if (sk_save)
			sock_hold(sk_save);

		batch_skb = alloc_skb(headroom + total_len, GFP_KERNEL);
		if (!batch_skb) {
			kfree_skb(skb);
			MCTP_SOCK_STAT_INC(sk_save, dev_net(dst->dev->dev), tx_dropped_no_memory);
			if (sk_save)
				sock_put(sk_save);
			return -ENOMEM;
		}

		if (sk_save)
			sock_put(sk_save);

		/* Copy generic SKB properties */
		batch_skb->protocol = htons(ETH_P_MCTP | 0x8000); /* Mark as batched */
		batch_skb->priority = skb->priority;
		batch_skb->dev = skb->dev;
		memcpy(batch_skb->cb, skb->cb, sizeof(batch_skb->cb));

		if (skb->sk)
			skb_set_owner_w(batch_skb, skb->sk);

		skb_reserve(batch_skb, headroom);
		skb_reset_network_header(batch_skb);
		batch_data = skb_put(batch_skb, total_len);

		cb = mctp_cb(batch_skb);
		cb->net = mctp_cb(skb)->net;

		skb_ext_copy(batch_skb, skb);

		pos = skb_pos;
		while (num_frags--) {
			unsigned int pkt_len;
			bool is_last_fragment_in_message;
			void *transport_hdr;

			size = min(mtu - hlen, skb->len - pos);
			is_last_fragment_in_message = (pos + size >= skb->len);
			pkt_len = batch_hdr_len + hlen + size;

			transport_hdr = batch_data;
			batch_data += batch_hdr_len;

			hdr2 = (struct mctp_hdr *)batch_data;
			hdr2->ver = hdr->ver;
			hdr2->dest = hdr->dest;
			hdr2->src = hdr->src;
			hdr2->flags_seq_tag = tag &
				(MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);

			if (pos == 0)
				hdr2->flags_seq_tag |= MCTP_HDR_FLAG_SOM;
			if (is_last_fragment_in_message)
				hdr2->flags_seq_tag |= MCTP_HDR_FLAG_EOM;

			hdr2->flags_seq_tag |= seq << MCTP_HDR_SEQ_SHIFT;

			skb_copy_bits(skb, pos, batch_data + hlen, size);

			if (dst->dev->ops && dst->dev->ops->fill_batch_hdr)
				dst->dev->ops->fill_batch_hdr(transport_hdr, pkt_len);

			batch_data += hlen + size;
			seq = (seq + 1) & MCTP_HDR_SEQ_MASK;
			pos += size;
		}

		skb_pos = pos;

		rc = dst->output(dst, batch_skb);
		if (rc) {
			rc = net_xmit_errno(rc);
			break;
		}
	}

	consume_skb(skb);
	return rc;
}

static int mctp_do_fragment_route(struct mctp_dst *dst, struct sk_buff *skb,
				  unsigned int mtu, u8 tag)
{
	const unsigned int hlen = sizeof(struct mctp_hdr);
	struct mctp_hdr *hdr, *hdr2;
	unsigned int pos, size, headroom;
	struct sk_buff *skb2;
	unsigned int batch_hdr_len;
	unsigned int batch_max_xfer;
	int rc;
	u8 seq;

	hdr = mctp_hdr(skb);
	seq = 0;
	rc = 0;

	if (mtu < hlen + 1) {
		/* Increment stat before freeing — skb->sk is invalid after kfree_skb */
		MCTP_SOCK_STAT_INC(skb->sk, dev_net(dst->dev->dev), tx_dropped_mtu_exceeded);
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	/* If batching is enabled on the device, use the batch path */
	if (dst->dev && dst->dev->tx_batching_enabled) {
		batch_hdr_len = dst->dev->tx_batch_hdr_len;
		batch_max_xfer = dst->dev->tx_batch_max_xfer;
		if (batch_hdr_len && batch_max_xfer)
			return mctp_do_fragment_route_batch(dst, skb, mtu, tag,
							   batch_hdr_len,
							   batch_max_xfer);
	}

	/* keep same headroom as the original skb */
	headroom = skb_headroom(skb);

	/* we've got the header */
	skb_pull(skb, hlen);

	for (pos = 0; pos < skb->len;) {
		bool is_last_fragment;

		/* size of message payload */
		size = min(mtu - hlen, skb->len - pos);
		is_last_fragment = (pos + size >= skb->len);

		skb2 = alloc_skb(headroom + hlen + size, GFP_KERNEL);
		if (!skb2) {
			MCTP_SOCK_STAT_INC(skb->sk, dev_net(dst->dev->dev), tx_dropped_no_memory);
			rc = -ENOMEM;
			break;
		}

		/* generic skb copy */
		skb2->protocol = skb->protocol;
		skb2->priority = skb->priority;
		skb2->dev = skb->dev;
		memcpy(skb2->cb, skb->cb, sizeof(skb2->cb));

		if (skb->sk)
			skb_set_owner_w(skb2, skb->sk);

		/* establish packet */
		skb_reserve(skb2, headroom);
		skb_reset_network_header(skb2);
		skb_put(skb2, hlen + size);
		skb2->transport_header = skb2->network_header + hlen;

		/* copy header fields, calculate SOM/EOM flags & seq */
		hdr2 = mctp_hdr(skb2);
		hdr2->ver = hdr->ver;
		hdr2->dest = hdr->dest;
		hdr2->src = hdr->src;
		hdr2->flags_seq_tag = tag &
			(MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);

		if (pos == 0)
			hdr2->flags_seq_tag |= MCTP_HDR_FLAG_SOM;

		if (is_last_fragment)
			hdr2->flags_seq_tag |= MCTP_HDR_FLAG_EOM;

		hdr2->flags_seq_tag |= seq << MCTP_HDR_SEQ_SHIFT;

		/* copy message payload */
		skb_copy_bits(skb, pos, skb_transport_header(skb2), size);

		/* we need to copy the extensions, for MCTP flow data */
		skb_ext_copy(skb2, skb);

		/* do route */
		rc = dst->output(dst, skb2);
		if (rc)
			break;

		seq = (seq + 1) & MCTP_HDR_SEQ_MASK;
		pos += size;
	}

	consume_skb(skb);
	return rc;
}

int mctp_local_output(struct sock *sk, struct mctp_dst *dst,
		      struct sk_buff *skb, mctp_eid_t daddr, u8 req_tag,
		      unsigned int tag_timeout_ms)
{
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct mctp_sk_key *key;
	struct mctp_hdr *hdr;
	unsigned long flags;
	unsigned int netid;
	unsigned int mtu;
	unsigned int pkt_len;
	mctp_eid_t saddr;
	int rc;
	u8 tag;

	KUNIT_STATIC_STUB_REDIRECT(mctp_local_output, sk, dst, skb, daddr,
				   req_tag, tag_timeout_ms);

	rc = -ENODEV;

	spin_lock_irqsave(&dst->dev->addrs_lock, flags);
	if (dst->dev->num_addrs == 0) {
		rc = -EHOSTUNREACH;
	} else {
		/* use the outbound interface's first address as our source */
		saddr = dst->dev->addrs[0];
		rc = 0;
	}
	spin_unlock_irqrestore(&dst->dev->addrs_lock, flags);
	netid = READ_ONCE(dst->dev->net);

	if (rc) {
		struct net *net = sock_net(sk);

		/* Track no-route drops */
		spin_lock_bh(&msk->stats_lock);
		msk->stats.tx_drops++;
		msk->stats.tx_dropped_no_route++;
		spin_unlock_bh(&msk->stats_lock);

		atomic64_inc(&net->mctp.tx_drops);
		atomic64_inc(&net->mctp.tx_dropped_no_route);

		goto out_release;
	}

	if (req_tag & MCTP_TAG_OWNER) {
		if (req_tag & MCTP_TAG_PREALLOC)
			key = mctp_lookup_prealloc_tag(msk, netid, daddr,
						       req_tag, &tag);
		else {
			unsigned long lifetime = mctp_effective_key_lifetime(msk,
									     dst->dev,
									     tag_timeout_ms);

			key = mctp_alloc_local_tag(msk, netid, saddr, daddr,
						   false, &tag, lifetime);
		}

		if (IS_ERR(key)) {
			rc = PTR_ERR(key);
			goto out_release;
		}
		mctp_skb_set_flow(skb, key);
		/* done with the key in this scope */
		mctp_key_unref(key);
		tag |= MCTP_HDR_FLAG_TO;
	} else {
		key = NULL;
		tag = req_tag & MCTP_TAG_MASK;
	}

	skb->pkt_type = PACKET_OUTGOING;
	skb->protocol = htons(ETH_P_MCTP);
	skb->priority = 0;
	skb_reset_transport_header(skb);
	skb_push(skb, sizeof(struct mctp_hdr));
	skb_reset_network_header(skb);
	skb->dev = dst->dev->dev;

	/* set up common header fields */
	hdr = mctp_hdr(skb);
	hdr->ver = 1;
	hdr->dest = daddr;
	hdr->src = saddr;

	/* Capture original message header for error reporting on fragmented messages.
	 * This ensures that if a middle or end fragment fails, we can still report
	 * the original header (PLDM Instance ID, etc.) to the application.
	 */
	if (key && skb->len > sizeof(struct mctp_hdr)) {
		unsigned long flags2;
		size_t payload_offset, available, capture_len;

		spin_lock_irqsave(&key->lock, flags2);

		/* Capture message type (first byte after MCTP header) */
		key->orig_msg_type = *((u8 *)(skb->data + sizeof(struct mctp_hdr)));

		/* Capture first 32 bytes of payload (after message type) */
		payload_offset = sizeof(struct mctp_hdr) + 1;
		available = skb->len - payload_offset;
		capture_len = min_t(size_t, available, sizeof(key->orig_payload));

		if (capture_len > 0) {
			memcpy(key->orig_payload, skb->data + payload_offset, capture_len);
			key->orig_payload_len = capture_len;
		} else {
			key->orig_payload_len = 0;
		}

		spin_unlock_irqrestore(&key->lock, flags2);
	}

	mtu = dst->mtu;

	/* Save length before transmit — rt->output() and mctp_do_fragment_route()
	 * consume the skb, making skb->len invalid afterward.
	 */
	pkt_len = skb->len;

	trace_mctp_local_output(saddr, daddr, tag, pkt_len);
	if (pkt_len <= mtu) {
		hdr->flags_seq_tag = MCTP_HDR_FLAG_SOM |
			MCTP_HDR_FLAG_EOM | tag;
		rc = dst->output(dst, skb);
	} else {
		rc = mctp_do_fragment_route(dst, skb, mtu, tag);
	}

	/* Update statistics based on transmission result */
	if (rc == 0) {
		struct net *net = sock_net(sk);
		u64 tx_time = ktime_get_ns();

		/* Update per-socket stats */
		spin_lock_bh(&msk->stats_lock);
		msk->stats.tx_packets++;
		msk->stats.tx_bytes += pkt_len;
		msk->stats.tx_messages++;
		msk->stats.last_tx_time = tx_time;
		spin_unlock_bh(&msk->stats_lock);

		/* Update global stats */
		atomic64_inc(&net->mctp.tx_packets);
		atomic64_add(pkt_len, &net->mctp.tx_bytes);
		atomic64_inc(&net->mctp.tx_messages);
	} else {
		struct net *net = sock_net(sk);

		/* Update error stats with specific reasons */
		spin_lock_bh(&msk->stats_lock);
		msk->stats.tx_errors++;
		msk->stats.tx_drops++;
		if (rc == -ENETDOWN || rc == -ENODEV)
			msk->stats.tx_dropped_device_down++;
		else if (rc == -EPERM || rc == -EACCES)
			msk->stats.tx_dropped_permission++;
		else if (rc == -EMSGSIZE)
			msk->stats.tx_dropped_mtu_exceeded++;
		spin_unlock_bh(&msk->stats_lock);

		atomic64_inc(&net->mctp.tx_errors);
		atomic64_inc(&net->mctp.tx_drops);
		if (rc == -ENETDOWN || rc == -ENODEV)
			atomic64_inc(&net->mctp.tx_dropped_device_down);
		else if (rc == -EPERM || rc == -EACCES)
			atomic64_inc(&net->mctp.tx_dropped_permission);
		else if (rc == -EMSGSIZE)
			atomic64_inc(&net->mctp.tx_dropped_mtu_exceeded);
	}

	/* route output functions consume the skb, even on error */
	skb = NULL;

out_release:
	kfree_skb(skb);
	return rc;
}

/* route management */

/* mctp_route_add(): Add the provided route, previously allocated via
 * mctp_route_alloc(). On success, takes ownership of @rt, which includes a
 * hold on rt->dev for usage in the route table. On failure a caller will want
 * to mctp_route_release().
 *
 * We expect that the caller has set rt->type, rt->dst_type, rt->min, rt->max,
 * rt->mtu and either rt->dev (with a reference held appropriately) or
 * rt->gateway. Other fields will be populated.
 */
static int mctp_route_add(struct net *net, struct mctp_route *rt)
{
	struct mctp_route *ert;

	if (!mctp_address_unicast(rt->min) || !mctp_address_unicast(rt->max))
		return -EINVAL;

	if (rt->dst_type == MCTP_ROUTE_DIRECT && !rt->dev)
		return -EINVAL;

	if (rt->dst_type == MCTP_ROUTE_GATEWAY && !rt->gateway.eid)
		return -EINVAL;

	switch (rt->type) {
	case RTN_LOCAL:
		rt->output = mctp_dst_input;
		break;
	case RTN_UNICAST:
		rt->output = mctp_dst_output;
		break;
	default:
		return -EINVAL;
	}

	ASSERT_RTNL();

	/* Prevent duplicate identical routes. */
	list_for_each_entry(ert, &net->mctp.routes, list) {
		if (mctp_rt_compare_exact(rt, ert)) {
			return -EEXIST;
		}
	}

	list_add_rcu(&rt->list, &net->mctp.routes);

	if (rt->dev)
		trace_mctp_route_add(rt->dev->dev, rt->min, rt->max - rt->min, rt->mtu);
	return 0;
}

static int mctp_route_remove(struct net *net, unsigned int netid,
			     mctp_eid_t daddr_start, unsigned int daddr_extent,
			     unsigned char type)
{
	struct mctp_route *rt, *tmp;
	mctp_eid_t daddr_end;
	bool dropped;

	if (daddr_extent > 0xff || daddr_start + daddr_extent >= 255)
		return -EINVAL;

	daddr_end = daddr_start + daddr_extent;
	dropped = false;

	ASSERT_RTNL();

	list_for_each_entry_safe(rt, tmp, &net->mctp.routes, list) {
		if (mctp_route_netid(rt) == netid &&
		    rt->min == daddr_start && rt->max == daddr_end &&
		    rt->type == type) {
			if (rt->dev)
				trace_mctp_route_del(rt->dev->dev, rt->min, rt->max - rt->min);
			list_del_rcu(&rt->list);
			/* TODO: immediate RTM_DELROUTE */
			mctp_route_release(rt);
			dropped = true;
		}
	}

	return dropped ? 0 : -ENOENT;
}

int mctp_route_add_local(struct mctp_dev *mdev, mctp_eid_t addr)
{
	struct mctp_route *rt;
	int rc;

	rt = mctp_route_alloc();
	if (!rt)
		return -ENOMEM;

	rt->min = addr;
	rt->max = addr;
	rt->dst_type = MCTP_ROUTE_DIRECT;
	rt->dev = mdev;
	rt->type = RTN_LOCAL;

	mctp_dev_hold(rt->dev);

	rc = mctp_route_add(dev_net(mdev->dev), rt);
	if (rc)
		mctp_route_release(rt);

	return rc;
}

int mctp_route_remove_local(struct mctp_dev *mdev, mctp_eid_t addr)
{
	return mctp_route_remove(dev_net(mdev->dev), mdev->net,
				 addr, 0, RTN_LOCAL);
}

/* removes all entries for a given device */
void mctp_route_remove_dev(struct mctp_dev *mdev)
{
	struct net *net = dev_net(mdev->dev);
	struct mctp_route *rt, *tmp;

	ASSERT_RTNL();
	list_for_each_entry_safe(rt, tmp, &net->mctp.routes, list) {
		if (rt->dst_type == MCTP_ROUTE_DIRECT && rt->dev == mdev) {
			list_del_rcu(&rt->list);
			/* TODO: immediate RTM_DELROUTE */
			mctp_route_release(rt);
		}
	}
}

/* Release device references from all keys bound to a departing mctp_dev.
 * Called during NETDEV_UNREGISTER to ensure the mctp_dev (and thus the
 * underlying net_device) refcount can reach zero. Without this,
 * manually-allocated keys (SIOCMCTPALLOCTAG) hold mctp_dev refs
 * indefinitely, blocking unregister_netdevice().
 */
void mctp_key_remove_dev(struct mctp_dev *mdev)
{
	struct net *net = dev_net(mdev->dev);
	struct mctp_sk_key *key;
	unsigned long flags, fl2;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry(key, &net->mctp.keys, hlist) {
		spin_lock_irqsave(&key->lock, fl2);
		if (key->dev == mdev)
			mctp_dev_release_key(key->dev, key);
		spin_unlock_irqrestore(&key->lock, fl2);
	}
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);
}

void mctp_key_remove_addr(struct mctp_dev *mdev, mctp_eid_t addr)
{
	struct net *net = dev_net(mdev->dev);
	unsigned int netid = READ_ONCE(mdev->net);
	int ifindex = mdev->dev->ifindex;
	struct mctp_sk_key *key;
	struct hlist_node *tmp;
	unsigned long flags, fl2;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry_safe(key, tmp, &net->mctp.keys, hlist) {
		struct sk_buff *skb;
		int bound_if;
		bool remove;

		spin_lock_irqsave(&key->lock, fl2);
		bound_if = READ_ONCE(key->sk->sk_bound_dev_if);
		/* Exact-address keys on unbound sockets are network-scoped and
		 * have no device association, as is usual for RX reassembly.
		 * For wildcard keys, explicit device ownership takes precedence
		 * over the socket binding.
		 */
		remove = key->net == netid &&
			 ((key->local_addr == addr &&
			   (key->dev == mdev ||
			    (!key->dev &&
			     (!bound_if || bound_if == ifindex)))) ||
			  (key->local_addr == MCTP_ADDR_ANY &&
			   (key->dev == mdev ||
			    (!key->dev && bound_if == ifindex))));
		if (!remove) {
			spin_unlock_irqrestore(&key->lock, fl2);
			continue;
		}

		trace_mctp_key_release(key, MCTP_TRACE_KEY_INVALIDATED);
		skb = key->reasm_head;
		key->reasm_head = NULL;
		key->reasm_dead = true;
		key->valid = false;
		mctp_dev_release_key(key->dev, key);
		spin_unlock_irqrestore(&key->lock, fl2);

		if (!hlist_unhashed(&key->hlist)) {
			hlist_del_init(&key->hlist);
			hlist_del_init(&key->sklist);
			mctp_key_unref(key);
		}

		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);
}

/* Lookup bound socket for packet delivery when no route exists */
static struct mctp_route *mctp_route_lookup_bound_socket(struct net *net, struct sk_buff *skb)
{
	struct mctp_hdr *mh;
	struct mctp_sock *msk;
	struct mctp_route *rt = NULL;

	WARN_ON(!rcu_read_lock_held());

	mh = mctp_hdr(skb);

	if (!skb_headlen(skb))
		return NULL;

	/* Look for bound sockets that match this packet */
	msk = mctp_lookup_bind(net, skb);
	if (msk) {
		/* Create a temporary route for socket delivery */
		rt = mctp_route_alloc();
		if (rt) {
			rt->min = mh->dest;
			rt->max = mh->dest;
			rt->type = RTN_LOCAL;
			rt->output = mctp_dst_input;
			rt->dev = NULL;
		}
	}

	return rt;
}

/* Incoming packet-handling */

static int mctp_pkttype_receive(struct sk_buff *skb, struct net_device *dev,
				struct packet_type *pt,
				struct net_device *orig_dev)
{
	struct net *net = dev_net(dev);
	struct mctp_dev *mdev;
	struct mctp_skb_cb *cb;
	struct mctp_dst dst;
	struct mctp_hdr *mh;
	int rc;
	u8 ver;

	rcu_read_lock();
	mdev = __mctp_dev_get(dev);
	rcu_read_unlock();
	if (!mdev) {
		/* basic non-data sanity checks */
		goto err_drop;
	}

	if (!pskb_may_pull(skb, sizeof(struct mctp_hdr))) {
		MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_invalid_header);
		goto err_drop;
	}

	skb_reset_transport_header(skb);
	skb_reset_network_header(skb);

	/* We have enough for a header; decode and route */
	mh = mctp_hdr(skb);
	ver = mh->ver & MCTP_HDR_VER_MASK;
	if (ver < MCTP_VER_MIN || ver > MCTP_VER_MAX)
		goto err_invalid_hdr;

	/* source must be valid unicast or null; drop reserved ranges and
	 * broadcast
	 */
	if (!(mctp_address_unicast(mh->src) || mctp_address_null(mh->src)))
		goto err_invalid_hdr;

	/* dest address: as above, but allow broadcast */
	if (!(mctp_address_unicast(mh->dest) || mctp_address_null(mh->dest) ||
	      mctp_address_broadcast(mh->dest)))
		goto err_invalid_hdr;

	/* MCTP drivers must populate halen/haddr */
	if (dev->type == ARPHRD_MCTP) {
		cb = mctp_cb(skb);
	} else {
		cb = __mctp_cb(skb);
		cb->halen = 0;
	}
	cb->net = READ_ONCE(mdev->net);
	cb->ifindex = dev->ifindex;

	if (!mctp_nf_ingress_check(skb, mdev, mh)) {
		netdev_dbg(dev, "MCTP nf: ingress filter dropped packet src=%u dst=%u\n",
			  mh->src, mh->dest);
		goto err_drop;
	}

	rc = mctp_route_lookup(net, cb->net, mh->dest, &dst);

	/* NULL EID, but addressed to our physical address */
	if (rc && mh->dest == MCTP_ADDR_NULL && skb->pkt_type == PACKET_HOST) {
		rc = mctp_route_lookup_null(net, dev, &dst);
		if (rc) {
			/* No RTN_LOCAL route: try bound sockets so NULL EID reaches them */
			struct mctp_route *rt;

			rcu_read_lock();
			rt = mctp_route_lookup_bound_socket(net, skb);
			rcu_read_unlock();
			if (rt) {
				rt->dev = mdev;
				mctp_dev_hold(rt->dev);
				mctp_dst_from_route(&dst, mh->dest, 0, rt);
				mctp_route_release(rt);
				rc = 0;
			}
		}
	}

	if (rc) {
		trace_mctp_drop_packet(skb, "no_route_found");
		MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_no_route);
		goto err_drop;
	}

	dst.output(&dst, skb);
	mctp_dst_release(&dst);
	mctp_dev_put(mdev);

	return NET_RX_SUCCESS;

err_invalid_hdr:
	MCTP_SOCK_STAT_INC(NULL, net, rx_dropped_invalid_header);
err_drop:
	kfree_skb(skb);
	mctp_dev_put(mdev);
	return NET_RX_DROP;
}

static struct packet_type mctp_packet_type = {
	.type = cpu_to_be16(ETH_P_MCTP),
	.func = mctp_pkttype_receive,
};

/* netlink interface */

static const struct nla_policy rta_mctp_policy[RTA_MAX + 1] = {
	[RTA_DST]		= { .type = NLA_U8 },
	[RTA_METRICS]		= { .type = NLA_NESTED },
	[RTA_OIF]		= { .type = NLA_U32 },
	[RTA_GATEWAY]		= NLA_POLICY_EXACT_LEN(sizeof(struct mctp_fq_addr)),
};

static const struct nla_policy rta_metrics_policy[RTAX_MAX + 1] = {
	[RTAX_MTU]		= { .type = NLA_U32 },
};

/* base parsing; common to both _lookup and _populate variants.
 *
 * For gateway routes (which have a RTA_GATEWAY, and no RTA_OIF), we populate
 * *gatweayp. for direct routes (RTA_OIF, no RTA_GATEWAY), we populate *mdev.
 */
static int mctp_route_nlparse_common(struct net *net, struct nlmsghdr *nlh,
				     struct netlink_ext_ack *extack,
				     struct nlattr **tb, struct rtmsg **rtm,
				     struct mctp_dev **mdev,
				     struct mctp_fq_addr *gatewayp,
				     mctp_eid_t *daddr_start)
{
	struct mctp_fq_addr *gateway = NULL;
	unsigned int ifindex = 0;
	struct net_device *dev;
	int rc;

	rc = nlmsg_parse(nlh, sizeof(struct rtmsg), tb, RTA_MAX,
			 rta_mctp_policy, extack);
	if (rc < 0) {
		NL_SET_ERR_MSG(extack, "incorrect format");
		return rc;
	}

	if (!tb[RTA_DST]) {
		NL_SET_ERR_MSG(extack, "dst EID missing");
		return -EINVAL;
	}
	*daddr_start = nla_get_u8(tb[RTA_DST]);

	if (tb[RTA_OIF])
		ifindex = nla_get_u32(tb[RTA_OIF]);

	if (tb[RTA_GATEWAY])
		gateway = nla_data(tb[RTA_GATEWAY]);

	if (ifindex && gateway) {
		NL_SET_ERR_MSG(extack,
			       "cannot specify both ifindex and gateway");
		return -EINVAL;

	} else if (ifindex) {
		dev = __dev_get_by_index(net, ifindex);
		if (!dev) {
			NL_SET_ERR_MSG(extack, "bad ifindex");
			return -ENODEV;
		}
		*mdev = mctp_dev_get_rtnl(dev);
		if (!*mdev)
			return -ENODEV;
		gatewayp->eid = 0;

	} else if (gateway) {
		if (!mctp_address_unicast(gateway->eid)) {
			NL_SET_ERR_MSG(extack, "bad gateway");
			return -EINVAL;
		}

		gatewayp->eid = gateway->eid;
		gatewayp->net = gateway->net != MCTP_NET_ANY ?
			gateway->net :
			READ_ONCE(net->mctp.default_net);
		*mdev = NULL;

	} else {
		NL_SET_ERR_MSG(extack, "no route output provided");
		return -EINVAL;
	}

	*rtm = nlmsg_data(nlh);
	if ((*rtm)->rtm_family != AF_MCTP) {
		NL_SET_ERR_MSG(extack, "route family must be AF_MCTP");
		return -EINVAL;
	}

	if ((*rtm)->rtm_type != RTN_UNICAST) {
		NL_SET_ERR_MSG(extack, "rtm_type must be RTN_UNICAST");
		return -EINVAL;
	}

	return 0;
}

/* Route parsing for lookup operations; we only need the "route target"
 * components (ie., network and dest-EID range).
 */
static int mctp_route_nlparse_lookup(struct net *net, struct nlmsghdr *nlh,
				     struct netlink_ext_ack *extack,
				     unsigned char *type, unsigned int *netid,
				     mctp_eid_t *daddr_start,
				     unsigned int *daddr_extent)
{
	struct nlattr *tb[RTA_MAX + 1];
	struct mctp_fq_addr gw;
	struct mctp_dev *mdev;
	struct rtmsg *rtm;
	int rc;

	rc = mctp_route_nlparse_common(net, nlh, extack, tb, &rtm,
				       &mdev, &gw, daddr_start);
	if (rc)
		return rc;

	if (mdev) {
		*netid = mdev->net;
	} else if (gw.eid) {
		*netid = gw.net;
	} else {
		/* bug: _nlparse_common should not allow this */
		return -1;
	}

	*type = rtm->rtm_type;
	*daddr_extent = rtm->rtm_dst_len;

	return 0;
}

/* Full route parse for RTM_NEWROUTE: populate @rt. On success,
 * MCTP_ROUTE_DIRECT routes (ie, those with a direct dev) will hold a reference
 * to that dev.
 */
static int mctp_route_nlparse_populate(struct net *net, struct nlmsghdr *nlh,
				       struct netlink_ext_ack *extack,
				       struct mctp_route *rt)
{
	struct nlattr *tbx[RTAX_MAX + 1];
	struct nlattr *tb[RTA_MAX + 1];
	unsigned int daddr_extent;
	struct mctp_fq_addr gw;
	mctp_eid_t daddr_start;
	struct mctp_dev *dev;
	struct rtmsg *rtm;
	u32 mtu = 0;
	int rc;

	rc = mctp_route_nlparse_common(net, nlh, extack, tb, &rtm,
				       &dev, &gw, &daddr_start);
	if (rc)
		return rc;

	daddr_extent = rtm->rtm_dst_len;

	if (daddr_extent > 0xff || daddr_extent + daddr_start >= 255) {
		NL_SET_ERR_MSG(extack, "invalid eid range");
		return -EINVAL;
	}

	if (tb[RTA_METRICS]) {
		rc = nla_parse_nested(tbx, RTAX_MAX, tb[RTA_METRICS],
				      rta_metrics_policy, NULL);
		if (rc < 0) {
			NL_SET_ERR_MSG(extack, "incorrect RTA_METRICS format");
			return rc;
		}
		if (tbx[RTAX_MTU])
			mtu = nla_get_u32(tbx[RTAX_MTU]);
	}

	rt->type = rtm->rtm_type;
	rt->min = daddr_start;
	rt->max = daddr_start + daddr_extent;
	rt->mtu = mtu;
	if (gw.eid) {
		rt->dst_type = MCTP_ROUTE_GATEWAY;
		rt->gateway.eid = gw.eid;
		rt->gateway.net = gw.net;
	} else {
		rt->dst_type = MCTP_ROUTE_DIRECT;
		rt->dev = dev;
		mctp_dev_hold(rt->dev);
	}

	return 0;
}

static int mctp_newroute(struct sk_buff *skb, struct nlmsghdr *nlh,
			 struct netlink_ext_ack *extack)
{
	struct net *net = sock_net(skb->sk);
	struct mctp_route *rt;
	int rc;

	rt = mctp_route_alloc();
	if (!rt)
		return -ENOMEM;

	rc = mctp_route_nlparse_populate(net, nlh, extack, rt);
	if (rc < 0)
		goto err_free;

	if (rt->dst_type == MCTP_ROUTE_DIRECT &&
	    rt->dev->dev->flags & IFF_LOOPBACK) {
		NL_SET_ERR_MSG(extack, "no routes to loopback");
		rc = -EINVAL;
		goto err_free;
	}

	rc = mctp_route_add(net, rt);
	if (!rc)
		return 0;

err_free:
	mctp_route_release(rt);
	return rc;
}

static int mctp_delroute(struct sk_buff *skb, struct nlmsghdr *nlh,
			 struct netlink_ext_ack *extack)
{
	struct net *net = sock_net(skb->sk);
	unsigned int netid, daddr_extent;
	unsigned char type = RTN_UNSPEC;
	mctp_eid_t daddr_start;
	int rc;

	rc = mctp_route_nlparse_lookup(net, nlh, extack, &type, &netid,
				       &daddr_start, &daddr_extent);
	if (rc < 0)
		return rc;

	/* we only have unicast routes */
	if (type != RTN_UNICAST)
		return -EINVAL;

	rc = mctp_route_remove(net, netid, daddr_start, daddr_extent, type);
	return rc;
}

static int mctp_fill_rtinfo(struct sk_buff *skb, struct mctp_route *rt,
			    u32 portid, u32 seq, int event, unsigned int flags)
{
	struct nlmsghdr *nlh;
	struct rtmsg *hdr;
	void *metrics;

	nlh = nlmsg_put(skb, portid, seq, event, sizeof(*hdr), flags);
	if (!nlh)
		return -EMSGSIZE;

	hdr = nlmsg_data(nlh);
	memset(hdr, 0, sizeof(*hdr));
	hdr->rtm_family = AF_MCTP;

	/* we use the _len fields as a number of EIDs, rather than
	 * a number of bits in the address
	 */
	hdr->rtm_dst_len = rt->max - rt->min;
	hdr->rtm_src_len = 0;
	hdr->rtm_tos = 0;
	hdr->rtm_table = RT_TABLE_DEFAULT;
	hdr->rtm_protocol = RTPROT_STATIC; /* everything is user-defined */
	hdr->rtm_type = rt->type;

	if (nla_put_u8(skb, RTA_DST, rt->min))
		goto cancel;

	metrics = nla_nest_start_noflag(skb, RTA_METRICS);
	if (!metrics)
		goto cancel;

	if (rt->mtu) {
		if (nla_put_u32(skb, RTAX_MTU, rt->mtu))
			goto cancel;
	}

	nla_nest_end(skb, metrics);

	if (rt->dst_type == MCTP_ROUTE_DIRECT) {
		hdr->rtm_scope = RT_SCOPE_LINK;
		if (nla_put_u32(skb, RTA_OIF, rt->dev->dev->ifindex))
			goto cancel;
	} else if (rt->dst_type == MCTP_ROUTE_GATEWAY) {
		hdr->rtm_scope = RT_SCOPE_UNIVERSE;
		if (nla_put(skb, RTA_GATEWAY,
			    sizeof(rt->gateway), &rt->gateway))
			goto cancel;
	}

	nlmsg_end(skb, nlh);

	return 0;

cancel:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static int mctp_dump_rtinfo(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct mctp_route *rt;
	int s_idx, idx;

	/* TODO: allow filtering on route data, possibly under
	 * cb->strict_check
	 */

	/* TODO: change to struct overlay */
	s_idx = cb->args[0];
	idx = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(rt, &net->mctp.routes, list) {
		if (idx++ < s_idx)
			continue;
		if (mctp_fill_rtinfo(skb, rt,
				     NETLINK_CB(cb->skb).portid,
				     cb->nlh->nlmsg_seq,
				     RTM_NEWROUTE, NLM_F_MULTI) < 0)
			break;
	}

	rcu_read_unlock();
	cb->args[0] = idx;

	return skb->len;
}

/* net namespace implementation */
static int __net_init mctp_routes_net_init(struct net *net)
{
	struct netns_mctp *ns = &net->mctp;

	INIT_LIST_HEAD(&ns->routes);
	INIT_HLIST_HEAD(&ns->binds);
	mutex_init(&ns->bind_lock);
	INIT_HLIST_HEAD(&ns->keys);
	INIT_HLIST_HEAD(&ns->tag_hints);
	spin_lock_init(&ns->keys_lock);
	WARN_ON(mctp_default_net_set(net, MCTP_INITIAL_DEFAULT_NET));
	return 0;
}

static void __net_exit mctp_routes_net_exit(struct net *net)
{
	struct netns_mctp *ns = &net->mctp;
	struct mctp_route *rt, *tmp;
	struct mctp_tag_hint *hint;
	struct hlist_node *hint_tmp;
	unsigned long flags;

	spin_lock_irqsave(&ns->keys_lock, flags);
	hlist_for_each_entry_safe(hint, hint_tmp, &ns->tag_hints, hlist) {
		hlist_del(&hint->hlist);
		kfree(hint);
	}
	spin_unlock_irqrestore(&ns->keys_lock, flags);

	ASSERT_RTNL();

	list_for_each_entry_safe(rt, tmp, &net->mctp.routes, list) {
		list_del_rcu(&rt->list);
		mctp_route_release(rt);
	}
}

static void __net_exit mctp_routes_net_exit_batch(struct list_head *net_exit_list,
						  struct list_head *dev_kill_list)
{
	struct net *net;

	list_for_each_entry(net, net_exit_list, exit_list)
		mctp_routes_net_exit(net);
}

static struct pernet_operations mctp_net_ops = {
	.init = mctp_routes_net_init,
	.exit_batch_rtnl = mctp_routes_net_exit_batch,
};

static const struct rtnl_msg_handler mctp_route_rtnl_msg_handlers[] = {
	{THIS_MODULE, PF_MCTP, RTM_NEWROUTE, mctp_newroute, NULL, 0},
	{THIS_MODULE, PF_MCTP, RTM_DELROUTE, mctp_delroute, NULL, 0},
	{THIS_MODULE, PF_MCTP, RTM_GETROUTE, NULL, mctp_dump_rtinfo, 0},
};

int __init mctp_routes_init(void)
{
	int err;

	dev_add_pack(&mctp_packet_type);

	err = register_pernet_subsys(&mctp_net_ops);
	if (err)
		goto err_pernet;

	err = rtnl_register_many(mctp_route_rtnl_msg_handlers);
	if (err)
		goto err_rtnl;

	return 0;

err_rtnl:
	unregister_pernet_subsys(&mctp_net_ops);
err_pernet:
	dev_remove_pack(&mctp_packet_type);
	return err;
}

void mctp_routes_exit(void)
{
	rtnl_unregister_many(mctp_route_rtnl_msg_handlers);
	unregister_pernet_subsys(&mctp_net_ops);
	dev_remove_pack(&mctp_packet_type);
}

/**
 * mctp_lookup_sock_by_key - Find socket using MCTP key (tag-based lookup)
 * @skb: The SKB to look up
 * @dev: The network device
 * @found_key: Output parameter to return the matched key (optional, can be NULL)
 *
 * Uses MCTP tag to uniquely identify which socket sent the packet.
 * This properly handles multiple applications sending to the same remote EID.
 * If found_key is non-NULL, returns the matched key via output parameter.
 * The returned key is still in the hash table (caller should not free it).
 * Returns a reference to the sock if found (caller must sock_put), NULL otherwise.
 */
struct sock *mctp_lookup_sock_by_key(struct sk_buff *skb, struct net_device *dev,
				     struct mctp_sk_key **found_key)
{
	struct net *net = dev_net(dev);
	struct mctp_dev *mdev;
	struct mctp_hdr *mh;
	struct mctp_sk_key *key;
	struct sock *sk = NULL;
	unsigned long flags;
	unsigned int netid;
	u8 tag;

	/* Initialize output parameter */
	if (found_key)
		*found_key = NULL;

	if (!skb || skb->len < sizeof(struct mctp_hdr)) {
		pr_err("MCTP: lookup_sock_by_key: invalid skb (skb=%p len=%u)\n",
			skb, skb ? skb->len : 0);
		return NULL;
	}

	mh = mctp_hdr(skb);
	
	/* Get network ID from device rather than SKB control block.
	 * In URB completion context, skb->cb may not have valid magic,
	 * causing WARN_ON in mctp_cb(). We can safely get netid from
	 * the MCTP device instead.
	 */
	rcu_read_lock();
	mdev = __mctp_dev_get(dev);
	if (!mdev) {
		rcu_read_unlock();
		pr_err("MCTP: lookup_sock_by_key: no mctp_dev for %s\n", dev->name);
		return NULL;
	}
	netid = READ_ONCE(mdev->net);
	mctp_dev_put(mdev);
	rcu_read_unlock();

	/* Extract tag from MCTP header (bits 2-0) */
	tag = mh->flags_seq_tag & MCTP_HDR_TAG_MASK;

	pr_debug("MCTP: lookup_sock_by_key: netid=%u src=%u dest=%u tag=%u\n",
		netid, mh->src, mh->dest, tag);

	/* Look up key in global key table */
	spin_lock_irqsave(&net->mctp.keys_lock, flags);

	hlist_for_each_entry(key, &net->mctp.keys, hlist) {
		/* Match by: network, local EID, peer EID, tag */
		if (key->net != netid)
			continue;

		/* For TX errors: source = local, dest = peer
		 * Use wildcard matching to handle MCTP_ADDR_ANY in keys.
		 * This is critical for Get Endpoint ID (dest=0/NULL) which
		 * creates keys with peer_addr=MCTP_ADDR_ANY.
		 */
		if (!mctp_address_matches(key->local_addr, mh->src))
			continue;
		if (!mctp_address_matches(key->peer_addr, mh->dest))
			continue;

		/* The critical differentiator: TAG */
		if (key->tag != tag)
			continue;

	/* Found exact match! */
	sk = key->sk;
	if (sk) {
		sock_hold(sk);
		/* Return the matched key via output parameter */
		if (found_key)
			*found_key = key;
		pr_debug("MCTP: lookup_sock_by_key: FOUND socket %p (key: net=%u local=%u peer=%u tag=%u)\n",
			sk, key->net, key->local_addr, key->peer_addr, key->tag);
	}
	break;
}

spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

if (!sk) {
	pr_debug("MCTP: lookup_sock_by_key: NO MATCH found for netid=%u src=%u dest=%u tag=%u\n",
		netid, mh->src, mh->dest, tag);
}

	return sk;
}
EXPORT_SYMBOL_GPL(mctp_lookup_sock_by_key);

/**
 * mctp_lookup_sock_for_error - Find socket for error reporting
 * @skb: Packet that failed (contains addressing info)
 * @dev: Network device
 * @key: Existing key (if available, may be NULL)
 * @found_key: Output parameter to return the matched key (optional, can be NULL)
 *
 * Finds which socket should receive an error notification for failed
 * operations WE initiated (our requests/responses).
 *
 * Uses two-tier lookup:
 * 1. Use socket from existing key (if key provided and valid)
 *    - For RX reassembly errors where key is from reassembly context
 * 2. Try TX key lookup by tag (for TX errors from drivers)
 *    - TX key existence indicates operation we initiated
 *    - No TX key = not our transaction = don't report
 *    - Returns found key via found_key output parameter
 *
 * With BMC-to-device behavior where device uses TO=0 in responses,
 * TX keys naturally match responses (both have tag value 0-7 only).
 *
 * Device-initiated messages won't have TX keys, so errors on those
 * messages are correctly not reported (not our operations).
 *
 * DEADLOCK PREVENTION: Method 2 is skipped if key parameter is provided,
 * as this indicates call from __mctp_key_remove() which holds keys_lock.
 *
 * Returns: Socket with refcount held, or NULL if no socket found or
 *          error should not be reported. Caller must call sock_put()
 *          if non-NULL returned.
 */
struct sock *mctp_lookup_sock_for_error(struct sk_buff *skb,
					struct net_device *dev,
					struct mctp_sk_key *key,
					struct mctp_sk_key **found_key)
{
	struct net *net;
	struct sock *sk = NULL;
	struct mctp_hdr *mh;
	u8 tag;

	/* Initialize output parameter */
	if (found_key)
		*found_key = NULL;

	if (!skb || !dev)
		return NULL;

	if (skb->len < sizeof(struct mctp_hdr))
		return NULL;

	net = dev_net(dev);
	mh = mctp_hdr(skb);
	/* Extract tag value only (0-7), not TO bit.
	 * With BMC-to-device behavior where device uses TO=0 in responses,
	 * this naturally matches TX keys which are also stored without TO bit.
	 */
	tag = mh->flags_seq_tag & MCTP_HDR_TAG_MASK;

	/* Method 1: Use existing key socket (fastest path)
	 * If caller already has a key (e.g., from RX reassembly), use it directly.
	 */
	if (key && key->sk) {
		struct mctp_sock *check_msk = container_of(key->sk, struct mctp_sock, sk);
		
		/* Check if socket is still valid and open */
		if (sock_flag(&check_msk->sk, SOCK_DEAD)) {
			pr_debug("MCTP error: key socket is dead (closed)\n");
			return NULL;
		}
		
		/* Skip if this socket doesn't have error queue enabled */
		if (!check_msk->enable_errqueue) {
			pr_debug("MCTP error: key socket found but error queue disabled (src=%u, dest=%u, tag=%u)\n",
				 mh->src, mh->dest, tag);
			return NULL;
		}
		
		sk = key->sk;
		sock_hold(sk);
		/* Return the key via output parameter */
		if (found_key)
			*found_key = key;
		pr_debug("MCTP error: socket via key (src=%u, dest=%u, tag=%u)\n",
			 mh->src, mh->dest, tag);
		return sk;
	}

	/* Method 2: TX key lookup (best effort to get key with orig_payload)
	 * 
	 * Try to find TX key first because it provides:
	 * 1. Socket for error reporting
	 * 2. orig_payload for accurate error message content
	 * 
	 * TX key existence indicates operation WE initiated (request packet).
	 * This is the preferred method for TX errors because it gives complete context.
	 * 
	 * This is slower than Method 1 (requires hash table lookup + lock),
	 * but needed when skb->sk is lost (e.g., old drivers, special cases).
	 * 
	 * DEADLOCK PREVENTION: Skip this method if key parameter is provided,
	 * as this indicates call from __mctp_key_remove() which holds keys_lock.
	 * mctp_lookup_sock_by_key() also needs keys_lock, causing deadlock.
	 */
	if (!key) {
		struct mctp_sk_key *tx_key = NULL;
		
		/* Safe to do key lookup - not called from __mctp_key_remove() */
		sk = mctp_lookup_sock_by_key(skb, dev, &tx_key);
		if (sk) {
			struct mctp_sock *check_msk = container_of(sk, struct mctp_sock, sk);
			
			/* Check if socket is still valid and open */
			if (sock_flag(&check_msk->sk, SOCK_DEAD)) {
				pr_debug("MCTP error: TX key socket is dead (closed)\n");
				sock_put(sk);
				return NULL;
			}
			
			/* Skip if this socket doesn't have error queue enabled */
			if (!check_msk->enable_errqueue) {
				pr_debug("MCTP error: TX key socket found but error queue disabled (src=%u, dest=%u, tag=%u)\n",
					 mh->src, mh->dest, tag);
				sock_put(sk);
				return NULL;
			}
			
			/* Return the found TX key via output parameter */
			if (found_key)
				*found_key = tx_key;
			
			pr_debug("MCTP error: socket via TX key lookup (src=%u, dest=%u, tag=%u) with orig_payload\n",
				 mh->src, mh->dest, tag);
			return sk;
		}
		
		/* No TX key found - fall through to Method 3 (skb->sk fallback) */
		pr_debug("MCTP error: No TX key found, trying skb->sk fallback (src=%u, dest=%u, tag=%u)\n",
			 mh->src, mh->dest, tag);
	}

	/* Method 3: Check skb->sk directly (fallback for TX errors without key)
	 * 
	 * If TX key lookup failed but SKB has socket pointer, use it as fallback.
	 * This is faster than TX key lookup but provides NO key (no orig_payload).
	 * Only used when TX key doesn't exist (e.g., response packets, old code paths).
	 * 
	 * DEADLOCK PREVENTION: Skip this method if key parameter is provided,
	 * as this indicates call from __mctp_key_remove() which holds keys_lock.
	 * 
	 * NOTE: found_key will be NULL when returning via this path.
	 * Error reporting will work but without orig_payload context.
	 */
	if (!key && skb->sk && skb->sk->sk_family == AF_MCTP) {
		struct mctp_sock *msk = container_of(skb->sk, struct mctp_sock, sk);
		
		/* Check if socket is still valid and open */
		if (sock_flag(&msk->sk, SOCK_DEAD)) {
			pr_debug("MCTP error: skb->sk socket is dead (closed)\n");
			return NULL;
		}
		
		/* Skip if this socket doesn't have error queue enabled */
		if (!msk->enable_errqueue) {
			pr_debug("MCTP error: skb->sk socket found but error queue disabled (src=%u, dest=%u)\n",
				 mh->src, mh->dest);
			return NULL;
		}
		
		/* This is the socket that sent this packet - safe to report error.
		 * NOTE: found_key stays NULL - no orig_payload available.
		 */
		sock_hold(skb->sk);
		pr_debug("MCTP error: socket via skb->sk fallback (src=%u, dest=%u) - no key, no orig_payload\n",
			 mh->src, mh->dest);
		return skb->sk;
	}

	/* If we reach here:
	 * - key parameter provided: called from __mctp_key_remove()
	 *   Methods 2 and 3 skipped for deadlock prevention
	 * - OR Method 2 found no valid skb->sk
	 * - OR Method 3 found no TX key: not our transaction
	 * 
	 * In any case, don't report error.
	 */
	return NULL;
}
EXPORT_SYMBOL_GPL(mctp_lookup_sock_for_error);

/**
 * mctp_lookup_tx_key_for_rx_error - Look up TX key for RX error reporting
 * @net: Network namespace
 * @netid: MCTP network ID
 * @local_eid: Local EID (from RX packet dest)
 * @peer_eid: Peer EID (from RX packet src)
 * @tag: Tag value (without TO bit)
 *
 * For RX errors, we need to find the original TX request to report the
 * correct payload to the application. This function looks up the TX key
 * by reversing the addressing from the RX packet.
 *
 * Returns: TX key with refcount incremented, or NULL if not found
 */
static struct mctp_sk_key *mctp_lookup_tx_key_for_rx_error(struct net *net,
							    unsigned int netid,
							    mctp_eid_t local_eid,
							    mctp_eid_t peer_eid,
							    u8 tag)
{
	struct mctp_sk_key *key, *ret = NULL;
	unsigned long flags;

	/* For RX errors, the TX key was created with:
	 * - local_addr = local_eid (our address when we sent request)
	 * - peer_addr = peer_eid (who we sent to)
	 * - tag = tag value (without TO bit, as TX key stores incoming perspective)
	 */
	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry(key, &net->mctp.keys, hlist) {
		if (!mctp_key_match(key, netid, local_eid, peer_eid, tag))
			continue;

		spin_lock(&key->lock);
		if (key->valid && key->orig_payload_len > 0) {
			refcount_inc(&key->refs);
			ret = key;
			spin_unlock(&key->lock);
			break;
		}
		spin_unlock(&key->lock);
	}
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

	pr_debug("mctp_lookup_tx_key_for_rx_error: netid=%u local=%u peer=%u tag=%u -> %s\n",
		 netid, local_eid, peer_eid, tag, ret ? "FOUND" : "NOT FOUND");

	return ret;
}

/**
 * mctp_queue_error - Queue error to socket error queue
 * @sk: Socket to report error to
 * @skb: SKB that failed (contains addressing info)
 * @error_code: errno value (EPROTO, ETIMEDOUT, EMSGSIZE for RX; EHOSTUNREACH, ENXIO for TX)
 * @dev: Network device (used to extract MCTP network ID)
 * @direction: MCTP_DIR_TX or MCTP_DIR_RX
 * @binding: enum mctp_phys_binding value (MCTP_PHYS_BINDING_USB, MCTP_PHYS_BINDING_SMBUS, etc.)
 * @key: Key for error context. TX key for TX errors, RX key for RX errors (may be TX key).
 *
 * Builds an mctp_error structure and queues it to the socket's error queue.
 * Applications can read this via recvmsg(MSG_ERRQUEUE).
 *
 * Strategy:
 * - TX errors: Use provided TX key's orig_payload (captured before fragmentation)
 * - RX errors: Look up TX key using addressing from RX key/SKB, use TX key's orig_payload
 *              (the original request we sent). Only report if TX key found - this ensures
 *              we only report errors for responses to OUR requests, not unsolicited packets.
 *
 * This ensures applications always receive the REQUEST payload for both TX and RX errors,
 * allowing them to identify which transaction failed.
 *
 * Common error codes:
 *   RX errors: EPROTO (sequence error), ETIMEDOUT (reassembly timeout), EMSGSIZE (too large)
 *   TX errors: EHOSTUNREACH, ENXIO, ENODEV, ESHUTDOWN, ENOMEM, EPROTO, etc.
 */
void mctp_queue_error(struct sock *sk, struct sk_buff *skb,
		      int error_code, struct net_device *dev, u8 direction, u8 binding,
		      struct mctp_sk_key *key)
{
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct mctp_sk_key *tx_key = NULL;
	struct mctp_error *mctp_err;
	struct sk_buff *err_skb;
	struct mctp_hdr *mh;
	struct mctp_dev *mdev;
	unsigned int netid;
	size_t capture_len;
	bool key_found = false;

	/* Extract network ID from device.
	 * This is safer than using skb->cb which may be corrupted by qdisc.
	 */
	rcu_read_lock();
	mdev = __mctp_dev_get(dev);
	if (!mdev) {
		rcu_read_unlock();
		pr_debug("MCTP: Failed to get mctp_dev for error reporting\n");
		return;
	}
	netid = READ_ONCE(mdev->net);
	mctp_dev_put(mdev);
	rcu_read_unlock();

	if (!msk->enable_errqueue) {
		pr_debug("MCTP: Error queue not enabled, skipping error report\n");
		return;
	}

	/* Extract addressing from SKB */
	if (!skb || skb->len < sizeof(struct mctp_hdr)) {
		pr_debug("MCTP: Invalid SKB for error reporting\n");
		return;
	}
	mh = mctp_hdr(skb);

	/* Handle TX vs RX errors differently.
	 * 
	 * TX errors: Use TX key if available, otherwise extract from SKB
	 * RX errors: Always need TX key to identify our original transaction
	 */
	if (direction == MCTP_DIR_TX) {
		/* ===== TX ERROR PATH =====
		 * TX errors can occur on:
		 *   1. REQUEST packets (owner=1) - TX key exists with orig_payload
		 *   2. RESPONSE packets (owner=0) - No TX key, extract from SKB
		 *   3. Fragmented packets - Middle/end need key, first has SKB payload
		 */
		tx_key = key;  /* For TX errors, key parameter holds TX key */
		
		if (tx_key && tx_key->orig_payload_len > 0) {
			/* Have TX key with payload - use it (fragmented requests) */
			key_found = true;
			pr_debug("mctp_queue_error: TX error - using TX key orig_payload (len=%u)\n",
				 tx_key->orig_payload_len);
		} else {
			/* No key or empty key - must extract from SKB.
			 * This handles:
			 *   - Unfragmented messages (complete payload in SKB)
			 *   - First fragment (SOM=1, has msg_type in SKB)
			 *   - Response packets (no TX key created)
			 * 
			 * CRITICAL: Only works for first fragment or unfragmented.
			 * Middle/end fragments without key cannot be reported.
			 */
			pr_debug("mctp_queue_error: TX error - no TX key, will extract from SKB\n");
			key_found = false;
		}
	} else {
		/* ===== RX ERROR PATH =====
		 * RX errors occur when receiving responses to OUR requests.
		 * We MUST find the TX key to identify our original transaction.
		 * If no TX key exists, this is an unsolicited packet - don't report.
		 */
		if (error_code == ETIMEDOUT) {
			/* RX reassembly timeout: fragments never completed.
			 * RX key IS the TX key (same object, reused for response tracking).
			 * It contains orig_payload from when we sent the original REQUEST.
			 */
			tx_key = key;  /* For RX timeout, key parameter holds RX key (which is TX key) */
			
			if (tx_key && tx_key->orig_payload_len > 0) {
				key_found = true;
				pr_debug("mctp_queue_error: RX timeout - has TX orig_payload (REQUEST, len=%u)\n",
					 tx_key->orig_payload_len);
			} else {
				/* No orig_payload = unsolicited request from device, not our transaction */
				pr_debug("mctp_queue_error: RX timeout - no TX origin (unsolicited), NOT REPORTING\n");
				return;
			}
		} else {
			/* RX sequence/SOM error: look up TX key by reversing addressing.
			 * Response came FROM peer (mh->src) TO us (mh->dest).
			 * Our original request was FROM us (mh->dest) TO peer (mh->src).
			 */
			u8 tag = mh->flags_seq_tag & MCTP_HDR_TAG_MASK;
			
			pr_debug("mctp_queue_error: RX error - looking up TX key (local=%u, peer=%u, tag=%u)\n",
				 mh->dest, mh->src, tag);
			
			tx_key = mctp_lookup_tx_key_for_rx_error(dev_net(dev), netid,
								 mh->dest, mh->src, tag);
			if (tx_key && tx_key->orig_payload_len > 0) {
				key_found = true;
				pr_debug("mctp_queue_error: RX error - TX key found with payload\n");
			} else {
				if (tx_key) {
					pr_debug("mctp_queue_error: RX error - TX key has no payload, NOT REPORTING\n");
					mctp_key_unref(tx_key);
				} else {
					pr_debug("mctp_queue_error: RX error - TX key NOT FOUND (unsolicited), NOT REPORTING\n");
				}
				return;
			}
		}
	}

	/* Allocate SKB for error */
	err_skb = alloc_skb(sizeof(*mctp_err), GFP_ATOMIC);
	if (!err_skb) {
		if (direction == MCTP_DIR_RX)
			mctp_key_unref(tx_key);
		return;
	}

	/* Build error structure */
	mctp_err = (struct mctp_error *)skb_put(err_skb, sizeof(*mctp_err));
	memset(mctp_err, 0, sizeof(*mctp_err));

	/* Fill basic error information */
	mctp_err->error_code = error_code;
	mctp_err->direction = direction;
	mctp_err->binding = binding;
	mctp_err->timestamp_ns = ktime_get_ns();

	/* Fill addressing from SKB */
	mctp_err->src_eid = mh->src;
	mctp_err->dest_eid = mh->dest;
	mctp_err->tag = mh->flags_seq_tag & MCTP_HDR_TAG_MASK;

	/* Extract payload for error report based on direction and availability. */
	if (direction == MCTP_DIR_TX) {
		/* ===== TX ERROR: Payload Extraction =====
		 * Try TX key first, fall back to SKB extraction
		 */
		if (key_found) {
			/* Use TX key payload (captured before fragmentation) */
			mctp_err->msg_type = tx_key->orig_msg_type;
			capture_len = min_t(size_t, tx_key->orig_payload_len,
					   MCTP_ERROR_PAYLOAD_SIZE);
			memcpy(mctp_err->payload, tx_key->orig_payload, capture_len);
			mctp_err->payload_len = capture_len;
			pr_debug("mctp_queue_error: TX - used key payload (len=%u)\n", capture_len);
		} else {
			/* No key - extract from SKB (responses, unfragmented, first fragments) */
			u8 flags = mh->flags_seq_tag;
			bool is_first_or_unfragmented = (flags & MCTP_HDR_FLAG_SOM);
			
			if (!is_first_or_unfragmented) {
				/* Middle/end fragment without key - cannot extract payload.
				 * Don't report - first fragment error already queued.
				 */
				pr_debug("mctp_queue_error: TX middle/end fragment without key - NOT REPORTING\n");
				kfree_skb(err_skb);
				return;
			}
			
			/* First fragment or unfragmented - SKB has msg_type and payload */
			size_t mctp_hdr_size = sizeof(struct mctp_hdr);
			size_t available = skb->len - mctp_hdr_size;
			
			if (available > 0) {
				u8 *payload_start = skb->data + mctp_hdr_size;
				
				/* First byte after MCTP header is message type */
				mctp_err->msg_type = *payload_start;
				
				/* Capture up to 32 bytes of payload (after message type) */
				if (available > 1) {
					capture_len = min_t(size_t, available - 1,
							   MCTP_ERROR_PAYLOAD_SIZE);
					memcpy(mctp_err->payload, payload_start + 1, capture_len);
					mctp_err->payload_len = capture_len;
				} else {
					mctp_err->payload_len = 0;
				}
				pr_debug("mctp_queue_error: TX - extracted from SKB (msg_type=%u, len=%u)\n",
					 mctp_err->msg_type, mctp_err->payload_len);
			} else {
				pr_debug("mctp_queue_error: TX - SKB too small, no payload\n");
				mctp_err->payload_len = 0;
			}
		}
	} else {
		/* ===== RX ERROR: Payload Extraction =====
		 * Always use TX key payload (original REQUEST we sent)
		 * We already validated key_found above, so tx_key is valid here.
		 */
		mctp_err->msg_type = tx_key->orig_msg_type;
		capture_len = min_t(size_t, tx_key->orig_payload_len,
				   MCTP_ERROR_PAYLOAD_SIZE);
		memcpy(mctp_err->payload, tx_key->orig_payload, capture_len);
		mctp_err->payload_len = capture_len;
		pr_debug("mctp_queue_error: RX - used TX key payload (REQUEST, len=%u)\n", capture_len);
		
		/* Release TX key if we looked it up (non-timeout RX errors) */
		if (error_code != ETIMEDOUT)
			mctp_key_unref(tx_key);
	}

	/* Queue error to socket */
	if (sock_queue_err_skb(sk, err_skb) == 0) {
		/* Successfully queued, trigger error report */
		sk_error_report(sk);
		pr_debug("mctp_queue_error: Error queued successfully (code=%d, %s, src=%u->dest=%u)\n",
			 error_code, direction == MCTP_DIR_TX ? "TX" : "RX",
			 mctp_err->src_eid, mctp_err->dest_eid);
	} else {
		/* Failed to queue - free the SKB to avoid memory leak */
		kfree_skb(err_skb);
		pr_debug("mctp_queue_error: Failed to queue error to socket\n");
	}
}
EXPORT_SYMBOL_GPL(mctp_queue_error);

#if IS_ENABLED(CONFIG_MCTP_TEST)
#include "test/route-test.c"
#endif
