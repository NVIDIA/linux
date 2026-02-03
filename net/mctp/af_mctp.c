// SPDX-License-Identifier: GPL-2.0
/*
 * Management Component Transport Protocol (MCTP)
 *
 * Copyright (c) 2021 Code Construct
 * Copyright (c) 2021 Google
 */

#include <linux/compat.h>
#include <linux/if_arp.h>
#include <linux/net.h>
#include <linux/mctp.h>
#include <linux/module.h>
#include <linux/socket.h>

#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <net/sock.h>

#define CREATE_TRACE_POINTS
#include <trace/events/mctp.h>

#include "mctp-socket-error-inject.h"

/* socket implementation */

static void mctp_sk_expire_keys(struct timer_list *timer);

static int mctp_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	if (sk) {
		sock->sk = NULL;
		sk->sk_prot->close(sk, 0);
	}

	return 0;
}

/* Generic sockaddr checks, padding checks only so far */
static bool mctp_sockaddr_is_ok(const struct sockaddr_mctp *addr)
{
	return !addr->__smctp_pad0 && !addr->__smctp_pad1;
}

static bool mctp_sockaddr_ext_is_ok(const struct sockaddr_mctp_ext *addr)
{
	return !addr->__smctp_pad0[0] &&
	       !addr->__smctp_pad0[1] &&
	       !addr->__smctp_pad0[2];
}

static int mctp_bind(struct socket *sock, struct sockaddr *addr, int addrlen)
{
	struct sock *sk = sock->sk;
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct sockaddr_mctp *smctp;
	int rc;

	if (addrlen < sizeof(*smctp))
		return -EINVAL;

	if (addr->sa_family != AF_MCTP)
		return -EAFNOSUPPORT;

	if (!capable(CAP_NET_BIND_SERVICE))
		return -EACCES;

	/* it's a valid sockaddr for MCTP, cast and do protocol checks */
	smctp = (struct sockaddr_mctp *)addr;

	if (!mctp_sockaddr_is_ok(smctp))
		return -EINVAL;

	lock_sock(sk);

	/* TODO: allow rebind */
	if (sk_hashed(sk)) {
		rc = -EADDRINUSE;
		goto out_release;
	}
	msk->bind_net = smctp->smctp_network;
	msk->bind_addr = smctp->smctp_addr.s_addr;
	msk->bind_type = smctp->smctp_type & 0x7f; /* ignore the IC bit */

	rc = sk->sk_prot->hash(sk);
	
out_release:
	release_sock(sk);

	return rc;
}

static int mctp_sendmsg(struct socket *sock, struct msghdr *msg, size_t len)
{
	DECLARE_SOCKADDR(struct sockaddr_mctp *, addr, msg->msg_name);
	int rc, addrlen = msg->msg_namelen;
	struct sock *sk = sock->sk;
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct mctp_skb_cb *cb;
	struct mctp_route *rt;
	struct sk_buff *skb = NULL;
	int hlen;

	if (addr) {
		const u8 tagbits = MCTP_TAG_MASK | MCTP_TAG_OWNER |
			MCTP_TAG_PREALLOC;

		if (addrlen < sizeof(struct sockaddr_mctp))
			return -EINVAL;
		if (addr->smctp_family != AF_MCTP)
			return -EINVAL;
		if (!mctp_sockaddr_is_ok(addr))
			return -EINVAL;
		if (addr->smctp_tag & ~tagbits)
			return -EINVAL;
		/* can't preallocate a non-owned tag */
		if (addr->smctp_tag & MCTP_TAG_PREALLOC &&
		    !(addr->smctp_tag & MCTP_TAG_OWNER))
			return -EINVAL;

	} else {
		/* TODO: connect()ed sockets */
		return -EDESTADDRREQ;
	}

	if (!capable(CAP_NET_RAW))
		return -EACCES;

	/* Socket-level error injection - test application error handling
	 * Simulates various sendto() failures at socket layer (binding-agnostic):
	 * - EBUSY: Tag allocation failure (all 8 tags in use)
	 * - EHOSTUNREACH: No route to destination
	 * - ENOBUFS/ENOMEM: Memory allocation failure
	 * - EAGAIN: Would block (non-blocking socket)
	 */
	rc = mctp_socket_error_inject_sendmsg();
	if (rc < 0)
		return rc;

	if (addr->smctp_network == MCTP_NET_ANY)
		addr->smctp_network = mctp_default_net(sock_net(sk));

	/* direct addressing */
	if (msk->addr_ext && addrlen >= sizeof(struct sockaddr_mctp_ext)) {
		DECLARE_SOCKADDR(struct sockaddr_mctp_ext *,
				 extaddr, msg->msg_name);
		struct net_device *dev;
		int bound_dev_if;

		rc = -EINVAL;
		rcu_read_lock();
		dev = dev_get_by_index_rcu(sock_net(sk), extaddr->smctp_ifindex);
		/* check for correct halen */
		if (dev && extaddr->smctp_halen == dev->addr_len) {
			/* Check SO_BINDTODEVICE constraint */
			bound_dev_if = READ_ONCE(sk->sk_bound_dev_if);
			if (bound_dev_if && bound_dev_if != dev->ifindex) {
				rc = -EINVAL;
			} else {
				hlen = LL_RESERVED_SPACE(dev) + sizeof(struct mctp_hdr);
				rc = 0;
			}
		}
		rcu_read_unlock();
		if (rc)
			goto err_free;
		rt = NULL;
	} else {
		int bound_dev_if;

		rt = mctp_route_lookup(sock_net(sk), addr->smctp_network,
				       addr->smctp_addr.s_addr);
		if (!rt) {
			rc = -EHOSTUNREACH;
			goto err_free;
		}
		
		/* Check SO_BINDTODEVICE constraint */
		bound_dev_if = READ_ONCE(sk->sk_bound_dev_if);
		if (bound_dev_if && rt->dev && 
		    bound_dev_if != rt->dev->dev->ifindex) {
			rc = -EINVAL;
			goto err_free;
		}
		
		hlen = LL_RESERVED_SPACE(rt->dev->dev) + sizeof(struct mctp_hdr);
	}

	skb = sock_alloc_send_skb(sk, hlen + 1 + len,
				  msg->msg_flags & MSG_DONTWAIT, &rc);
	if (!skb)
		return rc;

	skb_reserve(skb, hlen);

	/* set type as fist byte in payload */
	*(u8 *)skb_put(skb, 1) = addr->smctp_type;

	rc = memcpy_from_msg((void *)skb_put(skb, len), msg, len);
	if (rc < 0)
		goto err_free;

	/* set up cb */
	cb = __mctp_cb(skb);
	cb->net = addr->smctp_network;

	if (!rt) {
		/* fill extended address in cb */
		DECLARE_SOCKADDR(struct sockaddr_mctp_ext *,
				 extaddr, msg->msg_name);

		if (!mctp_sockaddr_ext_is_ok(extaddr) ||
		    extaddr->smctp_halen > sizeof(cb->haddr)) {
			rc = -EINVAL;
			goto err_free;
		}

		cb->ifindex = extaddr->smctp_ifindex;
		/* smctp_halen is checked above */
		cb->halen = extaddr->smctp_halen;
		memcpy(cb->haddr, extaddr->smctp_haddr, cb->halen);
	}

	trace_mctp_tx_packet(skb);
	rc = mctp_local_output(sk, rt, skb, addr->smctp_addr.s_addr,
			       addr->smctp_tag);

	return rc ? : len;

err_free:
	kfree_skb(skb);
	return rc;
}

static int mctp_recvmsg(struct socket *sock, struct msghdr *msg, size_t len,
		int flags)
{
	DECLARE_SOCKADDR(struct sockaddr_mctp *, addr, msg->msg_name);
	struct sock *sk = sock->sk;
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct sk_buff *skb;
	size_t msglen;
	u8 type;
	int rc;

	/* Handle error queue read */
	if (flags & MSG_ERRQUEUE)
		return sock_recv_errqueue(sk, msg, len, SOL_MCTP, MCTP_RECVERR);

	if (flags & ~(MSG_DONTWAIT | MSG_TRUNC | MSG_PEEK))
		return -EOPNOTSUPP;

	skb = skb_recv_datagram(sk, flags, &rc);
	if (!skb)
		return rc;

	if (!skb->len) {
		rc = 0;
		goto out_free;
	}

	/* extract message type, remove from data */
	type = *((u8 *)skb->data);
	msglen = skb->len - 1;

	if (len < msglen)
		msg->msg_flags |= MSG_TRUNC;
	else
		len = msglen;

	rc = skb_copy_datagram_msg(skb, 1, msg, len);
	if (rc < 0)
		goto out_free;

	sock_recv_cmsgs(msg, sk, skb);

	if (addr) {
		struct mctp_skb_cb *cb = mctp_cb(skb);
		/* TODO: expand mctp_skb_cb for header fields? */
		struct mctp_hdr *hdr = mctp_hdr(skb);

		addr = msg->msg_name;
		addr->smctp_family = AF_MCTP;
		addr->__smctp_pad0 = 0;
		addr->smctp_network = cb->net;
		addr->smctp_addr.s_addr = hdr->src;
		addr->smctp_type = type;
		addr->smctp_tag = hdr->flags_seq_tag &
					(MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO);
		addr->__smctp_pad1 = 0;
		msg->msg_namelen = sizeof(*addr);

		if (msk->addr_ext) {
			DECLARE_SOCKADDR(struct sockaddr_mctp_ext *, ae,
					 msg->msg_name);
			msg->msg_namelen = sizeof(*ae);
			ae->smctp_ifindex = cb->ifindex;
			ae->smctp_halen = cb->halen;
			memset(ae->__smctp_pad0, 0x0, sizeof(ae->__smctp_pad0));
			memset(ae->smctp_haddr, 0x0, sizeof(ae->smctp_haddr));
			memcpy(ae->smctp_haddr, cb->haddr, cb->halen);
		}
	}

	rc = len;

	if (flags & MSG_TRUNC)
		rc = msglen;

out_free:
	skb_free_datagram(sk, skb);
	return rc;
}

/* Work function - called by kernel worker thread to report errors
 * Context: Process context, NO locks held
 * 
 * This defers error reporting from timer/softirq context to avoid deadlock.
 * The timer path (mctp_sk_expire_keys) holds keys_lock, which would cause
 * deadlock if we call sk_error_report() directly (wake-up callback might
 * need keys_lock). By deferring to workqueue, we ensure error reporting
 * happens after keys_lock is released.
 */
static void mctp_error_report_work_fn(struct work_struct *work)
{
	struct mctp_sock *msk = container_of(work, struct mctp_sock,
					     error_report_work);
	struct mctp_pending_error *perr, *tmp;
	struct list_head local_list;
	
	/* Move all pending errors to local list to minimize lock time */
	INIT_LIST_HEAD(&local_list);
	
	spin_lock_bh(&msk->error_queue_lock);
	list_splice_init(&msk->pending_errors, &local_list);
	spin_unlock_bh(&msk->error_queue_lock);
	
	/* Process all errors without holding locks - safe! */
	list_for_each_entry_safe(perr, tmp, &local_list, list) {
		/* Report error to application - safe, no MCTP locks held.
		 *
		 * For RX reassembly timeout, we use the socket saved in perr->sk
		 * (from the RX key at timeout). This avoids redundant socket lookup
		 * and ensures we report to the correct socket even if RX key is gone.
		 *
		 * For ETIMEDOUT, we have orig_payload saved in perr, so we build
		 * the error structure directly instead of calling mctp_queue_error.
		 */
		if (perr->error_code == ETIMEDOUT && perr->sk && perr->orig_payload_len > 0) {
			/* RX timeout with saved request payload - build error directly */
			struct mctp_error *mctp_err;
			struct sk_buff *err_skb;
			struct mctp_hdr *mh;
			
			if (!perr->skb || perr->skb->len < sizeof(struct mctp_hdr))
				goto cleanup;
			
			err_skb = alloc_skb(sizeof(*mctp_err), GFP_KERNEL);
			if (!err_skb)
				goto cleanup;
			
			mctp_err = (struct mctp_error *)skb_put(err_skb, sizeof(*mctp_err));
			memset(mctp_err, 0, sizeof(*mctp_err));
			
			/* Fill error information */
			mctp_err->error_code = perr->error_code;
			mctp_err->direction = perr->direction;
			mctp_err->binding = perr->binding;
			mctp_err->timestamp_ns = ktime_get_ns();
			
			/* Fill addressing from first fragment SKB */
			mh = mctp_hdr(perr->skb);
			mctp_err->src_eid = mh->src;
			mctp_err->dest_eid = mh->dest;
			mctp_err->tag = mh->flags_seq_tag & MCTP_HDR_TAG_MASK;
			
			/* Use saved REQUEST payload from perr */
			mctp_err->msg_type = perr->orig_msg_type;
			mctp_err->payload_len = min_t(u16, perr->orig_payload_len,
						      MCTP_ERROR_PAYLOAD_SIZE);
			memcpy(mctp_err->payload, perr->orig_payload, mctp_err->payload_len);
			
			/* Queue error to socket */
			if (sock_queue_err_skb(perr->sk, err_skb) == 0) {
				sk_error_report(perr->sk);
				pr_debug("MCTP timeout: Error queued (REQUEST payload, len=%u)\n",
					 mctp_err->payload_len);
			} else {
				kfree_skb(err_skb);
				pr_debug("MCTP timeout: Failed to queue error to socket\n");
			}
cleanup:
			sock_put(perr->sk);
		} else if (perr->sk) {
			/* Other error types - use mctp_queue_error */
			mctp_queue_error(perr->sk, perr->skb, perr->error_code,
					 perr->dev, perr->direction, perr->binding, NULL);
			sock_put(perr->sk);
		} else {
			/* Fallback: Look up socket from SKB (future error types) */
			mctp_queue_error(&msk->sk, perr->skb, perr->error_code,
					 perr->dev, perr->direction, perr->binding, NULL);
		}
		
		/* Cleanup */
		list_del(&perr->list);
		kfree_skb(perr->skb);
		kfree(perr);
	}
}

/* We're done with the key; invalidate, stop reassembly, and remove from lists.
 */
static void __mctp_key_remove(struct mctp_sk_key *key, struct net *net,
			      unsigned long flags, unsigned long reason)
__releases(&key->lock)
__must_hold(&net->mctp.keys_lock)
{
	struct sk_buff *skb;

	trace_mctp_key_release(key, reason);
	skb = key->reasm_head;
	key->reasm_head = NULL;
	key->reasm_dead = true;
	key->valid = false;

	/* Reassembly timeout - queue error for deferred reporting */
	if (reason == MCTP_TRACE_KEY_TIMEOUT && skb && key->dev && key->dev->dev &&
	    key->sk) {
		struct mctp_sock *msk = container_of(key->sk, struct mctp_sock, sk);
		
		netdev_warn(key->dev->dev,
			   "MCTP RX: Reassembly timeout - partial message discarded "
			   "(src=%u, dest=%u, tag=%u, fragments incomplete)\n",
			   key->peer_addr, key->local_addr, key->tag);
		
		/* Queue error for deferred reporting via workqueue if enabled.
		 * Use ETIMEDOUT for reassembly timeout - semantically clearer than EPROTO
		 * (timeout is caused by missing fragments that never arrive).
		 *
		 * CRITICAL: Only report if this key originated from TX (has orig_payload).
		 * This ensures we only report errors for responses to OUR requests,
		 * not for unsolicited requests from device.
		 *
		 * For TX-originated keys:
		 * - key->sk = application that sent original request
		 * - key->reasm_head (skb) = first fragment of response (used for addressing)
		 * - key->orig_payload = original REQUEST payload (from TX phase)
		 *
		 * We pass socket + first fragment SKB (for addressing) to workqueue.
		 * Workqueue will extract REQUEST payload from key->orig_payload,
		 * allowing applications to identify which transaction timed out.
		 */
		if (msk->enable_errqueue && key->orig_payload_len > 0) {
			struct mctp_pending_error *perr;
			
			perr = kmalloc(sizeof(*perr), GFP_ATOMIC);
			if (perr) {
				/* Clone first fragment SKB for later use by workqueue */
				perr->skb = skb_clone(skb, GFP_ATOMIC);
				if (perr->skb) {
					/* Save socket with refcount - ensures it stays alive
					 * until workqueue processes the error.
					 * This is the application that sent the original request.
					 */
					perr->sk = key->sk;
					sock_hold(perr->sk);
					
					perr->error_code = ETIMEDOUT;
					perr->dev = key->dev->dev;
					perr->direction = MCTP_DIR_RX;
					perr->binding = mctp_get_binding_type(key->dev ? key->dev->dev : NULL);
					
				/* Copy original request payload from key for error reporting */
				perr->orig_msg_type = key->orig_msg_type;
				perr->orig_payload_len = key->orig_payload_len;
				memcpy(perr->orig_payload, key->orig_payload,
				       min_t(size_t, key->orig_payload_len, sizeof(perr->orig_payload)));
				
				/* Add to pending list and schedule work
				 * Work will run later in process context with no locks held
				 */
				spin_lock_bh(&msk->error_queue_lock);
				list_add_tail(&perr->list, &msk->pending_errors);
				spin_unlock_bh(&msk->error_queue_lock);
					
					pr_debug("MCTP: RX timeout queued for deferred error reporting (key has TX origin, orig_payload_len=%u)\n",
						key->orig_payload_len);
					
					/* Schedule work - returns immediately */
					schedule_work(&msk->error_report_work);
				} else {
					kfree(perr);
				}
			}
		} else if (msk->enable_errqueue && key->orig_payload_len == 0) {
			/* RX-only key (unsolicited request from device) - don't report */
			pr_debug("MCTP: RX timeout NOT reported - no TX origin (unsolicited request, orig_payload_len=0)\n");
		}
	}

	mctp_dev_release_key(key->dev, key);
	spin_unlock_irqrestore(&key->lock, flags);

	if (!hlist_unhashed(&key->hlist)) {
		hlist_del_init(&key->hlist);
		hlist_del_init(&key->sklist);
		/* unref for the lists */
		mctp_key_unref(key);
	}

	kfree_skb(skb);
}

static int mctp_setsockopt(struct socket *sock, int level, int optname,
			   sockptr_t optval, unsigned int optlen)
{
	struct mctp_sock *msk = container_of(sock->sk, struct mctp_sock, sk);
	int val;

	if (level != SOL_MCTP)
		return -EINVAL;

	if (optname == MCTP_OPT_ADDR_EXT) {
		if (optlen != sizeof(int))
			return -EINVAL;
		if (copy_from_sockptr(&val, optval, sizeof(int)))
			return -EFAULT;
		msk->addr_ext = val;
		return 0;
	}

	if (optname == MCTP_OPT_ENABLE_ERRQUEUE) {
		if (optlen != sizeof(int))
			return -EINVAL;
		if (copy_from_sockptr(&val, optval, sizeof(int)))
			return -EFAULT;
		
		msk->enable_errqueue = !!val;
		
		return 0;
	}

	return -ENOPROTOOPT;
}

static int mctp_getsockopt(struct socket *sock, int level, int optname,
			   char __user *optval, int __user *optlen)
{
	struct mctp_sock *msk = container_of(sock->sk, struct mctp_sock, sk);
	int len, val;

	if (level != SOL_MCTP)
		return -EINVAL;

	if (get_user(len, optlen))
		return -EFAULT;

	if (optname == MCTP_OPT_ADDR_EXT) {
		if (len != sizeof(int))
			return -EINVAL;
		val = !!msk->addr_ext;
		if (copy_to_user(optval, &val, len))
			return -EFAULT;
		return 0;
	}

	return -ENOPROTOOPT;
}

/* helpers for reading/writing the tag ioc, handling compatibility across the
 * two versions, and some basic API error checking
 */
static int mctp_ioctl_tag_copy_from_user(unsigned long arg,
					 struct mctp_ioc_tag_ctl2 *ctl,
					 bool tagv2)
{
	struct mctp_ioc_tag_ctl ctl_compat;
	unsigned long size;
	void *ptr;
	int rc;

	if (tagv2) {
		size = sizeof(*ctl);
		ptr = ctl;
	} else {
		size = sizeof(ctl_compat);
		ptr = &ctl_compat;
	}

	rc = copy_from_user(ptr, (void __user *)arg, size);
	if (rc)
		return -EFAULT;

	if (!tagv2) {
		/* compat, using defaults for new fields */
		ctl->net = MCTP_INITIAL_DEFAULT_NET;
		ctl->peer_addr = ctl_compat.peer_addr;
		ctl->local_addr = MCTP_ADDR_ANY;
		ctl->flags = ctl_compat.flags;
		ctl->tag = ctl_compat.tag;
	}

	if (ctl->flags)
		return -EINVAL;

	if (ctl->local_addr != MCTP_ADDR_ANY &&
	    ctl->local_addr != MCTP_ADDR_NULL)
		return -EINVAL;

	return 0;
}

static int mctp_ioctl_tag_copy_to_user(unsigned long arg,
				       struct mctp_ioc_tag_ctl2 *ctl,
				       bool tagv2)
{
	struct mctp_ioc_tag_ctl ctl_compat;
	unsigned long size;
	void *ptr;
	int rc;

	if (tagv2) {
		ptr = ctl;
		size = sizeof(*ctl);
	} else {
		ctl_compat.peer_addr = ctl->peer_addr;
		ctl_compat.tag = ctl->tag;
		ctl_compat.flags = ctl->flags;

		ptr = &ctl_compat;
		size = sizeof(ctl_compat);
	}

	rc = copy_to_user((void __user *)arg, ptr, size);
	if (rc)
		return -EFAULT;

	return 0;
}

static int mctp_ioctl_alloctag(struct mctp_sock *msk, bool tagv2,
			       unsigned long arg)
{
	struct net *net = sock_net(&msk->sk);
	struct mctp_sk_key *key = NULL;
	struct mctp_ioc_tag_ctl2 ctl;
	unsigned long flags;
	u8 tag;
	int rc;

	rc = mctp_ioctl_tag_copy_from_user(arg, &ctl, tagv2);
	if (rc)
		return rc;

	if (ctl.tag)
		return -EINVAL;

	key = mctp_alloc_local_tag(msk, ctl.net, MCTP_ADDR_ANY,
				   ctl.peer_addr, true, &tag,
				   MCTP_DEFAULT_LIFETIME);
	if (IS_ERR(key))
		return PTR_ERR(key);

	ctl.tag = tag | MCTP_TAG_OWNER | MCTP_TAG_PREALLOC;
	rc = mctp_ioctl_tag_copy_to_user(arg, &ctl, tagv2);
	if (rc) {
		unsigned long fl2;
		/* Unwind our key allocation: the keys list lock needs to be
		 * taken before the individual key locks, and we need a valid
		 * flags value (fl2) to pass to __mctp_key_remove, hence the
		 * second spin_lock_irqsave() rather than a plain spin_lock().
		 */
		spin_lock_irqsave(&net->mctp.keys_lock, flags);
		spin_lock_irqsave(&key->lock, fl2);
		__mctp_key_remove(key, net, fl2, MCTP_TRACE_KEY_DROPPED);
		mctp_key_unref(key);
		spin_unlock_irqrestore(&net->mctp.keys_lock, flags);
		return rc;
	}

	mctp_key_unref(key);
	return 0;
}

static int mctp_ioctl_droptag(struct mctp_sock *msk, bool tagv2,
			      unsigned long arg)
{
	struct net *net = sock_net(&msk->sk);
	struct mctp_ioc_tag_ctl2 ctl;
	unsigned long flags, fl2;
	struct mctp_sk_key *key;
	struct hlist_node *tmp;
	int rc;
	u8 tag;

	rc = mctp_ioctl_tag_copy_from_user(arg, &ctl, tagv2);
	if (rc)
		return rc;

	/* Must be a local tag, TO set, preallocated */
	if ((ctl.tag & ~MCTP_TAG_MASK) != (MCTP_TAG_OWNER | MCTP_TAG_PREALLOC))
		return -EINVAL;

	tag = ctl.tag & MCTP_TAG_MASK;
	rc = -EINVAL;

	if (ctl.peer_addr == MCTP_ADDR_NULL)
		ctl.peer_addr = MCTP_ADDR_ANY;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry_safe(key, tmp, &msk->keys, sklist) {
		/* we do an irqsave here, even though we know the irq state,
		 * so we have the flags to pass to __mctp_key_remove
		 */
		spin_lock_irqsave(&key->lock, fl2);
		if (key->manual_alloc &&
		    ctl.net == key->net &&
		    ctl.peer_addr == key->peer_addr &&
		    tag == key->tag) {
			__mctp_key_remove(key, net, fl2,
					  MCTP_TRACE_KEY_DROPPED);
			rc = 0;
		} else {
			spin_unlock_irqrestore(&key->lock, fl2);
		}
	}
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

	return rc;
}

static int mctp_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct mctp_sock *msk = container_of(sock->sk, struct mctp_sock, sk);
	bool tagv2 = false;

	switch (cmd) {
	case SIOCMCTPALLOCTAG2:
	case SIOCMCTPALLOCTAG:
		tagv2 = cmd == SIOCMCTPALLOCTAG2;
		return mctp_ioctl_alloctag(msk, tagv2, arg);
	case SIOCMCTPDROPTAG:
	case SIOCMCTPDROPTAG2:
		tagv2 = cmd == SIOCMCTPDROPTAG2;
		return mctp_ioctl_droptag(msk, tagv2, arg);
	}

	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static int mctp_compat_ioctl(struct socket *sock, unsigned int cmd,
			     unsigned long arg)
{
	void __user *argp = compat_ptr(arg);

	switch (cmd) {
	/* These have compatible ptr layouts */
	case SIOCMCTPALLOCTAG:
	case SIOCMCTPDROPTAG:
		return mctp_ioctl(sock, cmd, (unsigned long)argp);
	}

	return -ENOIOCTLCMD;
}
#endif

static __poll_t mctp_poll(struct file *file, struct socket *sock,
			  poll_table *wait)
{
	struct sock *sk = sock->sk;
	__poll_t mask;

	/* Use standard datagram_poll for normal data */
	mask = datagram_poll(file, sock, wait);

	/* Check error queue */
	if (!skb_queue_empty_lockless(&sk->sk_error_queue))
		mask |= EPOLLERR | EPOLLPRI;

	return mask;
}

static const struct proto_ops mctp_dgram_ops = {
	.family		= PF_MCTP,
	.release	= mctp_release,
	.bind		= mctp_bind,
	.connect	= sock_no_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= sock_no_getname,
	.poll		= mctp_poll,
	.ioctl		= mctp_ioctl,
	.gettstamp	= sock_gettstamp,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= mctp_setsockopt,
	.getsockopt	= mctp_getsockopt,
	.sendmsg	= mctp_sendmsg,
	.recvmsg	= mctp_recvmsg,
	.mmap		= sock_no_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= mctp_compat_ioctl,
#endif
};

static void mctp_sk_expire_keys(struct timer_list *timer)
{
	struct mctp_sock *msk = container_of(timer, struct mctp_sock,
					     key_expiry);
	struct net *net = sock_net(&msk->sk);
	unsigned long next_expiry, flags, fl2;
	struct mctp_sk_key *key;
	struct hlist_node *tmp;
	bool next_expiry_valid = false;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);

	hlist_for_each_entry_safe(key, tmp, &msk->keys, sklist) {
		/* don't expire. manual_alloc is immutable, no locking
		 * required.
		 */
		if (key->manual_alloc)
			continue;

		spin_lock_irqsave(&key->lock, fl2);
		if (!time_after_eq(key->expiry, jiffies)) {
			__mctp_key_remove(key, net, fl2,
					  MCTP_TRACE_KEY_TIMEOUT);
			continue;
		}

		if (next_expiry_valid) {
			if (time_before(key->expiry, next_expiry))
				next_expiry = key->expiry;
		} else {
			next_expiry = key->expiry;
			next_expiry_valid = true;
		}
		spin_unlock_irqrestore(&key->lock, fl2);
	}

	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

	if (next_expiry_valid)
		mod_timer(timer, next_expiry);
}

/* Forward declaration for workqueue callback */
static void mctp_error_report_work_fn(struct work_struct *work);

static int mctp_sk_init(struct sock *sk)
{
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);

	INIT_HLIST_HEAD(&msk->keys);
	timer_setup(&msk->key_expiry, mctp_sk_expire_keys, 0);
	
	/* Initialize deferred error reporting workqueue */
	INIT_WORK(&msk->error_report_work, mctp_error_report_work_fn);
	INIT_LIST_HEAD(&msk->pending_errors);
	spin_lock_init(&msk->error_queue_lock);
	
	return 0;
}

static void mctp_sk_close(struct sock *sk, long timeout)
{
	sk_common_release(sk);
}

static int mctp_sk_hash(struct sock *sk)
{
	struct net *net = sock_net(sk);

	/* Bind lookup runs under RCU, remain live during that. */
	sock_set_flag(sk, SOCK_RCU_FREE);

	/* Bind lookup runs under RCU, remain live during that. */
	sock_set_flag(sk, SOCK_RCU_FREE);

	mutex_lock(&net->mctp.bind_lock);
	sk_add_node_rcu(sk, &net->mctp.binds);
	mutex_unlock(&net->mctp.bind_lock);

	return 0;
}

static void mctp_sk_unhash(struct sock *sk)
{
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct net *net = sock_net(sk);
	unsigned long flags, fl2;
	struct mctp_sk_key *key;
	struct hlist_node *tmp;

	/* remove from any type-based binds */
	mutex_lock(&net->mctp.bind_lock);
	sk_del_node_init_rcu(sk);
	mutex_unlock(&net->mctp.bind_lock);

	/* remove tag allocations */
	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry_safe(key, tmp, &msk->keys, sklist) {
		spin_lock_irqsave(&key->lock, fl2);
		__mctp_key_remove(key, net, fl2, MCTP_TRACE_KEY_CLOSED);
	}
	sock_set_flag(sk, SOCK_DEAD);
	spin_unlock_irqrestore(&net->mctp.keys_lock, flags);

	/* Since there are no more tag allocations (we have removed all of the
	 * keys), stop any pending expiry events. the timer cannot be re-queued
	 * as the sk is no longer observable
	 */
	del_timer_sync(&msk->key_expiry);
	
	/* Cancel pending error reporting work and free any pending errors */
	cancel_work_sync(&msk->error_report_work);
	{
	struct mctp_pending_error *perr, *tmp_err;
		
	spin_lock_bh(&msk->error_queue_lock);
	list_for_each_entry_safe(perr, tmp_err, &msk->pending_errors, list) {
		list_del(&perr->list);
			if (perr->sk)
				sock_put(perr->sk);
		kfree_skb(perr->skb);
		kfree(perr);
	}
	spin_unlock_bh(&msk->error_queue_lock);
	}
}

static void mctp_sk_destruct(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);
}

static struct proto mctp_proto = {
	.name		= "MCTP",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct mctp_sock),
	.init		= mctp_sk_init,
	.close		= mctp_sk_close,
	.hash		= mctp_sk_hash,
	.unhash		= mctp_sk_unhash,
};

static int mctp_pf_create(struct net *net, struct socket *sock,
			  int protocol, int kern)
{
	const struct proto_ops *ops;
	struct proto *proto;
	struct sock *sk;
	int rc;

	if (protocol)
		return -EPROTONOSUPPORT;

	/* only datagram sockets are supported */
	if (sock->type != SOCK_DGRAM)
		return -ESOCKTNOSUPPORT;

	proto = &mctp_proto;
	ops = &mctp_dgram_ops;

	sock->state = SS_UNCONNECTED;
	sock->ops = ops;

	sk = sk_alloc(net, PF_MCTP, GFP_KERNEL, proto, kern);
	if (!sk)
		return -ENOMEM;

	sock_init_data(sock, sk);
	sk->sk_destruct = mctp_sk_destruct;

	rc = 0;
	if (sk->sk_prot->init)
		rc = sk->sk_prot->init(sk);

	if (rc)
		goto err_sk_put;

	return 0;

err_sk_put:
	sock_orphan(sk);
	sock_put(sk);
	return rc;
}

static struct net_proto_family mctp_pf = {
	.family = PF_MCTP,
	.create = mctp_pf_create,
	.owner = THIS_MODULE,
};

static __init int mctp_init(void)
{
	int rc;

	/* ensure our uapi tag definitions match the header format */
	BUILD_BUG_ON(MCTP_TAG_OWNER != MCTP_HDR_FLAG_TO);
	BUILD_BUG_ON(MCTP_TAG_MASK != MCTP_HDR_TAG_MASK);

	pr_info("mctp: management component transport protocol core\n");

	rc = sock_register(&mctp_pf);
	if (rc)
		return rc;

	rc = proto_register(&mctp_proto, 0);
	if (rc)
		goto err_unreg_sock;

	rc = mctp_routes_init();
	if (rc)
		goto err_unreg_proto;

	rc = mctp_neigh_init();
	if (rc)
		goto err_unreg_routes;

	rc = mctp_device_init();
	if (rc)
		goto err_unreg_neigh;

	rc = mctp_socket_error_inject_init();
	if (rc)
		pr_warn("MCTP: Socket error injection init failed, continuing without it\n");

	return 0;

err_unreg_neigh:
	mctp_neigh_exit();
err_unreg_routes:
	mctp_routes_exit();
err_unreg_proto:
	proto_unregister(&mctp_proto);
err_unreg_sock:
	sock_unregister(PF_MCTP);

	return rc;
}

static __exit void mctp_exit(void)
{
	mctp_socket_error_inject_cleanup();
	mctp_device_exit();
	mctp_neigh_exit();
	mctp_routes_exit();
	proto_unregister(&mctp_proto);
	sock_unregister(PF_MCTP);
}

subsys_initcall(mctp_init);
module_exit(mctp_exit);

MODULE_DESCRIPTION("MCTP core");
MODULE_AUTHOR("Jeremy Kerr <jk@codeconstruct.com.au>");

MODULE_ALIAS_NETPROTO(PF_MCTP);
