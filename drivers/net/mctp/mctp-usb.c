// SPDX-License-Identifier: GPL-2.0
/*
 * mctp-usb.c - MCTP-over-USB (DMTF DSP0283) transport binding driver.
 *
 * DSP0283 is available at:
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0283_1.0.1.pdf
 *
 * Copyright (C) 2024-2025 Code Construct Pty Ltd
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/usb/mctp-usb.h>
#include <linux/ethtool.h>

#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <net/pkt_sched.h>

#include <uapi/linux/if_arp.h>

#include "mctp-usb-error-inject.h"
#include "mctp-usb-internal.h"
#include <net/mctp-stats.h>

#include <trace/events/mctp.h>

static const unsigned int n_tx_queue = 8;

struct mctp_usb_batch_ctx {
	struct net_device *netdev;
	struct sk_buff_head skbs;
	unsigned int num_packets;
};

/* TX watchdog timeout - abort stuck URBs after this time
 */
#define MCTP_USB_TX_TIMEOUT (6 * HZ)

/**
 * mctp_usb_handle_tx_urb_status - Handle TX URB completion status
 * @mctp_usb: MCTP USB device
 * @netdev: Network device
 * @status: URB completion status code
 * @num_packets: Number of packets in the batch
 * @actual_length: Actual transfer length (for success case tracing)
 *
 * Processes TX URB completion status and updates per-EID statistics
 * accordingly; the netdev packet and drop counters are accounted by the
 * caller. All async URB errors are tracked under MCTP_EID_UNKNOWN since batch
 * completion cannot determine individual packet EIDs.
 */
static void mctp_usb_handle_tx_urb_status(struct mctp_usb *mctp_usb,
					   struct net_device *netdev,
					   int status,
					   unsigned int num_packets,
					   unsigned int actual_length)
{
	switch (status) {
	/* Valid USB error codes only */
	case -EBUSY:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_ebusy += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENODEV:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enodev += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOENT:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enoent += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENXIO:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enxio += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EINVAL:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_einval += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EXDEV:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_exdev += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EFBIG:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_efbig += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EPIPE:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_epipe += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EMSGSIZE:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_emsgsize += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOSPC:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enospc += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ESHUTDOWN:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_eshutdown += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EPERM:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_eperm += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EHOSTUNREACH:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_ehostunreach += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOEXEC:
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enoexec += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ECONNRESET:
		/* URB was unlinked rather than failing on the wire. This happens
		 * in two cases: the TX watchdog (mctp_usb_tx_timeout) aborting a
		 * stuck transfer while the link is up, or mctp_usb_stop() tearing
		 * down on interface down (->rx_stopped set). Only the former is a
		 * transmit timeout; either way it is deliberate, so don't log it
		 * as an "unexpected" status. Account the two cases separately so
		 * neither is conflated with a genuine -ENOENT.
		 */
		if (READ_ONCE(mctp_usb->rx_stopped)) {
			mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_unlinked += num_packets;
			trace_mctp_transport_error("usb", netdev, "tx_urb_unlinked", status);
		} else {
			netdev->stats.tx_errors += num_packets;
			mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_timeout += num_packets;
			trace_mctp_transport_error("usb", netdev, "tx_urb_timeout", status);
		}
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case 0:
		trace_mctp_transport_tx("usb", netdev, 0, actual_length);
		break;
	default:
		if (net_ratelimit())
			netdev_warn(netdev, "unexpected tx urb status: %d\n",
				    status);
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_urb_error += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		trace_mctp_transport_error("usb", netdev, "tx_urb_unexpected", status);
	}
}

/**
 * mctp_usb_handle_tx_sync_error - Handle synchronous TX error statistics
 * @mctp_usb: MCTP USB device
 * @dest_eid: Destination EID from MCTP header
 * @error_code: Error code from URB submission
 *
 * Maps synchronous TX errors (URB submission failures) to per-EID statistics.
 * Unlike async URB completion errors, these can be attributed to a specific
 * destination EID since the packet header is still available.
 * Only handles valid USB error codes per USB subsystem specification.
 */
static void mctp_usb_handle_tx_sync_error(struct mctp_usb *mctp_usb,
					   u8 dest_eid,
					   int error_code)
{
	switch (error_code) {
	/* Valid USB error codes only */
	case -ENOMEM:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_no_memory);
		break;
	case -EBUSY:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_ebusy);
		break;
	case -ENODEV:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_enodev);
		break;
	case -ENOENT:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_enoent);
		break;
	case -ENXIO:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_enxio);
		break;
	case -EINVAL:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_einval);
		break;
	case -EXDEV:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_exdev);
		break;
	case -EFBIG:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_efbig);
		break;
	case -EPIPE:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_epipe);
		break;
	case -EMSGSIZE:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_emsgsize);
		break;
	case -ENOSPC:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_enospc);
		break;
	case -ESHUTDOWN:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_eshutdown);
		break;
	case -EPERM:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_eperm);
		break;
	case -EHOSTUNREACH:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_ehostunreach);
		break;
	case -ENOEXEC:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_enoexec);
		break;
	default:
		MCTP_STAT_INC(mctp_usb, dest_eid, tx_drop_urb_error);
		break;
	}
}

/**
 * mctp_usb_report_tx_error - Report a TX failure to the socket error queue
 * @netdev: Network device
 * @skb: Failed TX skb, USB header still attached; may hold a batch of packets
 * @err: Positive error code to report
 *
 * Derives a single-packet skb from the first packet of @skb, strips the USB
 * header to expose the MCTP header, then looks up the owning socket and queues
 * the error there, so a subscriber polling POLLERR on its AF_MCTP socket sees
 * driver-level TX failures. @skb itself is left intact.
 */
static void mctp_usb_report_tx_error(struct net_device *netdev,
				     struct sk_buff *skb, int err)
{
	struct mctp_usb_hdr *hdr;
	struct sk_buff *pkt_skb;
	struct mctp_sk_key *key = NULL;
	struct sock *sk;
	unsigned int pkt_total_len;

	if (!skb || skb->len < sizeof(struct mctp_usb_hdr))
		return;

	hdr = (struct mctp_usb_hdr *)skb->data;
	if (be16_to_cpu(hdr->id) != MCTP_USB_DMTF_ID)
		return;

	pkt_total_len = hdr->len;
	if (pkt_total_len > skb->len ||
	    pkt_total_len <= sizeof(struct mctp_usb_hdr))
		return;

	/* Copy the first packet so the original (batched) skb is untouched */
	pkt_skb = alloc_skb(pkt_total_len, GFP_ATOMIC);
	if (!pkt_skb)
		return;

	skb_put_data(pkt_skb, skb->data, pkt_total_len);
	pkt_skb->dev = netdev;
	if (skb->sk)
		skb_set_owner_w(pkt_skb, skb->sk);

	/* Strip the USB header to expose the MCTP header for the socket lookup */
	if (skb_pull(pkt_skb, sizeof(struct mctp_usb_hdr))) {
		skb_reset_network_header(pkt_skb);

		sk = mctp_lookup_sock_for_error(pkt_skb, netdev, NULL, &key);
		if (sk) {
			mctp_queue_error(sk, pkt_skb, err, netdev,
					 MCTP_DIR_TX, MCTP_PHYS_BINDING_USB, key);
			sock_put(sk);
		}
		if (key)
			mctp_key_unref(key);
	}
	kfree_skb(pkt_skb);
}

static void mctp_usb_out_complete(struct urb *urb)
{
	struct mctp_usblib_tx_ctx *tx_ctx = urb->context;
	struct mctp_usb *mctp_usb = mctp_usblib_tx_ctx_priv(tx_ctx);
	struct net_device *netdev = mctp_usb->netdev;
	int status;

	/* ERROR INJECTION POINT: TX URB completion (asynchronous error)
	 * Note: Injection happens at URB level (may affect multiple batched
	 * packets). Pass urb so async path can apply EID filter from first
	 * packet.
	 */
	status = mctp_usb_error_inject_tx_async(mctp_usb, urb);

	/* Update per-EID statistics; usblib accounts the netdev stats, and
	 * reports failed packets to the socket error queue via ->tx_error.
	 */
	mctp_usb_handle_tx_urb_status(mctp_usb, netdev, status,
				      mctp_usblib_tx_ctx_npkts(tx_ctx),
				      urb->actual_length);

	mctp_usblib_tx_send_complete_status(tx_ctx, netdev, status);

	usb_free_urb(urb);

	if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
		netif_wake_queue(netdev);
}

static int mctp_usb_tx_send(struct mctp_usblib_tx_ctx *tx_ctx,
			    void *data, size_t len)
{
	struct mctp_usb *mctp_usb = mctp_usblib_tx_ctx_priv(tx_ctx);
	unsigned int npkts = mctp_usblib_tx_ctx_npkts(tx_ctx);
	struct net_device *netdev = mctp_usb->netdev;
	u16 eid = MCTP_EID_UNKNOWN;
	struct urb *urb;
	int rc;

	/* ERROR INJECTION POINT: TX URB submission (synchronous error) */
	rc = mctp_usb_error_inject_tx_sync(mctp_usb, data, len);
	if (rc)
		goto err;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		trace_mctp_transport_error("usb", netdev, "tx_urb_alloc_failed", 0);
		rc = -ENOMEM;
		goto err;
	}

	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  data, len, mctp_usb_out_complete, tx_ctx);

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	if (atomic_inc_return(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		netdev_dbg(netdev, "TX urb submit failed, %d\n", rc);
		trace_mctp_transport_error("usb", netdev, "tx_urb_submit_failed", rc);
		usb_unanchor_urb(urb);
		usb_free_urb(urb);
		if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
			netif_wake_queue(netdev);
		goto err;
	}

	return 0;

err:
	/* Track per-EID sync TX error. A single-packet transfer can be
	 * attributed to its destination EID; multi-packet transfers are
	 * tracked as UNKNOWN. usblib accounts the netdev drop, and reports
	 * the failed packets to the socket error queue via ->tx_error.
	 */
	if (npkts == 1 &&
	    len >= sizeof(struct mctp_usb_hdr) + sizeof(struct mctp_hdr)) {
		const struct mctp_hdr *mh = data + sizeof(struct mctp_usb_hdr);

		eid = mh->dest;
	}
	mctp_usb_handle_tx_sync_error(mctp_usb, eid, rc);
	trace_mctp_transport_error("usb", netdev,
				   npkts == 1 ? "tx_single_drop" : "tx_batch_drop",
				   rc);
	return rc;
}

/* usblib TX hook: called for each packet of a failed transfer */
static void mctp_usb_tx_error(struct mctp_usblib_tx_ctx *tx_ctx,
			      struct sk_buff *skb, int err)
{
	struct mctp_usb *mctp_usb = mctp_usblib_tx_ctx_priv(tx_ctx);

	/* Surface the TX failure on the socket error queue before the skb is
	 * freed.
	 */
	mctp_usb_report_tx_error(mctp_usb->netdev, skb, -err);
}

static const struct mctp_usblib_tx_ops tx_ops = {
	.send = mctp_usb_tx_send,
	.tx_error = mctp_usb_tx_error,
};

/* Completion for pre-framed batches from the route-level batching path */
static void mctp_usb_batch_out_complete(struct urb *urb)
{
	struct mctp_usb_batch_ctx *ctx = urb->context;
	struct net_device *netdev = ctx->netdev;
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	struct sk_buff *skb;
	int status;

	if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
		netif_wake_queue(netdev);

	status = urb->status;

	/* ERROR INJECTION POINT: TX URB completion (asynchronous error) */
	status = mctp_usb_error_inject_tx_async(mctp_usb, urb);

	/* Batched URBs increment counters by ctx->num_packets. */
	if (status)
		netdev->stats.tx_dropped += ctx->num_packets;
	else
		netdev->stats.tx_packets += ctx->num_packets;
	mctp_usb_handle_tx_urb_status(mctp_usb, netdev, status,
				      ctx->num_packets, urb->actual_length);

	while ((skb = skb_dequeue(&ctx->skbs)) != NULL) {
		if (status == 0) {
			consume_skb(skb);
		} else {
			/* Surface the async TX failure on the socket error
			 * queue before the skb is freed.
			 */
			mctp_usb_report_tx_error(netdev, skb, -status);
			kfree_skb(skb);
		}
	}

	kfree(ctx);
	usb_free_urb(urb);
}

static void mctp_usb_fill_batch_hdr(void *hdr, unsigned int pkt_len)
{
	struct mctp_usb_hdr *usb_hdr = (struct mctp_usb_hdr *)hdr;

	usb_hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	usb_hdr->rsvd = 0;
	usb_hdr->len = pkt_len;
}

static const struct mctp_netdev_ops mctp_usb_mctp_ops = {
	.fill_batch_hdr = mctp_usb_fill_batch_hdr,
};

static netdev_tx_t mctp_usb_send_batch(struct mctp_usb *mctp_usb,
				       struct sk_buff *skb)
{
	struct net_device *netdev = mctp_usb->netdev;
	struct mctp_usb_batch_ctx *ctx;
	struct urb *urb;
	/* Default to -ENOMEM so alloc-failure gotos attribute correctly */
	int rc = -ENOMEM;

	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	skb->protocol = htons(ETH_P_MCTP);

	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		goto err_drop;

	skb_queue_head_init(&ctx->skbs);
	ctx->netdev = netdev;
	ctx->num_packets = 1;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto err_free_ctx;

	skb_queue_tail(&ctx->skbs, skb);

	/* ERROR INJECTION POINT: TX URB submission (synchronous error) */
	rc = mctp_usb_error_inject_tx_sync(mctp_usb, skb->data, skb->len);
	if (rc)
		goto err_free_urb;

	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  skb->data, skb->len, mctp_usb_batch_out_complete,
			  ctx);

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	atomic_inc(&mctp_usb->tx_qlen);
	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		usb_unanchor_urb(urb);
		if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
			netif_wake_queue(netdev);
		trace_mctp_transport_error("usb", netdev, "tx_urb_submit_failed", rc);
		goto err_free_urb;
	}

	return NETDEV_TX_OK;

err_free_urb:
	usb_free_urb(urb);
err_free_ctx:
	skb_dequeue(&ctx->skbs);
	kfree(ctx);
err_drop:
	/* Batch TX errors are tracked as UNKNOWN (individual EIDs in the
	 * batch cannot be determined at this point).
	 */
	netdev->stats.tx_dropped++;
	mctp_usb_handle_tx_sync_error(mctp_usb, MCTP_EID_UNKNOWN, rc);
	trace_mctp_transport_error("usb", netdev, "tx_batch_drop", rc);
	if (rc != 0)
		mctp_usb_report_tx_error(netdev, skb, -rc);
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static netdev_tx_t mctp_usb_start_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);
	unsigned int pkt_len;
	int rc;

	/* Batched SKBs from route.c have a marker protocol */
	if (skb->protocol == htons(ETH_P_MCTP | 0x8000))
		return mctp_usb_send_batch(mctp_usb, skb);

	pkt_len = skb->len + sizeof(struct mctp_usb_hdr);

	if (pkt_len > MCTP_USB_1_0_PKTLEN_MAX) {
		/* Extract EID for tracking before dropping */
		struct mctp_hdr *mh = mctp_hdr(skb);

		MCTP_STAT_INC(mctp_usb, mh->dest, tx_drop_emsgsize);
		trace_mctp_transport_error("usb", dev, "tx_len_exceeds_max", pkt_len);
		goto err_drop;
	}

	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	/* usblib accounts drops, and frees @skb on error */
	rc = mctp_usblib_tx_push(dev, &mctp_usb->tx, skb, netdev_xmit_more());
	if (rc)
		trace_mctp_transport_error("usb", dev, "tx_dropped_start_xmit", rc);

	return NETDEV_TX_OK;

err_drop:
	dev_dstats_tx_dropped(dev);
	trace_mctp_transport_error("usb", dev, "tx_dropped_start_xmit", 0);
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void mctp_usb_in_complete(struct urb *urb);

/* If we fail to queue an in urb atomically (either due to skb allocation or
 * urb submission), we will schedule a rx queue in nonatomic context
 * after a delay, specified in jiffies
 */
static const unsigned long RX_RETRY_DELAY = HZ / 4;

static int mctp_usb_rx_queue(struct mctp_usb *mctp_usb, gfp_t gfp)
{
	unsigned long flags;
	size_t len;
	void *buf;
	int rc;

	rc = mctp_usblib_rx_prepare(mctp_usb->netdev, &mctp_usb->rx,
				    &buf, &len, gfp);
	if (rc) {
		trace_mctp_transport_error("usb", mctp_usb->netdev, "rx_alloc_skb_failed", 0);
		goto err_retry;
	}

	usb_fill_bulk_urb(mctp_usb->rx_urb, mctp_usb->usbdev,
			  usb_rcvbulkpipe(mctp_usb->usbdev, mctp_usb->ep_in),
			  buf, len, mctp_usb_in_complete, mctp_usb);

	rc = usb_submit_urb(mctp_usb->rx_urb, gfp);
	if (rc) {
		netdev_dbg(mctp_usb->netdev, "rx urb submit failure: %d\n", rc);
		trace_mctp_transport_error("usb", mctp_usb->netdev, "rx_urb_submit_failed", rc);
		mctp_usblib_rx_cancel(&mctp_usb->rx);
		if (rc == -ENOMEM)
			goto err_retry;
	}

	return rc;

err_retry:
	spin_lock_irqsave(&mctp_usb->rx_lock, flags);
	if (!mctp_usb->rx_stopped)
		schedule_delayed_work(&mctp_usb->rx_retry_work, RX_RETRY_DELAY);
	spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);
	return 0;
}

static const unsigned int rx_err_max = 10;

/* Returns -1 if we have hit excessive errors, zero otherwise. */
static int mctp_usb_in_urb_err(struct mctp_usb *mctp_usb, int status,
			       bool stalled)
{
	mctp_usblib_rx_cancel(&mctp_usb->rx);

	if (!mctp_usb->in_err_count++)
		mctp_usb->in_err_orig = status;

	if (mctp_usb->in_err_count >= rx_err_max) {
		netdev_err(mctp_usb->netdev,
			   "excessive errors from%s IN EP, first: %d\n",
			   stalled ? " (stalled)" : "",
			   mctp_usb->in_err_orig);
		return -1;
	}

	return 0;
}

/* Account a failed IN urb; @shutdown for expected shutdown/reset statuses */
static void mctp_usb_in_urb_err_stats(struct mctp_usb *mctp_usb, int status,
				      bool shutdown)
{
	struct net_device *netdev = mctp_usb->netdev;

	if (mctp_usb->error_inject.common.enable_rx)
		netdev_info(netdev,
			    "RX packet DROPPED (error %d) - error injection is ACTIVE\n",
			    status);
	else if (shutdown)
		netdev_dbg(netdev,
			   "RX packet DROPPED (error %d) - expected shutdown/reset\n",
			   status);

	if (!shutdown)
		netdev->stats.rx_errors++;
	netdev->stats.rx_dropped++;
	MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_urb_error);
	trace_mctp_transport_error("usb", netdev,
				   shutdown ? "rx_urb_error_shutdown" :
					      "rx_urb_error_unexpected",
				   status);
}

static void mctp_usb_in_complete(struct urb *urb)
{
	struct mctp_usb *mctp_usb = urb->context;
	struct net_device *netdev = mctp_usb->netdev;
	unsigned long flags;
	int rc, status;

	status = urb->status;

	/* ERROR INJECTION POINT: RX URB completion error */
	status = mctp_usb_error_inject_rx(mctp_usb, status);

	switch (status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* device shutdown, don't resubmit */
		mctp_usb_in_urb_err_stats(mctp_usb, status, true);
		mctp_usblib_rx_cancel(&mctp_usb->rx);
		return;

	case -EPIPE:
		/* endpoint stall: clear halt, which will cause a resubmit */
		mctp_usb_in_urb_err_stats(mctp_usb, status, false);
		rc = mctp_usb_in_urb_err(mctp_usb, status, true);
		if (rc)
			return;

		mctp_usb->clear_halt = true;
		spin_lock_irqsave(&mctp_usb->rx_lock, flags);
		if (!mctp_usb->rx_stopped)
			schedule_delayed_work(&mctp_usb->rx_retry_work,
					      RX_RETRY_DELAY);
		spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);
		return;

	default:
		netdev_dbg(netdev, "unexpected rx urb status: %d\n", status);
		fallthrough;
	case -ETIME:
	case -EPROTO:
	case -EILSEQ:
	case -EOVERFLOW:
		/* possibly transient; record first failure, resubmit */
		mctp_usb_in_urb_err_stats(mctp_usb, status, false);
		rc = mctp_usb_in_urb_err(mctp_usb, status, false);
		if (rc)
			return;
		break;

	case 0:
		mctp_usblib_rx_complete(netdev, &mctp_usb->rx, urb->actual_length);
		mctp_usb->in_err_count = 0;
		break;
	}

	mctp_usb_rx_queue(mctp_usb, GFP_ATOMIC);
}

/* usblib RX hook: called for each received packet before netif_rx() */
static int mctp_usb_rx_packet(struct net_device *netdev, struct sk_buff *skb)
{
	struct mctp_usb *mctp_usb = netdev_priv(netdev);

	/* ERROR INJECTION POINT: Fragment drop/corruption
	 * At this point:
	 * - USB header has been removed
	 * - skb->data points to MCTP header
	 * - Before packet sent to network stack
	 * This is the ideal location to inject fragment errors.
	 */
	if (mctp_usb->error_inject.common.enable_fragment_drop ||
	    mctp_usb->error_inject.common.enable_seq_corrupt ||
	    mctp_usb->error_inject.common.enable_som_clear ||
	    mctp_usb->error_inject.common.enable_rx) {
		int inject_action =
			mctp_usb_error_inject_fragment(mctp_usb, skb);

		if (inject_action == 1) {
			struct mctp_hdr *mh = mctp_hdr(skb);
			u8 src_eid = mh->src;

			/* Drop this fragment; usblib frees the skb */
			netdev->stats.rx_dropped++;
			MCTP_STAT_INC(mctp_usb, src_eid, rx_drop_fragment_error);
			trace_mctp_transport_error("usb", netdev, "rx_drop_fragment_error", 0);
			return 1;
		}
		/* inject_action == 0: pass through (normally or with
		 * corruption)
		 */
	}

	trace_mctp_transport_rx("usb", netdev, 0, skb->len);
	return 0;
}

/* usblib RX hook: called when received data is discarded */
static void mctp_usb_rx_error(struct net_device *netdev,
			      enum mctp_usblib_rx_err err,
			      const void *data, size_t len)
{
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	const struct mctp_usb_hdr *hdr = data;

	switch (err) {
	case MCTP_USBLIB_RX_ERR_ID:
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		/* Try to extract EID for per-EID tracking (MCTP hdr should
		 * follow the USB hdr)
		 */
		if (len >= sizeof(*hdr) + sizeof(struct mctp_hdr)) {
			const struct mctp_hdr *mh = (const void *)(hdr + 1);

			MCTP_STAT_INC(mctp_usb, mh->src, rx_drop_parse_error);
		} else {
			MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_parse_error);
		}
		trace_mctp_transport_error("usb", netdev, "rx_invalid_id", be16_to_cpu(hdr->id));
		break;
	case MCTP_USBLIB_RX_ERR_HDR_LEN:
		netdev->stats.rx_dropped++;
		MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_invalid_len);
		trace_mctp_transport_error("usb", netdev, "rx_short_packet_hdr", hdr->len);
		break;
	case MCTP_USBLIB_RX_ERR_XFER_LEN:
		netdev->stats.rx_dropped++;
		MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_invalid_len);
		trace_mctp_transport_error("usb", netdev, "rx_short_packet_xfer",
					   len - sizeof(*hdr));
		break;
	case MCTP_USBLIB_RX_ERR_NOMEM:
		/* usblib accounts the netdev rx drop */
		MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_no_memory);
		trace_mctp_transport_error("usb", netdev, "rx_split_nomem", 0);
		break;
	}
}

static const struct mctp_usblib_rx_ops mctp_usb_rx_ops = {
	.rx_packet = mctp_usb_rx_packet,
	.rx_error = mctp_usb_rx_error,
};

static void mctp_usb_rx_retry_work(struct work_struct *work)
{
	struct mctp_usb *mctp_usb = container_of(work, struct mctp_usb,
						 rx_retry_work.work);
	unsigned long flags;
	int rc;

	/* We are only called when rx completions are suspended */
	if (mctp_usb->clear_halt) {
		int pipe = usb_rcvbulkpipe(mctp_usb->usbdev, mctp_usb->ep_in);

		rc = usb_clear_halt(mctp_usb->usbdev, pipe);
		if (rc) {
			netdev_err(mctp_usb->netdev,
				   "can't clear IN EP halt: %d\n", rc);

			if (++mctp_usb->in_err_count >= rx_err_max)
				return;

			spin_lock_irqsave(&mctp_usb->rx_lock, flags);
			if (!mctp_usb->rx_stopped)
				schedule_delayed_work(&mctp_usb->rx_retry_work,
						      RX_RETRY_DELAY);
			spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);
			return;
		}
		mctp_usb->clear_halt = false;
	}

	mctp_usb_rx_queue(mctp_usb, GFP_KERNEL);
}

static int mctp_usb_open(struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);

	WRITE_ONCE(mctp_usb->rx_stopped, false);
	mctp_usb->clear_halt = false;
	mctp_usb->in_err_count = 0;

	netif_start_queue(dev);

	return mctp_usb_rx_queue(mctp_usb, GFP_KERNEL);
}

static int mctp_usb_stop(struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);
	unsigned long flags;

	netif_stop_queue(dev);

	/* prevent RX submission retry */
	spin_lock_irqsave(&mctp_usb->rx_lock, flags);
	mctp_usb->rx_stopped = true;
	cancel_delayed_work(&mctp_usb->rx_retry_work);
	spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);

	flush_delayed_work(&mctp_usb->rx_retry_work);

	usb_kill_urb(mctp_usb->rx_urb);

	usb_kill_anchored_urbs(&mctp_usb->tx_anchor);

	mctp_usblib_tx_cancel(&mctp_usb->tx, dev, SKB_DROP_REASON_DEV_READY);
	mctp_usblib_rx_cancel(&mctp_usb->rx);

	return 0;
}

static ssize_t tx_batching_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_dev(dev);
	struct mctp_usb *mctp_usb = netdev_priv(netdev);

	return sprintf(buf, "%d\n", mctp_usb->tx_batching_enabled ? 1 : 0);
}

static ssize_t tx_batching_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct net_device *netdev = to_net_dev(dev);
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	struct mctp_dev *mdev;
	bool enabled;
	int rc;

	rc = kstrtobool(buf, &enabled);
	if (rc)
		return rc;

	mctp_usb->tx_batching_enabled = enabled;

	rcu_read_lock();
	mdev = __mctp_dev_get(netdev);
	if (mdev) {
		mdev->tx_batching_enabled = enabled;
		mctp_dev_put(mdev);
	}
	rcu_read_unlock();

	return count;
}

static DEVICE_ATTR_RW(tx_batching);

static struct attribute *mctp_usb_attrs[] = {
	&dev_attr_tx_batching.attr,
	NULL,
};

static const struct attribute_group mctp_usb_attr_group = {
	.attrs = mctp_usb_attrs,
};

/* Ethtool statistics support */
/* Per-EID stat descriptors with abbreviated names */
struct mctp_usb_eid_stat_desc {
	const char *name;
	size_t offset;
};

#define MCTP_USB_EID_STAT(abbrev, field) { \
	.name = abbrev, \
	.offset = offsetof(struct mctp_usb_eid_stats, field) \
}

static const struct mctp_usb_eid_stat_desc mctp_usb_eid_stat_descs[] = {
	/* RX statistics */
	MCTP_USB_EID_STAT("rx_drop_no_memory",          rx_drop_no_memory),
	MCTP_USB_EID_STAT("rx_drop_urb_error",          rx_drop_urb_error),
	MCTP_USB_EID_STAT("rx_drop_invalid_len",        rx_drop_invalid_len),
	MCTP_USB_EID_STAT("rx_drop_parse_error",        rx_drop_parse_error),
	MCTP_USB_EID_STAT("rx_drop_fragment_error",     rx_drop_fragment_error),
	/* TX statistics - Valid USB error codes only */
	MCTP_USB_EID_STAT("tx_drop_no_memory",          tx_drop_no_memory),
	MCTP_USB_EID_STAT("tx_drop_urb_error",          tx_drop_urb_error),
	MCTP_USB_EID_STAT("tx_drop_ebusy",              tx_drop_ebusy),
	MCTP_USB_EID_STAT("tx_drop_enodev",             tx_drop_enodev),
	MCTP_USB_EID_STAT("tx_drop_enoent",             tx_drop_enoent),
	MCTP_USB_EID_STAT("tx_drop_enxio",              tx_drop_enxio),
	MCTP_USB_EID_STAT("tx_drop_einval",             tx_drop_einval),
	MCTP_USB_EID_STAT("tx_drop_exdev",              tx_drop_exdev),
	MCTP_USB_EID_STAT("tx_drop_efbig",              tx_drop_efbig),
	MCTP_USB_EID_STAT("tx_drop_epipe",              tx_drop_epipe),
	MCTP_USB_EID_STAT("tx_drop_emsgsize",           tx_drop_emsgsize),
	MCTP_USB_EID_STAT("tx_drop_enospc",             tx_drop_enospc),
	MCTP_USB_EID_STAT("tx_drop_eshutdown",          tx_drop_eshutdown),
	MCTP_USB_EID_STAT("tx_drop_eperm",              tx_drop_eperm),
	MCTP_USB_EID_STAT("tx_drop_ehostunreach",       tx_drop_ehostunreach),
	MCTP_USB_EID_STAT("tx_drop_enoexec",            tx_drop_enoexec),
	MCTP_USB_EID_STAT("tx_drop_queue_full",         tx_drop_queue_full),
	MCTP_USB_EID_STAT("tx_drop_timeout",            tx_drop_timeout),
	MCTP_USB_EID_STAT("tx_drop_unlinked",           tx_drop_unlinked),
	MCTP_USB_EID_STAT("tx_timeouts",                tx_timeouts),
	/* General statistics */
	MCTP_USB_EID_STAT("tx_requeued",                tx_requeued),
	MCTP_USB_EID_STAT("rx_requeued",                rx_requeued),
	MCTP_USB_EID_STAT("rx_urb_submitted",           rx_urb_submitted),
	MCTP_USB_EID_STAT("tx_urb_submitted",           tx_urb_submitted),
};

#define MCTP_USB_EID_NUM_STATS ARRAY_SIZE(mctp_usb_eid_stat_descs)

/* Generate the per-EID ethtool callbacks and mctp_usb_ethtool_ops */
MCTP_DEFINE_EID_ETHTOOL_OPS(mctp_usb, struct mctp_usb,
			    struct mctp_usb_eid_stats, mctp_usb_eid_stat_descs,
			    "UNKNOWN: URB/pre-parse errors ");

/* Asynchronously unlink every URB on an anchor.*/
static void mctp_usb_unlink_anchored_urbs(struct usb_anchor *anchor)
{
	struct urb *victim;

	while ((victim = usb_get_from_anchor(anchor)) != NULL) {
		usb_unlink_urb(victim);
		usb_put_urb(victim);
	}
}

static void mctp_usb_tx_timeout(struct net_device *netdev, unsigned int txqueue)
{
	struct mctp_usb *mctp_usb = netdev_priv(netdev);

	netdev_dbg(netdev, "TX WDT timeout, unlinking stuck URBs\n");

	/* Count the stall event. The aborted URBs are accounted per-packet as
	 * tx_drop_timeout when they complete with -ECONNRESET.
	 */
	mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_timeouts++;
	set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
	trace_mctp_transport_error("usb", netdev, "tx_watchdog_timeout", 0);
	mctp_usb_unlink_anchored_urbs(&mctp_usb->tx_anchor);
}

static const struct net_device_ops mctp_usb_netdev_ops = {
	.ndo_start_xmit = mctp_usb_start_xmit,
	.ndo_open = mctp_usb_open,
	.ndo_stop = mctp_usb_stop,
	.ndo_tx_timeout = mctp_usb_tx_timeout,
};

static void mctp_usb_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
	dev->ethtool_ops = &mctp_usb_ethtool_ops;
	dev->min_mtu = MCTP_USB_MTU_MIN;
	dev->max_mtu = MCTP_USB_1_0_MTU_MAX;

	dev->hard_header_len = sizeof(struct mctp_usb_hdr);
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_usb_netdev_ops;
	dev->watchdog_timeo = MCTP_USB_TX_TIMEOUT;
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_DSTATS;
}

static int mctp_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct usb_host_interface *iface_desc;
	struct net_device *netdev;
	struct mctp_usb *dev;
	int rc;

	/* only one alternate */
	iface_desc = intf->cur_altsetting;

	rc = usb_find_common_endpoints(iface_desc, &ep_in, &ep_out, NULL, NULL);
	if (rc) {
		dev_err(&intf->dev, "invalid endpoints on device?\n");
		return rc;
	}

	netdev = alloc_netdev(sizeof(*dev), "mctpusb%d", NET_NAME_ENUM,
			      mctp_usb_netdev_setup);
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, &intf->dev);
	dev = netdev_priv(netdev);
	dev->netdev = netdev;
	dev->usbdev = interface_to_usbdev(intf);
	dev->intf = intf;
	spin_lock_init(&dev->rx_lock);
	usb_set_intfdata(intf, dev);

	mctp_usblib_rx_init(&dev->rx);
	dev->rx.ops = &mctp_usb_rx_ops;
	mctp_usblib_tx_init(&dev->tx, &tx_ops, dev);

	dev->ep_in = ep_in->bEndpointAddress;
	dev->ep_out = ep_out->bEndpointAddress;

	init_usb_anchor(&dev->tx_anchor);

	dev->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->rx_urb) {
		rc = -ENOMEM;
		goto err_fini_rxtx;
	}

	INIT_DELAYED_WORK(&dev->rx_retry_work, mctp_usb_rx_retry_work);

	dev->tx_batching_enabled = true;

	rc = mctp_register_netdev(netdev, &mctp_usb_mctp_ops,
				  MCTP_PHYS_BINDING_USB);
	if (rc)
		goto err_free_urb;

	{
		struct mctp_dev *mdev;

		rcu_read_lock();
		mdev = __mctp_dev_get(netdev);
		if (mdev) {
			mdev->tx_batching_enabled = true;
			mdev->tx_batch_hdr_len = sizeof(struct mctp_usb_hdr);
			mdev->tx_batch_max_xfer = MCTP_USB_1_0_XFER_SIZE;
			mctp_dev_put(mdev);
		}
		rcu_read_unlock();
	}

	rc = sysfs_create_group(&netdev->dev.kobj, &mctp_usb_attr_group);
	if (rc) {
		netdev_err(netdev, "failed to create sysfs group: %d\n", rc);
		goto err_unregister_netdev;
	}

	/* Setup error injection after netdev registration (debugfs needs the
	 * netdev name)
	 */
	mctp_usb_error_inject_init(dev);

	return 0;

err_unregister_netdev:
	mctp_unregister_netdev(netdev);
err_free_urb:
	usb_free_urb(dev->rx_urb);
err_fini_rxtx:
	mctp_usblib_tx_fini(&dev->tx);
	mctp_usblib_rx_fini(&dev->rx);
	free_netdev(netdev);
	return rc;
}

static void mctp_usb_disconnect(struct usb_interface *intf)
{
	struct mctp_usb *dev = usb_get_intfdata(intf);

	sysfs_remove_group(&dev->netdev->dev.kobj, &mctp_usb_attr_group);

	/* Cleanup error injection */
	mctp_usb_error_inject_cleanup(dev);

	mctp_unregister_netdev(dev->netdev);
	mctp_usblib_rx_fini(&dev->rx);
	mctp_usblib_tx_fini(&dev->tx);
	usb_free_urb(dev->rx_urb);
	free_netdev(dev->netdev);
}

static const struct usb_device_id mctp_usb_devices[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_MCTP, 0x0, 0x1) },
	{ USB_INTERFACE_INFO(USB_CLASS_MCTP, 0x0, 0x2) },
	{ 0 },
};

MODULE_DEVICE_TABLE(usb, mctp_usb_devices);

static struct usb_driver mctp_usb_driver = {
	.name		= "mctp-usb",
	.id_table	= mctp_usb_devices,
	.probe		= mctp_usb_probe,
	.disconnect	= mctp_usb_disconnect,
};

static int __init mctp_usb_init(void)
{
	int rc;

	/* Initialize error injection infrastructure */
	rc = mctp_usb_error_inject_module_init();
	if (rc)
		pr_warn("MCTP USB: Error injection initialization failed, continuing without it\n");

	rc = usb_register(&mctp_usb_driver);
	if (rc)
		mctp_usb_error_inject_module_exit();

	return rc;
}

static void __exit mctp_usb_exit(void)
{
	usb_deregister(&mctp_usb_driver);
	mctp_usb_error_inject_module_exit();
}

module_init(mctp_usb_init);
module_exit(mctp_usb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeremy Kerr <jk@codeconstruct.com.au>");
MODULE_DESCRIPTION("MCTP USB transport");
