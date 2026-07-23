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

static const unsigned int n_rx_queue = 8;
static const unsigned int n_tx_queue = 8;

struct mctp_usb_batch_ctx {
	struct net_device *netdev;
	struct sk_buff_head skbs;
	unsigned int num_packets;
};

/**
 * mctp_usb_handle_tx_urb_status - Handle TX URB completion status
 * @mctp_usb: MCTP USB device
 * @netdev: Network device
 * @status: URB completion status code
 * @num_packets: Number of packets in the batch
 *
 * Processes TX URB completion status and updates statistics accordingly.
 * All async URB errors are tracked under MCTP_EID_UNKNOWN since batch
 * completion cannot determine individual packet EIDs.
 */
static void mctp_usb_handle_tx_urb_status(struct mctp_usb *mctp_usb,
					   struct net_device *netdev,
					   int status,
					   unsigned int num_packets)
{
	switch (status) {
	/* Valid USB error codes only */
	case -EBUSY:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_ebusy += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENODEV:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enodev += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOENT:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enoent += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENXIO:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enxio += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EINVAL:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_einval += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EXDEV:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_exdev += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EFBIG:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_efbig += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EPIPE:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_epipe += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EMSGSIZE:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_emsgsize += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOSPC:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enospc += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ESHUTDOWN:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_eshutdown += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EPERM:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_eperm += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -EHOSTUNREACH:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_ehostunreach += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case -ENOEXEC:
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_enoexec += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
		break;
	case 0:
		netdev->stats.tx_packets += num_packets;
		/* tx_bytes already updated per packet during batching */
		break;
	default:
		if (net_ratelimit())
			netdev_warn(netdev, "unexpected tx urb status: %d\n",
				    status);
		netdev->stats.tx_dropped += num_packets;
		mctp_usb->eid_stats.eid[MCTP_EID_UNKNOWN].tx_drop_urb_error += num_packets;
		set_bit(MCTP_EID_UNKNOWN, mctp_usb->eid_stats.active);
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

static void mctp_usb_out_complete(struct urb *urb)
{
	struct mctp_usb_batch_ctx *ctx = urb->context;
	struct net_device *netdev = ctx->netdev;
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	struct sk_buff *skb;
	int status;

	usb_unanchor_urb(urb);
	if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
		netif_wake_queue(netdev);

	status = urb->status;

	/* ERROR INJECTION POINT: TX URB completion (asynchronous error)
	 * Note: Injection happens at URB level (may affect multiple batched
	 * packets). Pass urb so async path can apply EID filter from first
	 * packet.
	 */
	status = mctp_usb_error_inject_tx_async(mctp_usb, urb);

	/* Handle TX URB status and update netdev + per-EID statistics.
	 * Batched URBs increment counters by ctx->num_packets.
	 */
	mctp_usb_handle_tx_urb_status(mctp_usb, netdev, status,
				      ctx->num_packets);

	while ((skb = skb_dequeue(&ctx->skbs)) != NULL) {
		if (status == 0)
			consume_skb(skb);
		else
			kfree_skb(skb);
	}

	kfree(ctx);
	usb_free_urb(urb);
}

static netdev_tx_t mctp_usb_send_single(struct mctp_usb *mctp_usb,
					struct sk_buff *skb)
{
	struct net_device *netdev = mctp_usb->netdev;
	struct mctp_usb_batch_ctx *ctx;
	unsigned int pkt_len = skb->len;
	struct urb *urb;
	u8 *buf;
	/* Default to -ENOMEM so alloc-failure gotos attribute correctly */
	int rc = -ENOMEM;

	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		goto err_drop;

	skb_queue_head_init(&ctx->skbs);
	ctx->netdev = netdev;
	ctx->num_packets = 1;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto err_free_ctx;

	buf = kmalloc(pkt_len, GFP_ATOMIC);
	if (!buf)
		goto err_free_urb;

	skb_copy_bits(skb, 0, buf, pkt_len);
	skb_queue_tail(&ctx->skbs, skb);

	netdev->stats.tx_bytes += pkt_len - sizeof(struct mctp_usb_hdr);

	/* ERROR INJECTION POINT: TX URB submission (synchronous error) */
	rc = mctp_usb_error_inject_tx_sync(mctp_usb, skb);
	if (rc) {
		/* buf is not yet owned by the urb (URB_FREE_BUFFER unset) */
		kfree(buf);
		goto err_free_urb;
	}

	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  buf, pkt_len, mctp_usb_out_complete, ctx);
	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	atomic_inc(&mctp_usb->tx_qlen);
	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		usb_unanchor_urb(urb);
		if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
			netif_wake_queue(netdev);
		goto err_free_urb;
	}

	return NETDEV_TX_OK;

err_free_urb:
	usb_free_urb(urb);
err_free_ctx:
	skb_dequeue(&ctx->skbs);
	kfree(ctx);
err_drop:
	netdev->stats.tx_dropped++;

	/* Track per-EID sync TX error: pull the USB header (still present from
	 * mctp_usb_start_xmit) to expose the MCTP header for EID attribution.
	 */
	if (skb->len >= sizeof(struct mctp_usb_hdr)) {
		struct mctp_hdr *mh;

		skb_pull(skb, sizeof(struct mctp_usb_hdr));
		skb_reset_network_header(skb);
		mh = mctp_hdr(skb);
		mctp_usb_handle_tx_sync_error(mctp_usb, mh->dest, rc);
	} else {
		mctp_usb_handle_tx_sync_error(mctp_usb, MCTP_EID_UNKNOWN, rc);
	}

	kfree_skb(skb);
	return NETDEV_TX_OK;
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
	rc = mctp_usb_error_inject_tx_sync(mctp_usb, skb);
	if (rc)
		goto err_free_urb;

	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  skb->data, skb->len, mctp_usb_out_complete, ctx);

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	atomic_inc(&mctp_usb->tx_qlen);
	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		usb_unanchor_urb(urb);
		if (atomic_dec_return(&mctp_usb->tx_qlen) < n_tx_queue)
			netif_wake_queue(netdev);
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
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static netdev_tx_t mctp_usb_start_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);
	struct mctp_usb_hdr *hdr;
	unsigned int plen, pkt_len;
	int rc;

	/* Batched SKBs from route.c have a marker protocol */
	if (skb->protocol == htons(ETH_P_MCTP | 0x8000))
		return mctp_usb_send_batch(mctp_usb, skb);

	plen = skb->len;
	pkt_len = plen + sizeof(*hdr);

	if (pkt_len > MCTP_USB_XFER_SIZE) {
		/* Extract EID for tracking before dropping */
		struct mctp_hdr *mh = mctp_hdr(skb);

		MCTP_STAT_INC(mctp_usb, mh->dest, tx_drop_emsgsize);
		goto err_drop;
	}

	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	rc = skb_cow_head(skb, sizeof(*hdr));
	if (rc)
		goto err_drop;

	hdr = skb_push(skb, sizeof(*hdr));
	if (!hdr)
		goto err_drop;

	hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr->rsvd = 0;
	hdr->len = pkt_len;

	return mctp_usb_send_single(mctp_usb, skb);

err_drop:
	dev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void mctp_usb_in_complete(struct urb *urb);

static const unsigned long RX_RETRY_DELAY = HZ / 4;

static int mctp_usb_rx_queue(struct mctp_usb *mctp_usb, struct urb *urb,
			     gfp_t gfp)
{
	struct sk_buff *skb;
	int rc;

	if (READ_ONCE(mctp_usb->rx_stopped))
		return 0;

	skb = __netdev_alloc_skb(mctp_usb->netdev, MCTP_USB_XFER_SIZE, gfp);
	if (!skb)
		return -ENOMEM;

	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_rcvbulkpipe(mctp_usb->usbdev, mctp_usb->ep_in),
			  skb->data, MCTP_USB_XFER_SIZE,
			  mctp_usb_in_complete, skb);

	rc = usb_submit_urb(urb, gfp);
	if (rc) {
		kfree_skb(skb);
		return rc;
	}

	atomic_inc(&mctp_usb->rx_qlen);
	return 0;
}

static void mctp_usb_in_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct net_device *netdev = skb->dev;
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	struct mctp_skb_cb *cb;
	unsigned int len;
	int status, rc;
	unsigned long flags;

	status = urb->status;
	atomic_dec(&mctp_usb->rx_qlen);

	/* ERROR INJECTION POINT: RX URB completion error */
	status = mctp_usb_error_inject_rx(mctp_usb, status);

	switch (status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -EPROTO:
		if (mctp_usb->error_inject.common.enable_rx)
			netdev_info(netdev,
				    "RX packet DROPPED (error %d) - error injection is ACTIVE\n",
				    status);
		else
			netdev_dbg(netdev,
				   "RX packet DROPPED (error %d) - expected shutdown/reset\n",
				   status);
		usb_unanchor_urb(urb);
		usb_free_urb(urb);
		netdev->stats.rx_dropped++;
		MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_urb_error);
		kfree_skb(skb);
		return;
	case 0:
		break;
	default:
		if (mctp_usb->error_inject.common.enable_rx)
			netdev_info(netdev,
				    "RX packet DROPPED (error %d) - error injection is ACTIVE\n",
				    status);
		else
			netdev_dbg(netdev, "unexpected rx urb status: %d\n",
				   status);
		usb_unanchor_urb(urb);
		usb_free_urb(urb);
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_urb_error);
		kfree_skb(skb);
		return;
	}

	len = urb->actual_length;
	__skb_put(skb, len);

	while (skb) {
		struct sk_buff *skb2 = NULL;
		struct mctp_usb_hdr *hdr;
		u8 pkt_len;

		hdr = skb_pull_data(skb, sizeof(*hdr));
		if (!hdr)
			break;

		if (be16_to_cpu(hdr->id) != MCTP_USB_DMTF_ID) {
			netdev_dbg(netdev, "rx: invalid id %04x\n",
				   be16_to_cpu(hdr->id));
			netdev->stats.rx_errors++;
			netdev->stats.rx_dropped++;
			/* Try to extract EID for per-EID tracking (USB hdr was
			 * pulled, MCTP hdr should be accessible)
			 */
			if (skb->len >= sizeof(struct mctp_hdr)) {
				struct mctp_hdr *mh = (struct mctp_hdr *)skb->data;

				MCTP_STAT_INC(mctp_usb, mh->src, rx_drop_parse_error);
			} else {
				MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_parse_error);
			}
			break;
		}

		if (hdr->len <
		    sizeof(struct mctp_hdr) + sizeof(struct mctp_usb_hdr)) {
			netdev_dbg(netdev, "rx: short packet (hdr) %d\n",
				   hdr->len);
			netdev->stats.rx_dropped++;
			MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_invalid_len);
			break;
		}

		pkt_len = hdr->len - sizeof(struct mctp_usb_hdr);
		if (pkt_len > skb->len) {
			netdev_dbg(netdev,
				   "rx: short packet (xfer) %d, actual %d\n",
				   hdr->len, skb->len);
			netdev->stats.rx_dropped++;
			MCTP_STAT_INC(mctp_usb, MCTP_EID_UNKNOWN, rx_drop_invalid_len);
			break;
		}

		if (pkt_len < skb->len) {
			skb2 = skb_clone(skb, GFP_ATOMIC);
			if (skb2) {
				if (!skb_pull(skb2, pkt_len)) {
					kfree_skb(skb2);
					skb2 = NULL;
				}
			}
			skb_trim(skb, pkt_len);
		}

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += skb->len;

		skb->protocol = htons(ETH_P_MCTP);
		skb_reset_network_header(skb);
		cb = __mctp_cb(skb);
		cb->halen = 0;

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

				/* Drop this fragment */
				netdev->stats.rx_dropped++;
				MCTP_STAT_INC(mctp_usb, src_eid, rx_drop_fragment_error);
				kfree_skb(skb);
				skb = skb2;
				continue;
			}
			/* inject_action == 0: pass through (normally or with
			 * corruption)
			 */
		}

		netif_rx(skb);

		skb = skb2;
	}

	if (skb)
		kfree_skb(skb);

	rc = mctp_usb_rx_queue(mctp_usb, urb, GFP_ATOMIC);
	if (rc) {
		usb_free_urb(urb);
		spin_lock_irqsave(&mctp_usb->rx_lock, flags);
		if (!mctp_usb->rx_stopped)
			schedule_delayed_work(&mctp_usb->rx_retry_work, RX_RETRY_DELAY);
		spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);
	}
}

static int mctp_usb_rx_queue_fill(struct mctp_usb *mctp_usb)
{
	int i, qlen, rc = 0;

	qlen = atomic_read(&mctp_usb->rx_qlen);
	if (qlen < 0 || qlen >= n_rx_queue)
		return 0;

	for (i = 0; i < n_rx_queue - qlen; i++) {
		struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);

		if (!urb) {
			rc = -ENOMEM;
			break;
		}

		usb_anchor_urb(urb, &mctp_usb->rx_anchor);

		rc = mctp_usb_rx_queue(mctp_usb, urb, GFP_KERNEL);
		if (rc) {
			usb_unanchor_urb(urb);
			usb_free_urb(urb);
			break;
		}
	}

	return rc;
}

static void mctp_usb_rx_retry_work(struct work_struct *work)
{
	struct mctp_usb *mctp_usb = container_of(work, struct mctp_usb,
						 rx_retry_work.work);
	unsigned long flags;
	int rc;

	if (READ_ONCE(mctp_usb->rx_stopped))
		return;

	rc = mctp_usb_rx_queue_fill(mctp_usb);
	if (rc) {
		spin_lock_irqsave(&mctp_usb->rx_lock, flags);
		if (!mctp_usb->rx_stopped)
			schedule_delayed_work(&mctp_usb->rx_retry_work, RX_RETRY_DELAY);
		spin_unlock_irqrestore(&mctp_usb->rx_lock, flags);
	}
}

static int mctp_usb_open(struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);

	WRITE_ONCE(mctp_usb->rx_stopped, false);

	netif_start_queue(dev);

	return mctp_usb_rx_queue_fill(mctp_usb);
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

	usb_kill_anchored_urbs(&mctp_usb->rx_anchor);
	usb_kill_anchored_urbs(&mctp_usb->tx_anchor);

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

static const struct net_device_ops mctp_usb_netdev_ops = {
	.ndo_start_xmit = mctp_usb_start_xmit,
	.ndo_open = mctp_usb_open,
	.ndo_stop = mctp_usb_stop,
};

static void mctp_usb_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
	dev->ethtool_ops = &mctp_usb_ethtool_ops;
	dev->min_mtu = MCTP_USB_MTU_MIN;
	dev->max_mtu = MCTP_USB_MTU_MAX;

	dev->hard_header_len = sizeof(struct mctp_usb_hdr);
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_usb_netdev_ops;
}

static int mctp_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct usb_host_interface *iface_desc;
	struct net_device *netdev;
	struct mctp_usb *dev;
	int rc;

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

	dev->ep_in = ep_in->bEndpointAddress;
	dev->ep_out = ep_out->bEndpointAddress;

	init_usb_anchor(&dev->rx_anchor);
	init_usb_anchor(&dev->tx_anchor);

	INIT_DELAYED_WORK(&dev->rx_retry_work, mctp_usb_rx_retry_work);

	dev->tx_batching_enabled = true;

	rc = mctp_register_netdev(netdev, &mctp_usb_mctp_ops,
				  MCTP_PHYS_BINDING_USB);
	if (rc)
		goto err_free_netdev;

	{
		struct mctp_dev *mdev;

		rcu_read_lock();
		mdev = __mctp_dev_get(netdev);
		if (mdev) {
			mdev->tx_batching_enabled = true;
			mdev->tx_batch_hdr_len = sizeof(struct mctp_usb_hdr);
			mdev->tx_batch_max_xfer = MCTP_USB_XFER_SIZE;
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
err_free_netdev:
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
