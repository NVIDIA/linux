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

#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <net/pkt_sched.h>

#include <uapi/linux/if_arp.h>

/* number of IN/OUT urbs to queue */
const unsigned int n_rx_queue = 8;
const unsigned int n_tx_queue = 8;

struct mctp_usb {
	struct usb_device *usbdev;
	struct usb_interface *intf;
	bool stopped;

	struct net_device *netdev;

	u8 ep_in;
	u8 ep_out;

	struct usb_anchor rx_anchor;
	struct usb_anchor tx_anchor;
	/* number of urbs currently queued */
	atomic_t rx_qlen, tx_qlen;

	struct delayed_work rx_retry_work;

	/* TX batching support - controlled via sysfs */
	bool tx_batching_enabled;
};

/* Structure to track batched packets in URB context */
struct mctp_usb_batch_ctx {
	struct net_device *netdev;
	struct sk_buff_head skbs;
	unsigned int num_packets;
};

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

	switch (status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -EPROTO:
		netdev->stats.tx_dropped += ctx->num_packets;
		break;
	case 0:
		netdev->stats.tx_packets += ctx->num_packets;
		/* tx_bytes already updated per packet during batching */
		break;
	default:
		netdev_dbg(netdev, "unexpected tx urb status: %d\n", status);
		netdev->stats.tx_dropped += ctx->num_packets;
	}

	/* Free all batched skbs */
	while ((skb = skb_dequeue(&ctx->skbs)) != NULL) {
		if (status == 0)
			consume_skb(skb);
		else
			kfree_skb(skb);
	}

	kfree(ctx);
	usb_free_urb(urb);
}
/* Fast path: send a single packet without batching.
 * This avoids lock overhead when batching is disabled.
 */
static netdev_tx_t mctp_usb_send_single(struct mctp_usb *mctp_usb,
					struct sk_buff *skb)
{
	struct net_device *netdev = mctp_usb->netdev;
	struct mctp_usb_batch_ctx *ctx;
	struct urb *urb;
	u8 *buf;
	unsigned int pkt_len = skb->len;
	int rc;

	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	/* Allocate minimal context for single packet */
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		goto err_drop;

	skb_queue_head_init(&ctx->skbs);
	ctx->netdev = netdev;
	ctx->num_packets = 1;

	/* Allocate URB and buffer */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto err_free_ctx;

	buf = kmalloc(pkt_len, GFP_ATOMIC);
	if (!buf)
		goto err_free_urb;

	/* Copy packet data */
	skb_copy_bits(skb, 0, buf, pkt_len);
	skb_queue_tail(&ctx->skbs, skb);

	netdev->stats.tx_bytes += pkt_len - sizeof(struct mctp_usb_hdr);

	/* Submit URB */
	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  buf, pkt_len, mctp_usb_out_complete, ctx);
	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		usb_unanchor_urb(urb);
		goto err_free_buf;
	}

	atomic_inc(&mctp_usb->tx_qlen);
	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);

	return NETDEV_TX_OK;

err_free_buf:
	kfree(buf);
err_free_urb:
	usb_free_urb(urb);
err_free_ctx:
	skb_dequeue(&ctx->skbs);
	kfree(ctx);
err_drop:
	netdev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

/* Callback invoked by route.c to fill in USB transport header during batching.
 * This is called once per packet in the batch, with the exact location and size.
 */
static void mctp_usb_fill_batch_hdr(void *hdr, unsigned int pkt_len)
{
	struct mctp_usb_hdr *usb_hdr = (struct mctp_usb_hdr *)hdr;

	usb_hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	usb_hdr->rsvd = 0;
	usb_hdr->len = pkt_len;
}

static const struct mctp_netdev_ops mctp_usb_ops = {
	.fill_batch_hdr = mctp_usb_fill_batch_hdr,
};

/* Send a pre-batched SKB (from route.c) with multiple MCTP packets.
 * The SKB data has headers pre-filled by route.c via our callback.
 */
static netdev_tx_t mctp_usb_send_batch(struct mctp_usb *mctp_usb,
				       struct sk_buff *skb)
{
	struct net_device *netdev = mctp_usb->netdev;
	struct mctp_usb_batch_ctx *ctx;
	struct urb *urb;
	int rc;

	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	/* Allocate context */
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		goto err_drop;

	skb_queue_head_init(&ctx->skbs);
	ctx->netdev = netdev;
	ctx->num_packets = 1; /* We'll count on completion */

	/* Allocate URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto err_free_ctx;

	skb_queue_tail(&ctx->skbs, skb);

	netdev_dbg(netdev, "Sending batched SKB: %u bytes\n", skb->len);

	/* Submit URB with pre-filled SKB data directly - headers already done by route.c! */
	usb_fill_bulk_urb(urb, mctp_usb->usbdev,
			  usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			  skb->data, skb->len, mctp_usb_out_complete, ctx);

	usb_anchor_urb(urb, &mctp_usb->tx_anchor);
	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		usb_unanchor_urb(urb);
		goto err_free_urb;
	}

	atomic_inc(&mctp_usb->tx_qlen);
	if (atomic_read(&mctp_usb->tx_qlen) >= n_tx_queue)
		netif_stop_queue(netdev);

	return NETDEV_TX_OK;

err_free_urb:
	usb_free_urb(urb);
err_free_ctx:
	skb_dequeue(&ctx->skbs);
	kfree(ctx);
err_drop:
	netdev->stats.tx_dropped++;
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

	/* Detect batched SKBs: route.c sets a special protocol marker
	 * (ETH_P_MCTP | 0x8000) to indicate a pre-batched multi-packet SKB.
	 * This avoids relying on skb->cb which may not be initialized.
	 */
	if (skb->protocol == htons(ETH_P_MCTP | 0x8000)) {
		/* This is a batched SKB - restore the correct protocol */
		skb->protocol = htons(ETH_P_MCTP);
		netdev_dbg(dev, "Detected batched SKB: len=%u\n", skb->len);
		return mctp_usb_send_batch(mctp_usb, skb);
	}

	/* Single packet path */
	plen = skb->len;
	pkt_len = plen + sizeof(*hdr);

	/* Single packet larger than max transfer size - can't send */
	if (pkt_len > MCTP_USB_XFER_SIZE)
		goto err_drop;

	rc = skb_cow_head(skb, sizeof(*hdr));
	if (rc)
		goto err_drop;

	hdr = skb_push(skb, sizeof(*hdr));
	if (!hdr)
		goto err_drop;

	hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr->rsvd = 0;
	hdr->len = pkt_len;

	/* Send single packet */
	return mctp_usb_send_single(mctp_usb, skb);

err_drop:
	dev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void mctp_usb_in_complete(struct urb *urb);

/* If we fail to queue an in urb atomically (either due to skb allocation or
 * urb submission), we will schedule a rx queue in nonatomic context
 * after a delay, specified in jiffies
 */
static const unsigned long RX_RETRY_DELAY = HZ / 4;

static int mctp_usb_rx_queue(struct mctp_usb *mctp_usb, struct urb *urb,
			     gfp_t gfp)
{
	struct sk_buff *skb;
	int rc;

	/* no point allocating if the queue is going to be rejected */
	if (READ_ONCE(mctp_usb->stopped))
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
		netdev_dbg(mctp_usb->netdev, "rx urb submit failure: %d\n", rc);
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

	status = urb->status;
	atomic_dec(&mctp_usb->rx_qlen);

	switch (status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -EPROTO:
		usb_unanchor_urb(urb);
		usb_free_urb(urb);
		kfree_skb(skb);
		return;
	case 0:
		break;
	default:
		netdev_dbg(netdev, "unexpected rx urb status: %d\n", status);
		usb_unanchor_urb(urb);
		usb_free_urb(urb);
		kfree_skb(skb);
		return;
	}

	len = urb->actual_length;
	__skb_put(skb, len);

	while (skb) {
		struct sk_buff *skb2 = NULL;
		struct mctp_usb_hdr *hdr;
		u8 pkt_len; /* length of MCTP packet, no USB header */

		hdr = skb_pull_data(skb, sizeof(*hdr));
		if (!hdr)
			break;

		if (be16_to_cpu(hdr->id) != MCTP_USB_DMTF_ID) {
			netdev_dbg(netdev, "rx: invalid id %04x\n",
				   be16_to_cpu(hdr->id));
			break;
		}

		if (hdr->len <
		    sizeof(struct mctp_hdr) + sizeof(struct mctp_usb_hdr)) {
			netdev_dbg(netdev, "rx: short packet (hdr) %d\n",
				   hdr->len);
			break;
		}

		/* we know we have at least sizeof(struct mctp_usb_hdr) here */
		pkt_len = hdr->len - sizeof(struct mctp_usb_hdr);
		if (pkt_len > skb->len) {
			netdev_dbg(netdev,
				   "rx: short packet (xfer) %d, actual %d\n",
				   hdr->len, skb->len);
			break;
		}

		if (pkt_len < skb->len) {
			/* more packets may follow - clone to a new
			 * skb to use on the next iteration
			 */
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
		netif_rx(skb);

		skb = skb2;
	}

	if (skb)
		kfree_skb(skb);

	rc = mctp_usb_rx_queue(mctp_usb, urb, GFP_ATOMIC);
	if (rc) {
		usb_free_urb(urb);
		schedule_delayed_work(&mctp_usb->rx_retry_work, RX_RETRY_DELAY);
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
	int rc;

	if (READ_ONCE(mctp_usb->stopped))
		return;

	rc = mctp_usb_rx_queue_fill(mctp_usb);
	if (rc)
		schedule_delayed_work(&mctp_usb->rx_retry_work, RX_RETRY_DELAY);
}

static int mctp_usb_open(struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);

	WRITE_ONCE(mctp_usb->stopped, false);

	netif_start_queue(dev);

	return mctp_usb_rx_queue_fill(mctp_usb);
}

static int mctp_usb_stop(struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);

	netif_stop_queue(dev);

	/* prevent RX submission retry */
	WRITE_ONCE(mctp_usb->stopped, true);

	usb_kill_anchored_urbs(&mctp_usb->rx_anchor);
	usb_kill_anchored_urbs(&mctp_usb->tx_anchor);

	cancel_delayed_work_sync(&mctp_usb->rx_retry_work);

	return 0;
}

/* sysfs attribute for runtime control of TX batching */
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

	/* Also update the mctp_dev flag for use in route.c */
	rcu_read_lock();
	mdev = __mctp_dev_get(netdev);
	if (mdev) {
		mdev->tx_batching_enabled = enabled;
		netdev_info(netdev,
			    "TX batching %s (hdr_len=%u, max_xfer=%u)\n",
			    enabled ? "enabled" : "disabled",
			    mdev->tx_batch_hdr_len, mdev->tx_batch_max_xfer);
		mctp_dev_put(
			mdev); /* Release reference taken by __mctp_dev_get() */
	} else {
		netdev_err(netdev, "Failed to get mctp_dev!\n");
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

static const struct net_device_ops mctp_usb_netdev_ops = {
	.ndo_start_xmit = mctp_usb_start_xmit,
	.ndo_open = mctp_usb_open,
	.ndo_stop = mctp_usb_stop,
};

static void mctp_usb_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
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
	dev->usbdev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	usb_set_intfdata(intf, dev);

	dev->ep_in = ep_in->bEndpointAddress;
	dev->ep_out = ep_out->bEndpointAddress;

	init_usb_anchor(&dev->rx_anchor);
	init_usb_anchor(&dev->tx_anchor);

	INIT_DELAYED_WORK(&dev->rx_retry_work, mctp_usb_rx_retry_work);

	/* Enable TX batching by default */
	dev->tx_batching_enabled = true;

	rc = mctp_register_netdev(netdev, &mctp_usb_ops, MCTP_PHYS_BINDING_USB);
	if (rc)
		goto err_free_netdev;

	/* Set the mctp_dev batching parameters for use in route.c */
	{
		struct mctp_dev *mdev;

		rcu_read_lock();
		mdev = __mctp_dev_get(netdev);
		if (mdev) {
			mdev->tx_batching_enabled = true;
			mdev->tx_batch_hdr_len = sizeof(struct mctp_usb_hdr);
			mdev->tx_batch_max_xfer = MCTP_USB_XFER_SIZE;
			mctp_dev_put(
				mdev); /* Release reference taken by __mctp_dev_get() */
		}
		rcu_read_unlock();
	}

	/* Register sysfs attribute for runtime control */
	rc = sysfs_create_group(&netdev->dev.kobj, &mctp_usb_attr_group);
	if (rc) {
		netdev_err(netdev, "failed to create sysfs group: %d\n", rc);
		goto err_unregister_netdev;
	}

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
	mctp_unregister_netdev(dev->netdev);
	usb_put_dev(dev->usbdev);
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

module_usb_driver(mctp_usb_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeremy Kerr <jk@codeconstruct.com.au>");
MODULE_DESCRIPTION("MCTP USB transport");
