// SPDX-License-Identifier: GPL-2.0+
/*
 * f_mctp.c - USB peripheral MCTP driver
 *
 * Copyright (C) 2024 Code Construct Pty Ltd
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/list.h>
#include <linux/ethtool.h>
#include <linux/bitmap.h>

#include <net/mctp.h>
#include <net/pkt_sched.h>

#include <linux/usb/func_utils.h>
#include <linux/usb/mctp-usb.h>

#include <uapi/linux/if_arp.h>

#define MCTP_USB_PREALLOC	4

/**
 * MCTP_EID_UNKNOWN - Special EID for errors where EID is not yet known
 *
 * Value: 256 (out-of-band value to avoid conflict with valid MCTP EIDs 0-254,
 *        and distinct from broadcast address 255)
 *
 * Used for pre-parse errors, allocation failures, and async EP completion
 * callbacks where the actual endpoint ID cannot be determined.
 */
#define MCTP_EID_UNKNOWN 256

/**
 * MCTP_STAT_INC - Increment per-EID statistics and mark EID as active
 * @dev_ptr: Pointer to f_mctp structure
 * @eid_val: EID value (0-256) - use MCTP_EID_UNKNOWN if EID not yet known
 * @stat_field: Name of the statistics field
 */
#define MCTP_STAT_INC(dev_ptr, eid_val, stat_field) do { \
	typeof(dev_ptr) __dev = (dev_ptr); \
	unsigned int __eid = (eid_val); \
	set_bit(__eid, __dev->eid_stats.active); \
	__dev->eid_stats.eid[__eid].stat_field++; \
} while (0)

/* Per-EID statistics structure for the gadget function */
struct f_mctp_eid_stats {
	/* RX stats */
	u64 rx_drop_no_memory;		/* SKB allocation failure */
	u64 rx_drop_invalid_id;		/* Invalid MCTP USB header ID */
	u64 rx_drop_short_hdr;		/* Packet too short (header check) */
	u64 rx_drop_short_xfer;		/* Packet too short (transfer) */
	u64 rx_drop_parse_error;	/* Generic parse error (no hdr) */
	u64 rx_drop_econnaborted;	/* EP completion: -ECONNABORTED */
	u64 rx_drop_econnreset;		/* EP completion: -ECONNRESET */
	u64 rx_drop_eshutdown;		/* EP completion: -ESHUTDOWN */
	u64 rx_drop_unknown_error;	/* EP completion: other errors */
	u64 rx_requeued;		/* RX requests requeued */
	/* TX stats */
	u64 tx_drop_too_large;		/* SKB too large for transfer */
	u64 tx_drop_no_req;		/* No TX request available */
	u64 tx_drop_ep_queue_error;	/* usb_ep_queue() failure */
	u64 tx_drop_econnaborted;	/* EP completion: -ECONNABORTED */
	u64 tx_drop_econnreset;		/* EP completion: -ECONNRESET */
	u64 tx_drop_eshutdown;		/* EP completion: -ESHUTDOWN */
	u64 tx_drop_unknown_error;	/* EP completion: other errors */
};

struct f_mctp {
	struct usb_function	function;

	struct usb_ep		*in_ep;
	struct usb_ep		*out_ep;

	struct net_device	*dev;

	/* Updates to skb_free_list and the req lists are performed under
	 * ->lock
	 */
	spinlock_t		lock;
	struct sk_buff_head	skb_free_list;
	struct list_head	rx_reqs;
	struct list_head	tx_reqs;

	struct work_struct	prealloc_work;

	/* Per-EID statistics tracking - SINGLE source of truth
	 *
	 * All statistics are tracked per-endpoint-ID (EID). Special EIDs:
	 * - EID 0: "null endpoint" - valid packets with EID=0
	 * - EID 256 (MCTP_EID_UNKNOWN): errors where EID could not be
	 *   determined (EP completion callbacks, allocation failures,
	 *   pre-parse errors)
	 */
	struct {
		DECLARE_BITMAP(active, 257);  /* Which EIDs have activity */
		struct f_mctp_eid_stats eid[257];
	} eid_stats;
};

struct f_mctp_opts {
	struct usb_function_instance	function_instance;
};

static inline struct f_mctp *func_to_mctp(struct usb_function *f)
{
	return container_of(f, struct f_mctp, function);
}

static struct usb_interface_descriptor mctp_usbg_intf = {
	.bLength =		sizeof(mctp_usbg_intf),
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_MCTP,
	.bInterfaceSubClass =	0x0, /* todo: allow host-interface mode? */
	.bInterfaceProtocol =	0x1, /* MCTP version 1 */
	/* .iInterface = DYNAMIC */
};

/* descriptors, full speed only */

static struct usb_endpoint_descriptor hs_mctp_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.wMaxPacketSize =	cpu_to_le16(MCTP_USB_XFER_SIZE),

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor hs_mctp_sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.wMaxPacketSize =	cpu_to_le16(MCTP_USB_XFER_SIZE),

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *hs_mctp_descs[] = {
	(struct usb_descriptor_header *) &mctp_usbg_intf,
	(struct usb_descriptor_header *) &hs_mctp_sink_desc,
	(struct usb_descriptor_header *) &hs_mctp_source_desc,
	NULL,
};

/* strings */
static struct usb_string mctp_usbg_strings[] = {
	{ .s = "MCTP over USB" },
	{ 0 }
};

static struct usb_gadget_strings mctp_usbg_stringtab = {
	.language	= 0x0409,	/* en-us */
	.strings	= mctp_usbg_strings,
};

static struct usb_gadget_strings *mctp_usbg_gadget_strings[] = {
	&mctp_usbg_stringtab,
	NULL,
};

static int mctp_usbg_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_mctp *mctp = func_to_mctp(f);
	int id, rc;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	mctp_usbg_intf.bInterfaceNumber = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;

	mctp_usbg_strings[0].id = id;

	mctp->in_ep = usb_ep_autoconfig(cdev->gadget, &hs_mctp_source_desc);
	if (!mctp->in_ep) {
		ERROR(cdev, "%s in_ep autoconfig failed\n", f->name);
		return -ENODEV;
	}
	hs_mctp_source_desc.wMaxPacketSize = cpu_to_le16(MCTP_USB_XFER_SIZE);

	mctp->out_ep = usb_ep_autoconfig(cdev->gadget, &hs_mctp_sink_desc);
	if (!mctp->out_ep) {
		ERROR(cdev, "%s out_ep autoconfig failed\n", f->name);
		return -ENODEV;
	}
	hs_mctp_sink_desc.wMaxPacketSize = cpu_to_le16(MCTP_USB_XFER_SIZE);

	rc = usb_assign_descriptors(f, NULL, hs_mctp_descs, NULL, NULL);
	if (rc) {
		ERROR(cdev, "assign_descriptors failed %d\n", rc);
		return rc;
	}

	DBG(cdev, "%s: in %s, out %s\n", f->name, mctp->in_ep->name,
	     mctp->out_ep->name);

	return 0;
}

static void mctp_usbg_prealloc(struct f_mctp *mctp)
{
	struct sk_buff_head skbs;
	struct usb_request *req;
	struct list_head reqs;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int n_skb;

	/* allocate SKBs for requests that have been consumed and don't
	 * have an associated skb, then requeue.
	 */
	spin_lock_irqsave(&mctp->lock, flags);
	list_replace_init(&mctp->rx_reqs, &reqs);
	spin_unlock_irqrestore(&mctp->lock, flags);

	while (!list_empty(&reqs)) {
		req = list_first_entry(&reqs, struct usb_request, list);
		list_del(&req->list);

		skb = __netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE,
					 GFP_KERNEL);
		if (!skb) {
			MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN,
				      rx_drop_no_memory);
			/* Put it back for next time */
			spin_lock_irqsave(&mctp->lock, flags);
			list_add(&req->list, &mctp->rx_reqs);
			spin_unlock_irqrestore(&mctp->lock, flags);
			break;
		}

		req->buf = skb->data;
		req->context = skb;
		usb_ep_queue(mctp->out_ep, req, GFP_KERNEL);
	}

	/* next, allocate our pool of spare skbs */
	n_skb = skb_queue_len_lockless(&mctp->skb_free_list);
	__skb_queue_head_init(&skbs);

	for (; n_skb < MCTP_USB_PREALLOC; n_skb++) {
		skb = __netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE,
					 GFP_KERNEL);
		if (!skb)
			break;

		__skb_queue_tail(&skbs, skb);
	}

	spin_lock_irqsave(&mctp->lock, flags);
	skb_queue_splice_tail(&skbs, &mctp->skb_free_list);
	spin_unlock_irqrestore(&mctp->lock, flags);
}

static void mctp_usbg_prealloc_work(struct work_struct *work)
{
	struct f_mctp *mctp = container_of(work, struct f_mctp, prealloc_work);

	mctp_usbg_prealloc(mctp);
}

static int mctp_usbg_requeue(struct f_mctp *mctp, struct usb_ep *ep,
			     struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb;
	int rc = 0;

	req->buf = NULL;

	spin_lock_irqsave(&mctp->lock, flags);

	/* Do we have a preallocated skb available? if so, we can requeue
	 * immediately; otherwise wait for the workqueue to populate.
	 */
	skb = __skb_dequeue(&mctp->skb_free_list);
	if (skb) {
		req->buf = skb->data;
		req->context = skb;
		rc = usb_ep_queue(ep, req, GFP_ATOMIC);
		MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN, rx_requeued);
	} else {
		/* keep for later allocation */
		list_add_tail(&req->list, &mctp->rx_reqs);
	}

	spin_unlock_irqrestore(&mctp->lock, flags);

	schedule_work(&mctp->prealloc_work);

	return rc;
}

static void mctp_usbg_handle_rx_urb(struct f_mctp *mctp,
				    struct usb_request *req)
{
	struct device *dev = &mctp->function.config->cdev->gadget->dev;
	struct net_device *netdev = mctp->dev;
	struct sk_buff *skb = req->context;
	struct mctp_usb_hdr *hdr;
	struct mctp_skb_cb *cb;
	unsigned int len;
	u16 id;

	len = req->actual;
	__skb_put(skb, len);

	hdr = skb_pull_data(skb, sizeof(*hdr));
	if (!hdr) {
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN, rx_drop_parse_error);
		goto err;
	}

	id = be16_to_cpu(hdr->id);
	if (id != MCTP_USB_DMTF_ID) {
		dev_dbg(dev, "%s: invalid id %04x\n", __func__, id);
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		/* Try to extract EID for per-EID tracking */
		if (skb->len >= sizeof(struct mctp_hdr)) {
			struct mctp_hdr *mh = (struct mctp_hdr *)skb->data;

			MCTP_STAT_INC(mctp, mh->src, rx_drop_invalid_id);
		} else {
			MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN,
				      rx_drop_invalid_id);
		}
		goto err;
	}

	if (hdr->len < sizeof(struct mctp_hdr) + sizeof(struct mctp_usb_hdr)) {
		dev_dbg(dev, "%s: short packet (hdr) %d\n",
			__func__, hdr->len);
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		/* USB hdr was pulled; try to extract EID from MCTP data */
		if (skb->len >= sizeof(struct mctp_hdr)) {
			struct mctp_hdr *mh = (struct mctp_hdr *)skb->data;

			MCTP_STAT_INC(mctp, mh->src, rx_drop_short_hdr);
		} else {
			MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN,
				      rx_drop_short_hdr);
		}
		goto err;
	}

	/* todo: multi-packet transfers */
	if (hdr->len - sizeof(struct mctp_usb_hdr) < skb->len) {
		dev_dbg(dev, "%s: short packet (xfer) %d, actual %d\n",
			__func__, hdr->len, skb->len);
		netdev->stats.rx_errors++;
		netdev->stats.rx_dropped++;
		/* USB hdr was pulled, MCTP hdr should be accessible */
		if (skb->len >= sizeof(struct mctp_hdr)) {
			struct mctp_hdr *mh = (struct mctp_hdr *)skb->data;

			MCTP_STAT_INC(mctp, mh->src, rx_drop_short_xfer);
		} else {
			MCTP_STAT_INC(mctp, MCTP_EID_UNKNOWN,
				      rx_drop_short_xfer);
		}
		goto err;
	}

	skb->protocol = htons(ETH_P_MCTP);
	skb_reset_network_header(skb);
	cb = __mctp_cb(skb);
	cb->halen = 0;
	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += skb->len;
	netif_rx(skb);

	return;

err:
	/* todo: return to free list */
	kfree_skb(skb);
}
/* Try to extract source EID from partially-received RX buffer data.
 * On error completion, __skb_put() was never called (skb->len == 0),
 * but the raw buffer may contain data up to req->actual bytes.
 * Returns MCTP_EID_UNKNOWN if not enough data was received.
 */
static unsigned int mctp_usbg_rx_eid_from_req(struct usb_request *req)
{
	unsigned int min_len = sizeof(struct mctp_usb_hdr) +
			       sizeof(struct mctp_hdr);

	if (req->actual >= min_len && req->buf) {
		struct mctp_hdr *mh = (struct mctp_hdr *)
			((u8 *)req->buf + sizeof(struct mctp_usb_hdr));
		return mh->src;
	}
	return MCTP_EID_UNKNOWN;
}

static void mctp_usbg_out_ep_complete(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct f_mctp *mctp = ep->driver_data;
	struct usb_composite_dev *cdev = mctp->function.config->cdev;
	unsigned int eid;
	int rc;

	switch (req->status) {
	case 0:
		mctp_usbg_handle_rx_urb(mctp, req);

		/* re-queue out request */
		rc = mctp_usbg_requeue(mctp, ep, req);
		if (rc) {
			WARNING(cdev, "%s: unable to re-queue out req\n",
				__func__);
			usb_ep_free_request(ep, req);
		}

		break;

	case -ECONNABORTED:
		mctp->dev->stats.rx_dropped++;
		eid = mctp_usbg_rx_eid_from_req(req);
		MCTP_STAT_INC(mctp, eid, rx_drop_econnaborted);
		kfree_skb(req->context);
		usb_ep_free_request(ep, req);
		break;
	case -ECONNRESET:
		mctp->dev->stats.rx_dropped++;
		eid = mctp_usbg_rx_eid_from_req(req);
		MCTP_STAT_INC(mctp, eid, rx_drop_econnreset);
		kfree_skb(req->context);
		usb_ep_free_request(ep, req);
		break;
	case -ESHUTDOWN:
		mctp->dev->stats.rx_dropped++;
		eid = mctp_usbg_rx_eid_from_req(req);
		MCTP_STAT_INC(mctp, eid, rx_drop_eshutdown);
		kfree_skb(req->context);
		usb_ep_free_request(ep, req);
		break;

	default:
		WARNING(cdev, "%s: invalid status %d?", __func__, req->status);
		mctp->dev->stats.rx_dropped++;
		eid = mctp_usbg_rx_eid_from_req(req);
		MCTP_STAT_INC(mctp, eid, rx_drop_unknown_error);
		kfree_skb(req->context);
		usb_ep_free_request(ep, req);
	}
}

static void mctp_usbg_in_ep_complete(struct usb_ep *ep,
				     struct usb_request *req)
{
	struct f_mctp *mctp = ep->driver_data;
	struct usb_composite_dev *cdev = mctp->function.config->cdev;
	struct net_device *netdev = mctp->dev;
	struct sk_buff *skb = req->context;
	unsigned int tx_len;
	unsigned long flags;
	unsigned int dest_eid;

	/* Save length and dest EID before freeing SKB.
	 * The SKB contains [USB hdr | MCTP hdr | payload].
	 * mctp_hdr() uses skb_network_header() which still points
	 * to the MCTP header set by the upper stack.
	 */
	tx_len = skb ? skb->len : 0;
	if (skb && skb->len >= sizeof(struct mctp_usb_hdr) +
				sizeof(struct mctp_hdr)) {
		struct mctp_hdr *mh = mctp_hdr(skb);
		dest_eid = mh->dest;
	} else {
		// We cant extract the dest EID, so we set it to unknown
		dest_eid = MCTP_EID_UNKNOWN;
	}
	kfree_skb(skb);
	req->context = NULL;
	req->buf = NULL;

	switch (req->status) {
	case 0:
		netdev->stats.tx_bytes += tx_len;
		netdev->stats.tx_packets++;
		spin_lock_irqsave(&mctp->lock, flags);
		if (list_empty(&mctp->tx_reqs))
			netif_wake_queue(mctp->dev);
		list_add(&req->list, &mctp->tx_reqs);
		spin_unlock_irqrestore(&mctp->lock, flags);
		break;
	case -ECONNABORTED:
		netdev->stats.tx_dropped++;
		MCTP_STAT_INC(mctp, dest_eid, tx_drop_econnaborted);
		usb_ep_free_request(ep, req);
		break;
	case -ECONNRESET:
		netdev->stats.tx_dropped++;
		MCTP_STAT_INC(mctp, dest_eid, tx_drop_econnreset);
		usb_ep_free_request(ep, req);
		break;
	case -ESHUTDOWN:
		netdev->stats.tx_dropped++;
		MCTP_STAT_INC(mctp, dest_eid, tx_drop_eshutdown);
		usb_ep_free_request(ep, req);
		break;
	default:
		WARNING(cdev, "%s: invalid status %d?", __func__, req->status);
		netdev->stats.tx_dropped++;
		MCTP_STAT_INC(mctp, dest_eid, tx_drop_unknown_error);
		usb_ep_free_request(ep, req);
		break;
	}
}

static int mctp_usbg_enable_ep(struct usb_gadget *gadget, struct f_mctp *mctp,
			       struct usb_ep *ep)
{
	int rc;

	rc = config_ep_by_speed(gadget, &mctp->function, ep);
	if (rc)
		return rc;

	rc = usb_ep_enable(ep);
	if (rc)
		return rc;

	ep->driver_data = mctp;

	return 0;
}

static int mctp_usbg_enable(struct usb_composite_dev *cdev, struct f_mctp *mctp)
{
	struct usb_request *out_req, *in_req;
	unsigned long flags;
	struct sk_buff *skb;
	int rc;

	rc = mctp_usbg_enable_ep(cdev->gadget, mctp, mctp->out_ep);
	if (rc) {
		ERROR(cdev, "%s: out ep enable failed %d\n", __func__, rc);
		return rc;
	}

	rc = mctp_usbg_enable_ep(cdev->gadget, mctp, mctp->in_ep);
	if (rc) {
		ERROR(cdev, "%s: in ep enable failed %d\n", __func__, rc);
		goto err_disable_out;
	}

	/* todo: just one out queued req for now */
	out_req = alloc_ep_req(mctp->out_ep, MCTP_USB_XFER_SIZE);
	if (!out_req) {
		ERROR(cdev, "%s: out req alloc failed\n", __func__);
		goto err_disable_in;
	}

	spin_lock_irqsave(&mctp->lock, flags);
	skb = __skb_dequeue(&mctp->skb_free_list);
	spin_unlock_irqrestore(&mctp->lock, flags);

	if (!skb)
		skb = netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE);

	if (!skb)
		goto err_free_req;

	out_req->context = skb;
	out_req->buf = skb->data;
	out_req->complete = mctp_usbg_out_ep_complete;

	rc = usb_ep_queue(mctp->out_ep, out_req, GFP_ATOMIC);
	if (rc) {
		ERROR(cdev, "%s: out req queue failed %d\b", __func__, rc);
		goto err_free_skb;
	}

	/* todo: and just one in the in queue too */
	in_req = usb_ep_alloc_request(mctp->in_ep, GFP_ATOMIC);
	if (!in_req) {
		ERROR(cdev, "%s: out req alloc failed\n", __func__);
		goto err_disable_in;
	}
	in_req->complete = mctp_usbg_in_ep_complete;

	spin_lock_irqsave(&mctp->lock, flags);
	list_add(&in_req->list, &mctp->tx_reqs);
	spin_unlock_irqrestore(&mctp->lock, flags);

	netif_carrier_on(mctp->dev);
	netif_wake_queue(mctp->dev);
	dev_info(&cdev->gadget->dev, "mctp-usb: enabled\n");

	return 0;


err_free_skb:
	kfree_skb(skb);
err_free_req:
	free_ep_req(mctp->out_ep, out_req);
err_disable_in:
	usb_ep_disable(mctp->in_ep);
err_disable_out:
	usb_ep_disable(mctp->out_ep);

	return rc;
}

static netdev_tx_t mctp_usbg_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct f_mctp *mctp = netdev_priv(dev);
	struct net_device *netdev = mctp->dev;
	struct mctp_usb_hdr *hdr;
	struct usb_request *req;
	unsigned long flags;
	unsigned int plen;
	int rc;

	if (skb->len + sizeof(*hdr) > MCTP_USB_XFER_SIZE) {
		/* Extract EID for tracking before dropping */
		struct mctp_hdr *mh = mctp_hdr(skb);

		MCTP_STAT_INC(mctp, mh->dest, tx_drop_too_large);
		goto drop;
	}

	spin_lock_irqsave(&mctp->lock, flags);
	req = list_first_entry_or_null(&mctp->tx_reqs, struct usb_request, list);

	if (req)
		list_del(&req->list);
	if (list_empty(&mctp->tx_reqs))
		netif_stop_queue(dev);

	spin_unlock_irqrestore(&mctp->lock, flags);

	if (!req) {
		struct mctp_hdr *mh = mctp_hdr(skb);

		netdev_warn(dev, "no tx reqs available\n");
		MCTP_STAT_INC(mctp, mh->dest, tx_drop_no_req);
		goto drop;
	}

	plen = skb->len;
	hdr = skb_push(skb, sizeof(*hdr));
	hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr->rsvd = 0;
	hdr->len = plen + sizeof(*hdr);

	/* todo: just one skb per transfer.. */
	req->context = skb;
	req->buf = skb->data;
	req->length = skb->len;

	rc = usb_ep_queue(mctp->in_ep, req, GFP_ATOMIC);
	if (rc) {
		struct mctp_hdr *mh = mctp_hdr(skb);

		netdev_warn(dev, "tx queue failed: %d\n", rc);
		MCTP_STAT_INC(mctp, mh->dest, tx_drop_ep_queue_error);
		req->context = NULL;
		req->buf = NULL;
		spin_lock_irqsave(&mctp->lock, flags);
		list_add(&req->list, &mctp->tx_reqs);
		netif_wake_queue(dev);
		spin_unlock_irqrestore(&mctp->lock, flags);
		goto drop;
	}

	netdev->stats.tx_bytes += skb->len;
	netdev->stats.tx_packets++;

	return NETDEV_TX_OK;

drop:
	kfree_skb(skb);
	netdev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static int mctp_usbg_open(struct net_device *net)
{
	return 0;
}

static int mctp_usbg_stop(struct net_device *net)
{
	return 0;
}

/* ---- Ethtool statistics support ---- */

/* Per-EID stat descriptors for ethtool */
struct f_mctp_eid_stat_desc {
	const char *name;
	size_t offset;
};

#define F_MCTP_EID_STAT(field) { \
	.name = #field, \
	.offset = offsetof(struct f_mctp_eid_stats, field) \
}

static const struct f_mctp_eid_stat_desc f_mctp_eid_stat_descs[] = {
	F_MCTP_EID_STAT(rx_drop_no_memory),
	F_MCTP_EID_STAT(rx_drop_invalid_id),
	F_MCTP_EID_STAT(rx_drop_short_hdr),
	F_MCTP_EID_STAT(rx_drop_short_xfer),
	F_MCTP_EID_STAT(rx_drop_parse_error),
	F_MCTP_EID_STAT(rx_drop_econnaborted),
	F_MCTP_EID_STAT(rx_drop_econnreset),
	F_MCTP_EID_STAT(rx_drop_eshutdown),
	F_MCTP_EID_STAT(rx_drop_unknown_error),
	F_MCTP_EID_STAT(rx_requeued),
	F_MCTP_EID_STAT(tx_drop_too_large),
	F_MCTP_EID_STAT(tx_drop_no_req),
	F_MCTP_EID_STAT(tx_drop_ep_queue_error),
	F_MCTP_EID_STAT(tx_drop_econnaborted),
	F_MCTP_EID_STAT(tx_drop_econnreset),
	F_MCTP_EID_STAT(tx_drop_eshutdown),
	F_MCTP_EID_STAT(tx_drop_unknown_error),
};

#define F_MCTP_EID_NUM_STATS ARRAY_SIZE(f_mctp_eid_stat_descs)

/* Helper: count non-zero stats for one EID */
static int f_mctp_count_eid_nonzero(struct f_mctp *mctp, unsigned int eid)
{
	struct f_mctp_eid_stats *es = &mctp->eid_stats.eid[eid];
	u8 *base = (u8 *)es;
	int count = 0;
	unsigned int i;

	for (i = 0; i < F_MCTP_EID_NUM_STATS; i++) {
		if (*(u64 *)(base + f_mctp_eid_stat_descs[i].offset) != 0)
			count++;
	}
	return count;
}

/* Helper: count total ethtool entries for all active EIDs */
static int f_mctp_count_eid_stats(struct f_mctp *mctp)
{
	int count = 0;
	int eid;

	for_each_set_bit(eid, mctp->eid_stats.active, 257) {
		int nz = f_mctp_count_eid_nonzero(mctp, eid);

		if (nz > 0)
			count += 1 + nz;  /* header + non-zero stats */
	}
	return count;
}

/* Helper: sum all stats for one EID */
static u64 f_mctp_eid_stats_total(struct f_mctp *mctp, unsigned int eid)
{
	struct f_mctp_eid_stats *es = &mctp->eid_stats.eid[eid];
	u8 *base = (u8 *)es;
	u64 total = 0;
	unsigned int i;

	for (i = 0; i < F_MCTP_EID_NUM_STATS; i++)
		total += *(u64 *)(base + f_mctp_eid_stat_descs[i].offset);
	return total;
}

static void mctp_usbg_get_strings(struct net_device *ndev, u32 stringset,
				   u8 *data)
{
	struct f_mctp *mctp = netdev_priv(ndev);
	unsigned int i;
	int eid;

	if (stringset != ETH_SS_STATS)
		return;

	/* Aggregate stats (summed across all EIDs) */
	for (i = 0; i < F_MCTP_EID_NUM_STATS; i++) {
		snprintf(data, ETH_GSTRING_LEN, "%-30s",
			 f_mctp_eid_stat_descs[i].name);
		data += ETH_GSTRING_LEN;
	}

	/* Separator */
	snprintf(data, ETH_GSTRING_LEN,
		 "                              ");
	data += ETH_GSTRING_LEN;

	/* Per-EID stats (only active EIDs with non-zero values) */
	for_each_set_bit(eid, mctp->eid_stats.active, 257) {
		struct f_mctp_eid_stats *es = &mctp->eid_stats.eid[eid];
		u8 *base = (u8 *)es;
		int nz = f_mctp_count_eid_nonzero(mctp, eid);

		if (nz == 0)
			continue;

		/* EID header line */
		if (eid == MCTP_EID_UNKNOWN)
			snprintf(data, ETH_GSTRING_LEN,
				 "UNKNOWN: EP/pre-parse errors  ");
		else if (eid == 0)
			snprintf(data, ETH_GSTRING_LEN,
				 "EID_0: null endpoint          ");
		else
			snprintf(data, ETH_GSTRING_LEN,
				 "EID_%-3u                       ", eid);
		data += ETH_GSTRING_LEN;

		/* Individual non-zero stats for this EID */
		for (i = 0; i < F_MCTP_EID_NUM_STATS; i++) {
			u64 val = *(u64 *)(base +
					   f_mctp_eid_stat_descs[i].offset);
			if (val != 0) {
				snprintf(data, ETH_GSTRING_LEN, "%-30s",
					 f_mctp_eid_stat_descs[i].name);
				data += ETH_GSTRING_LEN;
			}
		}
	}
}

static int mctp_usbg_get_sset_count(struct net_device *ndev, int sset)
{
	struct f_mctp *mctp = netdev_priv(ndev);

	if (sset == ETH_SS_STATS)
		return F_MCTP_EID_NUM_STATS   /* aggregates */
		       + 1                     /* separator */
		       + f_mctp_count_eid_stats(mctp);  /* per-EID */

	return -EOPNOTSUPP;
}

static void mctp_usbg_get_ethtool_stats(struct net_device *ndev,
					 struct ethtool_stats *stats,
					 u64 *data)
{
	struct f_mctp *mctp = netdev_priv(ndev);
	unsigned int i, idx = 0;
	int eid;

	/* Aggregate stats (summed across all EIDs) */
	for (i = 0; i < F_MCTP_EID_NUM_STATS; i++) {
		u64 total = 0;

		for_each_set_bit(eid, mctp->eid_stats.active, 257) {
			u8 *base = (u8 *)&mctp->eid_stats.eid[eid];

			total += *(u64 *)(base +
					  f_mctp_eid_stat_descs[i].offset);
		}
		data[idx++] = total;
	}

	/* Separator */
	data[idx++] = 0;

	/* Per-EID stats (only active EIDs with non-zero values) */
	for_each_set_bit(eid, mctp->eid_stats.active, 257) {
		struct f_mctp_eid_stats *es = &mctp->eid_stats.eid[eid];
		u8 *base = (u8 *)es;
		int nz = f_mctp_count_eid_nonzero(mctp, eid);

		if (nz == 0)
			continue;

		/* EID header: value is sum of all stats for this EID */
		data[idx++] = f_mctp_eid_stats_total(mctp, eid);

		/* Individual non-zero stats */
		for (i = 0; i < F_MCTP_EID_NUM_STATS; i++) {
			u64 val = *(u64 *)(base +
					   f_mctp_eid_stat_descs[i].offset);
			if (val != 0)
				data[idx++] = val;
		}
	}
}

static const struct ethtool_ops mctp_usbg_ethtool_ops = {
	.get_strings = mctp_usbg_get_strings,
	.get_sset_count = mctp_usbg_get_sset_count,
	.get_ethtool_stats = mctp_usbg_get_ethtool_stats,
};

static const struct net_device_ops mctp_usbg_netdev_ops = {
	.ndo_open = mctp_usbg_open,
	.ndo_stop = mctp_usbg_stop,
	.ndo_start_xmit = mctp_usbg_start_xmit,
};

static void __mctp_usbg_disable(struct f_mctp *mctp)
{
	usb_ep_disable(mctp->in_ep);
	usb_ep_disable(mctp->out_ep);
}

static void mctp_usbg_purge_tx_reqs(struct f_mctp *mctp)
{
	LIST_HEAD(reqs);
	struct usb_request *req, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&mctp->lock, flags);
	list_splice_init(&mctp->tx_reqs, &reqs);
	spin_unlock_irqrestore(&mctp->lock, flags);

	list_for_each_entry_safe(req, tmp, &reqs, list) {
		list_del(&req->list);
		usb_ep_free_request(mctp->in_ep, req);
	}
}

static void mctp_usbg_disable(struct usb_function *f)
{
	struct f_mctp *mctp = func_to_mctp(f);
	struct usb_composite_dev *cdev = mctp->function.config->cdev;

	__mctp_usbg_disable(mctp);

	cancel_work_sync(&mctp->prealloc_work);
	mctp_usbg_purge_tx_reqs(mctp);

	netif_stop_queue(mctp->dev);
	netif_carrier_off(mctp->dev);
	dev_info(&cdev->gadget->dev, "mctp-usb: disabled\n");
}

static int mctp_usbg_set_alt(struct usb_function *f,
			     unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct f_mctp *mctp = func_to_mctp(f);

	__mctp_usbg_disable(mctp);
	return mctp_usbg_enable(cdev, mctp);
}

static void mctp_usbg_free_func(struct usb_function *f)
{
	struct f_mctp *mctp = func_to_mctp(f);

	/* Cancel pending work before cleanup */
	cancel_work_sync(&mctp->prealloc_work);

	/* Free any pending SKBs in the queues */
	__skb_queue_purge(&mctp->skb_free_list);

	/* The netdev (and f_mctp) will be freed automatically
	 * by the network core since needs_free_netdev = true.
	 */
	unregister_netdev(mctp->dev);
}

static void mctp_usbg_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
	dev->ethtool_ops = &mctp_usbg_ethtool_ops;
	dev->min_mtu = MCTP_USB_MTU_MIN;
	dev->max_mtu = MCTP_USB_MTU_MAX;

	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_usbg_netdev_ops;
	dev->needs_free_netdev = true;
}

static struct usb_function
*mctp_usbg_alloc_func(struct usb_function_instance *fi)
{
	struct f_mctp_opts *opts;
	struct net_device *dev;
	struct f_mctp *mctp;
	int rc;

	opts = container_of(fi, struct f_mctp_opts, function_instance);

	dev = alloc_netdev(sizeof(*mctp), "mctpusbg%d", NET_NAME_ENUM,
			    mctp_usbg_netdev_setup);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	mctp = netdev_priv(dev);
	mctp->dev = dev;

	spin_lock_init(&mctp->lock);
	INIT_LIST_HEAD(&mctp->rx_reqs);
	INIT_LIST_HEAD(&mctp->tx_reqs);
	__skb_queue_head_init(&mctp->skb_free_list);
	INIT_WORK(&mctp->prealloc_work, mctp_usbg_prealloc_work);

	mctp->function.name = "mctp";
	mctp->function.bind = mctp_usbg_bind;
	mctp->function.set_alt = mctp_usbg_set_alt;
	mctp->function.disable = mctp_usbg_disable;
	mctp->function.strings = mctp_usbg_gadget_strings;
	mctp->function.free_func = mctp_usbg_free_func;

	/* this will allocate our first pool of out (rx) skbs */
	mctp_usbg_prealloc(mctp);

	rc = register_netdev(dev);
	if (rc) {
		free_netdev(dev);
		return ERR_PTR(rc);
	}

	return &mctp->function;
}

static struct f_mctp_opts *to_f_mctp_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_mctp_opts,
			    function_instance.group);
}

static void mctp_usbg_attr_release(struct config_item *item)
{
	struct f_mctp_opts *opts = to_f_mctp_opts(item);

	usb_put_function_instance(&opts->function_instance);
}

static struct configfs_item_operations mctp_usbg_item_ops = {
	.release = mctp_usbg_attr_release,
};

static struct configfs_attribute *mctp_usbg_attrs[] = {
	NULL,
};

static const struct config_item_type mctp_usbg_func_type = {
	.ct_item_ops    = &mctp_usbg_item_ops,
	.ct_attrs	= mctp_usbg_attrs,
	.ct_owner       = THIS_MODULE,
};

static void mctp_usbg_free_instance(struct usb_function_instance *fi)
{
	struct f_mctp_opts *opts;

	opts = container_of(fi, struct f_mctp_opts, function_instance);
	kfree(opts);
}

static struct usb_function_instance *mctp_usbg_alloc_instance(void)
{
	struct f_mctp_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	opts->function_instance.free_func_inst = mctp_usbg_free_instance;

	config_group_init_type_name(&opts->function_instance.group, "",
				    &mctp_usbg_func_type);

	return &opts->function_instance;
}

DECLARE_USB_FUNCTION(mctp, mctp_usbg_alloc_instance, mctp_usbg_alloc_func);

static int __init mctp_usbg_mod_init(void)
{
	return usb_function_register(&mctpusb_func);
}

static void __exit mctp_usbg_mod_exit(void)
{
	usb_function_unregister(&mctpusb_func);
}

module_init(mctp_usbg_mod_init);
module_exit(mctp_usbg_mod_exit);
MODULE_LICENSE("GPL");
