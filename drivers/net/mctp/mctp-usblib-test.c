// SPDX-License-Identifier: GPL-2.0
/*
 * mctp-usblib-test.c - MCTP-over-USB (DMTF DSP0283) transport helper library,
 * unit test definitions.
 *
 * Copyright (C) 2026 Code Construct Pty Ltd
 */

#include <uapi/linux/netdevice.h>
#include <linux/netdevice.h>
#include <kunit/test.h>
#include <linux/if_arp.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <linux/usb/mctp-usb.h>

struct mctp_usblib_test_dev {
	struct net_device *ndev;
	struct mctp_dev *mdev;
	struct sk_buff_head rx_pkts;
};

struct mctp_usblib_test_ctx {
	struct mctp_usblib_test_dev *dev;
	struct mctp_route rt;
};

static netdev_tx_t mctp_usblib_dev_tx(struct sk_buff *skb,
				      struct net_device *ndev)
{
	/* we don't track any TXed packets at present */
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mctp_test_netdev_ops = {
	.ndo_start_xmit = mctp_usblib_dev_tx,
};

static const u16 ep_maxpacket = 512;
static const mctp_eid_t local_eid = 8;

static void mctp_usblib_dev_setup(struct net_device *ndev)
{
	ndev->type = ARPHRD_MCTP;
	ndev->mtu = 8192;
	ndev->flags = IFF_NOARP;
	ndev->netdev_ops = &mctp_test_netdev_ops;
	ndev->needs_free_netdev = true;
	ndev->pcpu_stat_type = NETDEV_PCPU_STAT_DSTATS;
}

static void mctp_usblib_test_dev_action(void *data)
{
	struct mctp_usblib_test_dev *dev = data;

	skb_queue_purge(&dev->rx_pkts);
	if (dev->mdev)
		mctp_dev_put(dev->mdev);
	unregister_netdev(dev->ndev);
}

static struct mctp_usblib_test_dev *
mctp_usblib_test_create_dev(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct net_device *ndev;
	int rc;

	ndev = alloc_netdev(sizeof(*dev), "mctptest%d", NET_NAME_ENUM,
			    mctp_usblib_dev_setup);
	if (!ndev)
		return NULL;

	dev = netdev_priv(ndev);
	dev->ndev = ndev;
	skb_queue_head_init(&dev->rx_pkts);

	rc = register_netdev(ndev);
	if (rc) {
		free_netdev(ndev);
		return NULL;
	}

	rc = kunit_add_action_or_reset(test, mctp_usblib_test_dev_action, dev);
	if (rc)
		return NULL;

	rcu_read_lock();
	dev->mdev = __mctp_dev_get(ndev);
	if (dev->mdev)
		dev->mdev->net = mctp_default_net(dev_net(ndev));
	rcu_read_unlock();

	if (!dev->mdev)
		return NULL;

	rtnl_lock();
	rc = dev_open(ndev, NULL);
	rtnl_unlock();
	if (rc)
		return NULL;

	return dev;
}

static int mctp_usblib_test_dst_output(struct mctp_dst *dst,
				       struct sk_buff *skb)
{
	struct mctp_usblib_test_dev *dev = netdev_priv(skb->dev);

	skb_queue_tail(&dev->rx_pkts, skb);

	return 0;
}

static void mctp_usblib_test_fini_action(void *data)
{
	struct mctp_usblib_test_ctx *ctx = data;

	/* The device will have been destroyed, so ->rt will be unlinked.
	 * Just ensure that the refcount is as expected.
	 */
	KUNIT_EXPECT_TRUE(current->kunit_test,
			  refcount_dec_and_test(&ctx->rt.refs));

	kfree(ctx);
}

static struct mctp_usblib_test_ctx *mctp_usblib_test_init(struct kunit *test)
{
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_route *rt;
	int rc;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	INIT_LIST_HEAD(&ctx->rt.list);
	rt = &ctx->rt;
	refcount_set(&rt->refs, 1);

	rc = kunit_add_action_or_reset(test, mctp_usblib_test_fini_action, ctx);
	KUNIT_ASSERT_EQ(test, rc, 0);

	ctx->dev = mctp_usblib_test_create_dev(test);
	KUNIT_ASSERT_NOT_NULL(test, ctx->dev);

	rt->min = local_eid;
	rt->max = local_eid;
	rt->dst_type = MCTP_ROUTE_DIRECT;
	rt->type = RTN_LOCAL;
	rt->dev = ctx->dev->mdev;
	rt->output = mctp_usblib_test_dst_output;

	rtnl_lock();
	list_add_rcu(&ctx->rt.list, &init_net.mctp.routes);
	refcount_inc(&rt->refs);
	rtnl_unlock();

	return ctx;
}

/* Init a MCTP-over-USB packet within a buffer. @len is the length of the
 * buffer to write, @payload_len is the reported size of the MCTP-over-USB
 * packet.
 */
static void mctp_usblib_test_init_pkt(void *data, size_t len,
				      size_t payload_len)
{
	struct {
		struct mctp_usb_hdr usb;
		struct mctp_hdr mctp;
	} hdr;

	hdr.usb.id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr.usb.len = cpu_to_be16(payload_len);
	hdr.mctp.ver = 1;
	hdr.mctp.dest = local_eid;
	hdr.mctp.src = 0;
	hdr.mctp.flags_seq_tag = 0;

	memcpy(data, &hdr, min(len, sizeof(hdr)));
	if (len > sizeof(hdr))
		memset(data + sizeof(hdr), 0, len - sizeof(hdr));
}

static void action_rx_fini(void *data)
{
	struct mctp_usblib_rx *rx = data;

	mctp_usblib_rx_fini(rx);
	kfree(rx);
}

static struct mctp_usblib_rx *
mctp_usblib_test_rx_init_mps(struct kunit *test, bool span, u16 maxpacket)
{
	struct mctp_usblib_rx *rx;
	int rc;

	rx = kzalloc(sizeof(*rx), GFP_KERNEL);
	if (rx) {
		rc = kunit_add_action_or_reset(test, action_rx_fini, rx);
		KUNIT_ASSERT_EQ(test, rc, 0);
	}
	KUNIT_ASSERT_NOT_NULL(test, rx);

	rc = mctp_usblib_rx_init(rx, maxpacket, span);
	KUNIT_ASSERT_EQ(test, rc, 0);

	return rx;
}

static struct mctp_usblib_rx *
mctp_usblib_test_rx_init(struct kunit *test, bool span)
{
	return mctp_usblib_test_rx_init_mps(test, span, ep_maxpacket);
}

/* Wrappers for usblib's rx_complete callback, which is intended to be called
 * from atomic context
 */
static int mctp_usblib_test_rx_complete(struct net_device *netdev,
					struct mctp_usblib_rx *rx, size_t len)
{
	int rc;

	local_bh_disable();
	rc = mctp_usblib_rx_complete(netdev, rx, len);
	local_bh_enable();

	return rc;
}

/* Single packet, starting on a transfer boundary, contained entirely within
 * the transfer
 */
static void mctp_usblib_test_rx_single(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	struct sk_buff *skb;
	size_t len;
	void *buf;
	int rc;

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;

	rx = mctp_usblib_test_rx_init(test, true);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx,
				    &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);

	/* we should always have a maxpacket of transfer available */
	KUNIT_ASSERT_GE(test, len, ep_maxpacket);

	mctp_usblib_test_init_pkt(buf, 8, 8);

	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_ASSERT_EQ(test, rc, 0);

	skb = __skb_dequeue(&dev->rx_pkts);
	KUNIT_EXPECT_NOT_NULL(test, skb);
	if (skb)
		KUNIT_EXPECT_EQ(test, skb->len, 4);
	kfree_skb(skb);
}

static void mctp_usblib_test_rx_init_validation(struct kunit *test)
{
	struct mctp_usblib_rx rx;
	int rc;

	rc = mctp_usblib_rx_init(&rx, 0, true);
	KUNIT_EXPECT_EQ(test, rc, -EINVAL);

	rc = mctp_usblib_rx_init(&rx, USB_ENDPOINT_MAXP_MASK + 1, true);
	KUNIT_EXPECT_EQ(test, rc, -EINVAL);

	rc = mctp_usblib_rx_init(&rx, 64, true);
	KUNIT_ASSERT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, rx.ep_pktlen, (u16)64);
	mctp_usblib_rx_fini(&rx);
}

static void mctp_usblib_test_rx_full_speed(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	struct sk_buff *skb;
	size_t len;
	void *buf;
	int rc;

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;
	rx = mctp_usblib_test_rx_init_mps(test, true, 64);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	KUNIT_EXPECT_GE(test, len, (size_t)MCTP_USB_1_1_PKTLEN_MAX);
	KUNIT_EXPECT_EQ(test, len % 64, (size_t)0);

	mctp_usblib_test_init_pkt(buf, 72, 72);
	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 72);
	KUNIT_ASSERT_EQ(test, rc, 0);

	skb = __skb_dequeue(&dev->rx_pkts);
	KUNIT_ASSERT_NOT_NULL(test, skb);
	KUNIT_EXPECT_EQ(test, skb->len, 68U);
	kfree_skb(skb);
}

static void mctp_usblib_test_rx_reserved_bits(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	struct mctp_usb_hdr *hdr;
	struct sk_buff *skb;
	size_t len;
	void *buf;
	int rc;

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;
	rx = mctp_usblib_test_rx_init(test, true);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	mctp_usblib_test_init_pkt(buf, 8, 8);
	hdr = buf;
	hdr->len = cpu_to_be16(0xe000 | 8);

	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_ASSERT_EQ(test, rc, 0);
	skb = __skb_dequeue(&dev->rx_pkts);
	KUNIT_ASSERT_NOT_NULL(test, skb);
	kfree_skb(skb);
}

static void mctp_usblib_test_rx_error_resets_state(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	struct mctp_usb_hdr *hdr;
	struct sk_buff *skb;
	size_t len;
	void *buf;
	int rc;

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;
	rx = mctp_usblib_test_rx_init(test, true);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	mctp_usblib_test_init_pkt(buf, 8, 8);
	hdr = buf;
	hdr->id = 0;
	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_EXPECT_EQ(test, rc, -EPROTO);
	KUNIT_EXPECT_PTR_EQ(test, rx->skb, NULL);
	KUNIT_EXPECT_EQ(test, dev->rx_pkts.qlen, 0U);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	mctp_usblib_test_init_pkt(buf, 8, 8);
	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_ASSERT_EQ(test, rc, 0);
	skb = __skb_dequeue(&dev->rx_pkts);
	KUNIT_ASSERT_NOT_NULL(test, skb);
	kfree_skb(skb);
}

static void mctp_usblib_test_rx_legacy_does_not_span(struct kunit *test)
{
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	struct sk_buff *skb;
	size_t len;
	void *buf;
	int rc;

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;
	rx = mctp_usblib_test_rx_init(test, false);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	mctp_usblib_test_init_pkt(buf, 8, 16);
	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_EXPECT_EQ(test, rc, -EPROTO);
	KUNIT_EXPECT_PTR_EQ(test, rx->skb, NULL);

	rc = mctp_usblib_rx_prepare(dev->ndev, rx, &buf, &len, GFP_KERNEL);
	KUNIT_ASSERT_EQ(test, rc, 0);
	mctp_usblib_test_init_pkt(buf, 8, 8);
	rc = mctp_usblib_test_rx_complete(dev->ndev, rx, 8);
	KUNIT_ASSERT_EQ(test, rc, 0);
	skb = __skb_dequeue(&dev->rx_pkts);
	KUNIT_ASSERT_NOT_NULL(test, skb);
	kfree_skb(skb);
}

struct mctp_usblib_test_tx {
	struct mctp_usblib_tx tx;
	struct mctp_usblib_tx_ctx *ctx;
	void *buf;
	size_t len;
};

static int mctp_usblib_test_tx_send(struct mctp_usblib_tx_ctx *tx_ctx,
				    void *data, size_t len)
{
	struct mctp_usblib_test_tx *test_tx;

	test_tx = mctp_usblib_tx_ctx_priv(tx_ctx);
	test_tx->buf = kmemdup(data, len, GFP_ATOMIC);
	if (!test_tx->buf)
		return -ENOMEM;

	test_tx->ctx = tx_ctx;
	test_tx->len = len;

	return 0;
}

static const struct mctp_usblib_tx_ops mctp_usblib_test_tx_ops = {
	.send = mctp_usblib_test_tx_send,
};

static void mctp_usblib_test_tx_lengths(struct kunit *test)
{
	const struct mctp_usblib_tx_ops *ops = &mctp_usblib_test_tx_ops;
	static const struct {
		bool span;
		size_t payload_len;
		bool send;
	} cases[] = {
		{ false, 251, true },
		{ false, 252, false },
		{ true, 252, true },
		{ true, 8187, true },
		{ true, 8188, false },
	};
	struct mctp_usblib_test_ctx *ctx;
	struct net_device *ndev;
	unsigned int i;

	ctx = mctp_usblib_test_init(test);
	ndev = ctx->dev->ndev;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct mctp_usblib_test_tx test_tx = {};
		struct mctp_usb_hdr *hdr;
		struct sk_buff *skb;
		int rc;

		mctp_usblib_tx_init(&test_tx.tx, ops, &test_tx, cases[i].span);
		skb = netdev_alloc_skb(ndev, cases[i].payload_len);
		KUNIT_ASSERT_NOT_NULL(test, skb);
		memset(skb_put(skb, cases[i].payload_len), 0,
		       cases[i].payload_len);

		rc = mctp_usblib_tx_push(ndev, &test_tx.tx, skb, false);
		KUNIT_EXPECT_EQ(test, rc, 0);
		KUNIT_EXPECT_EQ(test, !!test_tx.ctx, cases[i].send);

		if (test_tx.ctx) {
			KUNIT_ASSERT_NOT_NULL(test, test_tx.buf);
			hdr = test_tx.buf;
			KUNIT_EXPECT_EQ(test, be16_to_cpu(hdr->id),
					MCTP_USB_DMTF_ID);
			KUNIT_EXPECT_EQ(test, be16_to_cpu(hdr->len),
					cases[i].payload_len + sizeof(*hdr));
			KUNIT_EXPECT_EQ(test, test_tx.len,
					cases[i].payload_len + sizeof(*hdr));
			mctp_usblib_tx_send_complete(test_tx.ctx, ndev, true);
		}

		kfree(test_tx.buf);
		mctp_usblib_tx_fini(&test_tx.tx);
	}
}

struct mctp_usblib_test_pkt_span {
	const char *name;
	size_t n_pkts;
	size_t pkts[6];
	size_t n_xfers;
	size_t xfers[6];
};

static void
mctp_usblib_test_pkt_span_to_desc(const struct mctp_usblib_test_pkt_span *t,
				  char *desc)
{
	strscpy(desc, t->name, KUNIT_PARAM_DESC_SIZE);
}

static void
mctp_usblib_test_pkt_span_validate(struct kunit *test,
				   const struct mctp_usblib_test_pkt_span *span,
				   size_t *len)
{
	size_t pkt_len = 0, xfer_len = 0;
	unsigned int i;

	for (i = 0; i < span->n_pkts; i++) {
		KUNIT_ASSERT_GE_MSG(test, span->pkts[i], 8,
				    "pkt[%u] len too small (%zu) for %s",
				    i, span->pkts[i], span->name);
		pkt_len += span->pkts[i];
	}

	for (i = 0; i < span->n_xfers; i++)
		xfer_len += span->xfers[i];

	KUNIT_ASSERT_EQ_MSG(test, pkt_len, xfer_len,
			    "invalid pkt_len (%zu) != xfer_len (%zu) for %s",
			    pkt_len, xfer_len, span->name);

	*len = pkt_len;
}

static void mctp_usblib_test_rx_pkt_span(struct kunit *test)
{
	const struct mctp_usblib_test_pkt_span *pkt_span = test->param_value;
	size_t len, xfer_len, off, xfer_off;
	struct mctp_usblib_test_dev *dev;
	struct mctp_usblib_test_ctx *ctx;
	struct mctp_usblib_rx *rx;
	unsigned int i;
	u8 *pktbuf;
	void *buf;
	int rc;

	mctp_usblib_test_pkt_span_validate(test, pkt_span, &len);
	pktbuf = kunit_kmalloc_array(test, 1, len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pktbuf);

	/* lay out packets */
	for (off = 0, i = 0; i < pkt_span->n_pkts; i++) {
		len = pkt_span->pkts[i];
		mctp_usblib_test_init_pkt(pktbuf + off, len, len);
		off += len;
	}

	ctx = mctp_usblib_test_init(test);
	dev = ctx->dev;

	rx = mctp_usblib_test_rx_init(test, true);

	/* feed transfers */
	for (off = 0, xfer_off = 0, i = 0; i < pkt_span->n_xfers;) {
		xfer_len = pkt_span->xfers[i] - xfer_off;
		rc = mctp_usblib_rx_prepare(dev->ndev, rx,
					    &buf, &len, GFP_KERNEL);
		KUNIT_ASSERT_EQ(test, rc, 0);

		KUNIT_ASSERT_GE(test, len, ep_maxpacket);

		len = min(len, xfer_len);
		memcpy(buf, pktbuf + off, len);

		if (len == xfer_len) {
			/* whole/end xfer, proceed to next */
			xfer_off = 0;
			i++;
		} else {
			/* partial */
			xfer_off += len;
		}

		rc = mctp_usblib_test_rx_complete(dev->ndev, rx, len);
		KUNIT_ASSERT_EQ(test, rc, 0);
		off += len;
	}

	/* check received packets */
	KUNIT_EXPECT_EQ(test, dev->rx_pkts.qlen, pkt_span->n_pkts);
	for (i = 0; ; i++) {
		struct sk_buff *skb = __skb_dequeue(&dev->rx_pkts);

		if (!skb)
			break;

		if (i < pkt_span->n_pkts)
			KUNIT_EXPECT_EQ(test, skb->len, pkt_span->pkts[i] - 4);

		kfree_skb(skb);
	}
}

static const struct mctp_usblib_test_pkt_span mctp_usblib_test_pkt_spans[] = {
	/* One packet completely within a transfer */
	{ "1p1x-complete", 1, { 8 }, 1, { 8 } },
	/* Two small packets combined within one transfer */
	{ "2p1x-combined", 2, { 8, 8 }, 1, { 16 } },
	/* A packet split over two transfers, at the MCTP payload */
	{ "1p2x-split-payload", 1, { 16 }, 2, { 8, 8 } },
	/* A packet split over two transfers, at the USB transport header */
	{ "1p2x-split-usbhdr", 1, { 16 }, 2, { 2, 14 } },
	/* A packet split over two transfers, at the MCTP header */
	{ "1p2x-split-mctphdr", 1, { 16 }, 2, { 6, 10 } },
	/* Single packet split over 3 transfers, middle entirely continuation */
	{ "1p3x-split", 1, { 12 }, 3, { 4, 4, 4 } },
	/* Max-sized single transfer */
	{ "1p1x-large", 1, { 8191 }, 1, { 8191 } },
	/* Two large packets, split at the worst-case for allocation, with a
	 * single byte continuing the span
	 */
	{ "2p2x-large-split", 2, { 8190, 8190 }, 2, { 8191, 8189 } },
};

KUNIT_ARRAY_PARAM(mctp_usblib_test_rx_pkt_span, mctp_usblib_test_pkt_spans,
		  mctp_usblib_test_pkt_span_to_desc);

static struct kunit_case mctp_usblib_test_cases[] = {
	KUNIT_CASE(mctp_usblib_test_rx_single),
	KUNIT_CASE(mctp_usblib_test_rx_init_validation),
	KUNIT_CASE(mctp_usblib_test_rx_full_speed),
	KUNIT_CASE(mctp_usblib_test_rx_reserved_bits),
	KUNIT_CASE(mctp_usblib_test_rx_error_resets_state),
	KUNIT_CASE(mctp_usblib_test_rx_legacy_does_not_span),
	KUNIT_CASE_PARAM(mctp_usblib_test_rx_pkt_span,
			 mctp_usblib_test_rx_pkt_span_gen_params),
	KUNIT_CASE(mctp_usblib_test_tx_lengths),
	{}
};

static struct kunit_suite mctp_usblib_test_suite = {
	.name = "mctp-usblib",
	.test_cases = mctp_usblib_test_cases,
};

kunit_test_suite(mctp_usblib_test_suite);
