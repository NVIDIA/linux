// SPDX-License-Identifier: GPL-2.0

#include <kunit/static_stub.h>
#include <kunit/test.h>

/* keep clangd happy when compiled outside of the route.c include */
#include <net/mctp.h>
#include <net/mctpdevice.h>

#include "utils.h"

#define mctp_test_create_skb_data(h, d) \
	__mctp_test_create_skb_data(h, d, sizeof(*d))

struct mctp_frag_test {
	unsigned int mtu;
	unsigned int msgsize;
	unsigned int n_frags;
};

struct mctp_test_output_fail {
	unsigned int magic;
	struct sk_buff_head pkts;
	unsigned int calls;
	unsigned int fail_at;
	int err;
};

static const unsigned int mctp_test_output_fail_magic = 0x776d9a31;

static int mctp_test_dst_output_fail(struct mctp_dst *dst, struct sk_buff *skb)
{
	struct kunit *test = current->kunit_test;
	struct mctp_test_output_fail *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, ctx->magic, mctp_test_output_fail_magic);

	ctx->calls++;
	if (ctx->calls == ctx->fail_at) {
		kfree_skb(skb);
		return ctx->err;
	}

	skb_queue_tail(&ctx->pkts, skb);
	return 0;
}

static void mctp_test_dst_setup_fail(struct kunit *test, struct mctp_dst *dst,
				     struct mctp_test_dev *dev,
				     struct mctp_test_output_fail *ctx,
				     unsigned int mtu, unsigned int fail_at,
				     int err)
{
	memset(dst, 0, sizeof(*dst));
	memset(ctx, 0, sizeof(*ctx));

	ctx->magic = mctp_test_output_fail_magic;
	ctx->fail_at = fail_at;
	ctx->err = err;
	skb_queue_head_init(&ctx->pkts);

	dst->dev = dev->mdev;
	__mctp_dev_get(dst->dev->dev);
	dst->mtu = mtu;
	dst->output = mctp_test_dst_output_fail;
	test->priv = ctx;
}

static void mctp_test_dst_release_fail(struct mctp_dst *dst,
				       struct mctp_test_output_fail *ctx)
{
	mctp_dst_release(dst);
	skb_queue_purge(&ctx->pkts);
}

struct mctp_test_frag_alloc_fault {
	unsigned int magic;
	unsigned int alloc_calls;
	unsigned int fail_at;
	unsigned int output_calls;
	struct sk_buff_head pkts;
};

static const unsigned int mctp_test_frag_alloc_fault_magic = 0x8b25c31a;

static struct sk_buff *mctp_test_fragment_alloc_fault(unsigned int length,
						      gfp_t gfp_mask)
{
	struct kunit *test = kunit_get_current_test();
	struct mctp_test_frag_alloc_fault *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, ctx->magic, mctp_test_frag_alloc_fault_magic);

	ctx->alloc_calls++;
	if (ctx->alloc_calls == ctx->fail_at)
		return NULL;

	return alloc_skb(length, gfp_mask);
}

static int mctp_test_dst_output_count(struct mctp_dst *dst, struct sk_buff *skb)
{
	struct kunit *test = current->kunit_test;
	struct mctp_test_frag_alloc_fault *ctx = test->priv;

	KUNIT_EXPECT_EQ(test, ctx->magic, mctp_test_frag_alloc_fault_magic);

	ctx->output_calls++;
	skb_queue_tail(&ctx->pkts, skb);
	return 0;
}

static void mctp_test_dst_setup_count(struct mctp_dst *dst,
				      struct mctp_test_dev *dev,
				      struct mctp_test_frag_alloc_fault *ctx,
				      unsigned int mtu)
{
	memset(dst, 0, sizeof(*dst));
	memset(ctx, 0, sizeof(*ctx));

	ctx->magic = mctp_test_frag_alloc_fault_magic;
	skb_queue_head_init(&ctx->pkts);

	dst->dev = dev->mdev;
	__mctp_dev_get(dst->dev->dev);
	dst->mtu = mtu;
	dst->output = mctp_test_dst_output_count;
}

static void mctp_test_dst_release_count(struct mctp_dst *dst,
					struct mctp_test_frag_alloc_fault *ctx)
{
	mctp_dst_release(dst);
	skb_queue_purge(&ctx->pkts);
}

static void mctp_test_fragment(struct kunit *test)
{
	const struct mctp_frag_test *params;
	struct mctp_test_pktqueue tpq;
	int rc, i, n, mtu, msgsize;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	struct mctp_hdr hdr;
	u8 seq;

	params = test->param_value;
	mtu = params->mtu;
	msgsize = params->msgsize;

	hdr.ver = 1;
	hdr.src = 8;
	hdr.dest = 10;
	hdr.flags_seq_tag = MCTP_HDR_FLAG_TO;

	skb = mctp_test_create_skb(&hdr, msgsize);
	KUNIT_ASSERT_TRUE(test, skb);

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	mctp_test_dst_setup(test, &dst, dev, &tpq, mtu);

	rc = mctp_do_fragment_route(&dst, skb, mtu, MCTP_TAG_OWNER);
	KUNIT_EXPECT_FALSE(test, rc);

	n = tpq.pkts.qlen;

	KUNIT_EXPECT_EQ(test, n, params->n_frags);

	for (i = 0;; i++) {
		struct mctp_hdr *hdr2;
		struct sk_buff *skb2;
		u8 tag_mask, seq2;
		bool first, last;

		first = i == 0;
		last = i == (n - 1);

		skb2 = skb_dequeue(&tpq.pkts);

		if (!skb2)
			break;

		hdr2 = mctp_hdr(skb2);

		tag_mask = MCTP_HDR_TAG_MASK | MCTP_HDR_FLAG_TO;

		KUNIT_EXPECT_EQ(test, hdr2->ver, hdr.ver);
		KUNIT_EXPECT_EQ(test, hdr2->src, hdr.src);
		KUNIT_EXPECT_EQ(test, hdr2->dest, hdr.dest);
		KUNIT_EXPECT_EQ(test, hdr2->flags_seq_tag & tag_mask,
				hdr.flags_seq_tag & tag_mask);

		KUNIT_EXPECT_EQ(test,
				!!(hdr2->flags_seq_tag & MCTP_HDR_FLAG_SOM), first);
		KUNIT_EXPECT_EQ(test,
				!!(hdr2->flags_seq_tag & MCTP_HDR_FLAG_EOM), last);

		seq2 = (hdr2->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) &
			MCTP_HDR_SEQ_MASK;

		if (first) {
			seq = seq2;
		} else {
			seq++;
			KUNIT_EXPECT_EQ(test, seq2, seq & MCTP_HDR_SEQ_MASK);
		}

		if (!last)
			KUNIT_EXPECT_EQ(test, skb2->len, mtu);
		else
			KUNIT_EXPECT_LE(test, skb2->len, mtu);

		kfree_skb(skb2);
	}

	mctp_test_dst_release(&dst, &tpq);
	mctp_test_destroy_dev(dev);
}

static const struct mctp_frag_test mctp_frag_tests[] = {
	{.mtu = 68, .msgsize = 63, .n_frags = 1},
	{.mtu = 68, .msgsize = 64, .n_frags = 1},
	{.mtu = 68, .msgsize = 65, .n_frags = 2},
	{.mtu = 68, .msgsize = 66, .n_frags = 2},
	{.mtu = 68, .msgsize = 127, .n_frags = 2},
	{.mtu = 68, .msgsize = 128, .n_frags = 2},
	{.mtu = 68, .msgsize = 129, .n_frags = 3},
	{.mtu = 68, .msgsize = 130, .n_frags = 3},
};

static void mctp_frag_test_to_desc(const struct mctp_frag_test *t, char *desc)
{
	sprintf(desc, "mtu %d len %d -> %d frags",
		t->msgsize, t->mtu, t->n_frags);
}

KUNIT_ARRAY_PARAM(mctp_frag, mctp_frag_tests, mctp_frag_test_to_desc);

static void mctp_test_fragment_route_preserves_output_errno(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_output_fail ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 2, -EPERM);

	rc = mctp_do_fragment_route(&dst, skb, 68, MCTP_TAG_OWNER);
	KUNIT_EXPECT_EQ(test, rc, -EPERM);
	KUNIT_EXPECT_EQ(test, ctx.calls, 2U);
	KUNIT_EXPECT_EQ(test, skb_queue_len(&ctx.pkts), 1U);

	mctp_test_dst_release_fail(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_fragment_route_first_failure_preserves_errno(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_output_fail ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 1, -EMSGSIZE);

	kunit_info(test,
		   "KR-01 first-fragment TX proof: injecting output failure at fragment 1 for 130-byte message");
	rc = mctp_do_fragment_route(&dst, skb, 68, MCTP_TAG_OWNER);
	kunit_info(test,
		   "KR-01 first-fragment TX proof: rc=%d output_calls=%u queued_frags=%u",
		   rc, ctx.calls, skb_queue_len(&ctx.pkts));
	kunit_info(test,
		   "KR-01 first-fragment TX proof: fixed result sends no partial packet when the first fragment fails");

	KUNIT_EXPECT_EQ_MSG(test, rc, -EMSGSIZE,
			    "KR-01: first fragment failure errno was not preserved");
	KUNIT_EXPECT_EQ_MSG(test, ctx.calls, 1U,
			    "KR-01: unexpected extra TX calls after first failure");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&ctx.pkts), 0U,
			    "KR-01: partial fragment was queued after first failure");

	mctp_test_dst_release_fail(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_fragment_route_batch_preserves_output_errno(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_output_fail ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 2, -EPERM);

	rc = mctp_do_fragment_route_batch(&dst, skb, 68, MCTP_TAG_OWNER, 1, 74);
	KUNIT_EXPECT_EQ(test, rc, -EPERM);
	KUNIT_EXPECT_EQ(test, ctx.calls, 2U);
	KUNIT_EXPECT_EQ(test, skb_queue_len(&ctx.pkts), 1U);

	mctp_test_dst_release_fail(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_fragment_route_batch_first_failure_preserves_errno(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_output_fail ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 1, -EMSGSIZE);

	kunit_info(test,
		   "KR-01 batch first-fragment TX proof: injecting output failure at first batch for 130-byte message");
	rc = mctp_do_fragment_route_batch(&dst, skb, 68, MCTP_TAG_OWNER, 1, 74);
	kunit_info(test,
		   "KR-01 batch first-fragment TX proof: rc=%d output_calls=%u queued_frags=%u",
		   rc, ctx.calls, skb_queue_len(&ctx.pkts));
	kunit_info(test,
		   "KR-01 batch first-fragment TX proof: fixed result sends no partial batch when the first transfer fails");

	KUNIT_EXPECT_EQ_MSG(test, rc, -EMSGSIZE,
			    "KR-01: first batch failure errno was not preserved");
	KUNIT_EXPECT_EQ_MSG(test, ctx.calls, 1U,
			    "KR-01: unexpected extra batch TX calls after first failure");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&ctx.pkts), 0U,
			    "KR-01: partial batch was queued after first failure");

	mctp_test_dst_release_fail(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_fragment_alloc_failure_sends_no_partial(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_frag_alloc_fault ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_count(&dst, dev, &ctx, 68);
	ctx.fail_at = 2;
	test->priv = &ctx;
	kunit_activate_static_stub(test, mctp_fragment_alloc_skb,
				   mctp_test_fragment_alloc_fault);

	kunit_info(test,
		   "KR-01 alloc-fault proof: fail fragment alloc #%u for 130-byte message at mtu=%u",
		   ctx.fail_at, dst.mtu);
	rc = mctp_do_fragment_route(&dst, skb, 68, MCTP_TAG_OWNER);
	kunit_deactivate_static_stub(test, mctp_fragment_alloc_skb);
	kunit_info(test,
		   "KR-01 alloc-fault proof: rc=%d alloc_calls=%u output_calls=%u queued_frags=%u",
		   rc, ctx.alloc_calls, ctx.output_calls,
		   skb_queue_len(&ctx.pkts));
	kunit_info(test,
		   "KR-01 alloc-fault proof: fixed result is -ENOMEM with output_calls=0, proving no partial message was transmitted");

	KUNIT_EXPECT_EQ_MSG(test, rc, -ENOMEM,
			    "KR-01: allocation failure errno was not preserved");
	KUNIT_EXPECT_EQ_MSG(test, ctx.alloc_calls, 2U,
			    "KR-01: test did not inject the intended allocation failure");
	KUNIT_EXPECT_EQ_MSG(test, ctx.output_calls, 0U,
			    "KR-01: fragment output occurred before all fragments were allocated");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&ctx.pkts), 0U,
			    "KR-01: partial fragment escaped on allocation failure");

	mctp_test_dst_release_count(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_fragment_batch_alloc_failure_sends_no_partial(struct kunit *test)
{
	struct mctp_hdr hdr = { .ver = 1, .src = 8, .dest = 10 };
	struct mctp_test_frag_alloc_fault ctx;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dev->mdev->tx_batching_enabled = true;
	dev->mdev->tx_batch_hdr_len = 1;
	dev->mdev->tx_batch_max_xfer = 74;

	skb = mctp_test_create_skb(&hdr, 130);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_dst_setup_count(&dst, dev, &ctx, 68);
	ctx.fail_at = 2;
	test->priv = &ctx;
	kunit_activate_static_stub(test, mctp_fragment_alloc_skb,
				   mctp_test_fragment_alloc_fault);

	kunit_info(test,
		   "KR-01 batch alloc-fault proof: fail batch alloc #%u for 130-byte message at mtu=%u max_xfer=%u",
		   ctx.fail_at, dst.mtu, dev->mdev->tx_batch_max_xfer);
	rc = mctp_do_fragment_route(&dst, skb, 68, MCTP_TAG_OWNER);
	kunit_deactivate_static_stub(test, mctp_fragment_alloc_skb);
	kunit_info(test,
		   "KR-01 batch alloc-fault proof: rc=%d alloc_calls=%u output_calls=%u queued_batches=%u",
		   rc, ctx.alloc_calls, ctx.output_calls,
		   skb_queue_len(&ctx.pkts));
	kunit_info(test,
		   "KR-01 batch alloc-fault proof: fixed result is -ENOMEM with output_calls=0, proving no partial batch was transmitted");

	KUNIT_EXPECT_EQ_MSG(test, rc, -ENOMEM,
			    "KR-01: batch allocation failure errno was not preserved");
	KUNIT_EXPECT_EQ_MSG(test, ctx.alloc_calls, 2U,
			    "KR-01: batch test did not inject the intended allocation failure");
	KUNIT_EXPECT_EQ_MSG(test, ctx.output_calls, 0U,
			    "KR-01: batch output occurred before all batches were allocated");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&ctx.pkts), 0U,
			    "KR-01: partial batch escaped on allocation failure");

	mctp_test_dst_release_count(&dst, &ctx);
	mctp_test_destroy_dev(dev);
}

struct mctp_rx_input_test {
	struct mctp_hdr hdr;
	bool input;
};

static void mctp_test_rx_input(struct kunit *test)
{
	const struct mctp_rx_input_test *params;
	struct mctp_test_pktqueue tpq;
	struct mctp_test_route *rt;
	struct mctp_test_dev *dev;
	struct sk_buff *skb;

	params = test->param_value;
	test->priv = &tpq;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	rt = mctp_test_create_route_direct(&init_net, dev->mdev, 8, 68);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt);

	skb = mctp_test_create_skb(&params->hdr, 1);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_pktqueue_init(&tpq);

	mctp_pkttype_receive(skb, dev->ndev, &mctp_packet_type, NULL);

	KUNIT_EXPECT_EQ(test, !!tpq.pkts.qlen, params->input);

	skb_queue_purge(&tpq.pkts);
	mctp_test_route_destroy(test, rt);
	mctp_test_destroy_dev(dev);
}

#define RX_HDR(_ver, _src, _dest, _fst) \
	{ .ver = _ver, .src = _src, .dest = _dest, .flags_seq_tag = _fst }

/* we have a route for EID 8 only */
static const struct mctp_rx_input_test mctp_rx_input_tests[] = {
	{ .hdr = RX_HDR(1, 10, 8, 0), .input = true },
	{ .hdr = RX_HDR(1, 10, 9, 0), .input = false }, /* no input route */
	{ .hdr = RX_HDR(2, 10, 8, 0), .input = false }, /* invalid version */
};

static void mctp_rx_input_test_to_desc(const struct mctp_rx_input_test *t,
				       char *desc)
{
	sprintf(desc, "{%x,%x,%x,%x}", t->hdr.ver, t->hdr.src, t->hdr.dest,
		t->hdr.flags_seq_tag);
}

KUNIT_ARRAY_PARAM(mctp_rx_input, mctp_rx_input_tests,
		  mctp_rx_input_test_to_desc);

/* set up a local dev, route on EID 8, and a socket listening on type 0 */
static void __mctp_route_test_init(struct kunit *test,
				   struct mctp_test_dev **devp,
				   struct mctp_dst *dst,
				   struct mctp_test_pktqueue *tpq,
				   struct socket **sockp,
				   unsigned int netid)
{
	struct sockaddr_mctp addr = {0};
	struct mctp_test_dev *dev;
	struct socket *sock;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	if (netid != MCTP_NET_ANY)
		WRITE_ONCE(dev->mdev->net, netid);

	mctp_test_dst_setup(test, dst, dev, tpq, 68);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	addr.smctp_family = AF_MCTP;
	addr.smctp_network = netid;
	addr.smctp_addr.s_addr = 8;
	addr.smctp_type = 0;
	rc = kernel_bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	KUNIT_ASSERT_EQ(test, rc, 0);

	*devp = dev;
	*sockp = sock;
}

static void __mctp_route_test_fini(struct kunit *test,
				   struct mctp_test_dev *dev,
				   struct mctp_dst *dst,
				   struct mctp_test_pktqueue *tpq,
				   struct socket *sock)
{
	sock_release(sock);
	mctp_test_dst_release(dst, tpq);
	mctp_test_destroy_dev(dev);
}

struct mctp_route_input_sk_test {
	struct mctp_hdr hdr;
	u8 type;
	bool deliver;
};

static void mctp_test_route_input_sk(struct kunit *test)
{
	const struct mctp_route_input_sk_test *params;
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *skb2;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct socket *sock;
	int rc;

	params = test->param_value;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	skb = mctp_test_create_skb_data(&params->hdr, &params->type);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_skb_set_dev(skb, dev);
	mctp_test_pktqueue_init(&tpq);

	rc = mctp_dst_input(&dst, skb);

	if (params->deliver) {
		KUNIT_EXPECT_EQ(test, rc, 0);

		skb2 = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
		KUNIT_EXPECT_NOT_ERR_OR_NULL(test, skb2);
		KUNIT_EXPECT_EQ(test, skb2->len, 1);

		skb_free_datagram(sock->sk, skb2);

	} else {
		KUNIT_EXPECT_NE(test, rc, 0);
		skb2 = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
		KUNIT_EXPECT_NULL(test, skb2);
	}

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

#define FL_S	(MCTP_HDR_FLAG_SOM)
#define FL_E	(MCTP_HDR_FLAG_EOM)
#define FL_TO	(MCTP_HDR_FLAG_TO)
#define FL_T(t)	((t) & MCTP_HDR_TAG_MASK)

static const struct mctp_route_input_sk_test mctp_route_input_sk_tests[] = {
	{ .hdr = RX_HDR(1, 10, 8, FL_S | FL_E | FL_TO), .type = 0, .deliver = true },
	{ .hdr = RX_HDR(1, 10, 8, FL_S | FL_E | FL_TO), .type = 1, .deliver = false },
	{ .hdr = RX_HDR(1, 10, 8, FL_S | FL_E), .type = 0, .deliver = false },
	{ .hdr = RX_HDR(1, 10, 8, FL_E | FL_TO), .type = 0, .deliver = false },
	{ .hdr = RX_HDR(1, 10, 8, FL_TO), .type = 0, .deliver = false },
	{ .hdr = RX_HDR(1, 10, 8, 0), .type = 0, .deliver = false },
};

static void mctp_route_input_sk_to_desc(const struct mctp_route_input_sk_test *t,
					char *desc)
{
	sprintf(desc, "{%x,%x,%x,%x} type %d", t->hdr.ver, t->hdr.src,
		t->hdr.dest, t->hdr.flags_seq_tag, t->type);
}

KUNIT_ARRAY_PARAM(mctp_route_input_sk, mctp_route_input_sk_tests,
		  mctp_route_input_sk_to_desc);

struct mctp_route_input_sk_reasm_test {
	const char *name;
	struct mctp_hdr hdrs[4];
	int n_hdrs;
	int rx_len;
};

static void mctp_test_route_input_sk_reasm(struct kunit *test)
{
	const struct mctp_route_input_sk_reasm_test *params;
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *skb2;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct socket *sock;
	int i, rc;
	u8 c;

	params = test->param_value;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	for (i = 0; i < params->n_hdrs; i++) {
		c = i;
		skb = mctp_test_create_skb_data(&params->hdrs[i], &c);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

		mctp_test_skb_set_dev(skb, dev);

		rc = mctp_dst_input(&dst, skb);
	}

	skb2 = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);

	if (params->rx_len) {
		KUNIT_EXPECT_NOT_ERR_OR_NULL(test, skb2);
		KUNIT_EXPECT_EQ(test, skb2->len, params->rx_len);
		skb_free_datagram(sock->sk, skb2);

	} else {
		KUNIT_EXPECT_NULL(test, skb2);
	}

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

#define RX_FRAG(f, s) RX_HDR(1, 10, 8, FL_TO | (f) | ((s) << MCTP_HDR_SEQ_SHIFT))

static const struct mctp_route_input_sk_reasm_test mctp_route_input_sk_reasm_tests[] = {
	{
		.name = "single packet",
		.hdrs = {
			RX_FRAG(FL_S | FL_E, 0),
		},
		.n_hdrs = 1,
		.rx_len = 1,
	},
	{
		.name = "single packet, offset seq",
		.hdrs = {
			RX_FRAG(FL_S | FL_E, 1),
		},
		.n_hdrs = 1,
		.rx_len = 1,
	},
	{
		.name = "start & end packets",
		.hdrs = {
			RX_FRAG(FL_S, 0),
			RX_FRAG(FL_E, 1),
		},
		.n_hdrs = 2,
		.rx_len = 2,
	},
	{
		.name = "start & end packets, offset seq",
		.hdrs = {
			RX_FRAG(FL_S, 1),
			RX_FRAG(FL_E, 2),
		},
		.n_hdrs = 2,
		.rx_len = 2,
	},
	{
		.name = "start & end packets, out of order",
		.hdrs = {
			RX_FRAG(FL_E, 1),
			RX_FRAG(FL_S, 0),
		},
		.n_hdrs = 2,
		.rx_len = 0,
	},
	{
		.name = "start, middle & end packets",
		.hdrs = {
			RX_FRAG(FL_S, 0),
			RX_FRAG(0,    1),
			RX_FRAG(FL_E, 2),
		},
		.n_hdrs = 3,
		.rx_len = 3,
	},
	{
		.name = "missing seq",
		.hdrs = {
			RX_FRAG(FL_S, 0),
			RX_FRAG(FL_E, 2),
		},
		.n_hdrs = 2,
		.rx_len = 0,
	},
	{
		.name = "seq wrap",
		.hdrs = {
			RX_FRAG(FL_S, 3),
			RX_FRAG(FL_E, 0),
		},
		.n_hdrs = 2,
		.rx_len = 2,
	},
};

static void mctp_route_input_sk_reasm_to_desc(
				const struct mctp_route_input_sk_reasm_test *t,
				char *desc)
{
	sprintf(desc, "%s", t->name);
}

KUNIT_ARRAY_PARAM(mctp_route_input_sk_reasm, mctp_route_input_sk_reasm_tests,
		  mctp_route_input_sk_reasm_to_desc);

static void mctp_test_route_input_duplicate_som_restarts_reasm(struct kunit *test)
{
	const struct mctp_hdr hdrs[] = {
		RX_FRAG(FL_S, 0),
		RX_FRAG(FL_S, 0),
		RX_FRAG(FL_E, 1),
	};
	const u8 data0[] = { 0, 0xa1 };
	const u8 data1[] = { 0, 0xb1 };
	const u8 data2[] = { 0xb2 };
	const u8 expected[] = { 0, 0xb1, 0xb2 };
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *rx_skb;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct socket *sock;
	u8 actual[3];
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	skb = __mctp_test_create_skb_data(&hdrs[0], data0, sizeof(data0));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 duplicate-SOM proof: starting old reassembly with payload byte 0x%02x",
		   data0[1]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	skb = __mctp_test_create_skb_data(&hdrs[1], data1, sizeof(data1));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 duplicate-SOM proof: injecting second SOM with same key and new payload byte 0x%02x",
		   data1[1]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	skb = __mctp_test_create_skb_data(&hdrs[2], data2, sizeof(data2));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 duplicate-SOM proof: completing the new reassembly with EOM byte 0x%02x",
		   data2[0]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	rx_skb = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rx_skb);
	kunit_info(test,
		   "KR-03 duplicate-SOM proof: delivered_len=%u expected_new_len=%zu",
		   rx_skb->len, sizeof(expected));
	KUNIT_EXPECT_EQ(test, rx_skb->len, (unsigned int)sizeof(expected));

	rc = skb_copy_bits(rx_skb, 0, actual, sizeof(actual));
	KUNIT_ASSERT_EQ(test, rc, 0);
	kunit_info(test,
		   "KR-03 duplicate-SOM proof: delivered bytes=%02x %02x %02x; fixed result keeps the new SOM and drops the old partial",
		   actual[0], actual[1], actual[2]);
	KUNIT_EXPECT_MEMEQ_MSG(test, actual, expected, sizeof(expected),
			       "KR-03: duplicate SOM did not restart reassembly with the new message");
	skb_free_datagram(sock->sk, rx_skb);

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static void mctp_test_route_input_duplicate_som_drops_old_middle(struct kunit *test)
{
	const struct mctp_hdr hdrs[] = {
		RX_FRAG(FL_S, 0),
		RX_FRAG(0, 1),
		RX_FRAG(FL_S, 2),
		RX_FRAG(FL_E, 3),
	};
	const u8 old_start[] = { 0, 0xa1 };
	const u8 old_mid[] = { 0xa2 };
	const u8 new_start[] = { 0, 0xb1 };
	const u8 new_end[] = { 0xb2 };
	const u8 expected[] = { 0, 0xb1, 0xb2 };
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *rx_skb;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct socket *sock;
	u8 actual[3];
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	skb = __mctp_test_create_skb_data(&hdrs[0], old_start, sizeof(old_start));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 old-middle proof: starting old reassembly byte=0x%02x",
		   old_start[1]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	skb = __mctp_test_create_skb_data(&hdrs[1], old_mid, sizeof(old_mid));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 old-middle proof: queuing old middle byte=0x%02x before duplicate SOM",
		   old_mid[0]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	skb = __mctp_test_create_skb_data(&hdrs[2], new_start, sizeof(new_start));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 old-middle proof: injecting duplicate SOM with new byte=0x%02x",
		   new_start[1]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	skb = __mctp_test_create_skb_data(&hdrs[3], new_end, sizeof(new_end));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);
	kunit_info(test,
		   "KR-03 old-middle proof: completing replacement reassembly with byte=0x%02x",
		   new_end[0]);
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	rx_skb = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rx_skb);
	kunit_info(test,
		   "KR-03 old-middle proof: delivered_len=%u expected_new_len=%zu",
		   rx_skb->len, sizeof(expected));
	KUNIT_EXPECT_EQ(test, rx_skb->len, (unsigned int)sizeof(expected));

	rc = skb_copy_bits(rx_skb, 0, actual, sizeof(actual));
	KUNIT_ASSERT_EQ(test, rc, 0);
	kunit_info(test,
		   "KR-03 old-middle proof: delivered bytes=%02x %02x %02x; fixed result drops old middle byte 0x%02x",
		   actual[0], actual[1], actual[2], old_mid[0]);
	KUNIT_EXPECT_MEMEQ_MSG(test, actual, expected, sizeof(expected),
			       "KR-03: replacement reassembly included stale old fragments");
	skb_free_datagram(sock->sk, rx_skb);

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

struct mctp_route_input_sk_keys_test {
	const char	*name;
	mctp_eid_t	key_peer_addr;
	mctp_eid_t	key_local_addr;
	u8		key_tag;
	struct mctp_hdr hdr;
	bool		deliver;
};

/* test packet rx in the presence of various key configurations */
static void mctp_test_route_input_sk_keys(struct kunit *test)
{
	const struct mctp_route_input_sk_keys_test *params;
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *skb2;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct netns_mctp *mns;
	struct mctp_sock *msk;
	struct socket *sock;
	unsigned long flags;
	struct mctp_dst dst;
	unsigned int net;
	int rc;
	u8 c;

	params = test->param_value;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	net = READ_ONCE(dev->mdev->net);

	mctp_test_dst_setup(test, &dst, dev, &tpq, 68);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	msk = container_of(sock->sk, struct mctp_sock, sk);
	mns = &sock_net(sock->sk)->mctp;

	/* set the incoming tag according to test params */
	key = mctp_key_alloc(msk, net, params->key_local_addr,
			     params->key_peer_addr, params->key_tag,
			     GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);

	spin_lock_irqsave(&mns->keys_lock, flags);
	mctp_reserve_tag(&init_net, key, msk, MCTP_DEFAULT_LIFETIME);
	spin_unlock_irqrestore(&mns->keys_lock, flags);

	/* create packet and route */
	c = 0;
	skb = mctp_test_create_skb_data(&params->hdr, &c);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	mctp_test_skb_set_dev(skb, dev);

	rc = mctp_dst_input(&dst, skb);

	/* (potentially) receive message */
	skb2 = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);

	if (params->deliver)
		KUNIT_EXPECT_NOT_ERR_OR_NULL(test, skb2);
	else
		KUNIT_EXPECT_PTR_EQ(test, skb2, NULL);

	if (skb2)
		skb_free_datagram(sock->sk, skb2);

	mctp_key_unref(key);
	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static const struct mctp_route_input_sk_keys_test mctp_route_input_sk_keys_tests[] = {
	{
		.name = "direct match",
		.key_peer_addr = 9,
		.key_local_addr = 8,
		.key_tag = 1,
		.hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(1)),
		.deliver = true,
	},
	{
		.name = "flipped src/dest",
		.key_peer_addr = 8,
		.key_local_addr = 9,
		.key_tag = 1,
		.hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(1)),
		.deliver = false,
	},
	{
		.name = "peer addr mismatch",
		.key_peer_addr = 9,
		.key_local_addr = 8,
		.key_tag = 1,
		.hdr = RX_HDR(1, 10, 8, FL_S | FL_E | FL_T(1)),
		.deliver = false,
	},
	{
		.name = "tag value mismatch",
		.key_peer_addr = 9,
		.key_local_addr = 8,
		.key_tag = 1,
		.hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(2)),
		.deliver = false,
	},
	{
		.name = "TO mismatch",
		.key_peer_addr = 9,
		.key_local_addr = 8,
		.key_tag = 1,
		.hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(1) | FL_TO),
		.deliver = false,
	},
	{
		.name = "broadcast response",
		.key_peer_addr = MCTP_ADDR_ANY,
		.key_local_addr = 8,
		.key_tag = 1,
		.hdr = RX_HDR(1, 11, 8, FL_S | FL_E | FL_T(1)),
		.deliver = true,
	},
	{
		.name = "any local match",
		.key_peer_addr = 12,
		.key_local_addr = MCTP_ADDR_ANY,
		.key_tag = 1,
		.hdr = RX_HDR(1, 12, 8, FL_S | FL_E | FL_T(1)),
		.deliver = true,
	},
};

static void mctp_route_input_sk_keys_to_desc(
				const struct mctp_route_input_sk_keys_test *t,
				char *desc)
{
	sprintf(desc, "%s", t->name);
}

KUNIT_ARRAY_PARAM(mctp_route_input_sk_keys, mctp_route_input_sk_keys_tests,
		  mctp_route_input_sk_keys_to_desc);

struct test_net {
	unsigned int netid;
	struct mctp_test_dev *dev;
	struct mctp_test_pktqueue tpq;
	struct mctp_dst dst;
	struct socket *sock;
	struct sk_buff *skb;
	struct mctp_sk_key *key;
	struct {
		u8 type;
		unsigned int data;
	} msg;
};

static void
mctp_test_route_input_multiple_nets_bind_init(struct kunit *test,
					      struct test_net *t)
{
	struct mctp_hdr hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(1) | FL_TO);

	t->msg.data = t->netid;

	__mctp_route_test_init(test, &t->dev, &t->dst, &t->tpq, &t->sock,
			       t->netid);

	t->skb = mctp_test_create_skb_data(&hdr, &t->msg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->skb);
	mctp_test_skb_set_dev(t->skb, t->dev);
	mctp_test_pktqueue_init(&t->tpq);
}

static void
mctp_test_route_input_multiple_nets_bind_fini(struct kunit *test,
					      struct test_net *t)
{
	__mctp_route_test_fini(test, t->dev, &t->dst, &t->tpq, t->sock);
}

/* Test that skbs from different nets (otherwise identical) get routed to their
 * corresponding socket via the sockets' bind()
 */
static void mctp_test_route_input_multiple_nets_bind(struct kunit *test)
{
	struct sk_buff *rx_skb1, *rx_skb2;
	struct test_net t1, t2;
	int rc;

	t1.netid = 1;
	t2.netid = 2;

	t1.msg.type = 0;
	t2.msg.type = 0;

	mctp_test_route_input_multiple_nets_bind_init(test, &t1);
	mctp_test_route_input_multiple_nets_bind_init(test, &t2);

	rc = mctp_dst_input(&t1.dst, t1.skb);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = mctp_dst_input(&t2.dst, t2.skb);
	KUNIT_ASSERT_EQ(test, rc, 0);

	rx_skb1 = skb_recv_datagram(t1.sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, rx_skb1);
	KUNIT_EXPECT_EQ(test, rx_skb1->len, sizeof(t1.msg));
	KUNIT_EXPECT_EQ(test,
			*(unsigned int *)skb_pull(rx_skb1, sizeof(t1.msg.data)),
			t1.netid);
	kfree_skb(rx_skb1);

	rx_skb2 = skb_recv_datagram(t2.sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, rx_skb2);
	KUNIT_EXPECT_EQ(test, rx_skb2->len, sizeof(t2.msg));
	KUNIT_EXPECT_EQ(test,
			*(unsigned int *)skb_pull(rx_skb2, sizeof(t2.msg.data)),
			t2.netid);
	kfree_skb(rx_skb2);

	mctp_test_route_input_multiple_nets_bind_fini(test, &t1);
	mctp_test_route_input_multiple_nets_bind_fini(test, &t2);
}

static void
mctp_test_route_input_multiple_nets_key_init(struct kunit *test,
					     struct test_net *t)
{
	struct mctp_hdr hdr = RX_HDR(1, 9, 8, FL_S | FL_E | FL_T(1));
	struct mctp_sock *msk;
	struct netns_mctp *mns;
	unsigned long flags;

	t->msg.data = t->netid;

	__mctp_route_test_init(test, &t->dev, &t->dst, &t->tpq, &t->sock,
			       t->netid);

	msk = container_of(t->sock->sk, struct mctp_sock, sk);

	t->key = mctp_key_alloc(msk, t->netid, hdr.dest, hdr.src, 1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->key);

	mns = &sock_net(t->sock->sk)->mctp;
	spin_lock_irqsave(&mns->keys_lock, flags);
	mctp_reserve_tag(&init_net, t->key, msk, MCTP_DEFAULT_LIFETIME);
	spin_unlock_irqrestore(&mns->keys_lock, flags);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->key);
	t->skb = mctp_test_create_skb_data(&hdr, &t->msg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->skb);
	mctp_test_skb_set_dev(t->skb, t->dev);
}

static void
mctp_test_route_input_multiple_nets_key_fini(struct kunit *test,
					     struct test_net *t)
{
	mctp_key_unref(t->key);
	__mctp_route_test_fini(test, t->dev, &t->dst, &t->tpq, t->sock);
}

/* test that skbs from different nets (otherwise identical) get routed to their
 * corresponding socket via the sk_key
 */
static void mctp_test_route_input_multiple_nets_key(struct kunit *test)
{
	struct sk_buff *rx_skb1, *rx_skb2;
	struct test_net t1, t2;
	int rc;

	t1.netid = 1;
	t2.netid = 2;

	/* use type 1 which is not bound */
	t1.msg.type = 1;
	t2.msg.type = 1;

	mctp_test_route_input_multiple_nets_key_init(test, &t1);
	mctp_test_route_input_multiple_nets_key_init(test, &t2);

	rc = mctp_dst_input(&t1.dst, t1.skb);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = mctp_dst_input(&t2.dst, t2.skb);
	KUNIT_ASSERT_EQ(test, rc, 0);

	rx_skb1 = skb_recv_datagram(t1.sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, rx_skb1);
	KUNIT_EXPECT_EQ(test, rx_skb1->len, sizeof(t1.msg));
	KUNIT_EXPECT_EQ(test,
			*(unsigned int *)skb_pull(rx_skb1, sizeof(t1.msg.data)),
			t1.netid);
	kfree_skb(rx_skb1);

	rx_skb2 = skb_recv_datagram(t2.sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, rx_skb2);
	KUNIT_EXPECT_EQ(test, rx_skb2->len, sizeof(t2.msg));
	KUNIT_EXPECT_EQ(test,
			*(unsigned int *)skb_pull(rx_skb2, sizeof(t2.msg.data)),
			t2.netid);
	kfree_skb(rx_skb2);

	mctp_test_route_input_multiple_nets_key_fini(test, &t1);
	mctp_test_route_input_multiple_nets_key_fini(test, &t2);
}

/* Input route to socket, using a single-packet message, where sock delivery
 * fails. Ensure we're handling the failure appropriately.
 */
static void mctp_test_route_input_sk_fail_single(struct kunit *test)
{
	const struct mctp_hdr hdr = RX_HDR(1, 10, 8, FL_S | FL_E | FL_TO);
	struct mctp_test_pktqueue tpq;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct socket *sock;
	struct sk_buff *skb;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	/* No rcvbuf space, so delivery should fail. __sock_set_rcvbuf will
	 * clamp the minimum to SOCK_MIN_RCVBUF, so we open-code this.
	 */
	lock_sock(sock->sk);
	WRITE_ONCE(sock->sk->sk_rcvbuf, 0);
	release_sock(sock->sk);

	skb = mctp_test_create_skb(&hdr, 10);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	skb_get(skb);

	mctp_test_skb_set_dev(skb, dev);

	/* do route input, which should fail */
	rc = mctp_dst_input(&dst, skb);
	KUNIT_EXPECT_NE(test, rc, 0);

	/* we should hold the only reference to skb */
	KUNIT_EXPECT_EQ(test, refcount_read(&skb->users), 1);
	kfree_skb(skb);

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

/* Input route to socket, using a fragmented message, where sock delivery fails.
 */
static void mctp_test_route_input_sk_fail_frag(struct kunit *test)
{
	const struct mctp_hdr hdrs[2] = { RX_FRAG(FL_S, 0), RX_FRAG(FL_E, 1) };
	struct mctp_test_pktqueue tpq;
	struct mctp_test_dev *dev;
	struct sk_buff *skbs[2];
	struct mctp_dst dst;
	struct socket *sock;
	unsigned int i;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	lock_sock(sock->sk);
	WRITE_ONCE(sock->sk->sk_rcvbuf, 0);
	release_sock(sock->sk);

	for (i = 0; i < ARRAY_SIZE(skbs); i++) {
		skbs[i] = mctp_test_create_skb(&hdrs[i], 10);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skbs[i]);
		skb_get(skbs[i]);

		mctp_test_skb_set_dev(skbs[i], dev);
	}

	/* first route input should succeed, we're only queueing to the
	 * frag list
	 */
	rc = mctp_dst_input(&dst, skbs[0]);
	KUNIT_EXPECT_EQ(test, rc, 0);

	/* final route input should fail to deliver to the socket */
	rc = mctp_dst_input(&dst, skbs[1]);
	KUNIT_EXPECT_NE(test, rc, 0);

	/* we should hold the only reference to both skbs */
	KUNIT_EXPECT_EQ(test, refcount_read(&skbs[0]->users), 1);
	kfree_skb(skbs[0]);

	KUNIT_EXPECT_EQ(test, refcount_read(&skbs[1]->users), 1);
	kfree_skb(skbs[1]);

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

/* Input route to socket, using a fragmented message created from clones.
 */
static void mctp_test_route_input_cloned_frag(struct kunit *test)
{
	/* 5 packet fragments, forming 2 complete messages */
	const struct mctp_hdr hdrs[5] = {
		RX_FRAG(FL_S, 0),
		RX_FRAG(0, 1),
		RX_FRAG(FL_E, 2),
		RX_FRAG(FL_S, 0),
		RX_FRAG(FL_E, 1),
	};
	const size_t data_len = 3; /* arbitrary */
	u8 compare[3 * ARRAY_SIZE(hdrs)];
	u8 flat[3 * ARRAY_SIZE(hdrs)];
	struct mctp_test_pktqueue tpq;
	struct mctp_test_dev *dev;
	struct sk_buff *skb[5];
	struct sk_buff *rx_skb;
	struct mctp_dst dst;
	struct socket *sock;
	size_t total;
	void *p;
	int rc;

	total = data_len + sizeof(struct mctp_hdr);

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	/* Create a single skb initially with concatenated packets */
	skb[0] = mctp_test_create_skb(&hdrs[0], 5 * total);
	mctp_test_skb_set_dev(skb[0], dev);
	memset(skb[0]->data, 0 * 0x11, skb[0]->len);
	memcpy(skb[0]->data, &hdrs[0], sizeof(struct mctp_hdr));

	/* Extract and populate packets */
	for (int i = 1; i < 5; i++) {
		skb[i] = skb_clone(skb[i - 1], GFP_ATOMIC);
		KUNIT_ASSERT_TRUE(test, skb[i]);
		p = skb_pull(skb[i], total);
		KUNIT_ASSERT_TRUE(test, p);
		skb_reset_network_header(skb[i]);
		memcpy(skb[i]->data, &hdrs[i], sizeof(struct mctp_hdr));
		memset(&skb[i]->data[sizeof(struct mctp_hdr)], i * 0x11, data_len);
	}
	for (int i = 0; i < 5; i++)
		skb_trim(skb[i], total);

	/* SOM packets have a type byte to match the socket */
	skb[0]->data[4] = 0;
	skb[3]->data[4] = 0;

	skb_dump("pkt1 ", skb[0], false);
	skb_dump("pkt2 ", skb[1], false);
	skb_dump("pkt3 ", skb[2], false);
	skb_dump("pkt4 ", skb[3], false);
	skb_dump("pkt5 ", skb[4], false);

	for (int i = 0; i < 5; i++) {
		KUNIT_EXPECT_EQ(test, refcount_read(&skb[i]->users), 1);
		/* Take a reference so we can check refcounts at the end */
		skb_get(skb[i]);
	}

	/* Feed the fragments into MCTP core */
	for (int i = 0; i < 5; i++) {
		rc = mctp_dst_input(&dst, skb[i]);
		KUNIT_EXPECT_EQ(test, rc, 0);
	}

	/* Receive first reassembled message */
	rx_skb = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, rx_skb->len, 3 * data_len);
	rc = skb_copy_bits(rx_skb, 0, flat, rx_skb->len);
	for (int i = 0; i < rx_skb->len; i++)
		compare[i] = (i / data_len) * 0x11;
	/* Set type byte */
	compare[0] = 0;

	KUNIT_EXPECT_MEMEQ(test, flat, compare, rx_skb->len);
	KUNIT_EXPECT_EQ(test, refcount_read(&rx_skb->users), 1);
	kfree_skb(rx_skb);

	/* Receive second reassembled message */
	rx_skb = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_EXPECT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, rx_skb->len, 2 * data_len);
	rc = skb_copy_bits(rx_skb, 0, flat, rx_skb->len);
	for (int i = 0; i < rx_skb->len; i++)
		compare[i] = (i / data_len + 3) * 0x11;
	/* Set type byte */
	compare[0] = 0;

	KUNIT_EXPECT_MEMEQ(test, flat, compare, rx_skb->len);
	KUNIT_EXPECT_EQ(test, refcount_read(&rx_skb->users), 1);
	kfree_skb(rx_skb);

	/* Check input skb refcounts */
	for (int i = 0; i < 5; i++) {
		KUNIT_EXPECT_EQ(test, refcount_read(&skb[i]->users), 1);
		kfree_skb(skb[i]);
	}

	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static struct sk_buff *mctp_test_payload_skb(struct kunit *test,
					     unsigned int len)
{
	struct sk_buff *skb;
	u8 *buf;
	unsigned int i;

	skb = alloc_skb(sizeof(struct mctp_hdr) + len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	__mctp_cb(skb);
	skb_reserve(skb, sizeof(struct mctp_hdr));

	buf = skb_put(skb, len);
	if (!len)
		return skb;

	buf[0] = 0x05;
	for (i = 1; i < len; i++)
		buf[i] = i & 0xff;

	return skb;
}

static void mctp_test_remove_socket_keys(struct mctp_sock *msk)
{
	struct net *net = sock_net(&msk->sk);
	struct mctp_sk_key *key;
	struct hlist_node *tmp;
	unsigned long flags, fl2;

	spin_lock_irqsave(&net->mctp.keys_lock, flags);
	hlist_for_each_entry_safe(key, tmp, &msk->keys, sklist) {
		struct sk_buff *skb;

		spin_lock_irqsave(&key->lock, fl2);
		skb = key->reasm_head;
		key->reasm_head = NULL;
		key->reasm_tailp = NULL;
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

#if IS_ENABLED(CONFIG_MCTP_FLOWS)

static void mctp_test_flow_init(struct kunit *test,
				struct mctp_test_dev **devp,
				struct mctp_dst *dst,
				struct mctp_test_pktqueue *tpq,
				struct socket **sock,
				struct sk_buff **skbp,
				unsigned int len)
{
	struct mctp_test_dev *dev;
	struct sk_buff *skb;

	/* we have a slightly odd routing setup here; the test route
	 * is for EID 8, which is our local EID. We don't do a routing
	 * lookup, so that's fine - all we require is a path through
	 * mctp_local_output, which will call dst->output on whatever
	 * route we provide
	 */
	__mctp_route_test_init(test, &dev, dst, tpq, sock, MCTP_NET_ANY);

	/* Assign a single EID. ->addrs is freed on mctp netdev release */
	dev->mdev->addrs = kmalloc(sizeof(u8), GFP_KERNEL);
	dev->mdev->num_addrs = 1;
	dev->mdev->addrs[0] = 8;

	skb = alloc_skb(len + sizeof(struct mctp_hdr) + 1, GFP_KERNEL);
	KUNIT_ASSERT_TRUE(test, skb);
	__mctp_cb(skb);
	skb_reserve(skb, sizeof(struct mctp_hdr) + 1);
	memset(skb_put(skb, len), 0, len);


	*devp = dev;
	*skbp = skb;
}

static void mctp_test_flow_fini(struct kunit *test,
				struct mctp_test_dev *dev,
				struct mctp_dst *dst,
				struct mctp_test_pktqueue *tpq,
				struct socket *sock)
{
	__mctp_route_test_fini(test, dev, dst, tpq, sock);
}

/* test that an outgoing skb has the correct MCTP extension data set */
static void mctp_test_packet_flow(struct kunit *test)
{
	struct mctp_test_pktqueue tpq;
	struct sk_buff *skb, *skb2;
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct mctp_flow *flow;
	struct socket *sock;
	u8 dst_eid = 8;
	int n, rc;

	mctp_test_flow_init(test, &dev, &dst, &tpq, &sock, &skb, 30);

	rc = mctp_local_output(sock->sk, &dst, skb, dst_eid, MCTP_TAG_OWNER, 0);
	KUNIT_ASSERT_EQ(test, rc, 0);

	n = tpq.pkts.qlen;
	KUNIT_ASSERT_EQ(test, n, 1);

	skb2 = skb_dequeue(&tpq.pkts);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb2);

	flow = skb_ext_find(skb2, SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow->key);
	KUNIT_ASSERT_PTR_EQ(test, flow->key->sk, sock->sk);

	kfree_skb(skb2);
	mctp_test_flow_fini(test, dev, &dst, &tpq, sock);
}

/* test that outgoing skbs, after fragmentation, all have the correct MCTP
 * extension data set.
 */
static void mctp_test_fragment_flow(struct kunit *test)
{
	struct mctp_test_pktqueue tpq;
	struct mctp_flow *flows[2];
	struct sk_buff *tx_skbs[2];
	struct mctp_test_dev *dev;
	struct mctp_dst dst;
	struct sk_buff *skb;
	struct socket *sock;
	u8 dst_eid = 8;
	int n, rc;

	mctp_test_flow_init(test, &dev, &dst, &tpq, &sock, &skb, 100);

	rc = mctp_local_output(sock->sk, &dst, skb, dst_eid, MCTP_TAG_OWNER, 0);
	KUNIT_ASSERT_EQ(test, rc, 0);

	n = tpq.pkts.qlen;
	KUNIT_ASSERT_EQ(test, n, 2);

	/* both resulting packets should have the same flow data */
	tx_skbs[0] = skb_dequeue(&tpq.pkts);
	tx_skbs[1] = skb_dequeue(&tpq.pkts);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, tx_skbs[0]);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, tx_skbs[1]);

	flows[0] = skb_ext_find(tx_skbs[0], SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flows[0]);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flows[0]->key);
	KUNIT_ASSERT_PTR_EQ(test, flows[0]->key->sk, sock->sk);

	flows[1] = skb_ext_find(tx_skbs[1], SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flows[1]);
	KUNIT_ASSERT_PTR_EQ(test, flows[1]->key, flows[0]->key);

	kfree_skb(tx_skbs[0]);
	kfree_skb(tx_skbs[1]);
	mctp_test_flow_fini(test, dev, &dst, &tpq, sock);
}

static void __mctp_test_partial_tx_invalidates_key_and_queues_error(struct kunit *test,
								    bool batch,
								    int fail_err)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_output_fail ctx;
	struct mctp_error *merr;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_sock *msk;
	struct mctp_flow *flow;
	struct sk_buff *skb;
	struct sk_buff *sent;
	struct sk_buff *err;
	struct sk_buff *extra;
	struct mctp_dst dst;
	struct socket *sock;
	u8 *addrs;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	WRITE_ONCE(dev->mdev->net, netid);

	addrs = kmalloc(sizeof(*addrs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, addrs);
	addrs[0] = 8;
	dev->mdev->addrs = addrs;
	dev->mdev->num_addrs = 1;

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	msk = container_of(sock->sk, struct mctp_sock, sk);
	msk->enable_errqueue = true;

	if (batch) {
		dev->mdev->tx_batching_enabled = true;
		dev->mdev->tx_batch_hdr_len = 1;
		dev->mdev->tx_batch_max_xfer = 143;
	}

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 2, fail_err);

	skb = mctp_test_payload_skb(test, 130);
	rc = mctp_local_output(sock->sk, &dst, skb, 10, MCTP_TAG_OWNER);
	KUNIT_EXPECT_EQ(test, rc, fail_err);
	KUNIT_EXPECT_EQ(test, ctx.calls, 2U);
	KUNIT_ASSERT_EQ(test, skb_queue_len(&ctx.pkts), 1U);

	sent = skb_peek(&ctx.pkts);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sent);
	if (batch)
		KUNIT_EXPECT_GT(test, sent->len, 68U);
	flow = skb_ext_find(sent, SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow);
	key = flow->key;
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);
	KUNIT_EXPECT_FALSE(test, key->valid);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key->hlist));

	err = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, err);
	KUNIT_ASSERT_TRUE(test, err->len >= sizeof(*merr));
	merr = (struct mctp_error *)err->data;
	KUNIT_EXPECT_EQ(test, merr->error_code, (u32)EPIPE);
	KUNIT_EXPECT_EQ(test, merr->direction, (u8)MCTP_DIR_TX);
	KUNIT_EXPECT_EQ(test, merr->src_eid, (u8)8);
	KUNIT_EXPECT_EQ(test, merr->dest_eid, (u8)10);
	KUNIT_EXPECT_EQ(test, merr->msg_type, (u8)0x05);
	KUNIT_EXPECT_EQ(test, merr->payload_len, (u16)MCTP_ERROR_PAYLOAD_SIZE);
	kfree_skb(err);
	extra = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_EXPECT_PTR_EQ(test, extra, NULL);
	kfree_skb(extra);

	mctp_test_dst_release_fail(&dst, &ctx);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_partial_tx_invalidates_key_and_queues_error(struct kunit *test)
{
	__mctp_test_partial_tx_invalidates_key_and_queues_error(test, false,
								-EPERM);
}

static void mctp_test_batch_partial_tx_invalidates_key_and_queues_error(struct kunit *test)
{
	__mctp_test_partial_tx_invalidates_key_and_queues_error(test, true,
								-EPERM);
}

static void mctp_test_partial_tx_preserves_emsgsize_and_queues_error(struct kunit *test)
{
	__mctp_test_partial_tx_invalidates_key_and_queues_error(test, false,
								-EMSGSIZE);
}

static void mctp_test_batch_partial_tx_preserves_emsgsize_and_queues_error(struct kunit *test)
{
	__mctp_test_partial_tx_invalidates_key_and_queues_error(test, true,
								-EMSGSIZE);
}

static void __mctp_test_first_tx_failure_keeps_errqueue_empty(struct kunit *test,
							      bool batch)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_output_fail ctx;
	struct mctp_test_dev *dev;
	struct mctp_sock *msk;
	struct sk_buff *skb;
	struct sk_buff *err;
	struct mctp_dst dst;
	struct socket *sock;
	u8 *addrs;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	WRITE_ONCE(dev->mdev->net, netid);

	addrs = kmalloc(sizeof(*addrs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, addrs);
	addrs[0] = 8;
	dev->mdev->addrs = addrs;
	dev->mdev->num_addrs = 1;

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	msk = container_of(sock->sk, struct mctp_sock, sk);
	msk->enable_errqueue = true;

	if (batch) {
		dev->mdev->tx_batching_enabled = true;
		dev->mdev->tx_batch_hdr_len = 1;
		dev->mdev->tx_batch_max_xfer = 143;
	}

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 1, -EMSGSIZE);

	skb = mctp_test_payload_skb(test, 130);
	rc = mctp_local_output(sock->sk, &dst, skb, 10, MCTP_TAG_OWNER);
	KUNIT_EXPECT_EQ(test, rc, -EMSGSIZE);
	KUNIT_EXPECT_EQ(test, ctx.calls, 1U);
	KUNIT_EXPECT_EQ(test, skb_queue_len(&ctx.pkts), 0U);

	err = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_EXPECT_PTR_EQ(test, err, NULL);
	kfree_skb(err);

	mctp_test_dst_release_fail(&dst, &ctx);
	mctp_test_remove_socket_keys(msk);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_first_tx_failure_keeps_errqueue_empty(struct kunit *test)
{
	__mctp_test_first_tx_failure_keeps_errqueue_empty(test, false);
}

static void mctp_test_batch_first_tx_failure_keeps_errqueue_empty(struct kunit *test)
{
	__mctp_test_first_tx_failure_keeps_errqueue_empty(test, true);
}

static unsigned int mctp_test_partial_release_flow_calls;
static struct mctp_dev *mctp_test_partial_released_flow_dev;

static void mctp_test_partial_release_flow(struct mctp_dev *dev,
					   struct mctp_sk_key *key)
{
	mctp_test_partial_release_flow_calls++;
	mctp_test_partial_released_flow_dev = dev;
	key->dev_flow_state = 0;
}

static const struct mctp_netdev_ops mctp_test_partial_release_flow_ops = {
	.release_flow = mctp_test_partial_release_flow,
};

static unsigned int mctp_test_partial_key_count(struct netns_mctp *mns)
{
	struct mctp_sk_key *key;
	unsigned long flags;
	unsigned int count = 0;

	spin_lock_irqsave(&mns->keys_lock, flags);
	hlist_for_each_entry(key, &mns->keys, hlist)
		count++;
	spin_unlock_irqrestore(&mns->keys_lock, flags);

	return count;
}

static void __mctp_test_manual_partial_tx(struct kunit *test, bool batch)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_output_fail ctx;
	struct netns_mctp *mns = &init_net.mctp;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_error *merr;
	struct mctp_sock *msk;
	struct mctp_flow *flow;
	struct mctp_hdr *hdr;
	struct sk_buff *skb;
	struct sk_buff *sent;
	struct sk_buff *err;
	struct sk_buff *extra;
	struct mctp_dst dst;
	struct socket *sock;
	unsigned long flags;
	unsigned int key_count;
	u8 *addrs;
	u8 tag;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	WRITE_ONCE(dev->mdev->net, netid);

	addrs = kmalloc(sizeof(*addrs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, addrs);
	addrs[0] = 8;
	dev->mdev->addrs = addrs;
	dev->mdev->num_addrs = 1;

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	msk = container_of(sock->sk, struct mctp_sock, sk);
	msk->enable_errqueue = true;

	key = mctp_alloc_local_tag(msk, netid, 8, 10, true, &tag,
				   MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);
	key_count = mctp_test_partial_key_count(mns);

	dev->mdev->ops = &mctp_test_partial_release_flow_ops;
	mctp_test_partial_release_flow_calls = 0;
	mctp_test_partial_released_flow_dev = NULL;
	spin_lock_irqsave(&key->lock, flags);
	key->dev_flow_state = 1;
	mctp_dev_set_key(dev->mdev, key);
	spin_unlock_irqrestore(&key->lock, flags);

	if (batch) {
		dev->mdev->tx_batching_enabled = true;
		dev->mdev->tx_batch_hdr_len = 1;
		dev->mdev->tx_batch_max_xfer = 143;
	}

	mctp_test_dst_setup_fail(test, &dst, dev, &ctx, 68, 2, -EMSGSIZE);

	skb = mctp_test_payload_skb(test, 130);
	rc = mctp_local_output(sock->sk, &dst, skb, 10,
			       MCTP_TAG_OWNER | MCTP_TAG_PREALLOC | tag);
	KUNIT_EXPECT_EQ(test, rc, -EMSGSIZE);
	KUNIT_EXPECT_EQ(test, ctx.calls, 2U);
	KUNIT_EXPECT_EQ(test, skb_queue_len(&ctx.pkts), 1U);
	KUNIT_EXPECT_EQ(test, mctp_test_partial_release_flow_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, mctp_test_partial_released_flow_dev,
			    dev->mdev);
	KUNIT_EXPECT_EQ(test, key->dev_flow_state, 0UL);
	KUNIT_EXPECT_TRUE(test, key->valid);
	KUNIT_EXPECT_FALSE(test, key->reasm_dead);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key->hlist));
	KUNIT_EXPECT_PTR_EQ(test, key->dev, dev->mdev);
	KUNIT_EXPECT_EQ(test, mctp_test_partial_key_count(mns), key_count);

	err = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, err);
	KUNIT_ASSERT_TRUE(test, err->len >= sizeof(*merr));
	merr = (struct mctp_error *)err->data;
	KUNIT_EXPECT_EQ(test, merr->error_code, (u32)EPIPE);
	KUNIT_EXPECT_EQ(test, merr->direction, (u8)MCTP_DIR_TX);
	KUNIT_EXPECT_EQ(test, merr->src_eid, (u8)8);
	KUNIT_EXPECT_EQ(test, merr->dest_eid, (u8)10);
	KUNIT_EXPECT_EQ(test, merr->msg_type, (u8)0x05);
	kfree_skb(err);
	extra = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_EXPECT_PTR_EQ(test, extra, NULL);
	kfree_skb(extra);

	/* Drop the successfully submitted first fragment/batch, then prove that
	 * the still-hashed manual key is found and reused by a fresh message.
	 */
	skb_queue_purge(&ctx.pkts);
	ctx.calls = 0;
	ctx.fail_at = 0;

	skb = mctp_test_payload_skb(test, 16);
	rc = mctp_local_output(sock->sk, &dst, skb, 10,
			       MCTP_TAG_OWNER | MCTP_TAG_PREALLOC | tag);
	KUNIT_ASSERT_EQ(test, rc, 0);
	KUNIT_ASSERT_EQ(test, ctx.calls, 1U);
	KUNIT_ASSERT_EQ(test, skb_queue_len(&ctx.pkts), 1U);
	sent = skb_peek(&ctx.pkts);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sent);
	flow = skb_ext_find(sent, SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow);
	KUNIT_EXPECT_PTR_EQ(test, flow->key, key);
	hdr = mctp_hdr(sent);
	KUNIT_EXPECT_EQ(test,
			hdr->flags_seq_tag &
				(MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM),
			(u8)(MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM));
	KUNIT_EXPECT_EQ(test, hdr->flags_seq_tag & MCTP_HDR_TAG_MASK, tag);
	KUNIT_EXPECT_TRUE(test, hdr->flags_seq_tag & MCTP_HDR_FLAG_TO);
	KUNIT_EXPECT_TRUE(test, key->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key->hlist));
	KUNIT_EXPECT_EQ(test, mctp_test_partial_key_count(mns), key_count);
	KUNIT_EXPECT_EQ(test, mctp_test_partial_release_flow_calls, 1U);
	extra = skb_dequeue(&sock->sk->sk_error_queue);
	KUNIT_EXPECT_PTR_EQ(test, extra, NULL);
	kfree_skb(extra);

	mctp_test_dst_release_fail(&dst, &ctx);
	/* Avoid counting test cleanup as another completed transport flow. */
	dev->mdev->ops = NULL;
	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_prealloc_partial_tx_releases_manual_key(struct kunit *test)
{
	__mctp_test_manual_partial_tx(test, false);
}

static void mctp_test_batch_prealloc_partial_tx_releases_manual_key(struct kunit *test)
{
	__mctp_test_manual_partial_tx(test, true);
}

#else
static void mctp_test_packet_flow(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_fragment_flow(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_partial_tx_invalidates_key_and_queues_error(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_batch_partial_tx_invalidates_key_and_queues_error(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_partial_tx_preserves_emsgsize_and_queues_error(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_batch_partial_tx_preserves_emsgsize_and_queues_error(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_first_tx_failure_keeps_errqueue_empty(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_batch_first_tx_failure_keeps_errqueue_empty(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_prealloc_partial_tx_releases_manual_key(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

static void mctp_test_batch_prealloc_partial_tx_releases_manual_key(struct kunit *test)
{
	kunit_skip(test, "Requires CONFIG_MCTP_FLOWS=y");
}

#endif

/* Test that outgoing skbs cause a suitable tag to be created */
static void mctp_test_route_output_key_create(struct kunit *test)
{
	const u8 dst_eid = 26, src_eid = 15;
	struct mctp_test_pktqueue tpq;
	const unsigned int netid = 50;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct netns_mctp *mns;
	unsigned long flags;
	struct socket *sock;
	struct sk_buff *skb;
	struct mctp_dst dst;
	bool empty, single;
	const int len = 2;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	WRITE_ONCE(dev->mdev->net, netid);

	mctp_test_dst_setup(test, &dst, dev, &tpq, 68);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	dev->mdev->addrs = kmalloc(sizeof(u8), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev->mdev->addrs);
	dev->mdev->num_addrs = 1;
	dev->mdev->addrs[0] = src_eid;

	skb = alloc_skb(sizeof(struct mctp_hdr) + 1 + len, GFP_KERNEL);
	KUNIT_ASSERT_TRUE(test, skb);
	__mctp_cb(skb);
	skb_reserve(skb, sizeof(struct mctp_hdr) + 1 + len);
	memset(skb_put(skb, len), 0, len);

	mns = &sock_net(sock->sk)->mctp;

	/* We assume we're starting from an empty keys list, which requires
	 * preceding tests to clean up correctly!
	 */
	spin_lock_irqsave(&mns->keys_lock, flags);
	empty = hlist_empty(&mns->keys);
	spin_unlock_irqrestore(&mns->keys_lock, flags);
	KUNIT_ASSERT_TRUE(test, empty);

	rc = mctp_local_output(sock->sk, &dst, skb, dst_eid, MCTP_TAG_OWNER, 0);
	KUNIT_ASSERT_EQ(test, rc, 0);

	key = NULL;
	single = false;
	spin_lock_irqsave(&mns->keys_lock, flags);
	if (!hlist_empty(&mns->keys)) {
		key = hlist_entry(mns->keys.first, struct mctp_sk_key, hlist);
		single = hlist_is_singular_node(&key->hlist, &mns->keys);
	}
	spin_unlock_irqrestore(&mns->keys_lock, flags);

	KUNIT_ASSERT_NOT_NULL(test, key);
	KUNIT_ASSERT_TRUE(test, single);

	KUNIT_EXPECT_EQ(test, key->net, netid);
	KUNIT_EXPECT_EQ(test, key->local_addr, src_eid);
	KUNIT_EXPECT_EQ(test, key->peer_addr, dst_eid);
	/* key has incoming tag, so inverse of what we sent */
	KUNIT_EXPECT_FALSE(test, key->tag & MCTP_TAG_OWNER);

	sock_release(sock);
	mctp_test_dst_release(&dst, &tpq);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_max_keys_cap_blocks_new_key(struct kunit *test)
{
	struct netns_mctp *mns = &init_net.mctp;
	unsigned int old_max_keys;
	struct mctp_sk_key *key1;
	struct mctp_sk_key *key2;
	struct mctp_sock *msk;
	struct socket *sock;
	u8 tag;
	int rc;

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	key1 = mctp_alloc_local_tag(msk, MCTP_INITIAL_DEFAULT_NET, 8, 9,
				    false, &tag, MCTP_DEFAULT_LIFETIME);
	if (IS_ERR_OR_NULL(key1)) {
		WRITE_ONCE(mns->max_keys, old_max_keys);
		sock_release(sock);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key1);
		return;
	}

	kunit_info(test,
		   "KR-05 key-cap proof: max_keys=1 with one existing key, allocating second key local=8 peer=10");
	key2 = mctp_alloc_local_tag(msk, MCTP_INITIAL_DEFAULT_NET, 8, 10,
				    false, &tag, MCTP_DEFAULT_LIFETIME);
	kunit_info(test,
		   "KR-05 key-cap proof: key2_err=%ld tx_drops=%llu tx_dropped_no_memory=%llu",
		   IS_ERR(key2) ? PTR_ERR(key2) : 0L, msk->stats.tx_drops,
		   msk->stats.tx_dropped_no_memory);
	kunit_info(test,
		   "KR-05 key-cap proof: fixed result rejects new in-flight key with -ENOBUFS");

	KUNIT_EXPECT_TRUE_MSG(test, IS_ERR(key2),
			      "KR-05: key allocation succeeded past max_keys cap");
	if (IS_ERR(key2))
		KUNIT_EXPECT_EQ_MSG(test, PTR_ERR(key2), (long)-ENOBUFS,
				    "KR-05: key cap did not return -ENOBUFS");

	WRITE_ONCE(mns->max_keys, old_max_keys);
	mctp_key_unref(key1);
	sock_release(sock);
}

static void mctp_test_max_keys_cap_blocks_rx_reasm_key(struct kunit *test)
{
	struct netns_mctp *mns = &init_net.mctp;
	const struct mctp_hdr hdr = RX_FRAG(FL_S, 0);
	struct mctp_test_pktqueue tpq;
	unsigned int old_max_keys;
	unsigned long flags;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_sock *msk;
	struct mctp_dst dst;
	struct socket *sock;
	struct sk_buff *skb;
	bool empty;
	u8 type = 0;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	key = mctp_key_alloc(msk, MCTP_INITIAL_DEFAULT_NET, 2, 3, 0,
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);
	rc = mctp_key_add(key, msk, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_EQ(test, rc, 0);

	skb = mctp_test_create_skb_data(&hdr, &type);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	kunit_info(test,
		   "KR-05 SOM flood proof: max_keys=1 with one existing key, injecting new RX SOM reassembly");
	rc = mctp_dst_input(&dst, skb);
	spin_lock_irqsave(&mns->keys_lock, flags);
	empty = hlist_is_singular_node(&key->hlist, &mns->keys);
	spin_unlock_irqrestore(&mns->keys_lock, flags);
	WRITE_ONCE(mns->max_keys, old_max_keys);
	kunit_info(test,
		   "KR-05 SOM flood proof: rc=%d only_original_key_linked=%d",
		   rc, empty);
	kunit_info(test,
		   "KR-05 SOM flood proof: fixed result rejects new SOM allocation with -ENOBUFS and no extra key linked");

	KUNIT_EXPECT_EQ_MSG(test, rc, -ENOBUFS,
			    "KR-05: RX SOM allocation past max_keys was not rejected");
	KUNIT_EXPECT_TRUE_MSG(test, empty,
			      "KR-05: RX SOM added an extra in-flight key past cap");

	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key);
	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static void mctp_test_max_keys_cap_blocks_local_output(struct kunit *test)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct netns_mctp *mns = &init_net.mctp;
	struct mctp_test_pktqueue tpq;
	unsigned int old_max_keys;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_sock *msk;
	struct sk_buff *skb;
	struct mctp_dst dst;
	struct socket *sock;
	u8 *addrs;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, netid);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	addrs = kmalloc(sizeof(*addrs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, addrs);
	addrs[0] = 8;
	dev->mdev->addrs = addrs;
	dev->mdev->num_addrs = 1;

	key = mctp_key_alloc(msk, netid, 2, 3, 0, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);
	rc = mctp_key_add(key, msk, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_EQ(test, rc, 0);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	kunit_info(test,
		   "KR-05 local-output proof: max_keys=1 with one existing key, sending new owner-tag request");
	skb = mctp_test_payload_skb(test, 16);
	rc = mctp_local_output(sock->sk, &dst, skb, 10, MCTP_TAG_OWNER);

	WRITE_ONCE(mns->max_keys, old_max_keys);
	kunit_info(test,
		   "KR-05 local-output proof: rc=%d queued_pkts=%u tx_drops=%llu tx_dropped_no_memory=%llu",
		   rc, skb_queue_len(&tpq.pkts), msk->stats.tx_drops,
		   msk->stats.tx_dropped_no_memory);
	kunit_info(test,
		   "KR-05 local-output proof: fixed result drops before TX and records no-memory stats");

	KUNIT_EXPECT_EQ_MSG(test, rc, -ENOBUFS,
			    "KR-05: local output was not blocked by max_keys");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&tpq.pkts), 0U,
			    "KR-05: packet transmitted despite max_keys cap");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.tx_drops, 1ULL,
			    "KR-05: TX drop counter was not incremented");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.tx_dropped_no_memory, 1ULL,
			    "KR-05: no-memory TX drop counter was not incremented");

	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key);
	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static void mctp_test_max_keys_cap_allows_prealloc_send(struct kunit *test)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct netns_mctp *mns = &init_net.mctp;
	struct mctp_test_pktqueue tpq;
	unsigned int old_max_keys;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_sock *msk;
	struct sk_buff *skb;
	struct mctp_dst dst;
	struct socket *sock;
	u8 *addrs;
	u8 tag;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, netid);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	addrs = kmalloc(sizeof(*addrs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, addrs);
	addrs[0] = 8;
	dev->mdev->addrs = addrs;
	dev->mdev->num_addrs = 1;

	key = mctp_alloc_local_tag(msk, netid, 8, 10, true, &tag,
				   MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	kunit_info(test,
		   "KR-05 prealloc proof: max_keys=1 but reusing preallocated tag=%u for peer=10",
		   tag);
	skb = mctp_test_payload_skb(test, 16);
	rc = mctp_local_output(sock->sk, &dst, skb, 10,
			       MCTP_TAG_OWNER | MCTP_TAG_PREALLOC | tag);

	WRITE_ONCE(mns->max_keys, old_max_keys);
	kunit_info(test,
		   "KR-05 prealloc proof: rc=%d queued_pkts=%u key_valid=%d",
		   rc, skb_queue_len(&tpq.pkts), key->valid);
	kunit_info(test,
		   "KR-05 prealloc proof: fixed result still allows existing/preallocated sessions at the cap");

	KUNIT_EXPECT_EQ_MSG(test, rc, 0,
			    "KR-05: preallocated tag reuse was blocked by max_keys");
	KUNIT_EXPECT_EQ_MSG(test, skb_queue_len(&tpq.pkts), 1U,
			    "KR-05: preallocated tag reuse did not transmit");
	KUNIT_EXPECT_TRUE_MSG(test, key->valid,
			      "KR-05: preallocated key was invalidated incorrectly");

	mctp_test_dst_release(&dst, &tpq);
	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_max_keys_cap_rx_increments_no_memory_drop_counter(struct kunit *test)
{
	struct netns_mctp *mns = &init_net.mctp;
	const struct mctp_hdr hdr = RX_FRAG(FL_S, 0);
	struct mctp_test_pktqueue tpq;
	unsigned int old_max_keys;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct mctp_sock *msk;
	struct mctp_dst dst;
	struct socket *sock;
	struct sk_buff *skb;
	u8 type = 0;
	int rc;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	key = mctp_key_alloc(msk, MCTP_INITIAL_DEFAULT_NET, 2, 3, 0,
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);
	rc = mctp_key_add(key, msk, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_EQ(test, rc, 0);

	skb = mctp_test_create_skb_data(&hdr, &type);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	mctp_test_skb_set_dev(skb, dev);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	kunit_info(test,
		   "KR-05 RX counter proof: injecting RX SOM past max_keys=1");
	rc = mctp_dst_input(&dst, skb);

	WRITE_ONCE(mns->max_keys, old_max_keys);

	kunit_info(test,
		   "KR-05 RX counter proof: rc=%d rx_no_memory=%llu rx_seq_mismatch=%llu",
		   rc, msk->stats.rx_dropped_no_memory,
		   msk->stats.rx_dropped_seq_mismatch);
	kunit_info(test,
		   "KR-05 RX counter proof: fixed result records no-memory, not sequence-mismatch, for cap rejection");

	KUNIT_EXPECT_EQ_MSG(test, rc, -ENOBUFS,
			    "KR-05: RX cap rejection did not return -ENOBUFS");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.rx_dropped_no_memory, 1ULL,
			    "KR-05: RX no-memory drop counter was not incremented");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.rx_dropped_seq_mismatch, 0ULL,
			    "KR-05: RX cap rejection was misclassified as sequence mismatch");

	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key);
	__mctp_route_test_fini(test, dev, &dst, &tpq, sock);
}

static void mctp_test_max_keys_cap_new_key_increments_no_memory_drop_counter(struct kunit *test)
{
	struct netns_mctp *mns = &init_net.mctp;
	unsigned int old_max_keys;
	struct mctp_sk_key *key1;
	struct mctp_sk_key *key2;
	struct mctp_sock *msk;
	struct socket *sock;
	u8 tag;
	int rc;

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	msk = container_of(sock->sk, struct mctp_sock, sk);

	old_max_keys = READ_ONCE(mns->max_keys);
	WRITE_ONCE(mns->max_keys, 1);

	key1 = mctp_alloc_local_tag(msk, MCTP_INITIAL_DEFAULT_NET, 8, 9,
				    false, &tag, MCTP_DEFAULT_LIFETIME);
	if (IS_ERR_OR_NULL(key1)) {
		WRITE_ONCE(mns->max_keys, old_max_keys);
		sock_release(sock);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key1);
		return;
	}

	key2 = mctp_alloc_local_tag(msk, MCTP_INITIAL_DEFAULT_NET, 8, 10,
				    false, &tag, MCTP_DEFAULT_LIFETIME);

	WRITE_ONCE(mns->max_keys, old_max_keys);

	kunit_info(test,
		   "KR-05 TX counter proof: key2_err=%ld tx_drops=%llu tx_dropped_no_memory=%llu",
		   IS_ERR(key2) ? PTR_ERR(key2) : 0L, msk->stats.tx_drops,
		   msk->stats.tx_dropped_no_memory);
	kunit_info(test,
		   "KR-05 TX counter proof: fixed result records no-memory TX drop for cap rejection");

	KUNIT_EXPECT_TRUE_MSG(test, IS_ERR(key2),
			      "KR-05: second key allocation succeeded past cap");
	if (IS_ERR(key2))
		KUNIT_EXPECT_EQ_MSG(test, PTR_ERR(key2), (long)-ENOBUFS,
				    "KR-05: second key allocation did not return -ENOBUFS");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.tx_drops, 1ULL,
			    "KR-05: TX drop counter was not incremented");
	KUNIT_EXPECT_EQ_MSG(test, msk->stats.tx_dropped_no_memory, 1ULL,
			    "KR-05: no-memory TX drop counter was not incremented");

	mctp_test_remove_socket_keys(msk);
	mctp_key_unref(key1);
	sock_release(sock);
}

static struct mctp_sk_key *
mctp_test_add_key(struct kunit *test, struct socket *sock, unsigned int netid,
		  mctp_eid_t local, mctp_eid_t peer, u8 tag,
		  struct mctp_dev *mdev)
{
	struct mctp_sock *msk = container_of(sock->sk, struct mctp_sock, sk);
	struct mctp_sk_key *key;
	unsigned long flags;
	int rc;

	key = mctp_key_alloc(msk, netid, local, peer, tag, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);

	rc = mctp_key_add(key, msk, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_EQ(test, rc, 0);

	if (mdev) {
		spin_lock_irqsave(&key->lock, flags);
		mctp_dev_set_key(mdev, key);
		spin_unlock_irqrestore(&key->lock, flags);
	}

	return key;
}

static void mctp_test_key_remove_addr_scopes_to_device(struct kunit *test)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_dev *dev1, *dev2;
	struct mctp_sk_key *key_dev1;
	struct mctp_sk_key *key_dev2;
	struct mctp_sk_key *key_any1;
	struct mctp_sk_key *key_any2;
	struct mctp_sk_key *key_bound1;
	struct mctp_sk_key *key_bound2;
	struct socket *sock1, *sock2, *sock3;
	struct socket *sock4, *sock5, *sock6;
	int rc;

	dev1 = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev1);
	dev2 = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev2);
	WRITE_ONCE(dev1->mdev->net, netid);
	WRITE_ONCE(dev2->mdev->net, netid);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock1);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock2);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock3);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock4);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock5);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock6);
	KUNIT_ASSERT_EQ(test, rc, 0);

	WRITE_ONCE(sock3->sk->sk_bound_dev_if, dev1->ndev->ifindex);
	WRITE_ONCE(sock4->sk->sk_bound_dev_if, dev2->ndev->ifindex);
	WRITE_ONCE(sock5->sk->sk_bound_dev_if, dev1->ndev->ifindex);
	WRITE_ONCE(sock6->sk->sk_bound_dev_if, dev2->ndev->ifindex);

	key_dev1 = mctp_test_add_key(test, sock1, netid, 8, 9,
				     MCTP_HDR_FLAG_TO | 1, dev1->mdev);
	key_dev2 = mctp_test_add_key(test, sock2, netid, 8, 10,
				     MCTP_HDR_FLAG_TO | 2, dev2->mdev);
	key_any1 = mctp_test_add_key(test, sock3, netid, MCTP_ADDR_ANY, 11,
				     MCTP_HDR_FLAG_TO | 3, NULL);
	key_any2 = mctp_test_add_key(test, sock4, netid, MCTP_ADDR_ANY, 12,
				     MCTP_HDR_FLAG_TO | 4, NULL);
	key_bound1 = mctp_test_add_key(test, sock5, netid, 8, 13,
				       MCTP_HDR_FLAG_TO | 5, NULL);
	key_bound2 = mctp_test_add_key(test, sock6, netid, 8, 14,
				       MCTP_HDR_FLAG_TO | 6, NULL);

	mctp_key_remove_addr(dev1->mdev, 8);

	KUNIT_EXPECT_FALSE(test, key_dev1->valid);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key_dev1->hlist));
	KUNIT_EXPECT_PTR_EQ(test, key_dev1->dev, NULL);

	KUNIT_EXPECT_TRUE(test, key_dev2->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_dev2->hlist));
	KUNIT_EXPECT_PTR_EQ(test, key_dev2->dev, dev2->mdev);

	KUNIT_EXPECT_FALSE(test, key_any1->valid);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key_any1->hlist));

	KUNIT_EXPECT_TRUE(test, key_any2->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_any2->hlist));

	KUNIT_EXPECT_FALSE(test, key_bound1->valid);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key_bound1->hlist));

	KUNIT_EXPECT_TRUE(test, key_bound2->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_bound2->hlist));

	mctp_key_remove_addr(dev2->mdev, 8);

	mctp_key_unref(key_dev1);
	mctp_key_unref(key_dev2);
	mctp_key_unref(key_any1);
	mctp_key_unref(key_any2);
	mctp_key_unref(key_bound1);
	mctp_key_unref(key_bound2);
	sock_release(sock1);
	sock_release(sock2);
	sock_release(sock3);
	sock_release(sock4);
	sock_release(sock5);
	sock_release(sock6);
	mctp_test_destroy_dev(dev1);
	mctp_test_destroy_dev(dev2);
}

static void mctp_test_key_remove_addr_removes_bound_manual_any_key(struct kunit *test)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_dev *dev1, *dev2;
	struct mctp_sk_key *key_dev1;
	struct mctp_sk_key *key_dev2;
	struct mctp_sk_key *key_unbound;
	struct mctp_sock *msk1, *msk2, *msk3;
	struct socket *sock1, *sock2, *sock3;
	u8 tag;
	int rc;

	dev1 = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev1);
	dev2 = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev2);
	WRITE_ONCE(dev1->mdev->net, netid);
	WRITE_ONCE(dev2->mdev->net, netid);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock1);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock2);
	KUNIT_ASSERT_EQ(test, rc, 0);
	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock3);
	KUNIT_ASSERT_EQ(test, rc, 0);

	WRITE_ONCE(sock1->sk->sk_bound_dev_if, dev1->ndev->ifindex);
	WRITE_ONCE(sock2->sk->sk_bound_dev_if, dev2->ndev->ifindex);

	msk1 = container_of(sock1->sk, struct mctp_sock, sk);
	msk2 = container_of(sock2->sk, struct mctp_sock, sk);
	msk3 = container_of(sock3->sk, struct mctp_sock, sk);

	key_dev1 = mctp_alloc_local_tag(msk1, netid, MCTP_ADDR_ANY, 9,
					true, &tag, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key_dev1);
	key_dev2 = mctp_alloc_local_tag(msk2, netid, MCTP_ADDR_ANY, 9,
					true, &tag, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key_dev2);
	key_unbound = mctp_alloc_local_tag(msk3, netid, MCTP_ADDR_ANY, 9,
					   true, &tag, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key_unbound);

	mctp_key_remove_addr(dev1->mdev, 8);

	KUNIT_EXPECT_TRUE(test, key_dev1->manual_alloc);
	KUNIT_EXPECT_FALSE(test, key_dev1->valid);
	KUNIT_EXPECT_TRUE(test, key_dev1->reasm_dead);
	KUNIT_EXPECT_PTR_EQ(test, key_dev1->reasm_head, NULL);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key_dev1->hlist));
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key_dev1->sklist));

	KUNIT_EXPECT_TRUE(test, key_dev2->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_dev2->hlist));
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_dev2->sklist));

	KUNIT_EXPECT_TRUE(test, key_unbound->valid);
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_unbound->hlist));
	KUNIT_EXPECT_FALSE(test, hlist_unhashed(&key_unbound->sklist));

	mctp_key_remove_addr(dev2->mdev, 8);
	mctp_key_unref(key_dev1);
	mctp_key_unref(key_dev2);
	mctp_key_unref(key_unbound);
	sock_release(sock1);
	sock_release(sock2);
	sock_release(sock3);
	mctp_test_destroy_dev(dev1);
	mctp_test_destroy_dev(dev2);
}

static void mctp_test_key_remove_addr_frees_reasm_head(struct kunit *test)
{
	const unsigned int netid = MCTP_INITIAL_DEFAULT_NET;
	struct mctp_test_dev *dev;
	struct mctp_sk_key *key;
	struct sk_buff *trap;
	struct sk_buff *skb;
	struct socket *sock;
	unsigned long flags;
	struct mctp_hdr hdr;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	WRITE_ONCE(dev->mdev->net, netid);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	key = mctp_test_add_key(test, sock, netid, 8, 9,
				MCTP_HDR_FLAG_TO | 1, dev->mdev);

	hdr.ver = 1;
	hdr.src = 9;
	hdr.dest = 8;
	hdr.flags_seq_tag = MCTP_HDR_FLAG_TO | MCTP_HDR_FLAG_SOM | 1;
	skb = mctp_test_create_skb(&hdr, 8);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	/* Clone holds a separate ref on the same data area. KASAN/kmemleak
	 * will flag the head as leaked if a regression skips the kfree_skb
	 * on the removal path.
	 */
	trap = skb_clone(skb, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, trap);

	spin_lock_irqsave(&key->lock, flags);
	key->reasm_head = skb;
	key->reasm_tailp = &skb->next;
	spin_unlock_irqrestore(&key->lock, flags);

	mctp_key_remove_addr(dev->mdev, 8);

	KUNIT_EXPECT_FALSE(test, key->valid);
	KUNIT_EXPECT_TRUE(test, key->reasm_dead);
	KUNIT_EXPECT_PTR_EQ(test, key->reasm_head, NULL);
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key->hlist));
	KUNIT_EXPECT_TRUE(test, hlist_unhashed(&key->sklist));

	kfree_skb(trap);
	mctp_key_unref(key);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_route_extaddr_input(struct kunit *test)
{
	static const unsigned char haddr[] = { 0xaa, 0x55 };
	struct mctp_test_pktqueue tpq;
	struct mctp_skb_cb *cb, *cb2;
	const unsigned int len = 40;
	struct mctp_test_dev *dev;
	struct sk_buff *skb, *skb2;
	struct mctp_dst dst;
	struct mctp_hdr hdr;
	struct socket *sock;
	int rc;

	hdr.ver = 1;
	hdr.src = 10;
	hdr.dest = 8;
	hdr.flags_seq_tag = FL_S | FL_E | FL_TO;

	__mctp_route_test_init(test, &dev, &dst, &tpq, &sock, MCTP_NET_ANY);

	skb = mctp_test_create_skb(&hdr, len);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	/* set our hardware addressing data */
	cb = mctp_cb(skb);
	memcpy(cb->haddr, haddr, sizeof(haddr));
	cb->halen = sizeof(haddr);

	mctp_test_skb_set_dev(skb, dev);

	rc = mctp_dst_input(&dst, skb);
	KUNIT_ASSERT_EQ(test, rc, 0);

	mctp_test_dst_release(&dst, &tpq);

	skb2 = skb_recv_datagram(sock->sk, MSG_DONTWAIT, &rc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb2);
	KUNIT_ASSERT_EQ(test, skb2->len, len);

	cb2 = mctp_cb(skb2);

	/* Received SKB should have the hardware addressing as set above.
	 * We're likely to have the same actual cb here (ie., cb == cb2),
	 * but it's the comparison that we care about
	 */
	KUNIT_EXPECT_EQ(test, cb2->halen, sizeof(haddr));
	KUNIT_EXPECT_MEMEQ(test, cb2->haddr, haddr, sizeof(haddr));

	skb_free_datagram(sock->sk, skb2);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_route_gw_lookup(struct kunit *test)
{
	struct mctp_test_route *rt1, *rt2;
	struct mctp_dst dst = { 0 };
	struct mctp_test_dev *dev;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	/* 8 (local) -> 10 (gateway) via 9 (direct) */
	rt1 = mctp_test_create_route_direct(&init_net, dev->mdev, 9, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt1);
	rt2 = mctp_test_create_route_gw(&init_net, dev->mdev->net, 10, 9, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt2);

	rc = mctp_route_lookup(&init_net, dev->mdev->net, 10, &dst);
	KUNIT_EXPECT_EQ(test, rc, 0);
	KUNIT_EXPECT_PTR_EQ(test, dst.dev, dev->mdev);
	KUNIT_EXPECT_EQ(test, dst.mtu, dev->ndev->mtu);
	KUNIT_EXPECT_EQ(test, dst.nexthop, 9);
	KUNIT_EXPECT_EQ(test, dst.halen, 0);

	mctp_dst_release(&dst);

	mctp_test_route_destroy(test, rt2);
	mctp_test_route_destroy(test, rt1);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_route_gw_loop(struct kunit *test)
{
	struct mctp_test_route *rt1, *rt2;
	struct mctp_dst dst = { 0 };
	struct mctp_test_dev *dev;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	/* two routes using each other as the gw */
	rt1 = mctp_test_create_route_gw(&init_net, dev->mdev->net, 9, 10, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt1);
	rt2 = mctp_test_create_route_gw(&init_net, dev->mdev->net, 10, 9, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt2);

	/* this should fail, rather than infinite-loop */
	rc = mctp_route_lookup(&init_net, dev->mdev->net, 10, &dst);
	KUNIT_EXPECT_NE(test, rc, 0);

	mctp_test_route_destroy(test, rt2);
	mctp_test_route_destroy(test, rt1);
	mctp_test_destroy_dev(dev);
}

struct mctp_route_gw_mtu_test {
	/* working away from the local stack */
	unsigned int dev, neigh, gw, dst;
	unsigned int exp;
};

static void mctp_route_gw_mtu_to_desc(const struct mctp_route_gw_mtu_test *t,
				      char *desc)
{
	sprintf(desc, "dev %d, neigh %d, gw %d, dst %d -> %d",
		t->dev, t->neigh, t->gw, t->dst, t->exp);
}

static const struct mctp_route_gw_mtu_test mctp_route_gw_mtu_tests[] = {
	/* no route-specific MTUs */
	{ 68, 0, 0, 0, 68 },
	{ 100, 0, 0, 0, 100 },
	/* one route MTU (smaller than dev mtu), others unrestricted */
	{ 100, 68, 0, 0, 68 },
	{ 100, 0, 68, 0, 68 },
	{ 100, 0, 0, 68, 68 },
	/* smallest applied, regardless of order */
	{ 100, 99, 98, 68, 68 },
	{ 99, 100, 98, 68, 68 },
	{ 98, 99, 100, 68, 68 },
	{ 68, 98, 99, 100, 68 },
};

KUNIT_ARRAY_PARAM(mctp_route_gw_mtu, mctp_route_gw_mtu_tests,
		  mctp_route_gw_mtu_to_desc);

static void mctp_test_route_gw_mtu(struct kunit *test)
{
	const struct mctp_route_gw_mtu_test *mtus = test->param_value;
	struct mctp_test_route *rt1, *rt2, *rt3;
	struct mctp_dst dst = { 0 };
	struct mctp_test_dev *dev;
	struct mctp_dev *mdev;
	unsigned int netid;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dev->ndev->mtu = mtus->dev;
	mdev = dev->mdev;
	netid = mdev->net;

	/* 8 (local) -> 11 (dst) via 10 (gw) via 9 (neigh) */
	rt1 = mctp_test_create_route_direct(&init_net, mdev, 9, mtus->neigh);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt1);

	rt2 = mctp_test_create_route_gw(&init_net, netid, 10, 9, mtus->gw);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt2);

	rt3 = mctp_test_create_route_gw(&init_net, netid, 11, 10, mtus->dst);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, rt3);

	rc = mctp_route_lookup(&init_net, dev->mdev->net, 11, &dst);
	KUNIT_EXPECT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, dst.mtu, mtus->exp);

	mctp_dst_release(&dst);

	mctp_test_route_destroy(test, rt3);
	mctp_test_route_destroy(test, rt2);
	mctp_test_route_destroy(test, rt1);
	mctp_test_destroy_dev(dev);
}

#define MCTP_TEST_LLADDR_LEN 2
struct mctp_test_llhdr {
	unsigned int magic;
	unsigned char src[MCTP_TEST_LLADDR_LEN];
	unsigned char dst[MCTP_TEST_LLADDR_LEN];
};

static const unsigned int mctp_test_llhdr_magic = 0x5c78339c;

static int test_dev_header_create(struct sk_buff *skb, struct net_device *dev,
				  unsigned short type, const void *daddr,
				  const void *saddr, unsigned int len)
{
	struct kunit *test = current->kunit_test;
	struct mctp_test_llhdr *hdr;

	hdr = skb_push(skb, sizeof(*hdr));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, hdr);
	skb_reset_mac_header(skb);

	hdr->magic = mctp_test_llhdr_magic;
	memcpy(&hdr->src, saddr, sizeof(hdr->src));
	memcpy(&hdr->dst, daddr, sizeof(hdr->dst));

	return 0;
}

/* Test the dst_output path for a gateway-routed skb: we should have it
 * lookup the nexthop EID in the neighbour table, and call into
 * header_ops->create to resolve that to a lladdr. Our mock header_ops->create
 * will just set a synthetic link-layer header, which we check after transmit.
 */
static void mctp_test_route_gw_output(struct kunit *test)
{
	const unsigned char haddr_self[MCTP_TEST_LLADDR_LEN] = { 0xaa, 0x03 };
	const unsigned char haddr_peer[MCTP_TEST_LLADDR_LEN] = { 0xaa, 0x02 };
	const struct header_ops ops = {
		.create = test_dev_header_create,
	};
	struct mctp_neigh neigh = { 0 };
	struct mctp_test_llhdr *ll_hdr;
	struct mctp_dst dst = { 0 };
	struct mctp_hdr hdr = { 0 };
	struct mctp_test_dev *dev;
	struct sk_buff *skb;
	unsigned char *buf;
	int i, rc;

	dev = mctp_test_create_dev_lladdr(sizeof(haddr_self), haddr_self);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dev->ndev->header_ops = &ops;

	dst.dev = dev->mdev;
	__mctp_dev_get(dst.dev->dev);
	dst.mtu = 68;
	dst.nexthop = 9;

	/* simple mctp_neigh_add for the gateway (not dest!) endpoint */
	INIT_LIST_HEAD(&neigh.list);
	neigh.dev = dev->mdev;
	mctp_dev_hold(dev->mdev);
	neigh.eid = 9;
	neigh.source = MCTP_NEIGH_STATIC;
	memcpy(neigh.ha, haddr_peer, sizeof(haddr_peer));
	list_add_rcu(&neigh.list, &init_net.mctp.neighbours);

	hdr.ver = 1;
	hdr.src = 8;
	hdr.dest = 10;
	hdr.flags_seq_tag = FL_S | FL_E | FL_TO;

	/* construct enough for a future link-layer header, the provided
	 * mctp header, and 4 bytes of data
	 */
	skb = alloc_skb(sizeof(*ll_hdr) + sizeof(hdr) + 4, GFP_KERNEL);
	skb->dev = dev->ndev;
	__mctp_cb(skb);

	skb_reserve(skb, sizeof(*ll_hdr));

	memcpy(skb_put(skb, sizeof(hdr)), &hdr, sizeof(hdr));
	buf = skb_put(skb, 4);
	for (i = 0; i < 4; i++)
		buf[i] = i;

	/* extra ref over the dev_xmit */
	skb_get(skb);

	rc = mctp_dst_output(&dst, skb);
	KUNIT_EXPECT_EQ(test, rc, 0);

	mctp_dst_release(&dst);
	list_del_rcu(&neigh.list);
	mctp_dev_put(dev->mdev);

	/* check that we have our header created with the correct neighbour */
	ll_hdr = (void *)skb_mac_header(skb);
	KUNIT_EXPECT_EQ(test, ll_hdr->magic, mctp_test_llhdr_magic);
	KUNIT_EXPECT_MEMEQ(test, ll_hdr->src, haddr_self, sizeof(haddr_self));
	KUNIT_EXPECT_MEMEQ(test, ll_hdr->dst, haddr_peer, sizeof(haddr_peer));
	kfree_skb(skb);
}

static unsigned int mctp_test_route_list_count(struct net *net)
{
	struct mctp_route *rt;
	unsigned int count = 0;

	list_for_each_entry(rt, &net->mctp.routes, list)
		count++;

	return count;
}

static void mctp_test_routes_net_exit_unlinks_routes(struct kunit *test)
{
	struct mctp_route *rt1, *rt2, *rt, *tmp;
	unsigned int before, after;
	struct net *net;

	net = kunit_kzalloc(test, sizeof(*net), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, net);
	INIT_LIST_HEAD(&net->mctp.routes);
	/* mctp_routes_net_exit() now takes keys_lock and walks tag_hints while
	 * freeing tag hints; initialize both so this synthetic net does not trip
	 * "spinlock bad magic" under CONFIG_DEBUG_SPINLOCK.
	 */
	spin_lock_init(&net->mctp.keys_lock);
	INIT_HLIST_HEAD(&net->mctp.tag_hints);

	rt1 = mctp_route_alloc();
	KUNIT_ASSERT_NOT_NULL(test, rt1);
	rt2 = mctp_route_alloc();
	if (!rt2) {
		mctp_route_release(rt1);
		KUNIT_FAIL_AND_ABORT(test, "failed to allocate second route");
	}

	rt1->dst_type = MCTP_ROUTE_GATEWAY;
	rt1->min = 8;
	rt1->max = 8;
	refcount_inc(&rt1->refs);
	list_add_rcu(&rt1->list, &net->mctp.routes);
	kunit_info(test,
		   "KR-07 route net_exit proof: added route min=%u max=%u; KUnit holds an extra ref for safe post-exit inspection",
		   rt1->min, rt1->max);

	rt2->dst_type = MCTP_ROUTE_GATEWAY;
	rt2->min = 9;
	rt2->max = 9;
	refcount_inc(&rt2->refs);
	list_add_rcu(&rt2->list, &net->mctp.routes);
	kunit_info(test,
		   "KR-07 route net_exit proof: added route min=%u max=%u; production teardown would not keep this inspection ref",
		   rt2->min, rt2->max);

	before = mctp_test_route_list_count(net);
	kunit_info(test,
		   "KR-07 route net_exit proof: inserted %u routes into net->mctp.routes",
		   before);
	kunit_info(test,
		   "KR-07 route net_exit proof: calling mctp_routes_net_exit(); fixed code must list_del_rcu() before release");

	rtnl_lock();
	mctp_routes_net_exit(net);
	rtnl_unlock();

	after = mctp_test_route_list_count(net);
	kunit_info(test,
		   "KR-07 route net_exit proof: before=%u after=%u list_empty=%u",
		   before, after, list_empty(&net->mctp.routes));
	kunit_info(test,
		   "KR-07 route net_exit proof: after>0 means released routes are still reachable from net->mctp.routes");
	kunit_info(test,
		   "KR-07 route net_exit proof: an RCU reader or route dump can walk stale entries until they are unlinked");
	kunit_info(test,
		   "KR-07 route net_exit proof: fixed result is after=0 and list_empty=1 immediately after net_exit");

	KUNIT_EXPECT_EQ_MSG(test, after, 0U,
			    "KR-07: mctp_routes_net_exit() released routes but left %u stale route entries linked",
			    after);
	KUNIT_EXPECT_TRUE_MSG(test, list_empty(&net->mctp.routes),
			      "KR-07: net->mctp.routes is not empty after net_exit");

	if (!list_empty(&net->mctp.routes)) {
		list_for_each_entry_safe(rt, tmp, &net->mctp.routes, list)
			list_del_rcu(&rt->list);
	}

	mctp_route_release(rt1);
	mctp_route_release(rt2);
	rcu_barrier();
}

static const unsigned int mctp_neigh_addr_len_tests[] = {
	0, 1, 2, 3, 8, MAX_ADDR_LEN,
};

static void mctp_neigh_addr_len_test_to_desc(const unsigned int *addr_len,
					     char *desc)
{
	sprintf(desc, "addr_len %u", *addr_len);
}

KUNIT_ARRAY_PARAM(mctp_neigh_addr_len, mctp_neigh_addr_len_tests,
		  mctp_neigh_addr_len_test_to_desc);

static void mctp_test_neigh_lookup_copies_addr_len_only(struct kunit *test)
{
	const unsigned int addr_len = *(const unsigned int *)test->param_value;
	struct mctp_neigh neigh = { 0 };
	unsigned char haddr_self[MAX_ADDR_LEN];
	unsigned char haddr_peer[MAX_ADDR_LEN];
	struct mctp_test_dev *dev;
	unsigned char out[MAX_ADDR_LEN];
	unsigned int i;
	int rc;

	for (i = 0; i < MAX_ADDR_LEN; i++) {
		haddr_self[i] = 0xa0 + i;
		haddr_peer[i] = 0xc0 + i;
	}

	dev = mctp_test_create_dev_lladdr(addr_len, haddr_self);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	INIT_LIST_HEAD(&neigh.list);
	neigh.dev = dev->mdev;
	mctp_dev_hold(dev->mdev);
	neigh.eid = 9;
	neigh.source = MCTP_NEIGH_STATIC;
	memset(neigh.ha, 0xee, sizeof(neigh.ha));
	memcpy(neigh.ha, haddr_peer, addr_len);
	list_add_rcu(&neigh.list, &init_net.mctp.neighbours);

	memset(out, 0xa5, sizeof(out));
	kunit_info(test,
		   "KN-01 neighbour-copy proof: dev_addr_len=%u neigh_padding=0xee out_padding_initial=0xa5",
		   addr_len);
	rc = mctp_neigh_lookup(dev->mdev, 9, out);
	kunit_info(test,
		   "KN-01 neighbour-copy proof: rc=%d first_tail_byte=0x%02x",
		   rc, addr_len < MAX_ADDR_LEN ? out[addr_len] : 0);
	if (addr_len < MAX_ADDR_LEN)
		kunit_info(test,
			   "KN-01 neighbour-copy proof: fixed result copies only dev->addr_len and leaves bytes [%u..%u] unchanged",
			   addr_len, MAX_ADDR_LEN - 1);
	else
		kunit_info(test,
			   "KN-01 neighbour-copy proof: fixed result copies the full MAX_ADDR_LEN address with no padding bytes remaining");

	KUNIT_EXPECT_EQ_MSG(test, rc, 0,
			    "KN-01: neighbour lookup failed");
	if (addr_len)
		KUNIT_EXPECT_MEMEQ_MSG(test, out, haddr_peer, addr_len,
				       "KN-01: valid address bytes were not copied");
	for (i = addr_len; i < MAX_ADDR_LEN; i++)
		KUNIT_EXPECT_EQ_MSG(test, out[i], (unsigned char)0xa5,
				    "KN-01: lookup overwrote bytes beyond dev->addr_len");

	list_del_rcu(&neigh.list);
	mctp_dev_put(dev->mdev);
	mctp_test_destroy_dev(dev);
}

static struct kunit_case mctp_test_cases[] = {
	KUNIT_CASE_PARAM(mctp_test_fragment, mctp_frag_gen_params),
	KUNIT_CASE(mctp_test_fragment_route_preserves_output_errno),
	KUNIT_CASE(mctp_test_fragment_route_first_failure_preserves_errno),
	KUNIT_CASE(mctp_test_fragment_route_batch_preserves_output_errno),
	KUNIT_CASE(mctp_test_fragment_route_batch_first_failure_preserves_errno),
	KUNIT_CASE(mctp_test_fragment_alloc_failure_sends_no_partial),
	KUNIT_CASE(mctp_test_fragment_batch_alloc_failure_sends_no_partial),
	KUNIT_CASE_PARAM(mctp_test_rx_input, mctp_rx_input_gen_params),
	KUNIT_CASE_PARAM(mctp_test_route_input_sk, mctp_route_input_sk_gen_params),
	KUNIT_CASE_PARAM(mctp_test_route_input_sk_reasm,
			 mctp_route_input_sk_reasm_gen_params),
	KUNIT_CASE(mctp_test_route_input_duplicate_som_restarts_reasm),
	KUNIT_CASE(mctp_test_route_input_duplicate_som_drops_old_middle),
	KUNIT_CASE_PARAM(mctp_test_route_input_sk_keys,
			 mctp_route_input_sk_keys_gen_params),
	KUNIT_CASE(mctp_test_route_input_sk_fail_single),
	KUNIT_CASE(mctp_test_route_input_sk_fail_frag),
	KUNIT_CASE(mctp_test_route_input_multiple_nets_bind),
	KUNIT_CASE(mctp_test_route_input_multiple_nets_key),
	KUNIT_CASE(mctp_test_packet_flow),
	KUNIT_CASE(mctp_test_fragment_flow),
	KUNIT_CASE(mctp_test_partial_tx_invalidates_key_and_queues_error),
	KUNIT_CASE(mctp_test_batch_partial_tx_invalidates_key_and_queues_error),
	KUNIT_CASE(mctp_test_partial_tx_preserves_emsgsize_and_queues_error),
	KUNIT_CASE(mctp_test_batch_partial_tx_preserves_emsgsize_and_queues_error),
	KUNIT_CASE(mctp_test_first_tx_failure_keeps_errqueue_empty),
	KUNIT_CASE(mctp_test_batch_first_tx_failure_keeps_errqueue_empty),
	KUNIT_CASE(mctp_test_prealloc_partial_tx_releases_manual_key),
	KUNIT_CASE(mctp_test_batch_prealloc_partial_tx_releases_manual_key),
	KUNIT_CASE(mctp_test_route_output_key_create),
	KUNIT_CASE(mctp_test_max_keys_cap_blocks_new_key),
	KUNIT_CASE(mctp_test_max_keys_cap_blocks_rx_reasm_key),
	KUNIT_CASE(mctp_test_max_keys_cap_blocks_local_output),
	KUNIT_CASE(mctp_test_max_keys_cap_allows_prealloc_send),
	KUNIT_CASE(mctp_test_key_remove_addr_scopes_to_device),
	KUNIT_CASE(mctp_test_key_remove_addr_removes_bound_manual_any_key),
	KUNIT_CASE(mctp_test_key_remove_addr_frees_reasm_head),
	KUNIT_CASE(mctp_test_max_keys_cap_rx_increments_no_memory_drop_counter),
	KUNIT_CASE(mctp_test_max_keys_cap_new_key_increments_no_memory_drop_counter),
	KUNIT_CASE(mctp_test_route_input_cloned_frag),
	KUNIT_CASE(mctp_test_route_extaddr_input),
	KUNIT_CASE(mctp_test_route_gw_lookup),
	KUNIT_CASE(mctp_test_route_gw_loop),
	KUNIT_CASE_PARAM(mctp_test_route_gw_mtu, mctp_route_gw_mtu_gen_params),
	KUNIT_CASE(mctp_test_route_gw_output),
	KUNIT_CASE(mctp_test_routes_net_exit_unlinks_routes),
	KUNIT_CASE_PARAM(mctp_test_neigh_lookup_copies_addr_len_only,
			 mctp_neigh_addr_len_gen_params),
	{}
};

static struct kunit_suite mctp_test_suite = {
	.name = "mctp-route",
	.test_cases = mctp_test_cases,
};

kunit_test_suite(mctp_test_suite);
