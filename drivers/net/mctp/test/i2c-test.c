// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>
#include <kunit/resource.h>
#include <kunit/skbuff.h>

KUNIT_DEFINE_ACTION_WRAPPER(mctp_i2c_test_free_netdev, free_netdev,
			    struct net_device *);

static void mctp_i2c_test_device_deselect(void *data)
{
	mctp_i2c_device_select(data, NULL);
}

static void mctp_i2c_test_lock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
}

static int mctp_i2c_test_trylock_bus(struct i2c_adapter *adapter,
				     unsigned int flags)
{
	return 1;
}

static void mctp_i2c_test_unlock_bus(struct i2c_adapter *adapter,
				     unsigned int flags)
{
	unsigned int *unlocks = adapter->algo_data;

	(*unlocks)++;
}

static const struct i2c_lock_operations mctp_i2c_test_lock_ops = {
	.lock_bus = mctp_i2c_test_lock_bus,
	.trylock_bus = mctp_i2c_test_trylock_bus,
	.unlock_bus = mctp_i2c_test_unlock_bus,
};

struct mctp_i2c_test_xfer_ctx {
	unsigned int unlocks;
	unsigned int transfers;
	u16 addr;
	u16 len;
	u8 buf[MCTP_I2C_BUFSZ];
};

static int mctp_i2c_test_master_xfer(struct i2c_adapter *adapter,
				     struct i2c_msg *msgs, int num)
{
	struct mctp_i2c_test_xfer_ctx *ctx = adapter->algo_data;
	struct kunit *test = current->kunit_test;

	KUNIT_EXPECT_EQ(test, num, 1);
	if (num != 1)
		return -EINVAL;

	KUNIT_EXPECT_LE(test, msgs[0].len, (u16)sizeof(ctx->buf));
	if (msgs[0].len > sizeof(ctx->buf))
		return -EINVAL;

	ctx->transfers++;
	ctx->addr = msgs[0].addr;
	ctx->len = msgs[0].len;
	memcpy(ctx->buf, msgs[0].buf, msgs[0].len);

	return num;
}

static const struct i2c_algorithm mctp_i2c_test_algorithm = {
	.master_xfer = mctp_i2c_test_master_xfer,
};

static struct sk_buff *__mctp_i2c_test_create_tx_skb(struct kunit *test,
						     u8 dest_eid,
						     struct mctp_sk_key *key,
						     unsigned int alloc_len,
						     unsigned int payload_len)
{
	struct mctp_i2c_hdr *ihdr;
	struct mctp_flow *flow;
	struct mctp_hdr *mh;
	struct sk_buff *skb;
	unsigned int i;

	KUNIT_ASSERT_LE(test, sizeof(*mh) + payload_len + 1,
			(size_t)MCTP_I2C_MAXBLOCK);

	skb = alloc_skb(alloc_len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, kunit_action_kfree_skb,
						  skb), 0);
	__mctp_cb(skb);

	skb_reset_mac_header(skb);
	ihdr = skb_put_zero(skb, sizeof(*ihdr));
	ihdr->dest_slave = 0x70;
	ihdr->command = MCTP_I2C_COMMANDCODE;
	ihdr->byte_count = sizeof(*mh) + payload_len + 1;
	ihdr->source_slave = 0x21;

	skb_set_network_header(skb, sizeof(*ihdr));
	mh = skb_put_zero(skb, sizeof(*mh));
	mh->ver = 1;
	mh->dest = dest_eid;
	mh->src = 8;
	mh->flags_seq_tag = MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM |
			    MCTP_HDR_FLAG_TO;
	for (i = 0; i < payload_len; i++)
		*(u8 *)skb_put(skb, 1) = (u8)(0x40 + i);

	flow = skb_ext_add(skb, SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow);
	if (key)
		refcount_inc(&key->refs);
	flow->key = key;

	return skb;
}

static struct sk_buff *mctp_i2c_test_create_tx_skb(struct kunit *test,
						   u8 dest_eid,
						   struct mctp_sk_key *key)
{
	return __mctp_i2c_test_create_tx_skb(test, dest_eid, key,
					     MCTP_I2C_BUFSZ, 2);
}

static struct sk_buff *mctp_i2c_test_create_tx_fragment(struct kunit *test,
							u8 dest_eid,
							struct mctp_sk_key *key,
							u8 flags_seq)
{
	struct sk_buff *skb;

	skb = mctp_i2c_test_create_tx_skb(test, dest_eid, key);
	mctp_hdr(skb)->flags_seq_tag = flags_seq | MCTP_HDR_FLAG_TO;

	return skb;
}

static void mctp_i2c_test_clear_flow_key(struct sk_buff *skb)
{
	struct mctp_flow *flow;
	struct mctp_sk_key *key;

	flow = skb_ext_find(skb, SKB_EXT_MCTP);
	if (flow && flow->key) {
		key = flow->key;
		flow->key = NULL;
		mctp_key_unref(key);
	}
}

static void __mctp_i2c_test_invalid_flow_counts_drop_and_preserves_skb(struct kunit *test,
								       bool valid_key)
{
	struct i2c_adapter adapter = { 0 };
	struct net_device *ndev;
	struct mctp_i2c_dev *midev;
	struct mctp_sk_key key = { 0 };
	struct sk_buff *skb;
	unsigned int unlocks = 0;
	unsigned int orig_len;
	u8 orig[MCTP_I2C_BUFSZ];

	ndev = alloc_netdev(0, "mctpi2ctest%d", NET_NAME_ENUM,
			    mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev), 0);

	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo_data = &unlocks;

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);

	spin_lock_init(&midev->lock);
	midev->ndev = ndev;
	midev->adapter = &adapter;
	midev->flows_enabled = true;

	spin_lock_init(&key.lock);
	refcount_set(&key.refs, 1);
	key.valid = valid_key;
	key.dev_flow_state = MCTP_I2C_FLOW_STATE_INVALID;

	skb = mctp_i2c_test_create_tx_skb(test, 9, &key);
	orig_len = skb->len;
	KUNIT_ASSERT_LE(test, orig_len, MCTP_I2C_BUFSZ);
	memcpy(orig, skb->data, orig_len);

	kunit_info(test,
		   "KI-02 invalid-flow proof: valid_key=%u dev_flow_state=INVALID before mctp_i2c_xmit()",
		   valid_key);
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-02 invalid-flow proof: tx_dropped=%lu tx_errors=%lu tx_packets=%lu eid9_flow_invalid=%llu skb_len=%u orig_len=%u",
		   ndev->stats.tx_dropped, ndev->stats.tx_errors,
		   ndev->stats.tx_packets,
		   midev->eid_stats.eid[9].tx_drop_flow_invalid, skb->len,
		   orig_len);
	kunit_info(test,
		   "KI-02 invalid-flow proof: fixed result is tx_dropped=1, tx_errors=0, no TX packet, and skb data unchanged");

	KUNIT_EXPECT_EQ_MSG(test, ndev->stats.tx_dropped, 1UL,
			    "KI-02: invalid flow drop was not counted");
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 0UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_packets, 0UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_bytes, 0UL);
	KUNIT_EXPECT_EQ_MSG(test,
			    midev->eid_stats.eid[9].tx_drop_flow_invalid, 1ULL,
			    "KI-02: per-EID invalid-flow drop was not counted");
	KUNIT_EXPECT_TRUE(test, test_bit(9, midev->eid_stats.active));
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);
	KUNIT_EXPECT_EQ(test, unlocks, 0U);
	KUNIT_EXPECT_EQ(test, skb->len, orig_len);
	KUNIT_EXPECT_MEMEQ(test, skb->data, orig, orig_len);
	mctp_i2c_test_clear_flow_key(skb);

}

static void mctp_i2c_test_invalid_key_counts_drop_and_preserves_skb(struct kunit *test)
{
	__mctp_i2c_test_invalid_flow_counts_drop_and_preserves_skb(test, false);
}

static void mctp_i2c_test_invalid_state_counts_drop_and_preserves_skb(struct kunit *test)
{
	__mctp_i2c_test_invalid_flow_counts_drop_and_preserves_skb(test, true);
}

static void mctp_i2c_test_invalidate_manual_active_flow_resets_for_reuse(struct kunit *test)
{
	struct i2c_adapter adapter = { 0 };
	struct mctp_i2c_dev *midev;
	struct mctp_sk_key key = { 0 };
	unsigned int unlocks = 0;
	struct mctp_flow *flow;
	struct sk_buff *skb;

	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo_data = &unlocks;

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);

	spin_lock_init(&midev->lock);
	midev->adapter = &adapter;
	midev->i2c_lock_count = 1;

	spin_lock_init(&key.lock);
	refcount_set(&key.refs, 1);
	key.valid = true;
	key.manual_alloc = true;
	key.dev_flow_state = MCTP_I2C_FLOW_STATE_ACTIVE;

	skb = alloc_skb(1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, kunit_action_kfree_skb,
						  skb), 0);
	flow = skb_ext_add(skb, SKB_EXT_MCTP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, flow);
	refcount_inc(&key.refs);
	flow->key = &key;

	kunit_info(test,
		   "KI-07 manual-flow proof: manual key starts ACTIVE with i2c_lock_count=%d unlocks=%u",
		   midev->i2c_lock_count, unlocks);
	mctp_i2c_invalidate_tx_flow(midev, skb);
	kunit_info(test,
		   "KI-07 manual-flow proof: after invalidate state=%lu i2c_lock_count=%d unlocks=%u",
		   key.dev_flow_state, midev->i2c_lock_count, unlocks);
	kunit_info(test,
		   "KI-07 manual-flow proof: a failed manual flow releases its lock and resets to NEW for a later exchange");

	KUNIT_EXPECT_EQ_MSG(test, key.dev_flow_state,
			    MCTP_I2C_FLOW_STATE_NEW,
			    "KI-07: reusable manual flow did not reset to NEW");
	KUNIT_EXPECT_EQ_MSG(test, midev->i2c_lock_count, 0,
			    "KI-07: I2C bus lock remained held");
	KUNIT_EXPECT_EQ_MSG(test, unlocks, 1U,
			    "KI-07: TX failure did not unlock the bus");
	mctp_i2c_test_clear_flow_key(skb);
}

static void mctp_i2c_test_xmit_error_resets_manual_active_flow(struct kunit *test)
{
	struct i2c_adapter adapter = { 0 };
	struct net_device *ndev;
	struct mctp_i2c_dev *midev;
	struct mctp_sk_key key = { 0 };
	unsigned int unlocks = 0;
	struct sk_buff *skb;

	ndev = alloc_netdev(0, "mctpi2ctest%d", NET_NAME_ENUM,
			    mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev), 0);

	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo_data = &unlocks;

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);

	spin_lock_init(&midev->lock);
	spin_lock_init(&midev->error_inject.lock);
	midev->ndev = ndev;
	midev->adapter = &adapter;
	midev->flows_enabled = true;
	midev->i2c_lock_count = 1;
	midev->error_inject.enable_tx = true;
	midev->error_inject.mode = MCTP_ERR_MODE_ALWAYS;
	midev->error_inject.i2c_tx_error_code = EIO;

	spin_lock_init(&key.lock);
	refcount_set(&key.refs, 1);
	key.valid = true;
	key.manual_alloc = true;
	key.dev_flow_state = MCTP_I2C_FLOW_STATE_ACTIVE;

	skb = mctp_i2c_test_create_tx_skb(test, 9, &key);

	kunit_info(test,
		   "KI-07 active-flow error proof: manual ACTIVE flow starts with i2c_lock_count=%d unlocks=%u",
		   midev->i2c_lock_count, unlocks);
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-07 active-flow error proof: after forced EIO state=%lu i2c_lock_count=%d unlocks=%u tx_errors=%lu eid9_eio=%llu",
		   key.dev_flow_state, midev->i2c_lock_count, unlocks,
		   ndev->stats.tx_errors, midev->eid_stats.eid[9].tx_drop_eio);

	KUNIT_EXPECT_EQ(test, key.dev_flow_state, MCTP_I2C_FLOW_STATE_NEW);
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);
	KUNIT_EXPECT_EQ(test, unlocks, 1U);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 1UL);
	KUNIT_EXPECT_EQ(test, midev->eid_stats.eid[9].tx_drop_eio, 1ULL);
	mctp_i2c_test_clear_flow_key(skb);
}

static void mctp_i2c_test_manual_flow_error_drops_remainder_and_recovers_on_som(struct kunit *test)
{
	struct i2c_adapter adapter = { 0 };
	struct mctp_i2c_test_xfer_ctx ctx = { 0 };
	struct mctp_i2c_client *mcli;
	struct net_device *ndev;
	struct mctp_i2c_dev *midev;
	struct mctp_sk_key key = { 0 };
	unsigned int request_len;
	struct sk_buff *skb;

	ndev = alloc_netdev(0, "mctpi2ctest%d", NET_NAME_ENUM,
			    mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev), 0);

	adapter.algo = &mctp_i2c_test_algorithm;
	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo_data = &ctx;
	adapter.timeout = HZ;
	mutex_init(&adapter.hold_lock);

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);

	mcli = kunit_kzalloc(test, sizeof(*mcli), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, mcli);

	spin_lock_init(&mcli->sel_lock);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_device_deselect,
						  mcli), 0);
	spin_lock_init(&midev->lock);
	spin_lock_init(&midev->error_inject.lock);
	midev->ndev = ndev;
	midev->adapter = &adapter;
	midev->client = mcli;
	midev->flows_enabled = true;
	midev->error_inject.enable_tx = true;
	midev->error_inject.mode = MCTP_ERR_MODE_ALWAYS;
	midev->error_inject.i2c_tx_error_code = EIO;

	spin_lock_init(&key.lock);
	refcount_set(&key.refs, 1);
	key.valid = true;
	key.manual_alloc = true;
	key.dev_flow_state = MCTP_I2C_FLOW_STATE_NEW;

	/* Fail the first fragment of one multi-packet message. */
	skb = mctp_i2c_test_create_tx_fragment(test, 9, &key,
					       MCTP_HDR_FLAG_SOM);
	kunit_info(test,
		   "KI-07 message-boundary proof: failing SOM starts state=NEW i2c_lock_count=%d unlocks=%u",
		   midev->i2c_lock_count, ctx.unlocks);
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-07 message-boundary proof: after EIO state=%lu i2c_lock_count=%d unlocks=%u transfers=%u",
		   key.dev_flow_state, midev->i2c_lock_count, ctx.unlocks,
		   ctx.transfers);

	KUNIT_EXPECT_EQ_MSG(test, key.dev_flow_state,
			    MCTP_I2C_FLOW_STATE_NEW,
			    "KI-07: failed manual exchange permanently disabled its tag");
	KUNIT_EXPECT_TRUE(test, key.valid);
	KUNIT_EXPECT_TRUE(test, key.manual_alloc);
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);
	KUNIT_EXPECT_EQ(test, midev->release_count, 0);
	KUNIT_EXPECT_EQ(test, ctx.unlocks, 1U);
	KUNIT_EXPECT_EQ(test, ctx.transfers, 0U);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_dropped, 0UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_packets, 0UL);
	KUNIT_EXPECT_EQ(test, midev->eid_stats.eid[9].tx_drop_eio, 1ULL);
	KUNIT_EXPECT_EQ(test,
			midev->eid_stats.eid[9].tx_drop_flow_invalid, 0ULL);
	KUNIT_EXPECT_TRUE(test, test_bit(9, midev->eid_stats.active));
	mctp_i2c_test_clear_flow_key(skb);

	/* The rest of the failed message must not be transmitted. Keeping the
	 * key in NEW permits a later SOM while still rejecting this non-SOM.
	 */
	midev->error_inject.enable_tx = false;
	skb = mctp_i2c_test_create_tx_fragment(test, 9, &key,
					       MCTP_HDR_FLAG_EOM |
					       (1 << MCTP_HDR_SEQ_SHIFT));
	kunit_info(test,
		   "KI-07 message-boundary proof: sending non-SOM remainder of failed exchange");
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-07 message-boundary proof: remainder state=%lu transfers=%u tx_dropped=%lu",
		   key.dev_flow_state, ctx.transfers, ndev->stats.tx_dropped);

	KUNIT_EXPECT_EQ_MSG(test, key.dev_flow_state,
			    MCTP_I2C_FLOW_STATE_NEW,
			    "KI-07: rejecting a stale remainder must preserve recovery state");
	KUNIT_EXPECT_EQ_MSG(test, ctx.transfers, 0U,
			    "KI-07: fragment after failed SOM reached the I2C bus");
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);
	KUNIT_EXPECT_EQ(test, ctx.unlocks, 1U);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_dropped, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_packets, 0UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_bytes, 0UL);
	KUNIT_EXPECT_EQ(test,
			midev->eid_stats.eid[9].tx_drop_flow_invalid, 1ULL);
	mctp_i2c_test_clear_flow_key(skb);

	/* A new message boundary on the persistent manual tag must recover. */
	skb = mctp_i2c_test_create_tx_fragment(test, 9, &key,
					       MCTP_HDR_FLAG_SOM |
					       MCTP_HDR_FLAG_EOM);
	request_len = skb->len;
	kunit_info(test,
		   "KI-07 message-boundary proof: sending a new SOM on the reusable manual tag");
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-07 message-boundary proof: new request state=%lu transfers=%u tx_packets=%lu tx_dropped=%lu",
		   key.dev_flow_state, ctx.transfers, ndev->stats.tx_packets,
		   ndev->stats.tx_dropped);

	KUNIT_EXPECT_EQ_MSG(test, ctx.transfers, 1U,
			    "KI-07: next SOM was dropped after a prior exchange failed");
	KUNIT_EXPECT_EQ(test, key.dev_flow_state, MCTP_I2C_FLOW_STATE_ACTIVE);
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 1);
	KUNIT_EXPECT_EQ(test, ctx.unlocks, 1U);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_dropped, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_packets, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_bytes,
			(unsigned long)request_len);
	KUNIT_EXPECT_EQ(test, midev->eid_stats.eid[9].tx_drop_eio, 1ULL);
	KUNIT_EXPECT_EQ(test,
			midev->eid_stats.eid[9].tx_drop_flow_invalid, 1ULL);
	mctp_i2c_test_clear_flow_key(skb);
	KUNIT_EXPECT_EQ(test, refcount_read(&key.refs), 1);

	/* The successful request intentionally leaves the response flow active. */
	mctp_i2c_unlock_reset(midev);
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);
	KUNIT_EXPECT_EQ(test, ctx.unlocks, 2U);
}

static void mctp_i2c_test_slave_rx_uses_last_selected_device(struct kunit *test)
{
	struct i2c_client client = { 0 };
	struct mctp_i2c_client *mcli;
	struct net_device *ndev_a;
	struct net_device *ndev_b;
	struct mctp_i2c_dev *midev_a;
	struct mctp_i2c_dev *midev_b;
	u8 val;

	ndev_a = alloc_netdev(0, "mctpi2ca%d", NET_NAME_ENUM,
			      mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev_a);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev_a), 0);

	ndev_b = alloc_netdev(0, "mctpi2cb%d", NET_NAME_ENUM,
			      mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev_b);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev_b), 0);

	mcli = kunit_kzalloc(test, sizeof(*mcli), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, mcli);
	spin_lock_init(&mcli->sel_lock);
	mcli->lladdr = 0x31;
	i2c_set_clientdata(&client, mcli);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_device_deselect,
						  mcli), 0);

	midev_a = kunit_kzalloc(test, sizeof(*midev_a), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev_a);
	spin_lock_init(&midev_a->lock);
	midev_a->ndev = ndev_a;
	midev_a->client = mcli;
	midev_a->allow_rx = true;

	midev_b = kunit_kzalloc(test, sizeof(*midev_b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev_b);
	spin_lock_init(&midev_b->lock);
	midev_b->ndev = ndev_b;
	midev_b->client = mcli;
	midev_b->allow_rx = true;

	mctp_i2c_device_select(mcli, midev_a);

	val = 0;
	mctp_i2c_slave_cb(&client, I2C_SLAVE_WRITE_REQUESTED, &val);
	val = 0xa7;
	mctp_i2c_slave_cb(&client, I2C_SLAVE_WRITE_RECEIVED, &val);

	kunit_info(test,
		   "KI-04 mux-attribution proof: selected=A intended_unsolicited=B byte=0x%02x",
		   val);
	kunit_info(test,
		   "KI-04 mux-attribution proof: rx_pos_A=%zu rx_pos_B=%zu first_data_A=0x%02x",
		   midev_a->rx_pos, midev_b->rx_pos, midev_a->rx_buffer[1]);
	kunit_info(test,
		   "KI-04 mux-attribution proof: slave callback has no mux-channel input and attributes RX to mcli->sel");

	KUNIT_EXPECT_EQ_MSG(test, midev_a->rx_pos, (size_t)2,
			    "KI-04: selected device did not receive callback bytes");
	KUNIT_EXPECT_EQ(test, midev_a->rx_buffer[0], (u8)(mcli->lladdr << 1));
	KUNIT_EXPECT_EQ(test, midev_a->rx_buffer[1], (u8)0xa7);
	KUNIT_EXPECT_EQ_MSG(test, midev_b->rx_pos, (size_t)0,
			    "KI-04: non-selected device received bytes despite no channel selector");
}

static void mctp_i2c_test_flow_release_overflow_clamps_and_unlocks(struct kunit *test)
{
	struct i2c_adapter adapter = { 0 };
	struct mctp_i2c_dev *midev;
	unsigned int unlocks = 0;

	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo_data = &unlocks;

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);
	spin_lock_init(&midev->lock);
	midev->adapter = &adapter;
	midev->i2c_lock_count = 1;
	midev->release_count = 3;

	kunit_info(test,
		   "KI-06 release-overflow proof: before release i2c_lock_count=%d release_count=%d unlocks=%u",
		   midev->i2c_lock_count, midev->release_count, unlocks);
	mctp_i2c_flow_release(midev);
	kunit_info(test,
		   "KI-06 release-overflow proof: after release i2c_lock_count=%d release_count=%d unlocks=%u",
		   midev->i2c_lock_count, midev->release_count, unlocks);
	kunit_info(test,
		   "KI-06 release-overflow proof: overflow is clamped to held locks, then the bus is unlocked once");

	KUNIT_EXPECT_EQ_MSG(test, midev->i2c_lock_count, 0,
			    "KI-06: I2C bus remained logically locked after overflow clamp");
	KUNIT_EXPECT_EQ_MSG(test, midev->release_count, 0,
			    "KI-06: release_count was not cleared after flow release");
	KUNIT_EXPECT_EQ_MSG(test, unlocks, 1U,
			    "KI-06: clamped overflow did not unlock the held bus");
}

struct mctp_i2c_tx_scratch_test {
	bool cloned;
	bool max_len;
};

static const struct mctp_i2c_tx_scratch_test mctp_i2c_tx_scratch_tests[] = {
	{ .cloned = false, .max_len = false },
	{ .cloned = true, .max_len = false },
	{ .cloned = false, .max_len = true },
	{ .cloned = true, .max_len = true },
};

static void
mctp_i2c_tx_scratch_test_to_desc(const struct mctp_i2c_tx_scratch_test *t,
				 char *desc)
{
	sprintf(desc, "%s%s", t->cloned ? "cloned" : "uncloned",
		t->max_len ? "_maxlen" : "_short");
}

KUNIT_ARRAY_PARAM(mctp_i2c_tx_scratch, mctp_i2c_tx_scratch_tests,
		  mctp_i2c_tx_scratch_test_to_desc);

static void mctp_i2c_test_xmit_uses_scratch_without_mutating_skb(struct kunit *test)
{
	const struct mctp_i2c_tx_scratch_test *params = test->param_value;
	const unsigned int payload_len = params->max_len ?
		MCTP_I2C_MAXBLOCK - sizeof(struct mctp_hdr) - 1 : 2;
	const unsigned int skb_len = sizeof(struct mctp_i2c_hdr) +
				     sizeof(struct mctp_hdr) + payload_len;
	struct mctp_i2c_test_xfer_ctx ctx = { 0 };
	struct i2c_adapter adapter = { 0 };
	struct mctp_i2c_client *mcli;
	struct net_device *ndev;
	struct mctp_i2c_dev *midev;
	struct sk_buff *clone = NULL;
	struct sk_buff *skb;
	unsigned int orig_len;
	u8 orig[MCTP_I2C_BUFSZ];
	u8 expected_pec;

	ndev = alloc_netdev(0, "mctpi2ctest%d", NET_NAME_ENUM,
			    mctp_i2c_net_setup);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ndev);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_free_netdev,
						  ndev), 0);

	adapter.lock_ops = &mctp_i2c_test_lock_ops;
	adapter.algo = &mctp_i2c_test_algorithm;
	adapter.algo_data = &ctx;
	adapter.timeout = HZ;
	mutex_init(&adapter.hold_lock);

	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, midev);

	mcli = kunit_kzalloc(test, sizeof(*mcli), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, mcli);

	spin_lock_init(&mcli->sel_lock);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test,
						  mctp_i2c_test_device_deselect,
						  mcli), 0);
	spin_lock_init(&midev->lock);
	spin_lock_init(&midev->error_inject.lock);
	midev->ndev = ndev;
	midev->adapter = &adapter;
	midev->client = mcli;
	midev->flows_enabled = false;

	skb = __mctp_i2c_test_create_tx_skb(test, 9, NULL, MCTP_I2C_BUFSZ,
					    payload_len);
	KUNIT_EXPECT_EQ(test, skb->len, skb_len);

	if (params->cloned) {
		clone = skb_clone(skb, GFP_KERNEL);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, clone);
		KUNIT_ASSERT_EQ(test,
				kunit_add_action_or_reset(test,
							  kunit_action_kfree_skb,
							  clone), 0);
	}

	orig_len = skb->len;
	memcpy(orig, skb->data, orig_len);
	expected_pec = i2c_smbus_pec(0, orig, orig_len);

	kunit_info(test,
		   "KI-05 PEC scratch proof: cloned=%u max_len=%u skb_len=%u expected_pec=0x%02x",
		   params->cloned, params->max_len, orig_len, expected_pec);
	mctp_i2c_xmit(midev, skb);
	kunit_info(test,
		   "KI-05 PEC scratch proof: transfers=%u tx_len=%u skb_len_after=%u clone_present=%u tx_packets=%lu tx_errors=%lu",
		   ctx.transfers, ctx.len, skb->len, !!clone,
		   ndev->stats.tx_packets, ndev->stats.tx_errors);
	kunit_info(test,
		   "KI-05 PEC scratch proof: fixed result transmits PEC from tx_scratch while original skb/clone remain unchanged");

	KUNIT_EXPECT_EQ(test, ctx.transfers, 1U);
	KUNIT_EXPECT_EQ(test, ctx.addr, (u16)(0x70 >> 1));
	KUNIT_EXPECT_EQ(test, ctx.len, (u16)orig_len);
	KUNIT_EXPECT_MEMEQ(test, ctx.buf, orig + 1, orig_len - 1);
	KUNIT_EXPECT_EQ(test, ctx.buf[orig_len - 1], expected_pec);
	KUNIT_EXPECT_EQ(test, ctx.unlocks, 1U);
	KUNIT_EXPECT_EQ(test, midev->i2c_lock_count, 0);

	KUNIT_EXPECT_EQ(test, skb->len, orig_len);
	KUNIT_EXPECT_MEMEQ(test, skb->data, orig, orig_len);
	if (clone) {
		KUNIT_EXPECT_EQ(test, clone->len, orig_len);
		KUNIT_EXPECT_MEMEQ(test, clone->data, orig, orig_len);
	}

	KUNIT_EXPECT_EQ(test, ndev->stats.tx_packets, 1UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_bytes, (unsigned long)orig_len);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_errors, 0UL);
	KUNIT_EXPECT_EQ(test, ndev->stats.tx_dropped, 0UL);
}

static struct kunit_case mctp_i2c_fmea_test_cases[] = {
	KUNIT_CASE(mctp_i2c_test_invalid_key_counts_drop_and_preserves_skb),
	KUNIT_CASE(mctp_i2c_test_invalid_state_counts_drop_and_preserves_skb),
	KUNIT_CASE(mctp_i2c_test_invalidate_manual_active_flow_resets_for_reuse),
	KUNIT_CASE(mctp_i2c_test_xmit_error_resets_manual_active_flow),
	KUNIT_CASE(mctp_i2c_test_manual_flow_error_drops_remainder_and_recovers_on_som),
	KUNIT_CASE(mctp_i2c_test_slave_rx_uses_last_selected_device),
	KUNIT_CASE(mctp_i2c_test_flow_release_overflow_clamps_and_unlocks),
	KUNIT_CASE_PARAM(mctp_i2c_test_xmit_uses_scratch_without_mutating_skb,
			 mctp_i2c_tx_scratch_gen_params),
	{}
};

static struct kunit_suite mctp_i2c_fmea_test_suite = {
	.name = "mctp-i2c-fmea",
	.test_cases = mctp_i2c_fmea_test_cases,
};

kunit_test_suite(mctp_i2c_fmea_test_suite);
