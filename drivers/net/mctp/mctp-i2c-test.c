// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>

static void mctp_i2c_test_netdev_setup(struct net_device *ndev)
{
	ndev->type = ARPHRD_MCTP;
}

static void mctp_i2c_test_rx_overflow_counted_once(struct kunit *test)
{
	struct mctp_i2c_client *mcli;
	struct mctp_i2c_dev *midev;
	struct i2c_client *client;
	struct net_device *ndev;
	unsigned int bytes_written, overflow_bytes;
	int i, rc;
	u8 val;

	mcli = kunit_kzalloc(test, sizeof(*mcli), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, mcli);
	midev = kunit_kzalloc(test, sizeof(*midev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, midev);
	client = kunit_kzalloc(test, sizeof(*client), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, client);

	ndev = alloc_netdev(0, "mctpi2ctest%d", NET_NAME_ENUM,
			    mctp_i2c_test_netdev_setup);
	KUNIT_ASSERT_NOT_NULL(test, ndev);

	spin_lock_init(&mcli->sel_lock);
	INIT_LIST_HEAD(&mcli->devs);
	mcli->lladdr = 0x10;
	mcli->sel = midev;
	i2c_set_clientdata(client, mcli);

	spin_lock_init(&midev->lock);
	midev->ndev = ndev;
	midev->allow_rx = true;

	val = 0;
	rc = mctp_i2c_slave_cb(client, I2C_SLAVE_WRITE_REQUESTED, &val);
	KUNIT_EXPECT_EQ(test, rc, 0);
	kunit_info(test,
		   "KI-03 i2c overflow proof: WRITE_REQUESTED started packet, rx_pos=%zu, buffer=%u",
		   midev->rx_pos, MCTP_I2C_BUFSZ);
	kunit_info(test,
		   "KI-03 i2c overflow proof: packet starts with generated dest slave byte; following bytes are injected via WRITE_RECEIVED");

	bytes_written = MCTP_I2C_BUFSZ + 2;
	overflow_bytes = bytes_written - (MCTP_I2C_BUFSZ - midev->rx_pos);
	kunit_info(test,
		   "KI-03 i2c overflow proof: injecting %u bytes after start, which is %u bytes beyond remaining buffer capacity",
		   bytes_written, overflow_bytes);
	for (i = 0; i < bytes_written; i++) {
		val = i == 0 ? MCTP_I2C_COMMANDCODE : 0;
		rc = mctp_i2c_slave_cb(client, I2C_SLAVE_WRITE_RECEIVED,
				       &val);
		KUNIT_EXPECT_EQ(test, rc, 0);
	}

	kunit_info(test,
		   "KI-03 i2c overflow proof: wrote %u bytes after start; rx_pos=%zu rx_over_errors=%lu rx_length_errors=%lu before STOP",
		   bytes_written, midev->rx_pos, ndev->stats.rx_over_errors,
		   ndev->stats.rx_length_errors);
	kunit_info(test,
		   "KI-03 i2c overflow proof: buggy code increments rx_over_errors once per overflow byte before STOP");
	kunit_info(test,
		   "KI-03 i2c overflow proof: fixed code increments rx_over_errors once per packet and records overflow state");

	rc = mctp_i2c_slave_cb(client, I2C_SLAVE_STOP, &val);
	kunit_info(test,
		   "KI-03 i2c overflow proof: STOP rc=%d rx_over_errors=%lu rx_length_errors=%lu",
		   rc, ndev->stats.rx_over_errors,
		   ndev->stats.rx_length_errors);
	kunit_info(test,
		   "KI-03 i2c overflow proof: if STOP enters mctp_i2c_recv(), the saturated packet also increments rx_length_errors");
	kunit_info(test,
		   "KI-03 i2c overflow proof: fixed result for one oversized packet is rx_over_errors=1 and rx_length_errors=0");
	kunit_info(test,
		   "KI-03 i2c overflow proof: rx_over_errors > 1 or rx_length_errors > 0 proves double-counting/noise for one bad packet");

	KUNIT_EXPECT_EQ_MSG(test, rc, -EINVAL,
			    "KI-03: oversized packet STOP should be rejected");
	KUNIT_EXPECT_EQ_MSG(test, ndev->stats.rx_over_errors, 1UL,
			    "KI-03: one oversized packet should count one rx_over_errors, got %lu",
			    ndev->stats.rx_over_errors);
	KUNIT_EXPECT_EQ_MSG(test, ndev->stats.rx_length_errors, 0UL,
			    "KI-03: overflowed packet should not also count rx_length_errors, got %lu",
			    ndev->stats.rx_length_errors);

	free_netdev(ndev);
}

static struct kunit_case mctp_i2c_test_cases[] = {
	KUNIT_CASE(mctp_i2c_test_rx_overflow_counted_once),
	{}
};

static struct kunit_suite mctp_i2c_test_suite = {
	.name = "mctp-i2c",
	.test_cases = mctp_i2c_test_cases,
};

kunit_test_suite(mctp_i2c_test_suite);
