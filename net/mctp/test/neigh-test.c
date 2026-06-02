// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>

#include "utils.h"

static struct mctp_neigh *mctp_test_alloc_neigh(struct mctp_test_dev *dev,
						mctp_eid_t eid)
{
	struct mctp_neigh *neigh;

	neigh = kzalloc(sizeof(*neigh), GFP_KERNEL);
	if (!neigh)
		return NULL;

	INIT_LIST_HEAD(&neigh->list);
	neigh->dev = dev->mdev;
	mctp_dev_hold(neigh->dev);
	neigh->eid = eid;
	neigh->source = MCTP_NEIGH_STATIC;

	return neigh;
}

static void mctp_test_free_neigh(struct mctp_neigh *neigh)
{
	mctp_dev_put(neigh->dev);
	kfree(neigh);
}

static unsigned int mctp_test_neigh_list_count(struct net *net)
{
	struct mctp_neigh *neigh;
	unsigned int count = 0;

	list_for_each_entry(neigh, &net->mctp.neighbours, list)
		count++;

	return count;
}

static void mctp_test_neigh_net_exit_unlinks_neighbours(struct kunit *test)
{
	struct mctp_neigh *neigh1, *neigh2, *neigh, *tmp;
	struct mctp_test_dev *dev;
	unsigned int before, after;
	struct net *net;

	net = kunit_kzalloc(test, sizeof(*net), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, net);
	KUNIT_ASSERT_EQ(test, mctp_neigh_net_init(net), 0);

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	neigh1 = mctp_test_alloc_neigh(dev, 8);
	if (!neigh1) {
		mctp_test_destroy_dev(dev);
		KUNIT_FAIL_AND_ABORT(test, "failed to allocate first neighbour");
	}

	neigh2 = mctp_test_alloc_neigh(dev, 9);
	if (!neigh2) {
		mctp_test_free_neigh(neigh1);
		mctp_test_destroy_dev(dev);
		KUNIT_FAIL_AND_ABORT(test, "failed to allocate second neighbour");
	}

	list_add_rcu(&neigh1->list, &net->mctp.neighbours);
	kunit_info(test,
		   "KR-08 neigh net_exit proof: added neighbour eid=%u; callback would free it after an RCU grace period",
		   neigh1->eid);
	list_add_rcu(&neigh2->list, &net->mctp.neighbours);
	kunit_info(test,
		   "KR-08 neigh net_exit proof: added neighbour eid=%u; KUnit keeps the device alive for safe inspection",
		   neigh2->eid);

	before = mctp_test_neigh_list_count(net);
	kunit_info(test,
		   "KR-08 neigh net_exit proof: inserted %u neighbours into net->mctp.neighbours",
		   before);
	kunit_info(test,
		   "KR-08 neigh net_exit proof: calling mctp_neigh_net_exit(); fixed code must list_del_rcu() before call_rcu()");

	mctp_neigh_net_exit(net);

	after = mctp_test_neigh_list_count(net);
	kunit_info(test,
		   "KR-08 neigh net_exit proof: before=%u after=%u list_empty=%u",
		   before, after, list_empty(&net->mctp.neighbours));
	kunit_info(test,
		   "KR-08 neigh net_exit proof: after>0 means neighbours scheduled for RCU free are still reachable from net->mctp.neighbours");
	kunit_info(test,
		   "KR-08 neigh net_exit proof: a concurrent RCU lookup can traverse stale neighbour list nodes after the callback frees memory");
	kunit_info(test,
		   "KR-08 neigh net_exit proof: fixed result is after=0 and list_empty=1 immediately after net_exit");

	KUNIT_EXPECT_EQ_MSG(test, after, 0U,
			    "KR-08: mctp_neigh_net_exit() queued neighbours for RCU free but left %u stale entries linked",
			    after);
	KUNIT_EXPECT_TRUE_MSG(test, list_empty(&net->mctp.neighbours),
			      "KR-08: net->mctp.neighbours is not empty after net_exit");

	if (!list_empty(&net->mctp.neighbours)) {
		list_for_each_entry_safe(neigh, tmp, &net->mctp.neighbours, list)
			list_del_rcu(&neigh->list);
	}

	rcu_barrier();
	mctp_test_destroy_dev(dev);
}

static struct kunit_case mctp_neigh_test_cases[] = {
	KUNIT_CASE(mctp_test_neigh_net_exit_unlinks_neighbours),
	{}
};

static struct kunit_suite mctp_neigh_test_suite = {
	.name = "mctp-neigh",
	.test_cases = mctp_neigh_test_cases,
};

kunit_test_suite(mctp_neigh_test_suite);
