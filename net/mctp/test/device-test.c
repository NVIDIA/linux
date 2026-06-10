// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>

#include "utils.h"

static struct sk_buff *mctp_test_addr_nlmsg(struct kunit *test,
					    struct socket *sock,
					    struct nlmsghdr **nlhp,
					    int ifindex, mctp_eid_t eid,
					    int msgtype)
{
	struct ifaddrmsg *ifm;
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	size_t len;
	int rc;

	len = NLMSG_ALIGN(sizeof(*ifm)) + nla_total_size(sizeof(eid));
	skb = nlmsg_new(len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);

	nlh = nlmsg_put(skb, 0, 1, msgtype, sizeof(*ifm), 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, nlh);

	ifm = nlmsg_data(nlh);
	memset(ifm, 0, sizeof(*ifm));
	ifm->ifa_family = AF_MCTP;
	ifm->ifa_index = ifindex;

	rc = nla_put_u8(skb, IFA_LOCAL, eid);
	KUNIT_ASSERT_EQ(test, rc, 0);
	nlmsg_end(skb, nlh);

	skb->sk = sock->sk;
	*nlhp = nlh;
	return skb;
}

static void mctp_test_newaddr_rolls_back_on_route_failure(struct kunit *test)
{
	const mctp_eid_t eid = 42;
	struct mctp_test_dev *dev;
	struct nlmsghdr *nlh;
	struct socket *sock;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	rc = sock_create_kern(&init_net, AF_NETLINK, SOCK_RAW, NETLINK_ROUTE,
			      &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	rtnl_lock();
	rc = mctp_route_add_local(dev->mdev, eid);
	if (rc) {
		rtnl_unlock();
		sock_release(sock);
		mctp_test_destroy_dev(dev);
		KUNIT_ASSERT_EQ(test, rc, 0);
		return;
	}

	skb = mctp_test_addr_nlmsg(test, sock, &nlh, dev->ndev->ifindex, eid,
				   RTM_NEWADDR);
	kunit_info(test,
		   "KD-02 route-add rollback proof: injected existing local route for EID %u before RTM_NEWADDR",
		   eid);
	rc = mctp_rtm_newaddr(skb, nlh, NULL);
	kunit_info(test,
		   "KD-02 route-add rollback proof: rc=%d num_addrs=%zu addrs=%p",
		   rc, dev->mdev->num_addrs, dev->mdev->addrs);
	kunit_info(test,
		   "KD-02 route-add rollback proof: fixed result is -EEXIST with no address stored when route add fails");

	KUNIT_EXPECT_EQ_MSG(test, rc, -EEXIST,
			    "KD-02: duplicate local route did not fail as expected");
	KUNIT_EXPECT_EQ_MSG(test, dev->mdev->num_addrs, (size_t)0,
			    "KD-02: address remained present after route failure");
	KUNIT_EXPECT_PTR_EQ_MSG(test, dev->mdev->addrs, NULL,
				"KD-02: address array leaked after rollback");

	rc = mctp_route_remove_local(dev->mdev, eid);
	KUNIT_EXPECT_EQ(test, rc, 0);
	rtnl_unlock();

	kfree_skb(skb);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_newaddr_rollback_preserves_existing_addr(struct kunit *test)
{
	const mctp_eid_t old_eid = 41;
	const mctp_eid_t new_eid = 42;
	struct mctp_test_dev *dev;
	struct nlmsghdr *nlh;
	struct socket *sock;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	rc = sock_create_kern(&init_net, AF_NETLINK, SOCK_RAW, NETLINK_ROUTE,
			      &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	dev->mdev->addrs = kmalloc(sizeof(old_eid), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev->mdev->addrs);
	dev->mdev->addrs[0] = old_eid;
	dev->mdev->num_addrs = 1;

	rtnl_lock();
	rc = mctp_route_add_local(dev->mdev, new_eid);
	if (rc) {
		rtnl_unlock();
		sock_release(sock);
		mctp_test_destroy_dev(dev);
		KUNIT_ASSERT_EQ(test, rc, 0);
		return;
	}

	skb = mctp_test_addr_nlmsg(test, sock, &nlh, dev->ndev->ifindex,
				   new_eid, RTM_NEWADDR);
	kunit_info(test,
		   "KD-02 rollback-preserve proof: old_eid=%u new_eid=%u existing route forces route add failure",
		   old_eid, new_eid);
	rc = mctp_rtm_newaddr(skb, nlh, NULL);
	kunit_info(test,
		   "KD-02 rollback-preserve proof: rc=%d num_addrs=%zu addr0=%u",
		   rc, dev->mdev->num_addrs,
		   dev->mdev->num_addrs ? dev->mdev->addrs[0] : 0);
	kunit_info(test,
		   "KD-02 rollback-preserve proof: fixed result keeps old address and does not add failed EID");

	KUNIT_EXPECT_EQ_MSG(test, rc, -EEXIST,
			    "KD-02: duplicate route did not fail");
	KUNIT_EXPECT_EQ_MSG(test, dev->mdev->num_addrs, (size_t)1,
			    "KD-02: rollback changed existing address count");
	KUNIT_EXPECT_EQ_MSG(test, dev->mdev->addrs[0], old_eid,
			    "KD-02: rollback corrupted existing address");

	rc = mctp_route_remove_local(dev->mdev, new_eid);
	KUNIT_EXPECT_EQ(test, rc, 0);
	rtnl_unlock();

	kfree_skb(skb);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_deladdr_allows_missing_local_route(struct kunit *test)
{
	const mctp_eid_t eid = 42;
	struct mctp_test_dev *dev;
	struct nlmsghdr *nlh;
	struct socket *sock;
	struct sk_buff *skb;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	rc = sock_create_kern(&init_net, AF_NETLINK, SOCK_RAW, NETLINK_ROUTE,
			      &sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	dev->mdev->addrs = kmalloc(sizeof(eid), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev->mdev->addrs);
	dev->mdev->addrs[0] = eid;
	dev->mdev->num_addrs = 1;

	skb = mctp_test_addr_nlmsg(test, sock, &nlh, dev->ndev->ifindex, eid,
				   RTM_DELADDR);

	rtnl_lock();
	rc = mctp_route_add_local(dev->mdev, eid);
	if (rc) {
		KUNIT_FAIL(test, "KD-03: failed to add local route: %d", rc);
		rtnl_unlock();
		goto out;
	}
	rc = mctp_route_remove_local(dev->mdev, eid);
	if (rc) {
		KUNIT_FAIL(test, "KD-03: failed to pre-remove local route: %d", rc);
		rtnl_unlock();
		goto out;
	}

	kunit_info(test,
		   "KD-03 missing-route proof: local address EID %u remains after its local route was already removed",
		   eid);
	rc = mctp_rtm_deladdr(skb, nlh, NULL);
	rtnl_unlock();
	kunit_info(test,
		   "KD-03 missing-route proof: RTM_DELADDR rc=%d num_addrs=%zu",
		   rc, dev->mdev->num_addrs);
	kunit_info(test,
		   "KD-03 missing-route proof: -ENOENT from local route removal is a valid pre-existing state, so warning-only fix would add noise");

	KUNIT_EXPECT_EQ_MSG(test, rc, 0,
			    "KD-03: address deletion failed when local route was already absent");
	KUNIT_EXPECT_EQ_MSG(test, dev->mdev->num_addrs, (size_t)0,
			    "KD-03: address was not deleted after missing local route");

out:
	kfree_skb(skb);
	sock_release(sock);
	mctp_test_destroy_dev(dev);
}

static void mctp_test_deladdr_removes_matching_key(struct kunit *test)
{
	const mctp_eid_t eid = 42;
	struct mctp_sk_key *key;
	struct mctp_test_dev *dev;
	struct mctp_sock *msk;
	struct nlmsghdr *nlh;
	struct socket *nl_sock;
	struct socket *mctp_sock;
	struct sk_buff *skb;
	u8 tag;
	int rc;

	dev = mctp_test_create_dev();
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	rc = sock_create_kern(&init_net, AF_NETLINK, SOCK_RAW, NETLINK_ROUTE,
			      &nl_sock);
	KUNIT_ASSERT_EQ(test, rc, 0);

	rc = sock_create_kern(&init_net, AF_MCTP, SOCK_DGRAM, 0, &mctp_sock);
	KUNIT_ASSERT_EQ(test, rc, 0);
	WRITE_ONCE(mctp_sock->sk->sk_bound_dev_if, dev->ndev->ifindex);
	msk = container_of(mctp_sock->sk, struct mctp_sock, sk);

	key = mctp_alloc_local_tag(msk, MCTP_INITIAL_DEFAULT_NET, eid, 9,
				   true, &tag, MCTP_DEFAULT_LIFETIME);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, key);

	dev->mdev->addrs = kmalloc(sizeof(eid), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev->mdev->addrs);
	dev->mdev->addrs[0] = eid;
	dev->mdev->num_addrs = 1;

	skb = mctp_test_addr_nlmsg(test, nl_sock, &nlh, dev->ndev->ifindex,
				   eid, RTM_DELADDR);
	kunit_info(test,
		   "KR-09/KD-01 stale-key proof: key valid=%d local=%u peer=%u before deleting local EID %u",
		   key->valid, key->local_addr, key->peer_addr, eid);
	rtnl_lock();
	rc = mctp_rtm_deladdr(skb, nlh, NULL);
	rtnl_unlock();
	kunit_info(test,
		   "KR-09/KD-01 stale-key proof: rc=%d num_addrs=%zu key_valid=%d key_unhashed=%d",
		   rc, dev->mdev->num_addrs, key->valid,
		   hlist_unhashed(&key->hlist));
	kunit_info(test,
		   "KR-09/KD-01 stale-key proof: fixed result removes the EID and invalidates matching socket keys");

	KUNIT_EXPECT_EQ_MSG(test, rc, 0,
			    "KR-09/KD-01: RTM_DELADDR failed");
	KUNIT_EXPECT_EQ_MSG(test, dev->mdev->num_addrs, (size_t)0,
			    "KR-09/KD-01: EID was not deleted");
	KUNIT_EXPECT_FALSE_MSG(test, key->valid,
			       "KR-09/KD-01: key stayed valid after EID deletion");
	KUNIT_EXPECT_TRUE_MSG(test, hlist_unhashed(&key->hlist),
			      "KR-09/KD-01: stale key stayed linked after EID deletion");

	mctp_key_unref(key);
	kfree_skb(skb);
	sock_release(mctp_sock);
	sock_release(nl_sock);
	mctp_test_destroy_dev(dev);
}

static struct kunit_case mctp_device_test_cases[] = {
	KUNIT_CASE(mctp_test_newaddr_rolls_back_on_route_failure),
	KUNIT_CASE(mctp_test_newaddr_rollback_preserves_existing_addr),
	KUNIT_CASE(mctp_test_deladdr_allows_missing_local_route),
	KUNIT_CASE(mctp_test_deladdr_removes_matching_key),
	{}
};

static struct kunit_suite mctp_device_test_suite = {
	.name = "mctp-device",
	.test_cases = mctp_device_test_cases,
};

kunit_test_suite(mctp_device_test_suite);
