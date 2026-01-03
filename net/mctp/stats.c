// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP Socket Statistics
 *
 * Provides global statistics via:
 * 1. Generic Netlink interface (primary)
 * 2. /proc/net/mctp/stats (convenience/debugging)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <net/genetlink.h>
#include <net/net_namespace.h>
#include <net/mctp.h>

/* Generic Netlink family for MCTP statistics
 * Enums are defined in include/uapi/linux/mctp.h
 */

static struct nla_policy mctp_stats_genl_policy[MCTP_ATTR_MAX + 1] = {
	[MCTP_ATTR_STATS] = { .type = NLA_BINARY, .len = sizeof(struct mctp_global_stats) },
};

static int mctp_stats_genl_get(struct sk_buff *skb, struct genl_info *info)
{
	struct net *net = genl_info_net(info);
	struct netns_mctp *ns = &net->mctp;
	struct mctp_global_stats stats;
	struct sk_buff *msg;
	void *hdr;
	int ret;

	/* Gather global statistics */
	stats.num_sockets = atomic_read(&ns->num_sockets);
	stats.num_bound_sockets = atomic_read(&ns->num_bound_sockets);
	stats.tx_bytes = atomic64_read(&ns->tx_bytes);
	stats.tx_packets = atomic64_read(&ns->tx_packets);
	stats.tx_messages = atomic64_read(&ns->tx_messages);
	stats.tx_errors = atomic64_read(&ns->tx_errors);
	stats.tx_drops = atomic64_read(&ns->tx_drops);
	stats.rx_bytes = atomic64_read(&ns->rx_bytes);
	stats.rx_packets = atomic64_read(&ns->rx_packets);
	stats.rx_messages = atomic64_read(&ns->rx_messages);
	stats.rx_errors = atomic64_read(&ns->rx_errors);
	stats.rx_drops = atomic64_read(&ns->rx_drops);
	stats.drops_no_route = atomic64_read(&ns->drops_no_route);
	stats.drops_mtu_exceeded = atomic64_read(&ns->drops_mtu_exceeded);
	stats.drops_no_memory = atomic64_read(&ns->drops_no_memory);
	stats.drops_seq_mismatch = atomic64_read(&ns->drops_seq_mismatch);
	stats.drops_tag_mismatch = atomic64_read(&ns->drops_tag_mismatch);
	stats.drops_queue_full = atomic64_read(&ns->drops_queue_full);
	stats.drops_device_down = atomic64_read(&ns->drops_device_down);
	stats.drops_invalid_header = atomic64_read(&ns->drops_invalid_header);
	stats.drops_permission = atomic64_read(&ns->drops_permission);

	/* Build netlink reply */
	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &mctp_genl_family, 0, MCTP_CMD_GET_STATS);
	if (!hdr) {
		ret = -EMSGSIZE;
		goto out_free;
	}

	ret = nla_put(msg, MCTP_ATTR_STATS, sizeof(stats), &stats);
	if (ret < 0)
		goto out_free;

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);

out_free:
	nlmsg_free(msg);
	return ret;
}

static const struct genl_ops mctp_genl_ops[] = {
	{
		.cmd	= MCTP_CMD_GET_STATS,
		.doit	= mctp_stats_genl_get,
		.flags	= 0,
	},
};

struct genl_family mctp_genl_family __ro_after_init = {
	.name		= "mctp",
	.version	= 1,
	.maxattr	= MCTP_ATTR_MAX,
	.policy		= mctp_stats_genl_policy,
	.ops		= mctp_genl_ops,
	.n_ops		= ARRAY_SIZE(mctp_genl_ops),
	.module		= THIS_MODULE,
	.netnsok	= true,
};

/* /proc interface for convenience and debugging */
static int mctp_stats_show(struct seq_file *m, void *v)
{
	struct net *net = m->private;
	struct netns_mctp *ns = &net->mctp;

	seq_printf(m, "Sockets:\n");
	seq_printf(m, "  Active: %u\n", atomic_read(&ns->num_sockets));
	seq_printf(m, "  Bound:  %u\n", atomic_read(&ns->num_bound_sockets));
	seq_printf(m, "\n");

	seq_printf(m, "TX Statistics:\n");
	seq_printf(m, "  Bytes:    %llu\n", atomic64_read(&ns->tx_bytes));
	seq_printf(m, "  Packets:  %llu\n", atomic64_read(&ns->tx_packets));
	seq_printf(m, "  Messages: %llu\n", atomic64_read(&ns->tx_messages));
	seq_printf(m, "  Errors:   %llu\n", atomic64_read(&ns->tx_errors));
	seq_printf(m, "  Drops:    %llu\n", atomic64_read(&ns->tx_drops));
	seq_printf(m, "\n");

	seq_printf(m, "RX Statistics:\n");
	seq_printf(m, "  Bytes:    %llu\n", atomic64_read(&ns->rx_bytes));
	seq_printf(m, "  Packets:  %llu\n", atomic64_read(&ns->rx_packets));
	seq_printf(m, "  Messages: %llu\n", atomic64_read(&ns->rx_messages));
	seq_printf(m, "  Errors:   %llu\n", atomic64_read(&ns->rx_errors));
	seq_printf(m, "  Drops:    %llu\n", atomic64_read(&ns->rx_drops));
	seq_printf(m, "\n");

	seq_printf(m, "Drop Reasons:\n");
	seq_printf(m, "  No route:      %llu\n", atomic64_read(&ns->drops_no_route));
	seq_printf(m, "  MTU exceeded:  %llu\n", atomic64_read(&ns->drops_mtu_exceeded));
	seq_printf(m, "  No memory:     %llu\n", atomic64_read(&ns->drops_no_memory));
	seq_printf(m, "  Seq mismatch:  %llu\n", atomic64_read(&ns->drops_seq_mismatch));
	seq_printf(m, "  Tag mismatch:  %llu\n", atomic64_read(&ns->drops_tag_mismatch));
	seq_printf(m, "  Queue full:    %llu\n", atomic64_read(&ns->drops_queue_full));
	seq_printf(m, "  Device down:   %llu\n", atomic64_read(&ns->drops_device_down));
	seq_printf(m, "  Invalid hdr:   %llu\n", atomic64_read(&ns->drops_invalid_header));
	seq_printf(m, "  Permission:    %llu\n", atomic64_read(&ns->drops_permission));

	return 0;
}

static int mctp_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mctp_stats_show, pde_data(inode));
}

static const struct proc_ops mctp_stats_proc_ops = {
	.proc_open	= mctp_stats_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

/* Per-socket statistics display */
static int mctp_sockets_show(struct seq_file *m, void *v)
{
	struct net *net = m->private;
	struct sock *sk;
	int i = 0;

	/* Header for socket list */
	seq_printf(m, "Socket List:\n");
	seq_printf(m, "  Sock   EID  Type  Net  State      TX Pkts   RX Pkts  TX Drops  RX Drops  UID   Inode\n");
	seq_printf(m, "  ----   ---  ----  ---  -----      -------   -------  --------  --------  ---   -----\n");

	rcu_read_lock();
	sk_for_each_rcu(sk, &net->mctp.binds) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);

		/* Line 1: Basic socket info and summary stats */
		seq_printf(m, "  %4d   %3u  0x%02x  %3u  %-9s %9llu %9llu %9llu %9llu %4u %7lu\n",
			   i,
			   msk->bind_addr,
			   msk->bind_type,
			   msk->bind_net,
			   sk_hashed(sk) ? "BOUND" : "UNBOUND",
			   msk->stats.tx_packets,
			   msk->stats.rx_packets,
			   msk->stats.tx_drops,
			   msk->stats.rx_drops,
			   from_kuid_munged(seq_user_ns(m), sock_i_uid(sk)),
			   sock_i_ino(sk));
		i++;
	}
	rcu_read_unlock();

	seq_printf(m, "\nDetailed Drops (per socket):\n");
	
	i = 0;
	rcu_read_lock();
	sk_for_each_rcu(sk, &net->mctp.binds) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);

		/* Line 2: Detailed drop reasons */
		spin_lock_bh(&msk->stats_lock);
		seq_printf(m, "  Sock %4d: no_route:%llu mtu:%llu nomem:%llu seq:%llu tag:%llu queue:%llu dev:%llu hdr:%llu perm:%llu\n",
			   i,
			   msk->stats.drops_no_route,
			   msk->stats.drops_mtu_exceeded,
			   msk->stats.drops_no_memory,
			   msk->stats.drops_seq_mismatch,
			   msk->stats.drops_tag_mismatch,
			   msk->stats.drops_queue_full,
			   msk->stats.drops_device_down,
			   msk->stats.drops_invalid_header,
			   msk->stats.drops_permission);
		spin_unlock_bh(&msk->stats_lock);

		i++;
	}
	rcu_read_unlock();

	return 0;
}

static int mctp_sockets_open(struct inode *inode, struct file *file)
{
	return single_open(file, mctp_sockets_show, pde_data(inode));
}

static const struct proc_ops mctp_sockets_proc_ops = {
	.proc_open	= mctp_sockets_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int __net_init mctp_stats_net_init(struct net *net)
{
	struct proc_dir_entry *mctp_dir;
	struct proc_dir_entry *stats_file;
	struct proc_dir_entry *sockets_file;

	/* Create /proc/net/mctp directory if it doesn't exist */
	mctp_dir = proc_mkdir("mctp", net->proc_net);
	if (!mctp_dir)
		return -ENOMEM;

	/* Create /proc/net/mctp/stats file */
	stats_file = proc_create_data("stats", 0444, mctp_dir, &mctp_stats_proc_ops, net);
	if (!stats_file) {
		remove_proc_entry("mctp", net->proc_net);
		return -ENOMEM;
	}

	/* Create /proc/net/mctp/sockets file */
	sockets_file = proc_create_data("sockets", 0444, mctp_dir, &mctp_sockets_proc_ops, net);
	if (!sockets_file) {
		remove_proc_entry("stats", mctp_dir);
		remove_proc_entry("mctp", net->proc_net);
		return -ENOMEM;
	}

	return 0;
}

static void __net_exit mctp_stats_net_exit(struct net *net)
{
	/* Remove proc entries in reverse order of creation */
	remove_proc_subtree("mctp", net->proc_net);
}

static struct pernet_operations mctp_stats_net_ops = {
	.init = mctp_stats_net_init,
	.exit = mctp_stats_net_exit,
};

int __init mctp_stats_init(void)
{
	int ret;

	/* Register Generic Netlink family */
	ret = genl_register_family(&mctp_genl_family);
	if (ret) {
		pr_err("MCTP: Failed to register netlink family: %d\n", ret);
		return ret;
	}

	/* Register /proc interface (optional, for convenience) */
	ret = register_pernet_subsys(&mctp_stats_net_ops);
	if (ret) {
		pr_warn("MCTP: Failed to register /proc stats: %d\n", ret);
		genl_unregister_family(&mctp_genl_family);
		return ret;
	}

	return 0;
}

void mctp_stats_exit(void)
{
	unregister_pernet_subsys(&mctp_stats_net_ops);
	genl_unregister_family(&mctp_genl_family);
}

