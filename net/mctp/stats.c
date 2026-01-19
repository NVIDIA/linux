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

	/* Detailed RX drops */
	stats.rx_dropped_no_route = atomic64_read(&ns->rx_dropped_no_route);
	stats.rx_dropped_no_memory = atomic64_read(&ns->rx_dropped_no_memory);
	stats.rx_dropped_seq_mismatch = atomic64_read(&ns->rx_dropped_seq_mismatch);
	stats.rx_dropped_tag_mismatch = atomic64_read(&ns->rx_dropped_tag_mismatch);
	stats.rx_dropped_queue_full = atomic64_read(&ns->rx_dropped_queue_full);
	stats.rx_dropped_invalid_header = atomic64_read(&ns->rx_dropped_invalid_header);
	stats.rx_dropped_permission = atomic64_read(&ns->rx_dropped_permission);
	stats.rx_dropped_timeout = atomic64_read(&ns->rx_dropped_timeout);

	/* Detailed TX drops */
	stats.tx_dropped_no_route = atomic64_read(&ns->tx_dropped_no_route);
	stats.tx_dropped_mtu_exceeded = atomic64_read(&ns->tx_dropped_mtu_exceeded);
	stats.tx_dropped_no_memory = atomic64_read(&ns->tx_dropped_no_memory);
	stats.tx_dropped_queue_full = atomic64_read(&ns->tx_dropped_queue_full);
	stats.tx_dropped_device_down = atomic64_read(&ns->tx_dropped_device_down);
	stats.tx_dropped_tag_exhaustion = atomic64_read(&ns->tx_dropped_tag_exhaustion);
	stats.tx_dropped_permission = atomic64_read(&ns->tx_dropped_permission);

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
	seq_printf(m, "  Messages: %llu\n", atomic64_read(&ns->tx_messages));
	seq_printf(m, "  Errors:   %llu\n", atomic64_read(&ns->tx_errors));
	seq_printf(m, "  Drops:    %llu\n", atomic64_read(&ns->tx_drops));
	seq_printf(m, "\n");

	seq_printf(m, "RX Statistics:\n");
	seq_printf(m, "  Bytes:    %llu\n", atomic64_read(&ns->rx_bytes));
	seq_printf(m, "  Messages: %llu\n", atomic64_read(&ns->rx_messages));
	seq_printf(m, "  Errors:   %llu\n", atomic64_read(&ns->rx_errors));
	seq_printf(m, "  Drops:    %llu\n", atomic64_read(&ns->rx_drops));
	seq_printf(m, "\n");

	seq_printf(m, "TX Drop Reasons:\n");
	seq_printf(m, "  No route:      %llu\n", atomic64_read(&ns->tx_dropped_no_route));
	seq_printf(m, "  MTU exceeded:  %llu\n", atomic64_read(&ns->tx_dropped_mtu_exceeded));
	seq_printf(m, "  No memory:     %llu\n", atomic64_read(&ns->tx_dropped_no_memory));
	seq_printf(m, "  Queue full:    %llu\n", atomic64_read(&ns->tx_dropped_queue_full));
	seq_printf(m, "  Device down:   %llu\n", atomic64_read(&ns->tx_dropped_device_down));
	seq_printf(m, "  Tag exhaust:   %llu\n", atomic64_read(&ns->tx_dropped_tag_exhaustion));
	seq_printf(m, "  Permission:    %llu\n", atomic64_read(&ns->tx_dropped_permission));
	seq_printf(m, "\n");

	seq_printf(m, "RX Drop Reasons:\n");
	seq_printf(m, "  No route:      %llu\n", atomic64_read(&ns->rx_dropped_no_route));
	seq_printf(m, "  No memory:     %llu\n", atomic64_read(&ns->rx_dropped_no_memory));
	seq_printf(m, "  Seq mismatch:  %llu\n", atomic64_read(&ns->rx_dropped_seq_mismatch));
	seq_printf(m, "  Tag mismatch:  %llu\n", atomic64_read(&ns->rx_dropped_tag_mismatch));
	seq_printf(m, "  Queue full:    %llu\n", atomic64_read(&ns->rx_dropped_queue_full));
	seq_printf(m, "  Invalid hdr:   %llu\n", atomic64_read(&ns->rx_dropped_invalid_header));
	seq_printf(m, "  Permission:    %llu\n", atomic64_read(&ns->rx_dropped_permission));
	seq_printf(m, "  Timeout:       %llu\n", atomic64_read(&ns->rx_dropped_timeout));

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

/* Helper to find peer EID for connected sockets */
/* static int mctp_sock_get_peer_eid(struct sock *sk, mctp_eid_t *peer)
 * {
 *	// TODO: Implement connected socket logic if needed.
 *	// Currently we don't have a direct 'peer' field in mctp_sock
 *	// for connected datagram sockets (unlike TCP).
 *	// We would need to inspect the first key or a saved peer address.
 *	*peer = 0; // Unknown
 *	return 0;
 * }
 */

/* Helper to map MCTP message type to name */
static const char *mctp_msg_type_name(u8 type)
{
	switch (type) {
	case 0x00: return "Control";
	case 0x01: return "PLDM";
	case 0x02: return "NC-SI";
	case 0x03: return "Ethernet";
	case 0x04: return "NVMe";
	case 0x05: return "SPDM";
	case 0x7E: return "Vendor(7E)";
	case 0x7F: return "Vendor(7F)";
	default:   return NULL;
	}
}

/* Per-socket statistics display */
static int mctp_sockets_show(struct seq_file *m, void *v)
{
	struct net *net = m->private;
	struct sock *sk;
	int i = 0;

	/* Header for socket list */
	seq_printf(m, "Socket List:\n");
	seq_printf(m, "  PID    Net  Type         State      TX Msgs   RX Msgs\n");
	seq_printf(m, "  ---    ---  ----         -----      -------   -------\n");

	rcu_read_lock();
	sk_for_each_rcu(sk, &net->mctp.binds) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
		const char *type_name;
		char type_buf[8];

		type_name = mctp_msg_type_name(msk->bind_type);
		if (!type_name) {
			snprintf(type_buf, sizeof(type_buf), "0x%02x", msk->bind_type);
			type_name = type_buf;
		}

		/* Line 1: Basic socket info and summary stats */
		seq_printf(m, "  %-6d %-4u %-12s %-10s %9llu %9llu\n",
			   msk->pid,
			   msk->bind_net,
			   type_name,
			   sk_hashed(sk) ? "BOUND" : "UNBOUND",
			   msk->stats.tx_messages,
			   msk->stats.rx_messages);
		
		/* Detailed drops - only if non-zero */
		spin_lock_bh(&msk->stats_lock);
		
		if (msk->stats.tx_drops) {
			seq_printf(m, "    TX Drops: %llu (No Route:%llu, MTU:%llu, Mem:%llu, QFull:%llu, Down:%llu, Tag:%llu, Perm:%llu)\n",
				   msk->stats.tx_drops,
				   msk->stats.tx_dropped_no_route,
				   msk->stats.tx_dropped_mtu_exceeded,
				   msk->stats.tx_dropped_no_memory,
				   msk->stats.tx_dropped_queue_full,
				   msk->stats.tx_dropped_device_down,
				   msk->stats.tx_dropped_tag_exhaustion,
				   msk->stats.tx_dropped_permission);
		}

		if (msk->stats.rx_drops) {
			seq_printf(m, "    RX Drops: %llu (No Route:%llu, Mem:%llu, Seq:%llu, Tag:%llu, QFull:%llu, Hdr:%llu, Perm:%llu, Time:%llu)\n",
				   msk->stats.rx_drops,
				   msk->stats.rx_dropped_no_route,
				   msk->stats.rx_dropped_no_memory,
				   msk->stats.rx_dropped_seq_mismatch,
				   msk->stats.rx_dropped_tag_mismatch,
				   msk->stats.rx_dropped_queue_full,
				   msk->stats.rx_dropped_invalid_header,
				   msk->stats.rx_dropped_permission,
				   msk->stats.rx_dropped_timeout);
		}
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
