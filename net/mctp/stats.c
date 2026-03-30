// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP Socket Statistics
 *
 * Provides global statistics via /proc/net/mctp/stats (debugging)
 * and per-socket statistics via MCTP_OPT_SOCK_STATS socket option.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <net/net_namespace.h>
#include <net/mctp.h>

#include <linux/sched.h> /* for TASK_COMM_LEN */

struct mctp_sock_stats {
	u64 tx_bytes;
	u64 tx_packets;
	u64 tx_messages;
	u64 tx_errors;
	u64 tx_drops;

	u64 rx_bytes;
	u64 rx_packets;
	u64 rx_messages;
	u64 rx_errors;
	u64 rx_drops;

	/* Detailed drop reasons - RX */
	u64 rx_dropped_no_route;
	u64 rx_dropped_no_memory;
	u64 rx_dropped_seq_mismatch;
	u64 rx_dropped_tag_mismatch;
	u64 rx_dropped_queue_full;
	u64 rx_dropped_invalid_header;
	u64 rx_dropped_permission;
	u64 rx_dropped_timeout;

	/* Detailed drop reasons - TX */
	u64 tx_dropped_no_route;
	u64 tx_dropped_mtu_exceeded;
	u64 tx_dropped_no_memory;
	u64 tx_dropped_queue_full;
	u64 tx_dropped_device_down;
	u64 tx_dropped_tag_exhaustion;
	u64 tx_dropped_permission;
	u64 tx_dropped_bad_addrlen;

	/* Timestamps - unused for aggregation */
	u64 last_tx_time;
	u64 last_rx_time;
};

struct mctp_proc_stats {
	struct hlist_node hlist;
	char name[TASK_COMM_LEN];
	struct mctp_sock_stats stats;
};

static DEFINE_SPINLOCK(mctp_closed_stats_lock);
static HLIST_HEAD(mctp_closed_stats_list);
/* Maximum number of per-process entries kept in mctp_closed_stats_list.
 * When at capacity, the least-recently-used (tail) entry is evicted.
 */
static unsigned int mctp_closed_stats_max_entries = 32;
static unsigned int mctp_closed_stats_count;

void mctp_stats_aggregate_closed_sk(struct sock *sk)
{
	struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
	struct mctp_sock_stats snapshot;
	struct mctp_proc_stats *pstats = NULL, *tmp;
	char comm[TASK_COMM_LEN];
	unsigned long flags;

	/* Snapshot stats under stats_lock to avoid a race with route.c updates.
	 * We copy first, then check, so we never read fields without the lock.
	 */
	spin_lock_bh(&msk->stats_lock);
	memcpy(&snapshot, &msk->stats, sizeof(snapshot));
	spin_unlock_bh(&msk->stats_lock);

	/* If no activity, don't allocate memory to track it */
	if (snapshot.tx_messages == 0 && snapshot.rx_messages == 0 &&
	    snapshot.tx_drops == 0 && snapshot.rx_drops == 0)
		return;

	strscpy(comm, current->comm, TASK_COMM_LEN);

	spin_lock_irqsave(&mctp_closed_stats_lock, flags);

	/* Find existing entry for this process name */
	hlist_for_each_entry(tmp, &mctp_closed_stats_list, hlist) {
		if (strncmp(tmp->name, comm, TASK_COMM_LEN) == 0) {
			pstats = tmp;
			break;
		}
	}

	if (!pstats) {
		/* At capacity: evict the LRU entry (tail of list). */
		if (mctp_closed_stats_count >= mctp_closed_stats_max_entries) {
			struct mctp_proc_stats *lru = NULL, *iter;

			hlist_for_each_entry(iter, &mctp_closed_stats_list, hlist)
				lru = iter;
			if (lru) {
				hlist_del(&lru->hlist);
				kfree(lru);
				mctp_closed_stats_count--;
			}
		}

		pstats = kzalloc(sizeof(*pstats), GFP_ATOMIC);
		if (!pstats) {
			spin_unlock_irqrestore(&mctp_closed_stats_lock, flags);
			return;
		}
		strscpy(pstats->name, comm, TASK_COMM_LEN);
		hlist_add_head(&pstats->hlist, &mctp_closed_stats_list);
		mctp_closed_stats_count++;
	} else {
		/* Move to head to mark as most-recently used. */
		hlist_del(&pstats->hlist);
		hlist_add_head(&pstats->hlist, &mctp_closed_stats_list);
	}

	/* Aggregate from snapshot */
	pstats->stats.tx_bytes += snapshot.tx_bytes;
	pstats->stats.tx_packets += snapshot.tx_packets;
	pstats->stats.tx_messages += snapshot.tx_messages;
	pstats->stats.tx_errors += snapshot.tx_errors;
	pstats->stats.tx_drops += snapshot.tx_drops;

	pstats->stats.rx_bytes += snapshot.rx_bytes;
	pstats->stats.rx_packets += snapshot.rx_packets;
	pstats->stats.rx_messages += snapshot.rx_messages;
	pstats->stats.rx_errors += snapshot.rx_errors;
	pstats->stats.rx_drops += snapshot.rx_drops;

	pstats->stats.rx_dropped_no_route += snapshot.rx_dropped_no_route;
	pstats->stats.rx_dropped_no_memory += snapshot.rx_dropped_no_memory;
	pstats->stats.rx_dropped_seq_mismatch += snapshot.rx_dropped_seq_mismatch;
	pstats->stats.rx_dropped_tag_mismatch += snapshot.rx_dropped_tag_mismatch;
	pstats->stats.rx_dropped_queue_full += snapshot.rx_dropped_queue_full;
	pstats->stats.rx_dropped_invalid_header += snapshot.rx_dropped_invalid_header;
	pstats->stats.rx_dropped_permission += snapshot.rx_dropped_permission;
	pstats->stats.rx_dropped_timeout += snapshot.rx_dropped_timeout;

	pstats->stats.tx_dropped_no_route += snapshot.tx_dropped_no_route;
	pstats->stats.tx_dropped_mtu_exceeded += snapshot.tx_dropped_mtu_exceeded;
	pstats->stats.tx_dropped_no_memory += snapshot.tx_dropped_no_memory;
	pstats->stats.tx_dropped_queue_full += snapshot.tx_dropped_queue_full;
	pstats->stats.tx_dropped_device_down += snapshot.tx_dropped_device_down;
	pstats->stats.tx_dropped_tag_exhaustion += snapshot.tx_dropped_tag_exhaustion;
	pstats->stats.tx_dropped_permission += snapshot.tx_dropped_permission;
	pstats->stats.tx_dropped_bad_addrlen += snapshot.tx_dropped_bad_addrlen;

	spin_unlock_irqrestore(&mctp_closed_stats_lock, flags);
}
EXPORT_SYMBOL_GPL(mctp_stats_aggregate_closed_sk);

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
	seq_printf(m, "  Bad addrlen:   %llu\n", atomic64_read(&ns->tx_dropped_bad_addrlen));
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


static void mctp_print_drop_reasons(struct seq_file *m, struct mctp_sock_stats *s)
{
	bool first = true;

	if (s->tx_drops) {
		seq_printf(m, "    TX Drops: %llu (", s->tx_drops);
		if (s->tx_dropped_no_route) {
			seq_printf(m, "No Route:%llu", s->tx_dropped_no_route);
			first = false;
		}
		if (s->tx_dropped_mtu_exceeded) {
			seq_printf(m, "%sMTU Exceeded:%llu", first ? "" : ", ", s->tx_dropped_mtu_exceeded);
			first = false;
		}
		if (s->tx_dropped_no_memory) {
			seq_printf(m, "%sNo Memory:%llu", first ? "" : ", ", s->tx_dropped_no_memory);
			first = false;
		}
		if (s->tx_dropped_queue_full) {
			seq_printf(m, "%sQueue Full:%llu", first ? "" : ", ", s->tx_dropped_queue_full);
			first = false;
		}
		if (s->tx_dropped_device_down) {
			seq_printf(m, "%sDevice Down:%llu", first ? "" : ", ", s->tx_dropped_device_down);
			first = false;
		}
		if (s->tx_dropped_tag_exhaustion) {
			seq_printf(m, "%sTag Exhaustion:%llu", first ? "" : ", ", s->tx_dropped_tag_exhaustion);
			first = false;
		}
		if (s->tx_dropped_permission) {
			seq_printf(m, "%sPermission:%llu", first ? "" : ", ", s->tx_dropped_permission);
			first = false;
		}
		if (s->tx_dropped_bad_addrlen) {
			seq_printf(m, "%sBad Addrlen:%llu",
				   first ? "" : ", ", s->tx_dropped_bad_addrlen);
			first = false;
		}
		seq_printf(m, ")\n");
	}

	first = true;
	if (s->rx_drops) {
		seq_printf(m, "    RX Drops: %llu (", s->rx_drops);
		if (s->rx_dropped_no_route) {
			seq_printf(m, "No Route:%llu", s->rx_dropped_no_route);
			first = false;
		}
		if (s->rx_dropped_no_memory) {
			seq_printf(m, "%sNo Memory:%llu", first ? "" : ", ", s->rx_dropped_no_memory);
			first = false;
		}
		if (s->rx_dropped_seq_mismatch) {
			seq_printf(m, "%sSeq Mismatch:%llu", first ? "" : ", ", s->rx_dropped_seq_mismatch);
			first = false;
		}
		if (s->rx_dropped_tag_mismatch) {
			seq_printf(m, "%sTag Mismatch:%llu", first ? "" : ", ", s->rx_dropped_tag_mismatch);
			first = false;
		}
		if (s->rx_dropped_queue_full) {
			seq_printf(m, "%sQueue Full:%llu", first ? "" : ", ", s->rx_dropped_queue_full);
			first = false;
		}
		if (s->rx_dropped_invalid_header) {
			seq_printf(m, "%sInvalid Header:%llu", first ? "" : ", ", s->rx_dropped_invalid_header);
			first = false;
		}
		if (s->rx_dropped_permission) {
			seq_printf(m, "%sPermission:%llu", first ? "" : ", ", s->rx_dropped_permission);
			first = false;
		}
		if (s->rx_dropped_timeout) {
			seq_printf(m, "%sTimeout:%llu", first ? "" : ", ", s->rx_dropped_timeout);
			first = false;
		}
		seq_printf(m, ")\n");
	}
}

/* Per-socket statistics display */
static int mctp_sockets_show(struct seq_file *m, void *v)
{
	struct net *net = m->private;
	struct sock *sk;

	/* Header for socket list */
	seq_printf(m, "Socket List:\n");
	seq_puts(m, "  PID    Net  Type         State      TX Msgs   RX Msgs   Keys\n");
	seq_puts(m, "  ---    ---  ----         -----      -------   -------   ----\n");

	rcu_read_lock();
	sk_for_each_rcu(sk, &net->mctp.binds) {
		struct mctp_sock *msk = container_of(sk, struct mctp_sock, sk);
		struct mctp_sock_stats snap;
		struct hlist_node *ktmp;
		unsigned long kflags;
		unsigned int num_keys = 0;
		/* At most 8 tags exist, so at most 8 EID pairs. */
		struct {
			mctp_eid_t local;
			mctp_eid_t peer;
			unsigned int count;
		} eid_keys[8];
		unsigned int neid_pairs = 0, i;

		/* Snapshot stats under lock so tx/rx_messages and drop counts
		 * are consistent with each other (avoids torn u64 reads on
		 * 32-bit ARM and TOCTOU between the summary line and drops).
		 */
		spin_lock_bh(&msk->stats_lock);
		memcpy(&snap, &msk->stats, sizeof(snap));
		spin_unlock_bh(&msk->stats_lock);

		/* Count valid tags held by this socket, grouped by EID pair. */
		spin_lock_irqsave(&net->mctp.keys_lock, kflags);
		hlist_for_each(ktmp, &msk->keys) {
			struct mctp_sk_key *key =
				hlist_entry(ktmp, struct mctp_sk_key, sklist);

			spin_lock(&key->lock);
			if (key->valid) {
				num_keys++;
				for (i = 0; i < neid_pairs; i++) {
					if (eid_keys[i].local == key->local_addr &&
					    eid_keys[i].peer == key->peer_addr) {
						eid_keys[i].count++;
						break;
					}
				}
				if (i == neid_pairs && neid_pairs < ARRAY_SIZE(eid_keys)) {
					eid_keys[neid_pairs].local = key->local_addr;
					eid_keys[neid_pairs].peer  = key->peer_addr;
					eid_keys[neid_pairs].count = 1;
					neid_pairs++;
				}
			}
			spin_unlock(&key->lock);
		}
		spin_unlock_irqrestore(&net->mctp.keys_lock, kflags);

		seq_printf(m, "  %-6d %-4u 0x%02x         %-10s %9llu %9llu %5u\n",
			   msk->pid,
			   msk->bind_net,
			   msk->bind_type,
			   "BOUND",
			   snap.tx_messages,
			   snap.rx_messages,
			   num_keys);

		for (i = 0; i < neid_pairs; i++)
			seq_printf(m, "    local %3u -> peer %3u: %u key(s)\n",
				   eid_keys[i].local, eid_keys[i].peer,
				   eid_keys[i].count);

		if (snap.tx_drops || snap.rx_drops)
			mctp_print_drop_reasons(m, &snap);
	}
	rcu_read_unlock();

	/* Section for Closed Sockets (Aggregated by Name) */
	seq_printf(m, "\nClosed Sockets (Aggregate by Process):\n");
	seq_printf(m, "  Name             TX Msgs   RX Msgs   TX Drops  RX Drops\n");
	seq_printf(m, "  ----             -------   -------   --------  --------\n");

	{
		struct {
			char name[TASK_COMM_LEN];
			struct mctp_sock_stats stats;
		} *snaps = NULL;
		struct mctp_proc_stats *pstats;
		unsigned int nsnaps = 0, i;
		unsigned long flags;

		/* First pass: count entries (briefly under lock). */
		spin_lock_irqsave(&mctp_closed_stats_lock, flags);
		hlist_for_each_entry(pstats, &mctp_closed_stats_list, hlist)
			nsnaps++;
		spin_unlock_irqrestore(&mctp_closed_stats_lock, flags);

		if (nsnaps) {
			snaps = kmalloc_array(nsnaps, sizeof(*snaps), GFP_KERNEL);
			if (!snaps)
				return -ENOMEM;

			/* Second pass: copy data under lock — no sleeping allowed here. */
			i = 0;
			spin_lock_irqsave(&mctp_closed_stats_lock, flags);
			hlist_for_each_entry(pstats, &mctp_closed_stats_list, hlist) {
				if (i >= nsnaps)
					break;
				strscpy(snaps[i].name, pstats->name, TASK_COMM_LEN);
				snaps[i].stats = pstats->stats;
				i++;
			}
			spin_unlock_irqrestore(&mctp_closed_stats_lock, flags);
			nsnaps = i;
		}

		/* Print outside the lock — seq_printf may sleep. */
		for (i = 0; i < nsnaps; i++) {
			seq_printf(m, "  %-16s %-9llu %-9llu %-9llu %-9llu\n",
				   snaps[i].name,
				   snaps[i].stats.tx_messages,
				   snaps[i].stats.rx_messages,
				   snaps[i].stats.tx_drops,
				   snaps[i].stats.rx_drops);

			if (snaps[i].stats.tx_drops || snaps[i].stats.rx_drops)
				mctp_print_drop_reasons(m, &snaps[i].stats);
		}
		kfree(snaps);
	}

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

	ret = register_pernet_subsys(&mctp_stats_net_ops);
	if (ret)
		pr_warn("MCTP: Failed to register /proc stats: %d\n", ret);

	return ret;
}

void mctp_stats_exit(void)
{
	struct mctp_proc_stats *pstats;
	struct hlist_node *tmp;
	unsigned long flags;

	unregister_pernet_subsys(&mctp_stats_net_ops);

	/* Free all closed-socket aggregate entries accumulated at runtime. */
	spin_lock_irqsave(&mctp_closed_stats_lock, flags);
	hlist_for_each_entry_safe(pstats, tmp, &mctp_closed_stats_list, hlist) {
		hlist_del(&pstats->hlist);
		kfree(pstats);
	}
	mctp_closed_stats_count = 0;
	spin_unlock_irqrestore(&mctp_closed_stats_lock, flags);
}
