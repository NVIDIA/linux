// SPDX-License-Identifier: GPL-2.0
/*
 * Aspeed MCTP Controller Error Injection Implementation
 *
 * Copyright (c) 2026 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over the Aspeed PCIe VDM controller.
 *
 * Interface mirrored from drivers/net/mctp/mctp-i2c-error-inject.c.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <net/mctp.h>

#include "aspeed-mctp-ctrl-internal.h"

static struct dentry *aspeed_mctp_error_inject_root;

static bool aspeed_mctp_should_inject_error(struct aspeed_mctp_error_inject *ei,
					    u32 rate, u32 *count)
{
	bool inject = false;

	spin_lock_bh(&ei->lock);

	switch (ei->mode) {
	case ASPEED_MCTP_ERR_MODE_ALWAYS:
		inject = true;
		break;

	case ASPEED_MCTP_ERR_MODE_RANDOM:
		inject = (prandom_u32_state(&ei->rng) % 100) < rate;
		break;

	case ASPEED_MCTP_ERR_MODE_COUNT:
		if (*count > 0) {
			(*count)--;
			inject = true;
		}
		break;
	}

	spin_unlock_bh(&ei->lock);

	return inject;
}

static bool aspeed_mctp_error_inject_match_filter(struct aspeed_mctp_error_inject *ei,
						  struct sk_buff *skb)
{
	struct mctp_hdr *mh;

	if (!ei->eid_filter.enabled)
		return true;

	mh = mctp_hdr(skb);
	if (!mh)
		return false;

	if (ei->eid_filter.src_eid != 0 &&
	    ei->eid_filter.src_eid != mh->src)
		return false;

	if (ei->eid_filter.dest_eid != 0 &&
	    ei->eid_filter.dest_eid != mh->dest)
		return false;

	return true;
}

/**
 * aspeed_mctp_error_inject_tx - TX error injection hook
 * @priv: Aspeed MCTP controller instance
 * @skb: outgoing packet
 *
 * Returns: 0 for normal operation, non-zero when an error should be injected.
 *
 * The caller (aspeed_mctp_ctrl_start_xmit) must free the skb and return
 * NETDEV_TX_OK because the netdev TX contract does not permit propagating
 * an error to the stack. The specific negative value is informational and
 * is reported via netdev_dbg for trace parity with the I2C hook.
 */
int aspeed_mctp_error_inject_tx(struct aspeed_mctp_ctrl *priv, struct sk_buff *skb)
{
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	struct mctp_hdr *mh;

	if (!ei->enable_tx || ei->tx_error_code == 0)
		return 0;

	if (!aspeed_mctp_error_inject_match_filter(ei, skb))
		return 0;

	if (!aspeed_mctp_should_inject_error(ei, ei->tx_error_rate,
					     &ei->tx_inject_count))
		return 0;

	if (ei->delay_ms > 0)
		msleep(ei->delay_ms);

	spin_lock_bh(&ei->lock);
	ei->tx_errors_injected++;
	ei->total_errors_injected++;
	ei->total_packets_processed++;
	spin_unlock_bh(&ei->lock);

	mh = mctp_hdr(skb);

	netdev_dbg(priv->ndev,
		   "ERROR INJECTION: TX - error=%d, src_eid=%u, dest_eid=%u, total_injected=%u, mode=%s\n",
		   ei->tx_error_code, mh ? mh->src : 0, mh ? mh->dest : 0,
		   ei->tx_errors_injected,
		   ei->mode == ASPEED_MCTP_ERR_MODE_ALWAYS ? "always" :
		   ei->mode == ASPEED_MCTP_ERR_MODE_RANDOM ? "random" : "count");

	return -ei->tx_error_code;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_tx);

/**
 * aspeed_mctp_error_inject_fragment - RX fragment-level injection hook
 * @priv: Aspeed MCTP controller instance
 * @skb: received packet (header intact)
 *
 * Returns: 0 to pass packet through, 1 to drop the packet.
 *
 * Can drop 2nd+ fragments, corrupt sequence numbers, or clear the SOM bit
 * on first fragments. Single-packet messages (SOM+EOM) are always passed
 * through unchanged — error injection operates only on multi-fragment
 * messages because single-packet traffic has no fragment-level failure mode.
 */
int aspeed_mctp_error_inject_fragment(struct aspeed_mctp_ctrl *priv,
				      struct sk_buff *skb)
{
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	struct mctp_hdr *mh;
	u8 flags;
	u8 seq;

	if (!ei->enable_rx ||
	    (!ei->enable_fragment_drop && !ei->enable_seq_corrupt &&
	     !ei->enable_som_clear))
		return 0;

	mh = mctp_hdr(skb);
	if (!mh)
		return 0;

	flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	seq = (mh->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) & MCTP_HDR_SEQ_MASK;

	/* Single-packet message (SOM+EOM) — not a fragment */
	if (flags == (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM))
		return 0;

	if (!aspeed_mctp_error_inject_match_filter(ei, skb))
		return 0;

	if (ei->enable_som_clear && (flags & MCTP_HDR_FLAG_SOM)) {
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;

		spin_lock_bh(&ei->lock);
		ei->som_clears++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);

		netdev_dbg(priv->ndev,
			   "ERROR INJECTION: SOM bit CLEARED (src=%u, dest=%u, seq=%u)\n",
			   mh->src, mh->dest, seq);

		return 0;
	}

	if (ei->enable_seq_corrupt && !(flags & MCTP_HDR_FLAG_SOM)) {
		u8 old_seq = seq;
		u8 corrupted_seq = (seq + 1) & MCTP_HDR_SEQ_MASK;

		mh->flags_seq_tag &= ~(MCTP_HDR_SEQ_MASK << MCTP_HDR_SEQ_SHIFT);
		mh->flags_seq_tag |= (corrupted_seq << MCTP_HDR_SEQ_SHIFT);

		spin_lock_bh(&ei->lock);
		ei->seq_corruptions++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);

		netdev_dbg(priv->ndev,
			   "ERROR INJECTION: Sequence CORRUPTED (src=%u, dest=%u, %u -> %u)\n",
			   mh->src, mh->dest, old_seq, corrupted_seq);

		return 0;
	}

	if (ei->enable_fragment_drop && !(flags & MCTP_HDR_FLAG_SOM)) {
		spin_lock_bh(&ei->lock);
		ei->fragments_dropped++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);

		netdev_dbg(priv->ndev,
			   "ERROR INJECTION: Fragment DROPPED (src=%u, dest=%u, seq=%u)\n",
			   mh->src, mh->dest, seq);

		return 1;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_fragment);

/**
 * aspeed_mctp_error_inject_filter_list - apply fragment hook to a skb list
 * @priv: Aspeed MCTP controller instance
 * @skb_list: list of skbs about to be handed to netif_receive_skb_list
 *
 * Walks the list, invokes the per-skb fragment hook, and removes/frees
 * any skbs the hook elected to drop. No-op fast-path when RX injection
 * is disabled.
 */
void aspeed_mctp_error_inject_filter_list(struct aspeed_mctp_ctrl *priv,
					  struct list_head *skb_list)
{
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	struct sk_buff *skb, *tmp;

	if (!ei->enable_rx)
		return;

	list_for_each_entry_safe(skb, tmp, skb_list, list) {
		if (aspeed_mctp_error_inject_fragment(priv, skb)) {
			skb_list_del_init(skb);
			dev_core_stats_rx_dropped_inc(priv->ndev);
			kfree_skb(skb);
		}
	}
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_filter_list);

/* ===== Debugfs interface ===== */

static struct aspeed_mctp_ctrl *aspeed_mctp_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

static ssize_t aspeed_mctp_debugfs_enable_tx_read(struct file *file,
						  char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	int len;

	len = scnprintf(buf, sizeof(buf), "%d\n", priv->error_inject.enable_tx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t aspeed_mctp_debugfs_enable_tx_write(struct file *file,
						   const char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	bool enable;
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';
	rc = kstrtobool(buf, &enable);
	if (rc)
		return rc;

	spin_lock_bh(&priv->error_inject.lock);
	priv->error_inject.enable_tx = enable;
	spin_unlock_bh(&priv->error_inject.lock);

	netdev_dbg(priv->ndev, "TX error injection %s\n",
		   enable ? "enabled" : "disabled");

	return count;
}

static const struct file_operations aspeed_mctp_debugfs_enable_tx_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_mctp_debugfs_enable_tx_read,
	.write = aspeed_mctp_debugfs_enable_tx_write,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t aspeed_mctp_debugfs_enable_rx_read(struct file *file,
						  char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	int len;

	len = scnprintf(buf, sizeof(buf), "%d\n", priv->error_inject.enable_rx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t aspeed_mctp_debugfs_enable_rx_write(struct file *file,
						   const char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	bool enable;
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';
	rc = kstrtobool(buf, &enable);
	if (rc)
		return rc;

	spin_lock_bh(&priv->error_inject.lock);
	priv->error_inject.enable_rx = enable;
	spin_unlock_bh(&priv->error_inject.lock);

	netdev_dbg(priv->ndev, "RX error injection %s\n",
		   enable ? "enabled" : "disabled");

	return count;
}

static const struct file_operations aspeed_mctp_debugfs_enable_rx_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_mctp_debugfs_enable_rx_read,
	.write = aspeed_mctp_debugfs_enable_rx_write,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t aspeed_mctp_debugfs_mode_read(struct file *file,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	const char *mode_str;
	char buf[16];
	int len;

	switch (priv->error_inject.mode) {
	case ASPEED_MCTP_ERR_MODE_ALWAYS:
		mode_str = "always\n";
		break;
	case ASPEED_MCTP_ERR_MODE_RANDOM:
		mode_str = "random\n";
		break;
	case ASPEED_MCTP_ERR_MODE_COUNT:
		mode_str = "count\n";
		break;
	default:
		mode_str = "unknown\n";
		break;
	}

	len = scnprintf(buf, sizeof(buf), "%s", mode_str);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t aspeed_mctp_debugfs_mode_write(struct file *file,
					      const char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[16];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	spin_lock_bh(&priv->error_inject.lock);

	if (strncmp(buf, "always", 6) == 0) {
		priv->error_inject.mode = ASPEED_MCTP_ERR_MODE_ALWAYS;
	} else if (strncmp(buf, "random", 6) == 0) {
		priv->error_inject.mode = ASPEED_MCTP_ERR_MODE_RANDOM;
	} else if (strncmp(buf, "count", 5) == 0) {
		priv->error_inject.mode = ASPEED_MCTP_ERR_MODE_COUNT;
	} else {
		spin_unlock_bh(&priv->error_inject.lock);
		return -EINVAL;
	}

	spin_unlock_bh(&priv->error_inject.lock);

	return count;
}

static const struct file_operations aspeed_mctp_debugfs_mode_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_mctp_debugfs_mode_read,
	.write = aspeed_mctp_debugfs_mode_write,
	.open = simple_open,
	.llseek = default_llseek,
};

#define ASPEED_MCTP_DEBUGFS_U32_DEFINE(name, field)			\
static ssize_t aspeed_mctp_debugfs_##name##_read(struct file *file,	\
						 char __user *userbuf,	\
						 size_t count,		\
						 loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[32];							\
	int len;							\
									\
	len = scnprintf(buf, sizeof(buf), "%u\n",			\
			priv->error_inject.field);			\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
									\
static ssize_t aspeed_mctp_debugfs_##name##_write(struct file *file,	\
						  const char __user	\
							  *userbuf,	\
						  size_t count,		\
						  loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[32];							\
	u32 val;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
									\
	buf[count] = '\0';						\
	rc = kstrtou32(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
									\
	spin_lock_bh(&priv->error_inject.lock);				\
	priv->error_inject.field = val;					\
	spin_unlock_bh(&priv->error_inject.lock);			\
									\
	return count;							\
}									\
									\
static const struct file_operations aspeed_mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE,						\
	.read = aspeed_mctp_debugfs_##name##_read,			\
	.write = aspeed_mctp_debugfs_##name##_write,			\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

#define ASPEED_MCTP_DEBUGFS_INT_DEFINE(name, field)			\
static ssize_t aspeed_mctp_debugfs_##name##_read(struct file *file,	\
						 char __user *userbuf,	\
						 size_t count,		\
						 loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[32];							\
	int len;							\
									\
	len = scnprintf(buf, sizeof(buf), "%d\n",			\
			priv->error_inject.field);			\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
									\
static ssize_t aspeed_mctp_debugfs_##name##_write(struct file *file,	\
						  const char __user	\
							  *userbuf,	\
						  size_t count,		\
						  loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[32];							\
	int val;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
									\
	buf[count] = '\0';						\
	rc = kstrtoint(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
									\
	spin_lock_bh(&priv->error_inject.lock);				\
	priv->error_inject.field = val;					\
	spin_unlock_bh(&priv->error_inject.lock);			\
									\
	return count;							\
}									\
									\
static const struct file_operations aspeed_mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE,						\
	.read = aspeed_mctp_debugfs_##name##_read,			\
	.write = aspeed_mctp_debugfs_##name##_write,			\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

#define ASPEED_MCTP_DEBUGFS_BOOL_DEFINE(name, field)			\
static ssize_t aspeed_mctp_debugfs_##name##_read(struct file *file,	\
						 char __user *userbuf,	\
						 size_t count,		\
						 loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[8];							\
	int len;							\
									\
	len = scnprintf(buf, sizeof(buf), "%d\n",			\
			priv->error_inject.field ? 1 : 0);		\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
									\
static ssize_t aspeed_mctp_debugfs_##name##_write(struct file *file,	\
						  const char __user	\
							  *userbuf,	\
						  size_t count,		\
						  loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[8];							\
	int val;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
									\
	buf[count] = '\0';						\
	rc = kstrtoint(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
									\
	spin_lock_bh(&priv->error_inject.lock);				\
	priv->error_inject.field = (val != 0);				\
	spin_unlock_bh(&priv->error_inject.lock);			\
									\
	return count;							\
}									\
									\
static const struct file_operations aspeed_mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE,						\
	.read = aspeed_mctp_debugfs_##name##_read,			\
	.write = aspeed_mctp_debugfs_##name##_write,			\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

ASPEED_MCTP_DEBUGFS_INT_DEFINE(tx_error_code, tx_error_code);
ASPEED_MCTP_DEBUGFS_U32_DEFINE(tx_error_rate, tx_error_rate);
ASPEED_MCTP_DEBUGFS_BOOL_DEFINE(enable_fragment_drop, enable_fragment_drop);
ASPEED_MCTP_DEBUGFS_BOOL_DEFINE(enable_seq_corrupt, enable_seq_corrupt);
ASPEED_MCTP_DEBUGFS_BOOL_DEFINE(enable_som_clear, enable_som_clear);
ASPEED_MCTP_DEBUGFS_U32_DEFINE(delay_ms, delay_ms);

#define ASPEED_MCTP_DEBUGFS_U8_DEFINE(name, field)			\
static ssize_t aspeed_mctp_debugfs_##name##_read(struct file *file,	\
						 char __user *userbuf,	\
						 size_t count,		\
						 loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[8];							\
	int len;							\
									\
	len = scnprintf(buf, sizeof(buf), "%u\n",			\
			priv->error_inject.eid_filter.field);		\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
									\
static ssize_t aspeed_mctp_debugfs_##name##_write(struct file *file,	\
						  const char __user	\
							  *userbuf,	\
						  size_t count,		\
						  loff_t *ppos)		\
{									\
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);	\
	char buf[8];							\
	u8 val;								\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
									\
	buf[count] = '\0';						\
	rc = kstrtou8(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
									\
	spin_lock_bh(&priv->error_inject.lock);				\
	priv->error_inject.eid_filter.field = val;			\
	spin_unlock_bh(&priv->error_inject.lock);			\
									\
	return count;							\
}									\
									\
static const struct file_operations aspeed_mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE,						\
	.read = aspeed_mctp_debugfs_##name##_read,			\
	.write = aspeed_mctp_debugfs_##name##_write,			\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

static ssize_t
aspeed_mctp_debugfs_eid_filter_enable_read(struct file *file,
					   char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	int len;

	len = scnprintf(buf, sizeof(buf), "%d\n",
			priv->error_inject.eid_filter.enabled);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t
aspeed_mctp_debugfs_eid_filter_enable_write(struct file *file,
					    const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	char buf[8];
	bool enable;
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';
	rc = kstrtobool(buf, &enable);
	if (rc)
		return rc;

	spin_lock_bh(&priv->error_inject.lock);
	priv->error_inject.eid_filter.enabled = enable;
	spin_unlock_bh(&priv->error_inject.lock);

	return count;
}

static const struct file_operations aspeed_mctp_debugfs_eid_filter_enable_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_mctp_debugfs_eid_filter_enable_read,
	.write = aspeed_mctp_debugfs_eid_filter_enable_write,
	.open = simple_open,
	.llseek = default_llseek,
};

ASPEED_MCTP_DEBUGFS_U8_DEFINE(eid_filter_src_eid, src_eid);
ASPEED_MCTP_DEBUGFS_U8_DEFINE(eid_filter_dest_eid, dest_eid);
ASPEED_MCTP_DEBUGFS_U8_DEFINE(eid_filter_msg_type, msg_type);

static ssize_t aspeed_mctp_debugfs_stats_read(struct file *file,
					      char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	char *buf;
	int len = 0;
	ssize_t ret;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_bh(&ei->lock);

	len += scnprintf(buf + len, PAGE_SIZE - len, "enable_tx: %d\n",
			 ei->enable_tx);
	len += scnprintf(buf + len, PAGE_SIZE - len, "enable_rx: %d\n",
			 ei->enable_rx);
	len += scnprintf(buf + len, PAGE_SIZE - len, "mode: %s\n",
			 ei->mode == ASPEED_MCTP_ERR_MODE_ALWAYS ? "always" :
			 ei->mode == ASPEED_MCTP_ERR_MODE_RANDOM ? "random" :
			 "count");
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "tx_errors_injected: %u\n", ei->tx_errors_injected);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "fragments_dropped: %u\n", ei->fragments_dropped);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "seq_corruptions: %u\n", ei->seq_corruptions);
	len += scnprintf(buf + len, PAGE_SIZE - len, "som_clears: %u\n",
			 ei->som_clears);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "total_packets_processed: %llu\n",
			 ei->total_packets_processed);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "total_errors_injected: %llu\n",
			 ei->total_errors_injected);

	spin_unlock_bh(&ei->lock);

	ret = simple_read_from_buffer(userbuf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static const struct file_operations aspeed_mctp_debugfs_stats_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_mctp_debugfs_stats_read,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t aspeed_mctp_debugfs_reset_write(struct file *file,
					       const char __user *userbuf,
					       size_t count, loff_t *ppos)
{
	struct aspeed_mctp_ctrl *priv = aspeed_mctp_from_file(file);
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	char buf[8];
	int val;
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';
	rc = kstrtoint(buf, 0, &val);
	if (rc)
		return rc;
	if (val != 1)
		return -EINVAL;

	spin_lock_bh(&ei->lock);

	ei->enable_tx = false;
	ei->enable_rx = false;
	ei->mode = ASPEED_MCTP_ERR_MODE_ALWAYS;
	ei->tx_error_code = 0;
	ei->tx_error_rate = 0;
	ei->tx_inject_count = 0;
	ei->enable_fragment_drop = false;
	ei->enable_seq_corrupt = false;
	ei->enable_som_clear = false;
	ei->delay_ms = 0;
	ei->eid_filter.enabled = false;
	ei->eid_filter.src_eid = 0;
	ei->eid_filter.dest_eid = 0;
	ei->eid_filter.msg_type = 0;

	ei->tx_errors_injected = 0;
	ei->fragments_dropped = 0;
	ei->seq_corruptions = 0;
	ei->som_clears = 0;
	ei->total_packets_processed = 0;
	ei->total_errors_injected = 0;

	spin_unlock_bh(&ei->lock);

	netdev_dbg(priv->ndev, "Error injection reset\n");

	return count;
}

static const struct file_operations aspeed_mctp_debugfs_reset_fops = {
	.owner = THIS_MODULE,
	.write = aspeed_mctp_debugfs_reset_write,
	.open = simple_open,
	.llseek = default_llseek,
};

void aspeed_mctp_error_inject_init(struct aspeed_mctp_ctrl *priv)
{
	struct aspeed_mctp_error_inject *ei = &priv->error_inject;
	struct dentry *dir, *eid_dir;

	spin_lock_init(&ei->lock);
	prandom_seed_state(&ei->rng, (u64)jiffies);
	ei->enable_tx = false;
	ei->enable_rx = false;
	ei->mode = ASPEED_MCTP_ERR_MODE_ALWAYS;

	if (!aspeed_mctp_error_inject_root)
		return;

	dir = debugfs_create_dir(netdev_name(priv->ndev),
				 aspeed_mctp_error_inject_root);
	if (IS_ERR_OR_NULL(dir))
		return;

	priv->debugfs_dir = dir;

	debugfs_create_file("enable_tx", 0600, dir, priv,
			    &aspeed_mctp_debugfs_enable_tx_fops);
	debugfs_create_file("enable_rx", 0600, dir, priv,
			    &aspeed_mctp_debugfs_enable_rx_fops);
	debugfs_create_file("mode", 0600, dir, priv,
			    &aspeed_mctp_debugfs_mode_fops);
	debugfs_create_file("tx_error_code", 0600, dir, priv,
			    &aspeed_mctp_debugfs_tx_error_code_fops);
	debugfs_create_file("tx_error_rate", 0600, dir, priv,
			    &aspeed_mctp_debugfs_tx_error_rate_fops);
	debugfs_create_file("enable_fragment_drop", 0600, dir, priv,
			    &aspeed_mctp_debugfs_enable_fragment_drop_fops);
	debugfs_create_file("enable_seq_corrupt", 0600, dir, priv,
			    &aspeed_mctp_debugfs_enable_seq_corrupt_fops);
	debugfs_create_file("enable_som_clear", 0600, dir, priv,
			    &aspeed_mctp_debugfs_enable_som_clear_fops);
	debugfs_create_file("delay_ms", 0600, dir, priv,
			    &aspeed_mctp_debugfs_delay_ms_fops);
	debugfs_create_file("stats", 0400, dir, priv,
			    &aspeed_mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, dir, priv,
			    &aspeed_mctp_debugfs_reset_fops);

	eid_dir = debugfs_create_dir("eid_filter", dir);
	if (!IS_ERR_OR_NULL(eid_dir)) {
		debugfs_create_file("enable", 0600, eid_dir, priv,
				    &aspeed_mctp_debugfs_eid_filter_enable_fops);
		debugfs_create_file("src_eid", 0600, eid_dir, priv,
				    &aspeed_mctp_debugfs_eid_filter_src_eid_fops);
		debugfs_create_file("dest_eid", 0600, eid_dir, priv,
				    &aspeed_mctp_debugfs_eid_filter_dest_eid_fops);
		debugfs_create_file("msg_type", 0600, eid_dir, priv,
				    &aspeed_mctp_debugfs_eid_filter_msg_type_fops);
	}
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_init);

void aspeed_mctp_error_inject_cleanup(struct aspeed_mctp_ctrl *priv)
{
	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_cleanup);

int aspeed_mctp_error_inject_module_init(void)
{
	aspeed_mctp_error_inject_root = debugfs_create_dir("aspeed_mctp", NULL);
	if (IS_ERR_OR_NULL(aspeed_mctp_error_inject_root)) {
		pr_warn("aspeed-mctp: failed to create debugfs root, error injection disabled\n");
		aspeed_mctp_error_inject_root = NULL;
		return -ENODEV;
	}

	pr_info("aspeed-mctp: error injection enabled\n");
	return 0;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_module_init);

void aspeed_mctp_error_inject_module_exit(void)
{
	debugfs_remove_recursive(aspeed_mctp_error_inject_root);
	aspeed_mctp_error_inject_root = NULL;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_error_inject_module_exit);

MODULE_DESCRIPTION("Aspeed MCTP controller error injection");
MODULE_LICENSE("GPL");
