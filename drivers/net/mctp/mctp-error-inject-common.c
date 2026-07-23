// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP Error Injection - common downstream helper
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Shared diagnostic plumbing for the per-binding MCTP error-injection drivers.
 * See mctp-error-inject-common.h and MCTP_DOWNSTREAM_API.md.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/mctp.h>

#include <net/mctp-error-inject-common.h>

void mctp_ei_common_init(struct mctp_ei_common *c)
{
	spin_lock_init(&c->lock);
	prandom_seed_state(&c->rng, (u64)jiffies);
	c->enable_tx = false;
	c->enable_rx = false;
	c->mode = MCTP_ERR_MODE_ALWAYS;
	/* All other fields are expected to be zero-initialised by the caller. */
}
EXPORT_SYMBOL_GPL(mctp_ei_common_init);

const char *mctp_ei_mode_str(enum mctp_error_inject_mode mode)
{
	switch (mode) {
	case MCTP_ERR_MODE_ALWAYS:
		return "always";
	case MCTP_ERR_MODE_RANDOM:
		return "random";
	case MCTP_ERR_MODE_COUNT:
		return "count";
	default:
		return "unknown";
	}
}
EXPORT_SYMBOL_GPL(mctp_ei_mode_str);

bool mctp_ei_should_inject(struct mctp_ei_common *c, u32 rate, u32 *count)
{
	bool inject = false;

	spin_lock_bh(&c->lock);

	switch (c->mode) {
	case MCTP_ERR_MODE_ALWAYS:
		inject = true;
		break;

	case MCTP_ERR_MODE_RANDOM:
		/* Random injection based on rate percentage */
		inject = (prandom_u32_state(&c->rng) % 100) < rate;
		break;

	case MCTP_ERR_MODE_COUNT:
		/* Inject for count packets, then stop */
		if (*count > 0) {
			(*count)--;
			inject = true;
		}
		break;
	}

	spin_unlock_bh(&c->lock);

	return inject;
}
EXPORT_SYMBOL_GPL(mctp_ei_should_inject);

bool mctp_ei_match_filter(struct mctp_ei_common *c, const struct mctp_hdr *mh)
{
	if (!c->eid_filter.enabled)
		return true;

	if (!mh)
		return false;

	/* Check source EID (0 = any) */
	if (c->eid_filter.src_eid != 0 &&
	    c->eid_filter.src_eid != mh->src)
		return false;

	/* Check dest EID (0 = any) */
	if (c->eid_filter.dest_eid != 0 &&
	    c->eid_filter.dest_eid != mh->dest)
		return false;

	return true;
}
EXPORT_SYMBOL_GPL(mctp_ei_match_filter);

int mctp_ei_fragment(struct mctp_ei_common *c, struct sk_buff *skb,
		     struct net_device *ndev)
{
	struct mctp_hdr *mh;
	u8 flags;
	u8 seq;

	/* Early exit if RX injection disabled or no fragment mode enabled. */
	if (!c->enable_rx ||
	    (!c->enable_fragment_drop && !c->enable_seq_corrupt &&
	     !c->enable_som_clear))
		return 0;

	mh = mctp_hdr(skb);
	if (!mh)
		return 0;

	flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	seq = (mh->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) & MCTP_HDR_SEQ_MASK;

	/* Single-packet message (SOM+EOM) - pass through (not a fragment). */
	if (flags == (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM))
		return 0;

	if (!mctp_ei_match_filter(c, mh))
		return 0;

	/* ===== Injection Type 1: Clear SOM bit (first fragment) ===== */
	if (c->enable_som_clear && (flags & MCTP_HDR_FLAG_SOM)) {
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;

		spin_lock_bh(&c->lock);
		c->som_clears++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(ndev,
			   "ERROR INJECTION: SOM bit CLEARED (first fragment, src=%u, dest=%u, seq=%u)\n",
			   mh->src, mh->dest, seq);

		return 0;  /* Pass through with corrupted SOM */
	}

	/* ===== Injection Type 2: Corrupt sequence (middle/end fragments) ===== */
	if (c->enable_seq_corrupt && !(flags & MCTP_HDR_FLAG_SOM)) {
		u8 old_seq = seq;
		u8 corrupted_seq = (seq + 1) & MCTP_HDR_SEQ_MASK;

		mh->flags_seq_tag &= ~(MCTP_HDR_SEQ_MASK << MCTP_HDR_SEQ_SHIFT);
		mh->flags_seq_tag |= (corrupted_seq << MCTP_HDR_SEQ_SHIFT);

		spin_lock_bh(&c->lock);
		c->seq_corruptions++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(ndev,
			   "ERROR INJECTION: Sequence CORRUPTED (2nd+ fragment, src=%u, dest=%u, %u -> %u)\n",
			   mh->src, mh->dest, old_seq, corrupted_seq);

		return 0;  /* Pass through with corrupted sequence */
	}

	/* ===== Injection Type 3: Drop fragment (2nd+ fragments) ===== */
	if (c->enable_fragment_drop && !(flags & MCTP_HDR_FLAG_SOM)) {
		spin_lock_bh(&c->lock);
		c->fragments_dropped++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(ndev,
			   "ERROR INJECTION: Fragment DROPPED (2nd+ fragment, src=%u, dest=%u, seq=%u)\n",
			   mh->src, mh->dest, seq);

		return 1;  /* Drop this fragment */
	}

	return 0; /* Pass through normally */
}
EXPORT_SYMBOL_GPL(mctp_ei_fragment);

void mctp_ei_common_reset(struct mctp_ei_common *c)
{
	/* Caller holds c->lock. */
	c->enable_tx = false;
	c->enable_rx = false;
	c->mode = MCTP_ERR_MODE_ALWAYS;
	c->delay_ms = 0;
	c->eid_filter.enabled = false;
	c->eid_filter.src_eid = 0;
	c->eid_filter.dest_eid = 0;
	c->eid_filter.msg_type = 0;

	c->enable_fragment_drop = false;
	c->enable_seq_corrupt = false;
	c->enable_som_clear = false;
	c->fragments_dropped = 0;
	c->seq_corruptions = 0;
	c->som_clears = 0;

	c->total_packets_processed = 0;
	c->total_errors_injected = 0;
}
EXPORT_SYMBOL_GPL(mctp_ei_common_reset);

/* ===== Generic debugfs files operating on struct mctp_ei_common ===== */

static struct mctp_ei_common *ei_common_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

/* bool field written via kstrtobool (accepts y/n/1/0/true/false). */
#define MCTP_EI_BOOL_KSTRTOBOOL(name, field)				\
static ssize_t mctp_ei_##name##_read(struct file *file,			\
				     char __user *userbuf,		\
				     size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	int len;							\
									\
	len = snprintf(buf, sizeof(buf), "%d\n", c->field);		\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
static ssize_t mctp_ei_##name##_write(struct file *file,		\
				      const char __user *userbuf,	\
				      size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	bool enable;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
	buf[count] = '\0';						\
	rc = kstrtobool(buf, &enable);					\
	if (rc)								\
		return rc;						\
	spin_lock_bh(&c->lock);						\
	c->field = enable;						\
	spin_unlock_bh(&c->lock);					\
	return count;							\
}									\
static const struct file_operations mctp_ei_##name##_fops = {		\
	.owner = THIS_MODULE,						\
	.read = mctp_ei_##name##_read,					\
	.write = mctp_ei_##name##_write,				\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

/* bool field written via kstrtoint then (val != 0). */
#define MCTP_EI_BOOL_INT(name, field)					\
static ssize_t mctp_ei_##name##_read(struct file *file,			\
				     char __user *userbuf,		\
				     size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	int len;							\
									\
	len = snprintf(buf, sizeof(buf), "%d\n", c->field ? 1 : 0);	\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
static ssize_t mctp_ei_##name##_write(struct file *file,		\
				      const char __user *userbuf,	\
				      size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	int val;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
	buf[count] = '\0';						\
	rc = kstrtoint(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
	spin_lock_bh(&c->lock);						\
	c->field = (val != 0);						\
	spin_unlock_bh(&c->lock);					\
	return count;							\
}									\
static const struct file_operations mctp_ei_##name##_fops = {		\
	.owner = THIS_MODULE,						\
	.read = mctp_ei_##name##_read,					\
	.write = mctp_ei_##name##_write,				\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

/* u32 field. */
#define MCTP_EI_U32(name, field)					\
static ssize_t mctp_ei_##name##_read(struct file *file,			\
				     char __user *userbuf,		\
				     size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[32];							\
	int len;							\
									\
	len = snprintf(buf, sizeof(buf), "%u\n", c->field);		\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
static ssize_t mctp_ei_##name##_write(struct file *file,		\
				      const char __user *userbuf,	\
				      size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[32];							\
	u32 val;							\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
	buf[count] = '\0';						\
	rc = kstrtou32(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
	spin_lock_bh(&c->lock);						\
	c->field = val;							\
	spin_unlock_bh(&c->lock);					\
	return count;							\
}									\
static const struct file_operations mctp_ei_##name##_fops = {		\
	.owner = THIS_MODULE,						\
	.read = mctp_ei_##name##_read,					\
	.write = mctp_ei_##name##_write,				\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

/* u8 field inside eid_filter. */
#define MCTP_EI_FILTER_U8(name, field)					\
static ssize_t mctp_ei_##name##_read(struct file *file,			\
				     char __user *userbuf,		\
				     size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	int len;							\
									\
	len = snprintf(buf, sizeof(buf), "%u\n", c->eid_filter.field);	\
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);	\
}									\
static ssize_t mctp_ei_##name##_write(struct file *file,		\
				      const char __user *userbuf,	\
				      size_t count, loff_t *ppos)	\
{									\
	struct mctp_ei_common *c = ei_common_from_file(file);		\
	char buf[8];							\
	u8 val;								\
	int rc;								\
									\
	if (count >= sizeof(buf))					\
		return -EINVAL;						\
	if (copy_from_user(buf, userbuf, count))			\
		return -EFAULT;						\
	buf[count] = '\0';						\
	rc = kstrtou8(buf, 0, &val);					\
	if (rc)								\
		return rc;						\
	spin_lock_bh(&c->lock);						\
	c->eid_filter.field = val;					\
	spin_unlock_bh(&c->lock);					\
	return count;							\
}									\
static const struct file_operations mctp_ei_##name##_fops = {		\
	.owner = THIS_MODULE,						\
	.read = mctp_ei_##name##_read,					\
	.write = mctp_ei_##name##_write,				\
	.open = simple_open,						\
	.llseek = default_llseek,					\
}

MCTP_EI_BOOL_KSTRTOBOOL(enable_tx, enable_tx);
MCTP_EI_BOOL_KSTRTOBOOL(enable_rx, enable_rx);
MCTP_EI_BOOL_INT(enable_fragment_drop, enable_fragment_drop);
MCTP_EI_BOOL_INT(enable_seq_corrupt, enable_seq_corrupt);
MCTP_EI_BOOL_INT(enable_som_clear, enable_som_clear);
MCTP_EI_U32(delay_ms, delay_ms);
MCTP_EI_BOOL_KSTRTOBOOL(eid_filter_enable, eid_filter.enabled);
MCTP_EI_FILTER_U8(eid_filter_src_eid, src_eid);
MCTP_EI_FILTER_U8(eid_filter_dest_eid, dest_eid);
MCTP_EI_FILTER_U8(eid_filter_msg_type, msg_type);

/* mode attribute - string "always"/"random"/"count". */
static ssize_t mctp_ei_mode_read(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct mctp_ei_common *c = ei_common_from_file(file);
	char buf[16];
	int len;

	len = snprintf(buf, sizeof(buf), "%s\n", mctp_ei_mode_str(c->mode));
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_ei_mode_write(struct file *file, const char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	struct mctp_ei_common *c = ei_common_from_file(file);
	char buf[16];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = '\0';

	spin_lock_bh(&c->lock);
	if (strncmp(buf, "always", 6) == 0)
		c->mode = MCTP_ERR_MODE_ALWAYS;
	else if (strncmp(buf, "random", 6) == 0)
		c->mode = MCTP_ERR_MODE_RANDOM;
	else if (strncmp(buf, "count", 5) == 0)
		c->mode = MCTP_ERR_MODE_COUNT;
	else {
		spin_unlock_bh(&c->lock);
		return -EINVAL;
	}
	spin_unlock_bh(&c->lock);

	return count;
}

static const struct file_operations mctp_ei_mode_fops = {
	.owner = THIS_MODULE,
	.read = mctp_ei_mode_read,
	.write = mctp_ei_mode_write,
	.open = simple_open,
	.llseek = default_llseek,
};

void mctp_ei_common_debugfs_create(struct dentry *dir, struct mctp_ei_common *c,
				   u32 caps)
{
	struct dentry *eid_dir;

	debugfs_create_file("enable_tx", 0600, dir, c, &mctp_ei_enable_tx_fops);
	debugfs_create_file("mode", 0600, dir, c, &mctp_ei_mode_fops);

	if (caps & MCTP_EI_CAP_RX_ENABLE)
		debugfs_create_file("enable_rx", 0600, dir, c,
				    &mctp_ei_enable_rx_fops);

	if (caps & MCTP_EI_CAP_FRAGMENT) {
		debugfs_create_file("enable_fragment_drop", 0600, dir, c,
				    &mctp_ei_enable_fragment_drop_fops);
		debugfs_create_file("enable_seq_corrupt", 0600, dir, c,
				    &mctp_ei_enable_seq_corrupt_fops);
		debugfs_create_file("enable_som_clear", 0600, dir, c,
				    &mctp_ei_enable_som_clear_fops);
	}

	if (caps & MCTP_EI_CAP_DELAY)
		debugfs_create_file("delay_ms", 0600, dir, c,
				    &mctp_ei_delay_ms_fops);

	if (caps & MCTP_EI_CAP_EID_FILTER) {
		eid_dir = debugfs_create_dir("eid_filter", dir);
		if (!IS_ERR_OR_NULL(eid_dir)) {
			debugfs_create_file("enable", 0600, eid_dir, c,
					    &mctp_ei_eid_filter_enable_fops);
			debugfs_create_file("src_eid", 0600, eid_dir, c,
					    &mctp_ei_eid_filter_src_eid_fops);
			debugfs_create_file("dest_eid", 0600, eid_dir, c,
					    &mctp_ei_eid_filter_dest_eid_fops);
			debugfs_create_file("msg_type", 0600, eid_dir, c,
					    &mctp_ei_eid_filter_msg_type_fops);
		}
	}
}
EXPORT_SYMBOL_GPL(mctp_ei_common_debugfs_create);

/* ===== Generic transport-specific scalar debugfs file ===== */

static struct mctp_ei_scalar *ei_scalar_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

static ssize_t mctp_ei_scalar_read(struct file *file, char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	struct mctp_ei_scalar *desc = ei_scalar_from_file(file);
	char buf[32];
	int len;

	if (desc->type == MCTP_EI_SCALAR_U32)
		len = snprintf(buf, sizeof(buf), "%u\n", *(u32 *)desc->ptr);
	else
		len = snprintf(buf, sizeof(buf), "%d\n", *(int *)desc->ptr);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_ei_scalar_write(struct file *file,
				    const char __user *userbuf,
				    size_t count, loff_t *ppos)
{
	struct mctp_ei_scalar *desc = ei_scalar_from_file(file);
	char buf[32];
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (desc->type == MCTP_EI_SCALAR_U32) {
		u32 val;

		rc = kstrtou32(buf, 0, &val);
		if (rc)
			return rc;
		spin_lock_bh(desc->lock);
		*(u32 *)desc->ptr = val;
		spin_unlock_bh(desc->lock);
	} else {
		int val;

		rc = kstrtoint(buf, 0, &val);
		if (rc)
			return rc;
		spin_lock_bh(desc->lock);
		*(int *)desc->ptr = val;
		spin_unlock_bh(desc->lock);
	}

	return count;
}

static const struct file_operations mctp_ei_scalar_fops = {
	.owner = THIS_MODULE,
	.read = mctp_ei_scalar_read,
	.write = mctp_ei_scalar_write,
	.open = simple_open,
	.llseek = default_llseek,
};

void mctp_ei_create_scalar_file(struct dentry *dir, const char *name,
				umode_t mode, struct mctp_ei_scalar *desc,
				enum mctp_ei_scalar_type type, void *ptr,
				spinlock_t *lock)
{
	desc->type = type;
	desc->ptr = ptr;
	desc->lock = lock;
	debugfs_create_file(name, mode, dir, desc, &mctp_ei_scalar_fops);
}
EXPORT_SYMBOL_GPL(mctp_ei_create_scalar_file);

MODULE_DESCRIPTION("MCTP error injection common helpers");
MODULE_LICENSE("GPL");
