// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP I2C Error Injection Implementation
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over I2C binding.
 *
 * Interface unified with USB error injection where applicable.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/i2c.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/mctp.h>

#include "mctp-i2c-internal.h"

/* Global debugfs root for all MCTP I2C error injection */
static struct dentry *mctp_i2c_error_inject_root;

/* Check if error should be injected based on mode and rate */
static bool mctp_should_inject_error(struct mctp_i2c_error_inject *ei, u32 rate, u32 *count)
{
	bool inject = false;
	
	spin_lock_bh(&ei->lock);
	
	switch (ei->mode) {
	case MCTP_ERR_MODE_ALWAYS:
		inject = true;
		break;
		
	case MCTP_ERR_MODE_RANDOM:
		/* Random injection based on rate percentage */
		inject = (prandom_u32_state(&ei->rng) % 100) < rate;
		break;
		
	case MCTP_ERR_MODE_COUNT:
		/* Inject for count packets, then stop */
		if (*count > 0) {
			(*count)--;
			inject = true;
		}
		break;
	}
	
	spin_unlock_bh(&ei->lock);
	
	return inject;
}

/* Check if packet matches EID filter */
static bool mctp_error_inject_match_filter(struct mctp_i2c_error_inject *ei,
					    struct sk_buff *skb)
{
	struct mctp_hdr *mh;
	
	if (!ei->eid_filter.enabled)
		return true;
	
	mh = mctp_hdr(skb);
	if (!mh)
		return false;
	
	/* Check source EID */
	if (ei->eid_filter.src_eid != 0 &&
	    ei->eid_filter.src_eid != mh->src)
		return false;
	
	/* Check dest EID */
	if (ei->eid_filter.dest_eid != 0 &&
	    ei->eid_filter.dest_eid != mh->dest)
		return false;
	
	return true;
}

/**
 * mctp_i2c_error_inject_tx - Inject TX error
 * @midev: MCTP I2C device
 * @skb: Packet being transmitted
 *
 * Returns: 0 for normal operation, negative error code to simulate failure
 *
 * Called before __i2c_transfer() to potentially inject a TX error.
 * I2C transfer is synchronous, so there's only one injection point.
 */
int mctp_i2c_error_inject_tx(struct mctp_i2c_dev *midev, struct sk_buff *skb)
{
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
	struct mctp_hdr *mh;
	
	/* Early exit if TX error injection is disabled or code not set */
	if (!ei->enable_tx || ei->i2c_tx_error_code == 0)
		return 0;
	
	if (!mctp_error_inject_match_filter(ei, skb))
		return 0;
	
	if (!mctp_should_inject_error(ei, ei->i2c_tx_error_rate,
				      &ei->i2c_tx_inject_count))
		return 0;
	
	/* Inject delay if configured */
	if (ei->delay_ms > 0)
		msleep(ei->delay_ms);
	
	/* Update statistics */
	spin_lock_bh(&ei->lock);
	ei->i2c_tx_errors_injected++;
	ei->total_errors_injected++;
	ei->total_packets_processed++;
	spin_unlock_bh(&ei->lock);
	
	mh = mctp_hdr(skb);
	
	netdev_dbg(midev->ndev,
		    "ERROR INJECTION: TX - error=%d, src_eid=%u, dest_eid=%u, "
		    "total_injected=%u, mode=%s\n",
		    ei->i2c_tx_error_code, mh->src, mh->dest,
		    ei->i2c_tx_errors_injected,
		    ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
		    ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	
	return -ei->i2c_tx_error_code;
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_tx);

/**
 * mctp_i2c_error_inject_fragment - Inject fragment errors
 * @midev: MCTP I2C device
 * @skb: Packet being processed
 *
 * Returns: 0 to pass packet through, 1 to drop packet
 *
 * This function can:
 * 1. Drop fragments (simulates fragment loss)
 * 2. Corrupt sequence numbers (simulates protocol errors)
 * 3. Clear SOM flag (simulates missing start-of-message)
 */
int mctp_i2c_error_inject_fragment(struct mctp_i2c_dev *midev, struct sk_buff *skb)
{
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
	struct mctp_hdr *mh;
	u8 flags;
	u8 seq;
	
	/* Early exit if RX error injection disabled or no fragment injection enabled */
	if (!ei->enable_rx || (!ei->enable_fragment_drop && !ei->enable_seq_corrupt && !ei->enable_som_clear))
		return 0;
	
	mh = mctp_hdr(skb);
	if (!mh)
		return 0;
	
	flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	seq = (mh->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) & MCTP_HDR_SEQ_MASK;
	
	/* Single-packet message (SOM+EOM) - pass through (not a fragment) */
	if (flags == (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM))
		return 0;
	
	if (!mctp_error_inject_match_filter(ei, skb))
		return 0;
	
	/* ===== Injection Type 1: Clear SOM bit (first fragment) ===== */
	if (ei->enable_som_clear && (flags & MCTP_HDR_FLAG_SOM)) {
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;
		
		spin_lock_bh(&ei->lock);
		ei->som_clears++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(midev->ndev,
			    "ERROR INJECTION: SOM bit CLEARED (first fragment, src=%u, dest=%u, seq=%u)\n",
			    mh->src, mh->dest, seq);
		
		return 0;  /* Pass through with corrupted SOM */
	}
	
	/* ===== Injection Type 2: Corrupt sequence number (middle/end fragments) ===== */
	if (ei->enable_seq_corrupt && !(flags & MCTP_HDR_FLAG_SOM)) {
		u8 old_seq = seq;
		u8 corrupted_seq = (seq + 1) & MCTP_HDR_SEQ_MASK;
		
		mh->flags_seq_tag &= ~(MCTP_HDR_SEQ_MASK << MCTP_HDR_SEQ_SHIFT);
		mh->flags_seq_tag |= (corrupted_seq << MCTP_HDR_SEQ_SHIFT);
		
		spin_lock_bh(&ei->lock);
		ei->seq_corruptions++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(midev->ndev,
			    "ERROR INJECTION: Sequence CORRUPTED (2nd+ fragment, src=%u, dest=%u, %u -> %u)\n",
			    mh->src, mh->dest, old_seq, corrupted_seq);
		
		return 0;  /* Pass through with corrupted sequence */
	}
	
	/* ===== Injection Type 3: Drop fragment (2nd+ fragments) ===== */
	if (ei->enable_fragment_drop && !(flags & MCTP_HDR_FLAG_SOM)) {
		spin_lock_bh(&ei->lock);
		ei->fragments_dropped++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(midev->ndev,
			    "ERROR INJECTION: Fragment DROPPED (2nd+ fragment, src=%u, dest=%u, seq=%u)\n",
			    mh->src, mh->dest, seq);
		
		return 1;  /* Drop this fragment */
	}
	
	return 0; /* Pass through normally */
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_fragment);

/* ===== Debugfs Interface - Unified with USB ===== */

static struct mctp_i2c_dev *mctp_i2c_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

/* enable_tx attribute */
static ssize_t mctp_debugfs_enable_tx_read(struct file *file, char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.enable_tx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_enable_tx_write(struct file *file, const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.enable_tx = enable;
	spin_unlock_bh(&midev->error_inject.lock);
	
	netdev_dbg(midev->ndev, "TX error injection %s\n",
		    enable ? "enabled" : "disabled");
	
	return count;
}

static const struct file_operations mctp_debugfs_enable_tx_fops = {
	.owner = THIS_MODULE,
	.read = mctp_debugfs_enable_tx_read,
	.write = mctp_debugfs_enable_tx_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* enable_rx attribute */
static ssize_t mctp_debugfs_enable_rx_read(struct file *file, char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.enable_rx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_enable_rx_write(struct file *file, const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.enable_rx = enable;
	spin_unlock_bh(&midev->error_inject.lock);
	
	netdev_dbg(midev->ndev, "RX error injection %s\n",
		    enable ? "enabled" : "disabled");
	
	return count;
}

static const struct file_operations mctp_debugfs_enable_rx_fops = {
	.owner = THIS_MODULE,
	.read = mctp_debugfs_enable_rx_read,
	.write = mctp_debugfs_enable_rx_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* mode attribute */
static ssize_t mctp_debugfs_mode_read(struct file *file, char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	const char *mode_str;
	char buf[16];
	int len;
	
	switch (midev->error_inject.mode) {
	case MCTP_ERR_MODE_ALWAYS:
		mode_str = "always\n";
		break;
	case MCTP_ERR_MODE_RANDOM:
		mode_str = "random\n";
		break;
	case MCTP_ERR_MODE_COUNT:
		mode_str = "count\n";
		break;
	default:
		mode_str = "unknown\n";
		break;
	}
	
	len = snprintf(buf, sizeof(buf), "%s", mode_str);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_mode_write(struct file *file, const char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	char buf[16];
	
	if (count >= sizeof(buf))
		return -EINVAL;
	
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	
	buf[count] = '\0';
	
	spin_lock_bh(&midev->error_inject.lock);
	
	if (strncmp(buf, "always", 6) == 0)
		midev->error_inject.mode = MCTP_ERR_MODE_ALWAYS;
	else if (strncmp(buf, "random", 6) == 0)
		midev->error_inject.mode = MCTP_ERR_MODE_RANDOM;
	else if (strncmp(buf, "count", 5) == 0)
		midev->error_inject.mode = MCTP_ERR_MODE_COUNT;
	else {
		spin_unlock_bh(&midev->error_inject.lock);
		return -EINVAL;
	}
	
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}

static const struct file_operations mctp_debugfs_mode_fops = {
	.owner = THIS_MODULE,
	.read = mctp_debugfs_mode_read,
	.write = mctp_debugfs_mode_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Generic u32 read/write helpers */
#define MCTP_DEBUGFS_U32_DEFINE(name, field) \
static ssize_t mctp_debugfs_##name##_read(struct file *file, char __user *userbuf, \
					  size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[32]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%u\n", midev->error_inject.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
					   size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[32]; \
	u32 val; \
	int rc; \
	\
	if (count >= sizeof(buf)) \
		return -EINVAL; \
	\
	if (copy_from_user(buf, userbuf, count)) \
		return -EFAULT; \
	\
	buf[count] = '\0'; \
	rc = kstrtou32(buf, 0, &val); \
	if (rc) \
		return rc; \
	\
	spin_lock_bh(&midev->error_inject.lock); \
	midev->error_inject.field = val; \
	spin_unlock_bh(&midev->error_inject.lock); \
	\
	return count; \
} \
\
static const struct file_operations mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE, \
	.read = mctp_debugfs_##name##_read, \
	.write = mctp_debugfs_##name##_write, \
	.open = simple_open, \
	.llseek = default_llseek, \
};

/* Generic int read/write helpers (for error codes) */
#define MCTP_DEBUGFS_INT_DEFINE(name, field) \
static ssize_t mctp_debugfs_##name##_read(struct file *file, char __user *userbuf, \
					  size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[32]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
					   size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[32]; \
	int val; \
	int rc; \
	\
	if (count >= sizeof(buf)) \
		return -EINVAL; \
	\
	if (copy_from_user(buf, userbuf, count)) \
		return -EFAULT; \
	\
	buf[count] = '\0'; \
	rc = kstrtoint(buf, 0, &val); \
	if (rc) \
		return rc; \
	\
	spin_lock_bh(&midev->error_inject.lock); \
	midev->error_inject.field = val; \
	spin_unlock_bh(&midev->error_inject.lock); \
	\
	return count; \
} \
\
static const struct file_operations mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE, \
	.read = mctp_debugfs_##name##_read, \
	.write = mctp_debugfs_##name##_write, \
	.open = simple_open, \
	.llseek = default_llseek, \
};

/* Generic bool read/write helpers */
#define MCTP_DEBUGFS_BOOL_DEFINE(name, field) \
static ssize_t mctp_debugfs_##name##_read(struct file *file, char __user *userbuf, \
					  size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[8]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.field ? 1 : 0); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
					   size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[8]; \
	int val; \
	int rc; \
	\
	if (count >= sizeof(buf)) \
		return -EINVAL; \
	\
	if (copy_from_user(buf, userbuf, count)) \
		return -EFAULT; \
	\
	buf[count] = '\0'; \
	rc = kstrtoint(buf, 0, &val); \
	if (rc) \
		return rc; \
	\
	spin_lock_bh(&midev->error_inject.lock); \
	midev->error_inject.field = (val != 0); \
	spin_unlock_bh(&midev->error_inject.lock); \
	\
	return count; \
} \
\
static const struct file_operations mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE, \
	.read = mctp_debugfs_##name##_read, \
	.write = mctp_debugfs_##name##_write, \
	.open = simple_open, \
	.llseek = default_llseek, \
};

/* Define all the file operations */
MCTP_DEBUGFS_INT_DEFINE(i2c_tx_error_code, i2c_tx_error_code)
MCTP_DEBUGFS_U32_DEFINE(i2c_tx_error_rate, i2c_tx_error_rate)
MCTP_DEBUGFS_BOOL_DEFINE(enable_fragment_drop, enable_fragment_drop)
MCTP_DEBUGFS_BOOL_DEFINE(enable_seq_corrupt, enable_seq_corrupt)
MCTP_DEBUGFS_BOOL_DEFINE(enable_som_clear, enable_som_clear)
MCTP_DEBUGFS_U32_DEFINE(delay_ms, delay_ms)

/* EID filter attributes */
#define MCTP_DEBUGFS_U8_DEFINE(name, field) \
static ssize_t mctp_debugfs_##name##_read(struct file *file, char __user *userbuf, \
					  size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[8]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%u\n", midev->error_inject.eid_filter.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
					   size_t count, loff_t *ppos) \
{ \
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file); \
	char buf[8]; \
	u8 val; \
	int rc; \
	\
	if (count >= sizeof(buf)) \
		return -EINVAL; \
	\
	if (copy_from_user(buf, userbuf, count)) \
		return -EFAULT; \
	\
	buf[count] = '\0'; \
	rc = kstrtou8(buf, 0, &val); \
	if (rc) \
		return rc; \
	\
	spin_lock_bh(&midev->error_inject.lock); \
	midev->error_inject.eid_filter.field = val; \
	spin_unlock_bh(&midev->error_inject.lock); \
	\
	return count; \
} \
\
static const struct file_operations mctp_debugfs_##name##_fops = { \
	.owner = THIS_MODULE, \
	.read = mctp_debugfs_##name##_read, \
	.write = mctp_debugfs_##name##_write, \
	.open = simple_open, \
	.llseek = default_llseek, \
};

static ssize_t mctp_debugfs_eid_filter_enable_read(struct file *file, char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.eid_filter.enabled);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_eid_filter_enable_write(struct file *file, const char __user *userbuf,
						    size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.eid_filter.enabled = enable;
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}

static const struct file_operations mctp_debugfs_eid_filter_enable_fops = {
	.owner = THIS_MODULE,
	.read = mctp_debugfs_eid_filter_enable_read,
	.write = mctp_debugfs_eid_filter_enable_write,
	.open = simple_open,
	.llseek = default_llseek,
};

MCTP_DEBUGFS_U8_DEFINE(eid_filter_src_eid, src_eid)
MCTP_DEBUGFS_U8_DEFINE(eid_filter_dest_eid, dest_eid)
MCTP_DEBUGFS_U8_DEFINE(eid_filter_msg_type, msg_type)

/* stats attribute (read-only) - unified with USB */
static ssize_t mctp_debugfs_stats_read(struct file *file, char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
	char *buf;
	int len = 0;
	ssize_t ret;
	
	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	
	spin_lock_bh(&ei->lock);
	
	len += snprintf(buf + len, PAGE_SIZE - len, "enable_tx: %d\n", ei->enable_tx);
	len += snprintf(buf + len, PAGE_SIZE - len, "enable_rx: %d\n", ei->enable_rx);
	len += snprintf(buf + len, PAGE_SIZE - len, "mode: %s\n",
			ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
			ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	len += snprintf(buf + len, PAGE_SIZE - len, "i2c_tx_errors_injected: %u\n",
			ei->i2c_tx_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "fragments_dropped: %u\n",
			ei->fragments_dropped);
	len += snprintf(buf + len, PAGE_SIZE - len, "seq_corruptions: %u\n",
			ei->seq_corruptions);
	len += snprintf(buf + len, PAGE_SIZE - len, "som_clears: %u\n",
			ei->som_clears);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_packets_processed: %llu\n",
			ei->total_packets_processed);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_errors_injected: %llu\n",
			ei->total_errors_injected);
	
	spin_unlock_bh(&ei->lock);
	
	ret = simple_read_from_buffer(userbuf, count, ppos, buf, len);
	kfree(buf);
	
	return ret;
}

static const struct file_operations mctp_debugfs_stats_fops = {
	.owner = THIS_MODULE,
	.read = mctp_debugfs_stats_read,
	.open = simple_open,
	.llseek = default_llseek,
};

/* reset attribute (write-only) - unified with USB */
static ssize_t mctp_debugfs_reset_write(struct file *file, const char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
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
	
	/* Reset all state */
	ei->enable_tx = false;
	ei->enable_rx = false;
	ei->mode = MCTP_ERR_MODE_ALWAYS;
	ei->i2c_tx_error_code = 0;
	ei->i2c_tx_error_rate = 0;
	ei->i2c_tx_inject_count = 0;
	ei->enable_fragment_drop = false;
	ei->enable_seq_corrupt = false;
	ei->enable_som_clear = false;
	ei->delay_ms = 0;
	ei->eid_filter.enabled = false;
	ei->eid_filter.src_eid = 0;
	ei->eid_filter.dest_eid = 0;
	ei->eid_filter.msg_type = 0;
	
	/* Reset statistics */
	ei->i2c_tx_errors_injected = 0;
	ei->fragments_dropped = 0;
	ei->seq_corruptions = 0;
	ei->som_clears = 0;
	ei->total_packets_processed = 0;
	ei->total_errors_injected = 0;
	
	spin_unlock_bh(&ei->lock);
	
	netdev_dbg(midev->ndev, "Error injection reset\n");
	
	return count;
}

static const struct file_operations mctp_debugfs_reset_fops = {
	.owner = THIS_MODULE,
	.write = mctp_debugfs_reset_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/**
 * mctp_i2c_error_inject_init - Initialize per-device error injection
 * @midev: MCTP I2C device
 *
 * Creates debugfs directory and files for this device
 */
void mctp_i2c_error_inject_init(struct mctp_i2c_dev *midev)
{
	struct dentry *dir, *eid_dir;
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
	
	/* Initialize error injection state */
	spin_lock_init(&ei->lock);
	prandom_seed_state(&ei->rng, (u64)jiffies);
	ei->enable_tx = false;
	ei->enable_rx = false;
	ei->mode = MCTP_ERR_MODE_ALWAYS;
	/* All other fields are zero-initialized */
	
	if (!mctp_i2c_error_inject_root)
		return;
	
	/* Create per-device debugfs directory */
	dir = debugfs_create_dir(netdev_name(midev->ndev), mctp_i2c_error_inject_root);
	if (IS_ERR_OR_NULL(dir))
		return;
	
	midev->debugfs_dir = dir;
	
	/* Create error injection files - unified with USB */
	debugfs_create_file("enable_tx", 0600, dir, midev, &mctp_debugfs_enable_tx_fops);
	debugfs_create_file("enable_rx", 0600, dir, midev, &mctp_debugfs_enable_rx_fops);
	debugfs_create_file("mode", 0600, dir, midev, &mctp_debugfs_mode_fops);
	debugfs_create_file("i2c_tx_error_code", 0600, dir, midev, &mctp_debugfs_i2c_tx_error_code_fops);
	debugfs_create_file("i2c_tx_error_rate", 0600, dir, midev, &mctp_debugfs_i2c_tx_error_rate_fops);
	debugfs_create_file("enable_fragment_drop", 0600, dir, midev, &mctp_debugfs_enable_fragment_drop_fops);
	debugfs_create_file("enable_seq_corrupt", 0600, dir, midev, &mctp_debugfs_enable_seq_corrupt_fops);
	debugfs_create_file("enable_som_clear", 0600, dir, midev, &mctp_debugfs_enable_som_clear_fops);
	debugfs_create_file("delay_ms", 0600, dir, midev, &mctp_debugfs_delay_ms_fops);
	debugfs_create_file("stats", 0400, dir, midev, &mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, dir, midev, &mctp_debugfs_reset_fops);
	
	/* Create eid_filter subdirectory - unified with USB */
	eid_dir = debugfs_create_dir("eid_filter", dir);
	if (!IS_ERR_OR_NULL(eid_dir)) {
		debugfs_create_file("enable", 0600, eid_dir, midev, &mctp_debugfs_eid_filter_enable_fops);
		debugfs_create_file("src_eid", 0600, eid_dir, midev, &mctp_debugfs_eid_filter_src_eid_fops);
		debugfs_create_file("dest_eid", 0600, eid_dir, midev, &mctp_debugfs_eid_filter_dest_eid_fops);
		debugfs_create_file("msg_type", 0600, eid_dir, midev, &mctp_debugfs_eid_filter_msg_type_fops);
	}
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_init);

/**
 * mctp_i2c_error_inject_cleanup - Cleanup per-device error injection
 * @midev: MCTP I2C device
 */
void mctp_i2c_error_inject_cleanup(struct mctp_i2c_dev *midev)
{
	debugfs_remove_recursive(midev->debugfs_dir);
	midev->debugfs_dir = NULL;
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_cleanup);

/**
 * mctp_i2c_error_inject_module_init - Initialize global error injection
 *
 * Creates root debugfs directory for MCTP I2C error injection
 */
int mctp_i2c_error_inject_module_init(void)
{
	mctp_i2c_error_inject_root = debugfs_create_dir("mctp_i2c", NULL);
	if (IS_ERR_OR_NULL(mctp_i2c_error_inject_root)) {
		pr_warn("MCTP I2C: Failed to create debugfs root, error injection disabled\n");
		mctp_i2c_error_inject_root = NULL;
		return -ENODEV;
	}
	
	pr_info("MCTP I2C Error Injection enabled\n");
	return 0;
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_module_init);

/**
 * mctp_i2c_error_inject_module_exit - Cleanup global error injection
 */
void mctp_i2c_error_inject_module_exit(void)
{
	debugfs_remove_recursive(mctp_i2c_error_inject_root);
	mctp_i2c_error_inject_root = NULL;
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_module_exit);
