// SPDX-License-Identifier: GPL-2.0+
/*
 * mctp-usb-error-inject.c - Error injection infrastructure for MCTP USB
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/usb/mctp-usb.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <net/mctp.h>

#include "mctp-usb-internal.h"

/* Global debugfs root directory */
static struct dentry *mctp_usb_debugfs_root;

/* ===== Error Injection Helper Functions ===== */

/* Check if error should be injected based on mode and rate
 * Note: Caller must check ei->enable_tx or ei->enable_rx before calling this
 */
static bool mctp_should_inject_error(struct mctp_error_inject *ei, u32 rate, u32 *count)
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
static bool mctp_error_inject_match_filter(struct mctp_error_inject *ei,
                                           struct sk_buff *skb,
                                           struct net_device *netdev)
{
	struct mctp_hdr *mh;
	
	if (!ei->eid_filter.enabled)
		return true;  /* No filter, match all */
	
	/* Check if SKB has enough data for USB header + MCTP header */
	if (skb->len < (sizeof(struct mctp_usb_hdr) + sizeof(struct mctp_hdr)))
		return false;
	
	/*
	 * Access MCTP header manually - SKB has USB header prepended at this point.
	 * Cannot use mctp_hdr(skb) as it has WARN_ON check that expects MCTP header
	 * at skb->data, but we have: [USB HDR][MCTP HDR][PAYLOAD]
	 */
	mh = (struct mctp_hdr *)(skb->data + sizeof(struct mctp_usb_hdr));
	
	/* Check source EID */
	if (ei->eid_filter.src_eid != 0 &&
	    ei->eid_filter.src_eid != mh->src) {
		netdev_dbg(netdev,
		          "Error injection: Packet filtered out (src EID %u != filter %u)\n",
		          mh->src, ei->eid_filter.src_eid);
		return false;
	}
	
	/* Check dest EID */
	if (ei->eid_filter.dest_eid != 0 &&
	    ei->eid_filter.dest_eid != mh->dest) {
		netdev_dbg(netdev,
		          "Error injection: Packet filtered out (dest EID %u != filter %u)\n",
		          mh->dest, ei->eid_filter.dest_eid);
		return false;
	}
	
	/* Packet matches filter */
	netdev_dbg(netdev,
	          "Error injection: Packet matches filter (src=%u, dest=%u)\n",
	          mh->src, mh->dest);
	
	return true;
}

/* TX synchronous error injection (before URB submission) */
int mctp_usb_error_inject_tx_sync(struct mctp_usb *mctp_usb, struct sk_buff *skb)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_hdr *mh;
	
	/* Early exit if TX error injection is disabled or sync code not set - zero overhead */
	if (!ei->enable_tx || ei->urb_tx_sync_error_code == 0)
		return 0;
	
	if (!mctp_error_inject_match_filter(ei, skb, mctp_usb->netdev))
		return 0;
	
	if (!mctp_should_inject_error(ei, ei->urb_tx_error_rate,
	                              &ei->urb_tx_sync_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: TX URB submission (sync) - not injecting (mode/rate check)\n");
		return 0;
	}
	
	/* Inject delay if configured */
	if (ei->delay_ms > 0) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: Injecting %u ms delay\n",
		          ei->delay_ms);
		msleep(ei->delay_ms);
	}
	
	/* Update statistics */
	spin_lock_bh(&ei->lock);
	ei->urb_tx_sync_errors_injected++;
	ei->total_errors_injected++;
	ei->total_packets_processed++;
	spin_unlock_bh(&ei->lock);
	
	/* 
	 * Access MCTP header manually - SKB has USB header prepended at this point.
	 * Cannot use mctp_hdr(skb) as it has WARN_ON check that expects MCTP header
	 * at skb->data, but we have: [USB HDR][MCTP HDR][PAYLOAD]
	 */
	mh = (struct mctp_hdr *)(skb->data + sizeof(struct mctp_usb_hdr));
	
	netdev_dbg(mctp_usb->netdev,
	           "Error injection: TX URB submission failed (sync) - error=%d, src_eid=%u, dest_eid=%u, "
	           "total_injected=%u, mode=%s\n",
	           ei->urb_tx_sync_error_code, mh->src, mh->dest,
	           ei->urb_tx_sync_errors_injected,
	           ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
	           ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	
	return ei->urb_tx_sync_error_code;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_tx_sync);

/* TX asynchronous error injection (URB completion) 
 * Note: TX async injection happens at URB level, not packet level.
 * With batching, a single URB may contain multiple packets from different transactions.
 * Therefore, EID filtering is not applicable here - the error affects the entire URB.
 */
int mctp_usb_error_inject_tx_async(struct mctp_usb *mctp_usb,
                                    int original_status)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	
	/* Only inject if original status was success */
	if (original_status != 0) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: TX URB already has error status %d, not injecting\n",
		          original_status);
		return original_status;
	}
	
	/* Early exit if TX error injection is disabled or async code not set - zero overhead */
	if (!ei->enable_tx || ei->urb_tx_async_error_code == 0)
		return original_status;
	
	/* Check EID filter if enabled.
	 * For batched URBs: All fragments in batch are from SAME message (same EID pair).
	 * We parse the first packet's MCTP header to get src/dest EID for filtering.
	 */
	if (ei->eid_filter.enabled) {
		/* Parse first packet in URB buffer to get EID.
		 * Use transfer_buffer_length (size we allocated) not actual_length (may be 0 on error).
		 */
		if (urb->transfer_buffer && urb->transfer_buffer_length >= sizeof(struct mctp_usb_hdr) + sizeof(struct mctp_hdr)) {
			struct mctp_usb_hdr *usb_hdr = (struct mctp_usb_hdr *)urb->transfer_buffer;
			struct mctp_hdr *mctp_hdr = (struct mctp_hdr *)(usb_hdr + 1);
			
			/* Apply EID filter (check both src and dest) */
			if ((ei->eid_filter.eid != mctp_hdr->src) && 
			    (ei->eid_filter.eid != mctp_hdr->dest)) {
				netdev_dbg(mctp_usb->netdev,
				          "Error injection: TX URB async - EID filter mismatch (filter=%u, src=%u, dest=%u), not injecting\n",
				          ei->eid_filter.eid, mctp_hdr->src, mctp_hdr->dest);
				return original_status;
			}
			
			netdev_dbg(mctp_usb->netdev,
			          "Error injection: TX URB async - EID filter match (filter=%u, src=%u, dest=%u)\n",
			          ei->eid_filter.eid, mctp_hdr->src, mctp_hdr->dest);
		} else {
			/* Buffer too small to parse - can't apply filter, skip injection */
			netdev_dbg(mctp_usb->netdev,
			          "Error injection: TX URB async - buffer too small for EID parsing, not injecting\n");
			return original_status;
		}
	}
	
	if (!mctp_should_inject_error(ei, ei->urb_tx_error_rate,
	                              &ei->urb_tx_async_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: TX URB completion (async) - not injecting (mode/rate check)\n");
		return original_status;
	}
	
	/* Update statistics */
	spin_lock_bh(&ei->lock);
	ei->urb_tx_async_errors_injected++;
	ei->total_errors_injected++;
	spin_unlock_bh(&ei->lock);
	
	netdev_dbg(mctp_usb->netdev,
	           "Error injection: TX URB completion failed (async) - error=%d, total_injected=%u, mode=%s (URB-level, may affect multiple packets)\n",
	           ei->urb_tx_async_error_code,
	           ei->urb_tx_async_errors_injected,
	           ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
	           ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	
	return ei->urb_tx_async_error_code;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_tx_async);

/* RX error injection */
int mctp_usb_error_inject_rx(struct mctp_usb *mctp_usb, int original_status)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	
	if (original_status != 0) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: RX URB already has error status %d, not injecting\n",
		          original_status);
		return original_status;
	}
	
	/* Early exit if RX error injection is disabled - zero overhead */
	if (!ei->enable_rx || ei->urb_rx_error_code == 0)
		return original_status;
	
	/* RX: Can't check EID filter yet as packet not parsed */
	if (ei->eid_filter.enabled) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: RX - EID filter enabled but cannot filter (packet not parsed yet)\n");
	}
	
	if (!mctp_should_inject_error(ei, ei->urb_rx_error_rate,
	                              &ei->urb_rx_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
		          "Error injection: RX URB completion - not injecting (mode/rate check)\n");
		return original_status;
	}
	
	/* Update statistics */
	spin_lock_bh(&ei->lock);
	ei->urb_rx_errors_injected++;
	ei->total_errors_injected++;
	spin_unlock_bh(&ei->lock);
	
	netdev_dbg(mctp_usb->netdev,
	           "Error injection: RX URB completion failed - error=%d, total_injected=%u, mode=%s\n",
	           ei->urb_rx_error_code,
	           ei->urb_rx_errors_injected,
	           ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
	           ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	
	netdev_dbg(mctp_usb->netdev,
	           "Error injection: RX packet will be DROPPED due to injected error\n");
	
	return ei->urb_rx_error_code;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_rx);

/* Fragment error injection
 * Integrated with RX error injection infrastructure:
 * - Requires enable_rx to be set
 * - Supports EID filtering (src_eid, dest_eid, msg_type)
 * - Three injection modes:
 *   1. enable_fragment_drop: Drop 2nd+ fragments (triggers reassembly timeout)
 *   2. enable_seq_corrupt: Corrupt sequence number in middle/end fragments (triggers sequence error)
 *   3. enable_som_clear: Clear SOM bit in first fragment (triggers missing SOM error)
 * 
 * Returns:
 *   0 - Pass through (normally or with corruption)
 *   1 - Drop this fragment
 */
int mctp_usb_error_inject_fragment(struct mctp_usb *mctp_usb, struct sk_buff *skb)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_hdr *mh;
	u8 flags;
	u8 msg_type;
	u8 seq;
	
	/* Early exit if RX error injection disabled or no fragment injection enabled */
	if (!ei->enable_rx || (!ei->enable_fragment_drop && !ei->enable_seq_corrupt && !ei->enable_som_clear))
		return 0;
	
	/* Check if we have enough data for MCTP header + message type */
	if (skb->len < sizeof(struct mctp_hdr) + 1)
		return 0;
	
	/* Parse MCTP header (skb->data points to MCTP header after USB header removed) */
	mh = (struct mctp_hdr *)skb->data;
	flags = mh->flags_seq_tag & (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	seq = (mh->flags_seq_tag >> 4) & 0x03;
	
	/* Single-packet message (SOM+EOM) - pass through (not a fragment) */
	if (flags == (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM))
		return 0;
	
	/* Apply EID filtering if enabled */
	if (ei->eid_filter.enabled) {
		/* Check source EID filter */
		if (ei->eid_filter.src_eid != 0 &&
		    ei->eid_filter.src_eid != mh->src) {
			netdev_dbg(mctp_usb->netdev,
			          "Fragment injection: Packet filtered out (src EID %u != filter %u)\n",
			          mh->src, ei->eid_filter.src_eid);
			return 0;  /* Does not match filter, pass through */
		}
		
		/* Check destination EID filter */
		if (ei->eid_filter.dest_eid != 0 &&
		    ei->eid_filter.dest_eid != mh->dest) {
			netdev_dbg(mctp_usb->netdev,
			          "Fragment injection: Packet filtered out (dest EID %u != filter %u)\n",
			          mh->dest, ei->eid_filter.dest_eid);
			return 0;  /* Does not match filter, pass through */
		}
		
		/* Check message type filter (first byte after MCTP header) */
		msg_type = *((u8 *)(skb->data + sizeof(struct mctp_hdr)));
		if (ei->eid_filter.msg_type != 0 &&
		    ei->eid_filter.msg_type != msg_type) {
			netdev_dbg(mctp_usb->netdev,
			          "Fragment injection: Packet filtered out (msg_type 0x%02x != filter 0x%02x)\n",
			          msg_type, ei->eid_filter.msg_type);
			return 0;  /* Does not match filter, pass through */
		}
		
		netdev_dbg(mctp_usb->netdev,
		          "Fragment injection: Packet MATCHES filter (src=%u, dest=%u, msg_type=0x%02x)\n",
		          mh->src, mh->dest, msg_type);
	}
	
	/* ===== Injection Type 1: Clear SOM bit (first fragment) ===== */
	if (ei->enable_som_clear && (flags & MCTP_HDR_FLAG_SOM)) {
		/* Clear the SOM bit in first fragment */
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;
		
		spin_lock_bh(&ei->lock);
		ei->som_clears++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(mctp_usb->netdev,
		           "Error injection: SOM bit CLEARED (first fragment, src=%u, dest=%u, seq=%u)%s\n",
		           mh->src, mh->dest, seq,
		           ei->eid_filter.enabled ? " [EID filter ACTIVE]" : "");
		
		return 0;  /* Pass through with corrupted SOM */
	}
	
	/* ===== Injection Type 2: Corrupt sequence number (middle/end fragments) ===== */
	if (ei->enable_seq_corrupt && !(flags & MCTP_HDR_FLAG_SOM)) {
		/* This is a middle or end fragment - corrupt the sequence number */
		u8 corrupted_seq = (seq + 1) & 0x03;  /* Increment sequence (wrap at 4) */
		
		/* Clear old sequence and set corrupted one */
		mh->flags_seq_tag &= ~(0x03 << 4);  /* Clear bits 4-5 */
		mh->flags_seq_tag |= (corrupted_seq << 4);  /* Set corrupted sequence */
		
		spin_lock_bh(&ei->lock);
		ei->seq_corruptions++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(mctp_usb->netdev,
		           "Error injection: Sequence CORRUPTED (2nd+ fragment, src=%u, dest=%u, orig_seq=%u -> corrupted_seq=%u)%s\n",
		           mh->src, mh->dest, seq, corrupted_seq,
		           ei->eid_filter.enabled ? " [EID filter ACTIVE]" : "");
		
		return 0;  /* Pass through with corrupted sequence */
	}
	
	/* ===== Injection Type 3: Drop fragment (2nd+ fragments) ===== */
	if (ei->enable_fragment_drop && !(flags & MCTP_HDR_FLAG_SOM)) {
		/* Drop this fragment (2nd onwards) */
		spin_lock_bh(&ei->lock);
		ei->fragments_dropped++;
		ei->total_errors_injected++;
		spin_unlock_bh(&ei->lock);
		
		netdev_dbg(mctp_usb->netdev,
		           "Error injection: Fragment DROPPED (2nd+ fragment, src=%u, dest=%u, seq=%u)%s\n",
		           mh->src, mh->dest, seq,
		           ei->eid_filter.enabled ? " [EID filter ACTIVE]" : "");
		
		return 1;  /* Drop this fragment */
	}
	
	/* No injection applied - pass through normally */
	return 0;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_fragment);

/* ===== Debugfs Interface ===== */

/* Helper to get mctp_usb from file */
static struct mctp_usb *mctp_usb_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

/* enable_tx attribute */
static ssize_t mctp_debugfs_enable_tx_read(struct file *file, char __user *userbuf,
                                           size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_usb->error_inject.enable_tx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_enable_tx_write(struct file *file, const char __user *userbuf,
                                            size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
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
	
	spin_lock_bh(&mctp_usb->error_inject.lock);
	mctp_usb->error_inject.enable_tx = enable;
	spin_unlock_bh(&mctp_usb->error_inject.lock);
	
	netdev_dbg(mctp_usb->netdev, "TX error injection %s\n",
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_usb->error_inject.enable_rx);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_enable_rx_write(struct file *file, const char __user *userbuf,
                                            size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
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
	
	spin_lock_bh(&mctp_usb->error_inject.lock);
	mctp_usb->error_inject.enable_rx = enable;
	spin_unlock_bh(&mctp_usb->error_inject.lock);
	
	netdev_dbg(mctp_usb->netdev, "RX error injection %s\n",
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	const char *mode_str;
	char buf[16];
	int len;
	
	switch (mctp_usb->error_inject.mode) {
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	char buf[16];
	
	if (count >= sizeof(buf))
		return -EINVAL;
	
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	
	buf[count] = '\0';
	
	spin_lock_bh(&mctp_usb->error_inject.lock);
	
	if (strncmp(buf, "always", 6) == 0)
		mctp_usb->error_inject.mode = MCTP_ERR_MODE_ALWAYS;
	else if (strncmp(buf, "random", 6) == 0)
		mctp_usb->error_inject.mode = MCTP_ERR_MODE_RANDOM;
	else if (strncmp(buf, "count", 5) == 0)
		mctp_usb->error_inject.mode = MCTP_ERR_MODE_COUNT;
	else {
		spin_unlock_bh(&mctp_usb->error_inject.lock);
		return -EINVAL;
	}
	
	spin_unlock_bh(&mctp_usb->error_inject.lock);
	
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
	char buf[32]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%u\n", mctp_usb->error_inject.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
                                           size_t count, loff_t *ppos) \
{ \
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
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
	spin_lock_bh(&mctp_usb->error_inject.lock); \
	mctp_usb->error_inject.field = val; \
	spin_unlock_bh(&mctp_usb->error_inject.lock); \
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
	char buf[32]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_usb->error_inject.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
                                           size_t count, loff_t *ppos) \
{ \
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
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
	spin_lock_bh(&mctp_usb->error_inject.lock); \
	mctp_usb->error_inject.field = val; \
	spin_unlock_bh(&mctp_usb->error_inject.lock); \
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
	char buf[8]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_usb->error_inject.field ? 1 : 0); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
                                           size_t count, loff_t *ppos) \
{ \
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
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
	spin_lock_bh(&mctp_usb->error_inject.lock); \
	mctp_usb->error_inject.field = (val != 0); \
	spin_unlock_bh(&mctp_usb->error_inject.lock); \
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
MCTP_DEBUGFS_INT_DEFINE(urb_tx_sync_error_code, urb_tx_sync_error_code)
MCTP_DEBUGFS_INT_DEFINE(urb_tx_async_error_code, urb_tx_async_error_code)
MCTP_DEBUGFS_U32_DEFINE(urb_tx_error_rate, urb_tx_error_rate)
MCTP_DEBUGFS_INT_DEFINE(urb_rx_error_code, urb_rx_error_code)
MCTP_DEBUGFS_U32_DEFINE(urb_rx_error_rate, urb_rx_error_rate)
MCTP_DEBUGFS_BOOL_DEFINE(enable_fragment_drop, enable_fragment_drop)
MCTP_DEBUGFS_BOOL_DEFINE(enable_seq_corrupt, enable_seq_corrupt)
MCTP_DEBUGFS_BOOL_DEFINE(enable_som_clear, enable_som_clear)
MCTP_DEBUGFS_U32_DEFINE(delay_ms, delay_ms)

/* EID filter attributes */
#define MCTP_DEBUGFS_U8_DEFINE(name, field) \
static ssize_t mctp_debugfs_##name##_read(struct file *file, char __user *userbuf, \
                                          size_t count, loff_t *ppos) \
{ \
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
	char buf[8]; \
	int len; \
	\
	len = snprintf(buf, sizeof(buf), "%u\n", mctp_usb->error_inject.eid_filter.field); \
	return simple_read_from_buffer(userbuf, count, ppos, buf, len); \
} \
\
static ssize_t mctp_debugfs_##name##_write(struct file *file, const char __user *userbuf, \
                                           size_t count, loff_t *ppos) \
{ \
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file); \
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
	spin_lock_bh(&mctp_usb->error_inject.lock); \
	mctp_usb->error_inject.eid_filter.field = val; \
	spin_unlock_bh(&mctp_usb->error_inject.lock); \
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
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_usb->error_inject.eid_filter.enabled);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t mctp_debugfs_eid_filter_enable_write(struct file *file, const char __user *userbuf,
                                                    size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
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
	
	spin_lock_bh(&mctp_usb->error_inject.lock);
	mctp_usb->error_inject.eid_filter.enabled = enable;
	spin_unlock_bh(&mctp_usb->error_inject.lock);
	
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

/* stats attribute (read-only) */
static ssize_t mctp_debugfs_stats_read(struct file *file, char __user *userbuf,
                                       size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
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
	len += snprintf(buf + len, PAGE_SIZE - len, "tx_sync_errors_injected: %u\n",
	               ei->urb_tx_sync_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "tx_async_errors_injected: %u\n",
	               ei->urb_tx_async_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "rx_errors_injected: %u\n",
	               ei->urb_rx_errors_injected);
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

/* reset attribute (write-only) */
static ssize_t mctp_debugfs_reset_write(struct file *file, const char __user *userbuf,
                                        size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
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
	ei->urb_tx_sync_error_code = 0;
	ei->urb_tx_async_error_code = 0;
	ei->urb_tx_error_rate = 0;
	ei->urb_tx_sync_inject_count = 0;
	ei->urb_tx_async_inject_count = 0;
	ei->urb_rx_error_code = 0;
	ei->urb_rx_error_rate = 0;
	ei->urb_rx_inject_count = 0;
	ei->enable_fragment_drop = false;
	ei->enable_seq_corrupt = false;
	ei->enable_som_clear = false;
	ei->delay_ms = 0;
	ei->eid_filter.enabled = false;
	ei->eid_filter.src_eid = 0;
	ei->eid_filter.dest_eid = 0;
	ei->eid_filter.msg_type = 0;
	
	/* Reset statistics */
	ei->urb_tx_sync_errors_injected = 0;
	ei->urb_tx_async_errors_injected = 0;
	ei->urb_rx_errors_injected = 0;
	ei->fragments_dropped = 0;
	ei->seq_corruptions = 0;
	ei->som_clears = 0;
	ei->total_packets_processed = 0;
	ei->total_errors_injected = 0;
	
	spin_unlock_bh(&ei->lock);
	
	netdev_dbg(mctp_usb->netdev, "Error injection reset\n");
	
	return count;
}

static const struct file_operations mctp_debugfs_reset_fops = {
	.owner = THIS_MODULE,
	.write = mctp_debugfs_reset_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Initialize error injection - called from main driver probe */
void mctp_usb_error_inject_init(struct mctp_usb *mctp_usb)
{
	struct dentry *dir, *eid_dir;
	
	/* Initialize error injection state */
	spin_lock_init(&mctp_usb->error_inject.lock);
	prandom_seed_state(&mctp_usb->error_inject.rng, (u64)jiffies);
	mctp_usb->error_inject.enable_tx = false;
	mctp_usb->error_inject.enable_rx = false;
	mctp_usb->error_inject.mode = MCTP_ERR_MODE_ALWAYS;
	/* All other fields are zero-initialized */
	
	/* Setup debugfs */
	if (!mctp_usb_debugfs_root)
		return;
	
	/* Use USB interface device name (stable, unique, persistent across reboots)
	 * instead of netdev name (which can be renamed by udev after probe).
	 * This prevents debugfs directory name collisions when devices are hot-plugged.
	 * Example: "1-1.1.1.1:1.0" instead of "mctpusb0"
	 */
	dir = debugfs_create_dir(dev_name(&mctp_usb->intf->dev), mctp_usb_debugfs_root);
	if (IS_ERR_OR_NULL(dir)) {
		netdev_warn(mctp_usb->netdev,
			    "Failed to create debugfs directory '%s', error injection disabled\n",
			    dev_name(&mctp_usb->intf->dev));
		return;
	}
	
	mctp_usb->debugfs_dir = dir;
	
	/* Create error_inject files */
	debugfs_create_file("enable_tx", 0600, dir, mctp_usb, &mctp_debugfs_enable_tx_fops);
	debugfs_create_file("enable_rx", 0600, dir, mctp_usb, &mctp_debugfs_enable_rx_fops);
	debugfs_create_file("mode", 0600, dir, mctp_usb, &mctp_debugfs_mode_fops);
	debugfs_create_file("urb_tx_sync_error_code", 0600, dir, mctp_usb, &mctp_debugfs_urb_tx_sync_error_code_fops);
	debugfs_create_file("urb_tx_async_error_code", 0600, dir, mctp_usb, &mctp_debugfs_urb_tx_async_error_code_fops);
	debugfs_create_file("urb_tx_error_rate", 0600, dir, mctp_usb, &mctp_debugfs_urb_tx_error_rate_fops);
	debugfs_create_file("urb_rx_error_code", 0600, dir, mctp_usb, &mctp_debugfs_urb_rx_error_code_fops);
	debugfs_create_file("urb_rx_error_rate", 0600, dir, mctp_usb, &mctp_debugfs_urb_rx_error_rate_fops);
	debugfs_create_file("enable_fragment_drop", 0600, dir, mctp_usb, &mctp_debugfs_enable_fragment_drop_fops);
	debugfs_create_file("enable_seq_corrupt", 0600, dir, mctp_usb, &mctp_debugfs_enable_seq_corrupt_fops);
	debugfs_create_file("enable_som_clear", 0600, dir, mctp_usb, &mctp_debugfs_enable_som_clear_fops);
	debugfs_create_file("delay_ms", 0600, dir, mctp_usb, &mctp_debugfs_delay_ms_fops);
	debugfs_create_file("stats", 0400, dir, mctp_usb, &mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, dir, mctp_usb, &mctp_debugfs_reset_fops);
	
	/* Create eid_filter subdirectory */
	eid_dir = debugfs_create_dir("eid_filter", dir);
	if (!IS_ERR_OR_NULL(eid_dir)) {
		debugfs_create_file("enable", 0600, eid_dir, mctp_usb, &mctp_debugfs_eid_filter_enable_fops);
		debugfs_create_file("src_eid", 0600, eid_dir, mctp_usb, &mctp_debugfs_eid_filter_src_eid_fops);
		debugfs_create_file("dest_eid", 0600, eid_dir, mctp_usb, &mctp_debugfs_eid_filter_dest_eid_fops);
		debugfs_create_file("msg_type", 0600, eid_dir, mctp_usb, &mctp_debugfs_eid_filter_msg_type_fops);
	}
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_init);

/* Cleanup error injection - called from main driver disconnect */
void mctp_usb_error_inject_cleanup(struct mctp_usb *mctp_usb)
{
	debugfs_remove_recursive(mctp_usb->debugfs_dir);
	mctp_usb->debugfs_dir = NULL;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_cleanup);

/* Module init for debugfs root */
int __init mctp_usb_error_inject_module_init(void)
{
	/* Create debugfs root directory */
	mctp_usb_debugfs_root = debugfs_create_dir("mctp_usb", NULL);
	if (IS_ERR_OR_NULL(mctp_usb_debugfs_root)) {
		pr_warn("MCTP USB: Failed to create debugfs root, error injection disabled\n");
		mctp_usb_debugfs_root = NULL;
		return -ENODEV;
	}
	
	return 0;
}

/* Module exit for debugfs root
 * Note: Not marked __exit because it's called from __init error path in mctp-usb.c
 */
void mctp_usb_error_inject_module_exit(void)
{
	debugfs_remove_recursive(mctp_usb_debugfs_root);
	mctp_usb_debugfs_root = NULL;
}

