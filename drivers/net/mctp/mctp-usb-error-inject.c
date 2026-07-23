// SPDX-License-Identifier: GPL-2.0+
/*
 * mctp-usb-error-inject.c - Error injection infrastructure for MCTP USB
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Generic plumbing (injection decision, the generic debugfs files, the
 * transport scalar files and reset) is provided by mctp-error-inject-common.c.
 * USB keeps its transport-specific sync/async TX, RX and fragment hooks here,
 * because they have USB-specific error semantics, header offsets and (for the
 * fragment path) message-type filtering.
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

/* ===== Error injection hooks (USB-specific) ===== */

/* TX synchronous error injection (before URB submission) */
int mctp_usb_error_inject_tx_sync(struct mctp_usb *mctp_usb, struct sk_buff *skb)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;
	struct mctp_hdr *mh;

	/* Early exit if TX error injection is disabled or sync code not set - zero overhead */
	if (!c->enable_tx || ei->urb_tx_sync_error_code == 0)
		return 0;

	/*
	 * Access MCTP header manually - SKB has USB header prepended at this point.
	 * Cannot use mctp_hdr(skb) as it has a WARN_ON check that expects the MCTP
	 * header at skb->data, but we have: [USB HDR][MCTP HDR][PAYLOAD]
	 */
	if (c->eid_filter.enabled &&
	    skb->len < sizeof(struct mctp_usb_hdr) + sizeof(struct mctp_hdr))
		return 0;

	mh = (struct mctp_hdr *)(skb->data + sizeof(struct mctp_usb_hdr));

	if (!mctp_ei_match_filter(c, mh))
		return 0;

	if (!mctp_ei_should_inject(c, ei->urb_tx_error_rate,
				   &ei->urb_tx_sync_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: TX URB submission (sync) - not injecting (mode/rate check)\n");
		return 0;
	}

	/* Inject delay if configured */
	if (c->delay_ms > 0) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: Injecting %u ms delay\n", c->delay_ms);
		msleep(c->delay_ms);
	}

	/* Update statistics */
	spin_lock_bh(&c->lock);
	ei->urb_tx_sync_errors_injected++;
	c->total_errors_injected++;
	c->total_packets_processed++;
	spin_unlock_bh(&c->lock);

	netdev_dbg(mctp_usb->netdev,
		   "Error injection: TX URB submission failed (sync) - error=%d, src_eid=%u, dest_eid=%u, "
		   "total_injected=%u, mode=%s\n",
		   ei->urb_tx_sync_error_code, mh->src, mh->dest,
		   ei->urb_tx_sync_errors_injected, mctp_ei_mode_str(c->mode));

	return ei->urb_tx_sync_error_code;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_tx_sync);

/* TX asynchronous error injection (URB completion)
 * @mctp_usb: MCTP USB device
 * @urb: The completed URB (used for status and, when EID filter enabled, to parse first packet)
 *
 * Returns: Status to use (original urb->status or injected error code).
 *
 * EID filtering: When eid_filter.enabled, we parse the first packet in the URB buffer
 * and match src_eid/dest_eid (0 = any). Batched URBs contain fragments of one message
 * (same EID pair), so checking the first packet is sufficient.
 */
int mctp_usb_error_inject_tx_async(struct mctp_usb *mctp_usb, struct urb *urb)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;
	int original_status = urb->status;

	/* Only inject if original status was success */
	if (original_status != 0) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: TX URB already has error status %d, not injecting\n",
			   original_status);
		return original_status;
	}

	/* Early exit if TX error injection is disabled or async code not set - zero overhead */
	if (!c->enable_tx || ei->urb_tx_async_error_code == 0)
		return original_status;

	/* EID filter: parse first packet and match src_eid/dest_eid (0 = any) */
	if (c->eid_filter.enabled) {
		if (urb->transfer_buffer &&
		    urb->transfer_buffer_length >= sizeof(struct mctp_usb_hdr) + sizeof(struct mctp_hdr)) {
			struct mctp_usb_hdr *usb_hdr = urb->transfer_buffer;
			struct mctp_hdr *mh = (struct mctp_hdr *)(usb_hdr + 1);

			if (!mctp_ei_match_filter(c, mh)) {
				netdev_dbg(mctp_usb->netdev,
					   "Error injection: TX URB async - EID filter mismatch (src_eid=%u, dest_eid=%u, pkt src=%u dest=%u), not injecting\n",
					   c->eid_filter.src_eid, c->eid_filter.dest_eid,
					   mh->src, mh->dest);
				return original_status;
			}
			netdev_dbg(mctp_usb->netdev,
				   "Error injection: TX URB async - EID filter match (src=%u dest=%u)\n",
				   mh->src, mh->dest);
		} else {
			netdev_dbg(mctp_usb->netdev,
				   "Error injection: TX URB async - buffer too small for EID parsing, not injecting\n");
			return original_status;
		}
	}

	if (!mctp_ei_should_inject(c, ei->urb_tx_error_rate,
				   &ei->urb_tx_async_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: TX URB completion (async) - not injecting (mode/rate check)\n");
		return original_status;
	}

	/* Update statistics */
	spin_lock_bh(&c->lock);
	ei->urb_tx_async_errors_injected++;
	c->total_errors_injected++;
	spin_unlock_bh(&c->lock);

	netdev_dbg(mctp_usb->netdev,
		   "Error injection: TX URB completion failed (async) - error=%d, total_injected=%u, mode=%s (URB-level, may affect multiple packets)\n",
		   ei->urb_tx_async_error_code,
		   ei->urb_tx_async_errors_injected, mctp_ei_mode_str(c->mode));

	return ei->urb_tx_async_error_code;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_tx_async);

/* RX error injection */
int mctp_usb_error_inject_rx(struct mctp_usb *mctp_usb, int original_status)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;

	if (original_status != 0) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: RX URB already has error status %d, not injecting\n",
			   original_status);
		return original_status;
	}

	/* Early exit if RX error injection is disabled - zero overhead */
	if (!c->enable_rx || ei->urb_rx_error_code == 0)
		return original_status;

	/* RX: Can't check EID filter yet as packet not parsed */
	if (c->eid_filter.enabled) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: RX - EID filter enabled but cannot filter (packet not parsed yet)\n");
	}

	if (!mctp_ei_should_inject(c, ei->urb_rx_error_rate,
				   &ei->urb_rx_inject_count)) {
		netdev_dbg(mctp_usb->netdev,
			   "Error injection: RX URB completion - not injecting (mode/rate check)\n");
		return original_status;
	}

	/* Update statistics */
	spin_lock_bh(&c->lock);
	ei->urb_rx_errors_injected++;
	c->total_errors_injected++;
	spin_unlock_bh(&c->lock);

	netdev_dbg(mctp_usb->netdev,
		   "Error injection: RX URB completion failed - error=%d, total_injected=%u, mode=%s\n",
		   ei->urb_rx_error_code,
		   ei->urb_rx_errors_injected, mctp_ei_mode_str(c->mode));

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
 * USB keeps a local fragment hook (rather than the common mctp_ei_fragment)
 * because it additionally filters on message type and validates packet length.
 *
 * Returns:
 *   0 - Pass through (normally or with corruption)
 *   1 - Drop this fragment
 */
int mctp_usb_error_inject_fragment(struct mctp_usb *mctp_usb, struct sk_buff *skb)
{
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;
	struct mctp_hdr *mh;
	u8 flags;
	u8 msg_type;
	u8 seq;

	/* Early exit if RX error injection disabled or no fragment injection enabled */
	if (!c->enable_rx ||
	    (!c->enable_fragment_drop && !c->enable_seq_corrupt && !c->enable_som_clear))
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
	if (c->eid_filter.enabled) {
		/* Check source EID filter */
		if (c->eid_filter.src_eid != 0 &&
		    c->eid_filter.src_eid != mh->src) {
			netdev_dbg(mctp_usb->netdev,
				   "Fragment injection: Packet filtered out (src EID %u != filter %u)\n",
				   mh->src, c->eid_filter.src_eid);
			return 0;  /* Does not match filter, pass through */
		}

		/* Check destination EID filter */
		if (c->eid_filter.dest_eid != 0 &&
		    c->eid_filter.dest_eid != mh->dest) {
			netdev_dbg(mctp_usb->netdev,
				   "Fragment injection: Packet filtered out (dest EID %u != filter %u)\n",
				   mh->dest, c->eid_filter.dest_eid);
			return 0;  /* Does not match filter, pass through */
		}

		/* Check message type filter (first byte after MCTP header) */
		msg_type = *((u8 *)(skb->data + sizeof(struct mctp_hdr)));
		if (c->eid_filter.msg_type != 0 &&
		    c->eid_filter.msg_type != msg_type) {
			netdev_dbg(mctp_usb->netdev,
				   "Fragment injection: Packet filtered out (msg_type 0x%02x != filter 0x%02x)\n",
				   msg_type, c->eid_filter.msg_type);
			return 0;  /* Does not match filter, pass through */
		}

		netdev_dbg(mctp_usb->netdev,
			   "Fragment injection: Packet MATCHES filter (src=%u, dest=%u, msg_type=0x%02x)\n",
			   mh->src, mh->dest, msg_type);
	}

	/* ===== Injection Type 1: Clear SOM bit (first fragment) ===== */
	if (c->enable_som_clear && (flags & MCTP_HDR_FLAG_SOM)) {
		/* Clear the SOM bit in first fragment */
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;

		spin_lock_bh(&c->lock);
		c->som_clears++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(mctp_usb->netdev,
			   "Error injection: SOM bit CLEARED (first fragment, src=%u, dest=%u, seq=%u)%s\n",
			   mh->src, mh->dest, seq,
			   c->eid_filter.enabled ? " [EID filter ACTIVE]" : "");

		return 0;  /* Pass through with corrupted SOM */
	}

	/* ===== Injection Type 2: Corrupt sequence number (middle/end fragments) ===== */
	if (c->enable_seq_corrupt && !(flags & MCTP_HDR_FLAG_SOM)) {
		/* This is a middle or end fragment - corrupt the sequence number */
		u8 corrupted_seq = (seq + 1) & 0x03;  /* Increment sequence (wrap at 4) */

		/* Clear old sequence and set corrupted one */
		mh->flags_seq_tag &= ~(0x03 << 4);  /* Clear bits 4-5 */
		mh->flags_seq_tag |= (corrupted_seq << 4);  /* Set corrupted sequence */

		spin_lock_bh(&c->lock);
		c->seq_corruptions++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(mctp_usb->netdev,
			   "Error injection: Sequence CORRUPTED (2nd+ fragment, src=%u, dest=%u, orig_seq=%u -> corrupted_seq=%u)%s\n",
			   mh->src, mh->dest, seq, corrupted_seq,
			   c->eid_filter.enabled ? " [EID filter ACTIVE]" : "");

		return 0;  /* Pass through with corrupted sequence */
	}

	/* ===== Injection Type 3: Drop fragment (2nd+ fragments) ===== */
	if (c->enable_fragment_drop && !(flags & MCTP_HDR_FLAG_SOM)) {
		/* Drop this fragment (2nd onwards) */
		spin_lock_bh(&c->lock);
		c->fragments_dropped++;
		c->total_errors_injected++;
		spin_unlock_bh(&c->lock);

		netdev_dbg(mctp_usb->netdev,
			   "Error injection: Fragment DROPPED (2nd+ fragment, src=%u, dest=%u, seq=%u)%s\n",
			   mh->src, mh->dest, seq,
			   c->eid_filter.enabled ? " [EID filter ACTIVE]" : "");

		return 1;  /* Drop this fragment */
	}

	/* No injection applied - pass through normally */
	return 0;
}
EXPORT_SYMBOL_GPL(mctp_usb_error_inject_fragment);

/* ===== Debugfs Interface ===== */

static struct mctp_usb *mctp_usb_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

/* stats attribute (read-only) */
static ssize_t mctp_debugfs_stats_read(struct file *file, char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct mctp_usb *mctp_usb = mctp_usb_from_file(file);
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;
	char *buf;
	int len = 0;
	ssize_t ret;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_bh(&c->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "enable_tx: %d\n", c->enable_tx);
	len += snprintf(buf + len, PAGE_SIZE - len, "enable_rx: %d\n", c->enable_rx);
	len += snprintf(buf + len, PAGE_SIZE - len, "mode: %s\n",
			mctp_ei_mode_str(c->mode));
	len += snprintf(buf + len, PAGE_SIZE - len, "tx_sync_errors_injected: %u\n",
			ei->urb_tx_sync_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "tx_async_errors_injected: %u\n",
			ei->urb_tx_async_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "rx_errors_injected: %u\n",
			ei->urb_rx_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "fragments_dropped: %u\n",
			c->fragments_dropped);
	len += snprintf(buf + len, PAGE_SIZE - len, "seq_corruptions: %u\n",
			c->seq_corruptions);
	len += snprintf(buf + len, PAGE_SIZE - len, "som_clears: %u\n",
			c->som_clears);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_packets_processed: %llu\n",
			c->total_packets_processed);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_errors_injected: %llu\n",
			c->total_errors_injected);

	spin_unlock_bh(&c->lock);

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
	struct mctp_ei_common *c = &ei->common;
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

	spin_lock_bh(&c->lock);

	/* Reset common state and counters */
	mctp_ei_common_reset(c);

	/* Reset USB-specific state */
	ei->urb_tx_sync_error_code = 0;
	ei->urb_tx_async_error_code = 0;
	ei->urb_tx_error_rate = 0;
	ei->urb_tx_sync_inject_count = 0;
	ei->urb_tx_async_inject_count = 0;
	ei->urb_rx_error_code = 0;
	ei->urb_rx_error_rate = 0;
	ei->urb_rx_inject_count = 0;
	ei->urb_tx_sync_errors_injected = 0;
	ei->urb_tx_async_errors_injected = 0;
	ei->urb_rx_errors_injected = 0;

	spin_unlock_bh(&c->lock);

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
	struct mctp_error_inject *ei = &mctp_usb->error_inject;
	struct mctp_ei_common *c = &ei->common;
	struct dentry *dir;

	/* Initialize error injection state */
	mctp_ei_common_init(c);

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

	/* Generic files shared with other bindings */
	mctp_ei_common_debugfs_create(dir, c,
				      MCTP_EI_CAP_RX_ENABLE |
				      MCTP_EI_CAP_FRAGMENT |
				      MCTP_EI_CAP_DELAY |
				      MCTP_EI_CAP_EID_FILTER);

	/* USB-specific TX/RX error codes and rates */
	mctp_ei_create_scalar_file(dir, "urb_tx_sync_error_code", 0600,
				   &ei->tx_sync_error_code_desc, MCTP_EI_SCALAR_INT,
				   &ei->urb_tx_sync_error_code, &c->lock);
	mctp_ei_create_scalar_file(dir, "urb_tx_async_error_code", 0600,
				   &ei->tx_async_error_code_desc, MCTP_EI_SCALAR_INT,
				   &ei->urb_tx_async_error_code, &c->lock);
	mctp_ei_create_scalar_file(dir, "urb_tx_error_rate", 0600,
				   &ei->tx_error_rate_desc, MCTP_EI_SCALAR_U32,
				   &ei->urb_tx_error_rate, &c->lock);
	mctp_ei_create_scalar_file(dir, "urb_rx_error_code", 0600,
				   &ei->rx_error_code_desc, MCTP_EI_SCALAR_INT,
				   &ei->urb_rx_error_code, &c->lock);
	mctp_ei_create_scalar_file(dir, "urb_rx_error_rate", 0600,
				   &ei->rx_error_rate_desc, MCTP_EI_SCALAR_U32,
				   &ei->urb_rx_error_rate, &c->lock);

	debugfs_create_file("stats", 0400, dir, mctp_usb, &mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, dir, mctp_usb, &mctp_debugfs_reset_fops);
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
