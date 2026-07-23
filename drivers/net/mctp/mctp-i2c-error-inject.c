// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP I2C Error Injection Implementation
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over I2C binding.
 *
 * Generic plumbing (injection decision, EID filter matching, fragment
 * mutation, and the generic debugfs files) is provided by
 * mctp-error-inject-common.c.  Only the I2C-specific TX error code and the
 * debugfs wiring remain here.
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
	struct mctp_ei_common *c = &ei->common;
	struct mctp_hdr *mh;

	/* Early exit if TX error injection is disabled or code not set */
	if (!c->enable_tx || ei->i2c_tx_error_code == 0)
		return 0;

	mh = mctp_hdr(skb);

	if (!mctp_ei_match_filter(c, mh))
		return 0;

	if (!mctp_ei_should_inject(c, ei->i2c_tx_error_rate,
				   &ei->i2c_tx_inject_count))
		return 0;

	/* Inject delay if configured */
	if (c->delay_ms > 0)
		msleep(c->delay_ms);

	/* Update statistics */
	spin_lock_bh(&c->lock);
	ei->i2c_tx_errors_injected++;
	c->total_errors_injected++;
	c->total_packets_processed++;
	spin_unlock_bh(&c->lock);

	netdev_dbg(midev->ndev,
		   "ERROR INJECTION: TX - error=%d, src_eid=%u, dest_eid=%u, "
		   "total_injected=%u, mode=%s\n",
		   ei->i2c_tx_error_code, mh->src, mh->dest,
		   ei->i2c_tx_errors_injected, mctp_ei_mode_str(c->mode));

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
 * Delegates to the common fragment-mutation helper, which can drop fragments,
 * corrupt sequence numbers, or clear the SOM flag.
 */
int mctp_i2c_error_inject_fragment(struct mctp_i2c_dev *midev, struct sk_buff *skb)
{
	return mctp_ei_fragment(&midev->error_inject.common, skb, midev->ndev);
}
EXPORT_SYMBOL_GPL(mctp_i2c_error_inject_fragment);

/* ===== Debugfs Interface ===== */

static struct mctp_i2c_dev *mctp_i2c_from_file(struct file *file)
{
	return file->f_inode->i_private;
}

/* stats attribute (read-only) */
static ssize_t mctp_debugfs_stats_read(struct file *file, char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
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
	len += snprintf(buf + len, PAGE_SIZE - len, "i2c_tx_errors_injected: %u\n",
			ei->i2c_tx_errors_injected);
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
	struct mctp_i2c_dev *midev = mctp_i2c_from_file(file);
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
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

	/* Reset I2C-specific state */
	ei->i2c_tx_error_code = 0;
	ei->i2c_tx_error_rate = 0;
	ei->i2c_tx_inject_count = 0;
	ei->i2c_tx_errors_injected = 0;

	spin_unlock_bh(&c->lock);

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
	struct mctp_i2c_error_inject *ei = &midev->error_inject;
	struct mctp_ei_common *c = &ei->common;
	struct dentry *dir;

	/* Initialize error injection state */
	mctp_ei_common_init(c);

	if (!mctp_i2c_error_inject_root)
		return;

	/* Create per-device debugfs directory */
	dir = debugfs_create_dir(netdev_name(midev->ndev), mctp_i2c_error_inject_root);
	if (IS_ERR_OR_NULL(dir))
		return;

	midev->debugfs_dir = dir;

	/* Generic files shared with other bindings */
	mctp_ei_common_debugfs_create(dir, c,
				      MCTP_EI_CAP_RX_ENABLE |
				      MCTP_EI_CAP_FRAGMENT |
				      MCTP_EI_CAP_DELAY |
				      MCTP_EI_CAP_EID_FILTER);

	/* I2C-specific TX error code/rate */
	mctp_ei_create_scalar_file(dir, "i2c_tx_error_code", 0600,
				   &ei->tx_error_code_desc, MCTP_EI_SCALAR_INT,
				   &ei->i2c_tx_error_code, &c->lock);
	mctp_ei_create_scalar_file(dir, "i2c_tx_error_rate", 0600,
				   &ei->tx_error_rate_desc, MCTP_EI_SCALAR_U32,
				   &ei->i2c_tx_error_rate, &c->lock);

	debugfs_create_file("stats", 0400, dir, midev, &mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, dir, midev, &mctp_debugfs_reset_fops);
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
