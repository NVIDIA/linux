// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP SPI Error Injection Implementation
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over SPI binding.
 *
 * Generic plumbing (injection decision, EID filter matching, the enable_tx and
 * mode debugfs files, and reset) is provided by mctp-error-inject-common.c.
 * SPI is TX-only and returns SPB_AP status codes, so only the SPI-specific TX
 * hook and the (range-validated) error code/rate files remain here.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/mctp.h>

/*
 * error-inject only uses SpbAp as an opaque pointer (struct mctp_spi.ap, never
 * dereferenced here). Provide a dummy type and suppress the real
 * glacier-spb-ap.h include guard so its header-defined functions are not
 * emitted into this translation unit (they would clash at link with
 * mctp-spi.o).
 */
#define __GLACIER_SPB_AP_H__
typedef struct { int dummy; } SpbAp;

#include "mctp-spi-internal.h"

/* Global debugfs root for all MCTP SPI error injection */
static struct dentry *mctp_spi_error_inject_root;

/**
 * mctp_spi_error_inject_tx - Inject TX error
 * @midev: MCTP SPI device
 * @skb: Packet being transmitted
 * @injected_status: Output - SPB_AP status code to inject
 *
 * Returns: 0 for normal operation, 1 to inject error
 *
 * Called before spb_ap_send() to potentially inject a TX error.
 */
int mctp_spi_error_inject_tx(struct mctp_spi *midev, struct sk_buff *skb, int *injected_status)
{
	struct mctp_spi_error_inject *ei = &midev->error_inject;
	struct mctp_ei_common *c = &ei->common;
	struct mctp_hdr *mh;

	/* Early exit if TX error injection is disabled or code not set */
	if (!c->enable_tx || ei->spi_tx_error_code == 0)
		return 0;

	/* Skip the SPI transport header to reach the MCTP header */
	mh = (struct mctp_hdr *)(skb->data + sizeof(struct mctp_spi_hdr));

	if (!mctp_ei_match_filter(c, mh))
		return 0;

	if (!mctp_ei_should_inject(c, ei->spi_tx_error_rate,
				   &ei->spi_tx_inject_count))
		return 0;

	/* Inject delay if configured */
	if (c->delay_ms > 0)
		msleep(c->delay_ms);

	/* Update statistics */
	spin_lock_bh(&c->lock);
	ei->spi_tx_errors_injected++;
	c->total_errors_injected++;
	c->total_packets_processed++;
	spin_unlock_bh(&c->lock);

	netdev_info(midev->ndev,
		    "ERROR INJECTION: SPI TX - error=%d, src_eid=%u, dest_eid=%u, "
		    "total_injected=%u, mode=%s\n",
		    ei->spi_tx_error_code, mh->src, mh->dest,
		    ei->spi_tx_errors_injected, mctp_ei_mode_str(c->mode));

	*injected_status = ei->spi_tx_error_code;
	return 1;
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_tx);

/* ===== SPI-specific debugfs files (range-validated, base 10) ===== */

static ssize_t error_code_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	int len;

	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.spi_tx_error_code);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t error_code_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	int val;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = '\0';
	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	/* Valid SPB_AP error codes: 2=INVALID_ARG, 3=TIMEOUT, 4=UNKNOWN */
	if (val < 0 || val > 10)
		return -EINVAL;

	spin_lock_bh(&midev->error_inject.common.lock);
	midev->error_inject.spi_tx_error_code = val;
	spin_unlock_bh(&midev->error_inject.common.lock);

	return count;
}

static const struct file_operations mctp_debugfs_spi_tx_error_code_fops = {
	.owner = THIS_MODULE,
	.read = error_code_read,
	.write = error_code_write,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t rate_read(struct file *file, char __user *user_buf,
			  size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	int len;

	len = snprintf(buf, sizeof(buf), "%u\n", midev->error_inject.spi_tx_error_rate);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t rate_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	u32 val;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = '\0';
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 100)
		return -EINVAL;

	spin_lock_bh(&midev->error_inject.common.lock);
	midev->error_inject.spi_tx_error_rate = val;
	spin_unlock_bh(&midev->error_inject.common.lock);

	return count;
}

static const struct file_operations mctp_debugfs_spi_tx_error_rate_fops = {
	.owner = THIS_MODULE,
	.read = rate_read,
	.write = rate_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* stats attribute (read-only) */
static ssize_t stats_read(struct file *file, char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	struct mctp_spi_error_inject *ei = &midev->error_inject;
	struct mctp_ei_common *c = &ei->common;
	char *buf;
	int len = 0;
	ssize_t ret;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_bh(&c->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "enable_tx: %d\n", c->enable_tx);
	len += snprintf(buf + len, PAGE_SIZE - len, "mode: %s\n",
			mctp_ei_mode_str(c->mode));
	len += snprintf(buf + len, PAGE_SIZE - len, "spi_tx_errors_injected: %u\n",
			ei->spi_tx_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_packets_processed: %llu\n",
			c->total_packets_processed);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_errors_injected: %llu\n",
			c->total_errors_injected);

	spin_unlock_bh(&c->lock);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static const struct file_operations mctp_debugfs_stats_fops = {
	.owner = THIS_MODULE,
	.read = stats_read,
	.open = simple_open,
	.llseek = default_llseek,
};

/* reset attribute (write-only) */
static ssize_t reset_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	struct mctp_spi_error_inject *ei = &midev->error_inject;
	struct mctp_ei_common *c = &ei->common;
	char buf[8];
	int val;
	int rc;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
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

	/* Reset SPI-specific state */
	ei->spi_tx_error_code = 0;
	ei->spi_tx_error_rate = 0;
	ei->spi_tx_inject_count = 0;
	ei->spi_tx_errors_injected = 0;

	spin_unlock_bh(&c->lock);

	netdev_dbg(midev->ndev, "Error injection reset\n");

	return count;
}

static const struct file_operations mctp_debugfs_reset_fops = {
	.owner = THIS_MODULE,
	.write = reset_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/**
 * mctp_spi_error_inject_init - Initialize error injection for a device
 * @midev: MCTP SPI device
 */
void mctp_spi_error_inject_init(struct mctp_spi *midev)
{
	struct mctp_spi_error_inject *ei = &midev->error_inject;
	struct mctp_ei_common *c = &ei->common;

	/* Initialize error injection state */
	memset(ei, 0, sizeof(*ei));
	mctp_ei_common_init(c);

	/* SPI default configuration */
	ei->spi_tx_error_rate = 100;

	/* Create device-specific debugfs directory */
	if (!mctp_spi_error_inject_root)
		return;

	midev->debugfs_dir = debugfs_create_dir(midev->ndev->name,
						mctp_spi_error_inject_root);
	if (IS_ERR_OR_NULL(midev->debugfs_dir)) {
		netdev_warn(midev->ndev, "Failed to create debugfs directory\n");
		midev->debugfs_dir = NULL;
		return;
	}

	/* Generic files (enable_tx + mode); SPI is TX-only so no other caps. */
	mctp_ei_common_debugfs_create(midev->debugfs_dir, c, 0);

	/* SPI-specific TX error code/rate */
	debugfs_create_file("spi_tx_error_code", 0600, midev->debugfs_dir, midev,
			    &mctp_debugfs_spi_tx_error_code_fops);
	debugfs_create_file("spi_tx_error_rate", 0600, midev->debugfs_dir, midev,
			    &mctp_debugfs_spi_tx_error_rate_fops);
	debugfs_create_file("stats", 0400, midev->debugfs_dir, midev,
			    &mctp_debugfs_stats_fops);
	debugfs_create_file("reset", 0200, midev->debugfs_dir, midev,
			    &mctp_debugfs_reset_fops);

	netdev_info(midev->ndev, "Error injection debugfs created at /sys/kernel/debug/mctp_spi/%s\n",
		    midev->ndev->name);
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_init);

/**
 * mctp_spi_error_inject_cleanup - Cleanup error injection for a device
 * @midev: MCTP SPI device
 */
void mctp_spi_error_inject_cleanup(struct mctp_spi *midev)
{
	if (midev->debugfs_dir) {
		debugfs_remove_recursive(midev->debugfs_dir);
		midev->debugfs_dir = NULL;
	}
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_cleanup);

/**
 * mctp_spi_error_inject_module_init - Initialize module-wide debugfs
 */
int mctp_spi_error_inject_module_init(void)
{
	mctp_spi_error_inject_root = debugfs_create_dir("mctp_spi", NULL);
	if (IS_ERR_OR_NULL(mctp_spi_error_inject_root)) {
		pr_warn("MCTP SPI: Failed to create debugfs root, error injection disabled\n");
		mctp_spi_error_inject_root = NULL;
		return -ENODEV;
	}

	pr_info("MCTP SPI: Error injection initialized at /sys/kernel/debug/mctp_spi/\n");
	return 0;
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_module_init);

/**
 * mctp_spi_error_inject_module_exit - Cleanup module-wide debugfs
 */
void mctp_spi_error_inject_module_exit(void)
{
	if (mctp_spi_error_inject_root) {
		debugfs_remove_recursive(mctp_spi_error_inject_root);
		mctp_spi_error_inject_root = NULL;
	}
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCTP SPI Error Injection");
