// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP Socket Error Injection
 * 
 * Binding-agnostic error injection at AF_MCTP socket layer.
 * Allows testing application handling of sendto() API failures:
 * - EBUSY: Tag allocation failure (all tags in use)
 * - EHOSTUNREACH: No route to destination
 * - ENOBUFS/ENOMEM: Memory exhaustion
 * - EAGAIN: Would block
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include <linux/math64.h>
#include "mctp-socket-error-inject.h"

/* Global error injection state */
struct mctp_socket_error_inject mctp_socket_ei;

static struct dentry *mctp_socket_debugfs_root;

/* Main injection function called from sendmsg() */
int mctp_socket_error_inject_sendmsg(void)
{
	int ret = 0;
	
	spin_lock_bh(&mctp_socket_ei.lock);
	mctp_socket_ei.sendmsg_calls++;
	
	if (mctp_socket_ei.enabled) {
		mctp_socket_ei.errors_injected++;
		ret = mctp_socket_ei.error_code;
		
		pr_info("MCTP Socket Error Injection: sendmsg() returning %d, "
		        "total_injected=%llu\n",
		        ret, mctp_socket_ei.errors_injected);
	}
	
	spin_unlock_bh(&mctp_socket_ei.lock);
	
	return ret;
}
EXPORT_SYMBOL_GPL(mctp_socket_error_inject_sendmsg);

/* Debugfs: enable */
static ssize_t enable_read(struct file *file, char __user *userbuf,
                           size_t count, loff_t *ppos)
{
	char buf[8];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_socket_ei.enabled ? 1 : 0);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t enable_write(struct file *file, const char __user *userbuf,
                            size_t count, loff_t *ppos)
{
	char buf[8];
	size_t len = min(count, sizeof(buf) - 1);
	int value;
	
	if (copy_from_user(buf, userbuf, len))
		return -EFAULT;
	buf[len] = '\0';
	
	if (kstrtoint(buf, 0, &value))
		return -EINVAL;
	
	mctp_socket_ei.enabled = (value != 0);
	
	pr_info("MCTP Socket Error Injection: %s\n", 
	        mctp_socket_ei.enabled ? "ENABLED" : "DISABLED");
	return count;
}

static const struct file_operations enable_fops = {
	.read = enable_read,
	.write = enable_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Debugfs: stats */
static ssize_t stats_read(struct file *file, char __user *userbuf,
                          size_t count, loff_t *ppos)
{
	char buf[256];
	int len;
	
	spin_lock_bh(&mctp_socket_ei.lock);
	len = snprintf(buf, sizeof(buf),
	               "MCTP Socket Error Injection Statistics:\n"
	               "  Enabled: %s\n"
	               "  Error Code: %d\n"
	               "  Total sendmsg() calls: %llu\n"
	               "  Errors injected: %llu\n",
	               mctp_socket_ei.enabled ? "yes" : "no",
	               mctp_socket_ei.error_code,
	               mctp_socket_ei.sendmsg_calls,
	               mctp_socket_ei.errors_injected);
	spin_unlock_bh(&mctp_socket_ei.lock);
	
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations stats_fops = {
	.read = stats_read,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Debugfs: error_code */
static ssize_t error_code_read(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
	char buf[32];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", mctp_socket_ei.error_code);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t error_code_write(struct file *file, const char __user *userbuf,
                                 size_t count, loff_t *ppos)
{
	char buf[32];
	size_t len = min(count, sizeof(buf) - 1);
	int value;
	
	if (copy_from_user(buf, userbuf, len))
		return -EFAULT;
	buf[len] = '\0';
	
	if (kstrtoint(buf, 0, &value))
		return -EINVAL;
	
	mctp_socket_ei.error_code = value;
	return count;
}

static const struct file_operations error_code_fops = {
	.read = error_code_read,
	.write = error_code_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Debugfs: reset */
static ssize_t reset_write(struct file *file, const char __user *userbuf,
                           size_t count, loff_t *ppos)
{
	char buf[8];
	size_t len = min(count, sizeof(buf) - 1);
	int value;
	
	if (copy_from_user(buf, userbuf, len))
		return -EFAULT;
	buf[len] = '\0';
	
	if (kstrtoint(buf, 0, &value))
		return -EINVAL;
	
	if (value != 1)
		return -EINVAL;
	
	/* Reset all state */
	spin_lock_bh(&mctp_socket_ei.lock);
	mctp_socket_ei.enabled = false;
	mctp_socket_ei.error_code = 0;
	mctp_socket_ei.sendmsg_calls = 0;
	mctp_socket_ei.errors_injected = 0;
	spin_unlock_bh(&mctp_socket_ei.lock);
	
	pr_info("MCTP Socket Error Injection: Reset (disabled and cleared)\n");
	return count;
}

static const struct file_operations reset_fops = {
	.write = reset_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* Initialization */
int mctp_socket_error_inject_init(void)
{
	memset(&mctp_socket_ei, 0, sizeof(mctp_socket_ei));
	spin_lock_init(&mctp_socket_ei.lock);
	
	/* Default settings */
	mctp_socket_ei.enabled = false;
	mctp_socket_ei.error_code = -EBUSY;  /* Default: simulate tag exhaustion */
	
	/* Create debugfs directory */
	mctp_socket_debugfs_root = debugfs_create_dir("mctp_socket", NULL);
	if (IS_ERR_OR_NULL(mctp_socket_debugfs_root)) {
		pr_warn("MCTP Socket Error Injection: Failed to create debugfs directory\n");
		return PTR_ERR(mctp_socket_debugfs_root);
	}
	
	mctp_socket_ei.debugfs_dir = mctp_socket_debugfs_root;
	
	/* Create debugfs files */
	debugfs_create_file("enable", 0644, mctp_socket_debugfs_root, NULL, &enable_fops);
	debugfs_create_file("error_code", 0644, mctp_socket_debugfs_root, NULL, &error_code_fops);
	debugfs_create_file("stats", 0444, mctp_socket_debugfs_root, NULL, &stats_fops);
	debugfs_create_file("reset", 0200, mctp_socket_debugfs_root, NULL, &reset_fops);
	
	pr_info("MCTP Socket Error Injection: Initialized at /sys/kernel/debug/mctp_socket/\n");
	return 0;
}

void mctp_socket_error_inject_cleanup(void)
{
	debugfs_remove_recursive(mctp_socket_debugfs_root);
	pr_info("MCTP Socket Error Injection: Cleaned up\n");
}

