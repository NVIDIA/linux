// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP SPI Error Injection Implementation
 * 
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over SPI binding.
 * 
 * Interface unified with I2C/USB error injection where applicable.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/mctp.h>

#include "mctp-spi-error-inject.h"

/* Forward declare SpbApStatus enum values - avoid including glacier-spb-ap.h 
 * to prevent multiple definition linker errors */
#define SPB_AP_OK 0
#define SPB_AP_MESSAGE_AVAILABLE 1
#define SPB_AP_ERROR_INVALID_ARGUMENT 2
#define SPB_AP_ERROR_TIMEOUT 3
#define SPB_AP_ERROR_UNKNOWN 4

/* Forward declarations to avoid including glacier-spb-ap.h */
typedef struct SpbAp SpbAp;
struct spidev_data;
struct gpio_desc;

/* Constants from mctp-spi.c */
#define BUFSIZE 256

/* CRITICAL: This struct definition MUST match the one in mctp-spi.c exactly!
 * If you modify struct mctp_spi in mctp-spi.c, you MUST update this copy.
 */
struct mctp_spi {
	struct net_device	*ndev;
	struct spidev_data *spidev;

	struct task_struct *tx_thread;
	wait_queue_head_t main_thread_wq;
	struct sk_buff_head tx_queue;
	spinlock_t lock;
	bool allow_rx;
	struct completion rx_done;

	struct gpio_desc *rx_alert; //Input gpio to alert about the incoming package from SPI
	int	rx_alert_irq;

	SpbAp *ap;
	wait_queue_head_t gpio_intr_wq;
	bool gpio_intr_cond;
	spinlock_t gpio_intr_cond_lock;

	/* Per-EID statistics tracking - SINGLE source of truth
	 *
	 * All statistics are tracked per-endpoint-ID (EID). Two special EIDs:
	 * - EID 0: "null endpoint" - valid packets with EID=0 (unallocated endpoint)
	 * - EID 256 (MCTP_EID_UNKNOWN): errors where EID could not be determined
	 *   (GPIO interrupts, SPI transfer errors, allocation failures)
	 */
	struct {
		DECLARE_BITMAP(active, 257);  /* Which EIDs have activity */
		struct mctp_spi_eid_stats {
			/* RX stats */
			u64 rx_drop_no_memory;
			u64 rx_drop_not_ready;       /* Tracked as UNKNOWN (before EID known) */
			u64 rx_drop_spi_error;       /* Tracked as UNKNOWN */
			
			/* TX stats */
			u64 tx_drop_spi_error;
			u64 tx_drop_ebusy;
			u64 tx_drop_etimedout;
			u64 tx_drop_eio;
			u64 tx_drop_einval;
			u64 tx_drop_enomem;
			u64 tx_drop_emsgsize;
			
			/* GPIO interrupt tracking (UNKNOWN - no EID context) */
			u64 gpio_interrupts;
		} eid[257];
	} eid_stats;
	
	/* Error injection support */
	struct mctp_spi_error_inject error_inject;
	struct dentry *debugfs_dir;
};
/* Global debugfs root for all MCTP SPI error injection */
static struct dentry *mctp_spi_error_inject_root;

/* Check if error should be injected based on mode and rate */
static bool mctp_should_inject_error(struct mctp_spi_error_inject *ei, u32 rate, u32 *count)
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
static bool mctp_error_inject_match_filter(struct mctp_spi_error_inject *ei,
					    struct sk_buff *skb)
{
	struct mctp_hdr *mh;
	struct mctp_spi_hdr {
		u8 command_code;
		u8 byte_count;
		u8 resrv[2];
	} *spi_hdr;
	
	if (!ei->eid_filter.enabled)
		return true;
	
	/* Skip SPI header to get to MCTP header */
	spi_hdr = (void *)skb->data;
	mh = (struct mctp_hdr *)(skb->data + sizeof(*spi_hdr));
	
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
	struct mctp_hdr *mh;
	struct mctp_spi_hdr {
		u8 command_code;
		u8 byte_count;
		u8 resrv[2];
	} *spi_hdr;
	
	/* Early exit if TX error injection is disabled or code not set */
	if (!ei->enable_tx || ei->spi_tx_error_code == 0)
		return 0;
	
	if (!mctp_error_inject_match_filter(ei, skb))
		return 0;
	
	if (!mctp_should_inject_error(ei, ei->spi_tx_error_rate,
				      &ei->spi_tx_inject_count))
		return 0;
	
	/* Inject delay if configured */
	if (ei->delay_ms > 0)
		msleep(ei->delay_ms);
	
	/* Update statistics */
	spin_lock_bh(&ei->lock);
	ei->spi_tx_errors_injected++;
	ei->total_errors_injected++;
	ei->total_packets_processed++;
	spin_unlock_bh(&ei->lock);
	
	spi_hdr = (void *)skb->data;
	mh = (struct mctp_hdr *)(skb->data + sizeof(*spi_hdr));
	
	netdev_info(midev->ndev,
		    "ERROR INJECTION: SPI TX - error=%d, src_eid=%u, dest_eid=%u, "
		    "total_injected=%u, mode=%s\n",
		    ei->spi_tx_error_code, mh->src, mh->dest,
		    ei->spi_tx_errors_injected,
		    ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
		    ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	
	*injected_status = ei->spi_tx_error_code;
	return 1;
}
EXPORT_SYMBOL_GPL(mctp_spi_error_inject_tx);

/* Debugfs attribute files */
static ssize_t enable_tx_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[4];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%d\n", midev->error_inject.enable_tx);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t enable_tx_write(struct file *file, const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[4];
	bool val;
	int ret;
	
	if (count >= sizeof(buf))
		return -EINVAL;
	
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	
	buf[count] = '\0';
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.enable_tx = val;
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}

static const struct file_operations mctp_debugfs_enable_tx_fops = {
	.owner = THIS_MODULE,
	.read = enable_tx_read,
	.write = enable_tx_write,
	.open = simple_open,
	.llseek = default_llseek,
};

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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.spi_tx_error_code = val;
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}

static const struct file_operations mctp_debugfs_spi_tx_error_code_fops = {
	.owner = THIS_MODULE,
	.read = error_code_read,
	.write = error_code_write,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t mode_read(struct file *file, char __user *user_buf,
			  size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	const char *mode_str;
	char buf[32];
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
	}
	
	len = snprintf(buf, sizeof(buf), "%s", mode_str);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t mode_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	
	if (count >= sizeof(buf))
		return -EINVAL;
	
	if (copy_from_user(buf, user_buf, count))
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
	.read = mode_read,
	.write = mode_write,
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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.spi_tx_error_rate = val;
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}

static const struct file_operations mctp_debugfs_spi_tx_error_rate_fops = {
	.owner = THIS_MODULE,
	.read = rate_read,
	.write = rate_write,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t inject_count_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	char buf[16];
	int len;
	
	len = snprintf(buf, sizeof(buf), "%u\n", midev->error_inject.spi_tx_inject_count);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t inject_count_write(struct file *file, const char __user *user_buf,
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
	
	spin_lock_bh(&midev->error_inject.lock);
	midev->error_inject.spi_tx_inject_count = val;
	spin_unlock_bh(&midev->error_inject.lock);
	
	return count;
}


/* stats attribute (read-only) - unified with USB/I2C */
static ssize_t stats_read(struct file *file, char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	struct mctp_spi_error_inject *ei = &midev->error_inject;
	char *buf;
	int len = 0;
	ssize_t ret;
	
	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	
	spin_lock_bh(&ei->lock);
	
	len += snprintf(buf + len, PAGE_SIZE - len, "enable_tx: %d\n", ei->enable_tx);
	len += snprintf(buf + len, PAGE_SIZE - len, "mode: %s\n",
			ei->mode == MCTP_ERR_MODE_ALWAYS ? "always" :
			ei->mode == MCTP_ERR_MODE_RANDOM ? "random" : "count");
	len += snprintf(buf + len, PAGE_SIZE - len, "spi_tx_errors_injected: %u\n",
			ei->spi_tx_errors_injected);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_packets_processed: %llu\n",
			ei->total_packets_processed);
	len += snprintf(buf + len, PAGE_SIZE - len, "total_errors_injected: %llu\n",
			ei->total_errors_injected);
	
	spin_unlock_bh(&ei->lock);
	
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

/* reset attribute (write-only) - unified with USB/I2C */
static ssize_t reset_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct mctp_spi *midev = file->private_data;
	struct mctp_spi_error_inject *ei = &midev->error_inject;
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
	
	spin_lock_bh(&ei->lock);
	
	/* Reset all configuration - unified with USB/I2C */
	ei->enable_tx = false;
	ei->mode = MCTP_ERR_MODE_ALWAYS;
	ei->spi_tx_error_code = 0;
	ei->spi_tx_error_rate = 0;
	ei->spi_tx_inject_count = 0;
	ei->delay_ms = 0;
	ei->eid_filter.enabled = false;
	ei->eid_filter.src_eid = 0;
	ei->eid_filter.dest_eid = 0;
	ei->eid_filter.msg_type = 0;
	
	/* Reset all statistics */
	ei->spi_tx_errors_injected = 0;
	ei->total_packets_processed = 0;
	ei->total_errors_injected = 0;
	
	spin_unlock_bh(&ei->lock);
	
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
	
	/* Initialize error injection state */
	memset(ei, 0, sizeof(*ei));
	spin_lock_init(&ei->lock);
	prandom_seed_state(&ei->rng, get_random_u32());
	
	/* Default configuration */
	ei->mode = MCTP_ERR_MODE_ALWAYS;
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
	
	/* Create debugfs files - unified with USB/I2C */
	debugfs_create_file("enable_tx", 0600, midev->debugfs_dir, midev,
			    &mctp_debugfs_enable_tx_fops);
	debugfs_create_file("mode", 0600, midev->debugfs_dir, midev,
			    &mctp_debugfs_mode_fops);
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

