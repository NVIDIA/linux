// SPDX-License-Identifier: GPL-2.0
/*
 * MCTP-over-PCIe-VDM diagnostic error injection
 *
 * This is a downstream diagnostic interface used to exercise MCTP error
 * handling without changing the PCIe-VDM queueing or completion model.
 *
 * Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <net/mctp.h>

#include "mctp-pcie-vdm-error-inject.h"

enum mctp_pcie_vdm_inject_mode {
	MCTP_PCIE_VDM_INJECT_ALWAYS,
	MCTP_PCIE_VDM_INJECT_RANDOM,
	MCTP_PCIE_VDM_INJECT_COUNT,
};

enum mctp_pcie_vdm_inject_attr_id {
	MCTP_PCIE_VDM_ATTR_ENABLE_TX,
	MCTP_PCIE_VDM_ATTR_ENABLE_RX,
	MCTP_PCIE_VDM_ATTR_MODE,
	MCTP_PCIE_VDM_ATTR_TX_ERROR_CODE,
	MCTP_PCIE_VDM_ATTR_TX_ERROR_RATE,
	MCTP_PCIE_VDM_ATTR_TX_INJECT_COUNT,
	MCTP_PCIE_VDM_ATTR_ENABLE_FRAGMENT_DROP,
	MCTP_PCIE_VDM_ATTR_ENABLE_SEQ_CORRUPT,
	MCTP_PCIE_VDM_ATTR_ENABLE_SOM_CLEAR,
	MCTP_PCIE_VDM_ATTR_FILTER_ENABLE,
	MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID,
	MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID,
	MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE,
	MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID_ENABLE,
	MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID_ENABLE,
	MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE_ENABLE,
	MCTP_PCIE_VDM_ATTR_STATS,
	MCTP_PCIE_VDM_ATTR_RESET,
	MCTP_PCIE_VDM_ATTR_COUNT,
};

struct mctp_pcie_vdm_error_inject;

struct mctp_pcie_vdm_inject_attr {
	struct mctp_pcie_vdm_error_inject *inject;
	enum mctp_pcie_vdm_inject_attr_id id;
};

struct mctp_pcie_vdm_error_inject {
	struct kref ref;
	struct dentry *debugfs_dir;
	bool dead;

	bool enable_tx;
	bool enable_rx;
	atomic_t mode;
	s32 tx_error_code;
	u32 tx_error_rate;
	atomic_t tx_inject_count;

	bool enable_fragment_drop;
	bool enable_seq_corrupt;
	bool enable_som_clear;

	bool filter_enable;
	u8 filter_src_eid;
	u8 filter_dest_eid;
	u8 filter_msg_type;
	bool filter_src_eid_enable;
	bool filter_dest_eid_enable;
	bool filter_msg_type_enable;

	atomic64_t tx_errors_injected;
	atomic64_t fragments_dropped;
	atomic64_t seq_corruptions;
	atomic64_t som_clears;
	atomic64_t total_packets_processed;
	atomic64_t total_errors_injected;

	struct mctp_pcie_vdm_inject_attr
		attrs[MCTP_PCIE_VDM_ATTR_COUNT];
};

static DEFINE_MUTEX(mctp_pcie_vdm_inject_root_lock);
static struct dentry *mctp_pcie_vdm_inject_root;

static const char *
mctp_pcie_vdm_inject_mode_name(enum mctp_pcie_vdm_inject_mode mode)
{
	switch (mode) {
	case MCTP_PCIE_VDM_INJECT_ALWAYS:
		return "always";
	case MCTP_PCIE_VDM_INJECT_RANDOM:
		return "random";
	case MCTP_PCIE_VDM_INJECT_COUNT:
		return "count";
	default:
		return "unknown";
	}
}

static void
mctp_pcie_vdm_error_inject_release(struct kref *ref)
{
	struct mctp_pcie_vdm_error_inject *inject =
		container_of(ref, struct mctp_pcie_vdm_error_inject, ref);

	kfree(inject);
}

static int mctp_pcie_vdm_inject_attr_open(struct inode *inode,
					  struct file *file)
{
	struct mctp_pcie_vdm_inject_attr *attr = inode->i_private;
	struct mctp_pcie_vdm_error_inject *inject = attr->inject;

	if (!kref_get_unless_zero(&inject->ref))
		return -ENODEV;
	if (READ_ONCE(inject->dead)) {
		kref_put(&inject->ref, mctp_pcie_vdm_error_inject_release);
		return -ENODEV;
	}

	file->private_data = attr;
	return 0;
}

static int mctp_pcie_vdm_inject_attr_release(struct inode *unused,
					     struct file *file)
{
	struct mctp_pcie_vdm_inject_attr *attr = file->private_data;

	kref_put(&attr->inject->ref, mctp_pcie_vdm_error_inject_release);
	return 0;
}

static int
mctp_pcie_vdm_inject_stats_format(struct mctp_pcie_vdm_error_inject *inject,
				  char *buf, size_t size)
{
	enum mctp_pcie_vdm_inject_mode mode = atomic_read(&inject->mode);
	int len = 0;

	len += scnprintf(buf + len, size - len, "enable_tx: %u\n",
			 READ_ONCE(inject->enable_tx));
	len += scnprintf(buf + len, size - len, "enable_rx: %u\n",
			 READ_ONCE(inject->enable_rx));
	len += scnprintf(buf + len, size - len, "mode: %s\n",
			 mctp_pcie_vdm_inject_mode_name(mode));
	len += scnprintf(buf + len, size - len, "tx_inject_count: %d\n",
			 atomic_read(&inject->tx_inject_count));
	len += scnprintf(buf + len, size - len,
			 "tx_errors_injected: %lld\n",
			 atomic64_read(&inject->tx_errors_injected));
	len += scnprintf(buf + len, size - len, "fragments_dropped: %lld\n",
			 atomic64_read(&inject->fragments_dropped));
	len += scnprintf(buf + len, size - len, "seq_corruptions: %lld\n",
			 atomic64_read(&inject->seq_corruptions));
	len += scnprintf(buf + len, size - len, "som_clears: %lld\n",
			 atomic64_read(&inject->som_clears));
	len += scnprintf(buf + len, size - len,
			 "total_packets_processed: %lld\n",
			 atomic64_read(&inject->total_packets_processed));
	len += scnprintf(buf + len, size - len,
			 "total_errors_injected: %lld\n",
			 atomic64_read(&inject->total_errors_injected));

	return len;
}

static ssize_t mctp_pcie_vdm_inject_attr_read(struct file *file,
					      char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct mctp_pcie_vdm_inject_attr *attr = file->private_data;
	struct mctp_pcie_vdm_error_inject *inject = attr->inject;
	char buf[512];
	int len;

	if (READ_ONCE(inject->dead))
		return -ENODEV;

	switch (attr->id) {
	case MCTP_PCIE_VDM_ATTR_ENABLE_TX:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->enable_tx));
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_RX:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->enable_rx));
		break;
	case MCTP_PCIE_VDM_ATTR_MODE: {
		enum mctp_pcie_vdm_inject_mode mode;

		mode = atomic_read(&inject->mode);
		len = scnprintf(buf, sizeof(buf), "%s\n",
				mctp_pcie_vdm_inject_mode_name(mode));
		break;
	}
	case MCTP_PCIE_VDM_ATTR_TX_ERROR_CODE:
		len = scnprintf(buf, sizeof(buf), "%d\n",
				READ_ONCE(inject->tx_error_code));
		break;
	case MCTP_PCIE_VDM_ATTR_TX_ERROR_RATE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->tx_error_rate));
		break;
	case MCTP_PCIE_VDM_ATTR_TX_INJECT_COUNT:
		len = scnprintf(buf, sizeof(buf), "%d\n",
				atomic_read(&inject->tx_inject_count));
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_FRAGMENT_DROP:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->enable_fragment_drop));
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_SEQ_CORRUPT:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->enable_seq_corrupt));
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_SOM_CLEAR:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->enable_som_clear));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_ENABLE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_enable));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_src_eid));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_dest_eid));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_msg_type));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID_ENABLE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_src_eid_enable));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID_ENABLE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_dest_eid_enable));
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE_ENABLE:
		len = scnprintf(buf, sizeof(buf), "%u\n",
				READ_ONCE(inject->filter_msg_type_enable));
		break;
	case MCTP_PCIE_VDM_ATTR_STATS:
		len = mctp_pcie_vdm_inject_stats_format(inject, buf,
							sizeof(buf));
		break;
	default:
		return -EOPNOTSUPP;
	}

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static void
mctp_pcie_vdm_error_inject_reset(struct mctp_pcie_vdm_error_inject *inject)
{
	WRITE_ONCE(inject->enable_tx, false);
	WRITE_ONCE(inject->enable_rx, false);
	atomic_set(&inject->mode, MCTP_PCIE_VDM_INJECT_ALWAYS);
	WRITE_ONCE(inject->tx_error_code, 0);
	WRITE_ONCE(inject->tx_error_rate, 0);
	atomic_set(&inject->tx_inject_count, 0);

	WRITE_ONCE(inject->enable_fragment_drop, false);
	WRITE_ONCE(inject->enable_seq_corrupt, false);
	WRITE_ONCE(inject->enable_som_clear, false);

	WRITE_ONCE(inject->filter_enable, false);
	WRITE_ONCE(inject->filter_src_eid, 0);
	WRITE_ONCE(inject->filter_dest_eid, 0);
	WRITE_ONCE(inject->filter_msg_type, 0);
	WRITE_ONCE(inject->filter_src_eid_enable, false);
	WRITE_ONCE(inject->filter_dest_eid_enable, false);
	WRITE_ONCE(inject->filter_msg_type_enable, false);

	atomic64_set(&inject->tx_errors_injected, 0);
	atomic64_set(&inject->fragments_dropped, 0);
	atomic64_set(&inject->seq_corruptions, 0);
	atomic64_set(&inject->som_clears, 0);
	atomic64_set(&inject->total_packets_processed, 0);
	atomic64_set(&inject->total_errors_injected, 0);
}

static ssize_t mctp_pcie_vdm_inject_attr_write(struct file *file,
					       const char __user *userbuf,
					       size_t count, loff_t *unused)
{
	struct mctp_pcie_vdm_inject_attr *attr = file->private_data;
	struct mctp_pcie_vdm_error_inject *inject = attr->inject;
	char buf[32];
	unsigned int uval;
	bool bval;
	s32 sval;
	int ret;

	if (READ_ONCE(inject->dead))
		return -ENODEV;
	if (!count || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = '\0';

	switch (attr->id) {
	case MCTP_PCIE_VDM_ATTR_ENABLE_TX:
	case MCTP_PCIE_VDM_ATTR_ENABLE_RX:
	case MCTP_PCIE_VDM_ATTR_ENABLE_FRAGMENT_DROP:
	case MCTP_PCIE_VDM_ATTR_ENABLE_SEQ_CORRUPT:
	case MCTP_PCIE_VDM_ATTR_ENABLE_SOM_CLEAR:
	case MCTP_PCIE_VDM_ATTR_FILTER_ENABLE:
	case MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID_ENABLE:
	case MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID_ENABLE:
	case MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE_ENABLE:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	switch (attr->id) {
	case MCTP_PCIE_VDM_ATTR_ENABLE_TX:
		WRITE_ONCE(inject->enable_tx, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_RX:
		WRITE_ONCE(inject->enable_rx, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_MODE:
		if (sysfs_streq(buf, "always"))
			atomic_set(&inject->mode,
				   MCTP_PCIE_VDM_INJECT_ALWAYS);
		else if (sysfs_streq(buf, "random"))
			atomic_set(&inject->mode,
				   MCTP_PCIE_VDM_INJECT_RANDOM);
		else if (sysfs_streq(buf, "count"))
			atomic_set(&inject->mode,
				   MCTP_PCIE_VDM_INJECT_COUNT);
		else
			return -EINVAL;
		break;
	case MCTP_PCIE_VDM_ATTR_TX_ERROR_CODE:
		ret = kstrtos32(buf, 0, &sval);
		if (ret)
			return ret;
		WRITE_ONCE(inject->tx_error_code, sval);
		break;
	case MCTP_PCIE_VDM_ATTR_TX_ERROR_RATE:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > 100)
			return -ERANGE;
		WRITE_ONCE(inject->tx_error_rate, uval);
		break;
	case MCTP_PCIE_VDM_ATTR_TX_INJECT_COUNT:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > INT_MAX)
			return -ERANGE;
		atomic_set(&inject->tx_inject_count, uval);
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_FRAGMENT_DROP:
		WRITE_ONCE(inject->enable_fragment_drop, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_SEQ_CORRUPT:
		WRITE_ONCE(inject->enable_seq_corrupt, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_ENABLE_SOM_CLEAR:
		WRITE_ONCE(inject->enable_som_clear, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_ENABLE:
		WRITE_ONCE(inject->filter_enable, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > U8_MAX)
			return -ERANGE;
		WRITE_ONCE(inject->filter_src_eid, uval);
		/*
		 * Preserve the legacy ABI where zero means "any EID".
		 * src_eid_enable can still be set explicitly to match EID 0.
		 */
		WRITE_ONCE(inject->filter_src_eid_enable, uval != 0);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > U8_MAX)
			return -ERANGE;
		WRITE_ONCE(inject->filter_dest_eid, uval);
		WRITE_ONCE(inject->filter_dest_eid_enable, uval != 0);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > 0x7f)
			return -ERANGE;
		WRITE_ONCE(inject->filter_msg_type, uval);
		WRITE_ONCE(inject->filter_msg_type_enable, true);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID_ENABLE:
		WRITE_ONCE(inject->filter_src_eid_enable, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID_ENABLE:
		WRITE_ONCE(inject->filter_dest_eid_enable, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE_ENABLE:
		WRITE_ONCE(inject->filter_msg_type_enable, bval);
		break;
	case MCTP_PCIE_VDM_ATTR_RESET:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval != 1)
			return -EINVAL;
		mctp_pcie_vdm_error_inject_reset(inject);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return count;
}

static const struct file_operations mctp_pcie_vdm_inject_attr_fops = {
	.owner = THIS_MODULE,
	.open = mctp_pcie_vdm_inject_attr_open,
	.release = mctp_pcie_vdm_inject_attr_release,
	.read = mctp_pcie_vdm_inject_attr_read,
	.write = mctp_pcie_vdm_inject_attr_write,
	.llseek = default_llseek,
};

static void mctp_vdm_inject_file(struct mctp_pcie_vdm_error_inject *inject,
				 struct dentry *parent, const char *name,
				 umode_t mode,
				 enum mctp_pcie_vdm_inject_attr_id id)
{
	struct mctp_pcie_vdm_inject_attr *attr = &inject->attrs[id];

	attr->inject = inject;
	attr->id = id;
	debugfs_create_file(name, mode, parent, attr,
			    &mctp_pcie_vdm_inject_attr_fops);
}

static struct dentry *mctp_pcie_vdm_inject_get_root(void)
{
	struct dentry *root;

	mutex_lock(&mctp_pcie_vdm_inject_root_lock);
	root = mctp_pcie_vdm_inject_root;
	if (!root) {
		root = debugfs_create_dir("aspeed_mctp", NULL);
		if (IS_ERR_OR_NULL(root))
			root = NULL;
		mctp_pcie_vdm_inject_root = root;
	}
	mutex_unlock(&mctp_pcie_vdm_inject_root_lock);

	return root;
}

struct mctp_pcie_vdm_error_inject *
mctp_pcie_vdm_error_inject_init(const char *ifname)
{
	struct mctp_pcie_vdm_error_inject *inject;
	struct dentry *filter_dir;
	struct dentry *root;

	root = mctp_pcie_vdm_inject_get_root();
	if (!root)
		return NULL;

	inject = kzalloc(sizeof(*inject), GFP_KERNEL);
	if (!inject)
		return NULL;

	kref_init(&inject->ref);
	atomic_set(&inject->mode, MCTP_PCIE_VDM_INJECT_ALWAYS);
	atomic_set(&inject->tx_inject_count, 0);

	inject->debugfs_dir = debugfs_create_dir(ifname, root);
	if (IS_ERR_OR_NULL(inject->debugfs_dir)) {
		inject->debugfs_dir = NULL;
		kref_put(&inject->ref, mctp_pcie_vdm_error_inject_release);
		return NULL;
	}

	mctp_vdm_inject_file(inject, inject->debugfs_dir, "enable_tx", 0600,
			     MCTP_PCIE_VDM_ATTR_ENABLE_TX);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "enable_rx", 0600,
			     MCTP_PCIE_VDM_ATTR_ENABLE_RX);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "mode", 0600,
			     MCTP_PCIE_VDM_ATTR_MODE);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "tx_error_code",
			     0600, MCTP_PCIE_VDM_ATTR_TX_ERROR_CODE);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "tx_error_rate",
			     0600, MCTP_PCIE_VDM_ATTR_TX_ERROR_RATE);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "tx_inject_count",
			     0600, MCTP_PCIE_VDM_ATTR_TX_INJECT_COUNT);
	mctp_vdm_inject_file(inject, inject->debugfs_dir,
			     "enable_fragment_drop", 0600,
			     MCTP_PCIE_VDM_ATTR_ENABLE_FRAGMENT_DROP);
	mctp_vdm_inject_file(inject, inject->debugfs_dir,
			     "enable_seq_corrupt", 0600,
			     MCTP_PCIE_VDM_ATTR_ENABLE_SEQ_CORRUPT);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "enable_som_clear",
			     0600, MCTP_PCIE_VDM_ATTR_ENABLE_SOM_CLEAR);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "stats", 0400,
			     MCTP_PCIE_VDM_ATTR_STATS);
	mctp_vdm_inject_file(inject, inject->debugfs_dir, "reset", 0200,
			     MCTP_PCIE_VDM_ATTR_RESET);

	filter_dir = debugfs_create_dir("eid_filter", inject->debugfs_dir);
	if (!IS_ERR_OR_NULL(filter_dir)) {
		mctp_vdm_inject_file(inject, filter_dir, "enable", 0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_ENABLE);
		mctp_vdm_inject_file(inject, filter_dir, "src_eid", 0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID);
		mctp_vdm_inject_file(inject, filter_dir, "dest_eid", 0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID);
		mctp_vdm_inject_file(inject, filter_dir, "msg_type", 0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE);
		mctp_vdm_inject_file(inject, filter_dir, "src_eid_enable",
				     0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_SRC_EID_ENABLE);
		mctp_vdm_inject_file(inject, filter_dir, "dest_eid_enable",
				     0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_DEST_EID_ENABLE);
		mctp_vdm_inject_file(inject, filter_dir, "msg_type_enable",
				     0600,
				     MCTP_PCIE_VDM_ATTR_FILTER_MSG_TYPE_ENABLE);
	}

	return inject;
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_error_inject_init);

void mctp_pcie_vdm_error_inject_cleanup(struct mctp_pcie_vdm_error_inject *inject)
{
	if (!inject)
		return;

	WRITE_ONCE(inject->dead, true);
	debugfs_remove_recursive(inject->debugfs_dir);
	inject->debugfs_dir = NULL;
	kref_put(&inject->ref, mctp_pcie_vdm_error_inject_release);
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_error_inject_cleanup);

static bool
mctp_pcie_vdm_error_inject_match(struct mctp_pcie_vdm_error_inject *inject,
				 struct sk_buff *skb, int offset)
{
	struct mctp_hdr mh;
	u8 msg_type;

	if (!READ_ONCE(inject->filter_enable))
		return true;

	if (offset < 0 || offset > skb->len ||
	    skb_copy_bits(skb, offset, &mh, sizeof(mh)))
		return false;

	if (READ_ONCE(inject->filter_src_eid_enable) &&
	    READ_ONCE(inject->filter_src_eid) != mh.src)
		return false;
	if (READ_ONCE(inject->filter_dest_eid_enable) &&
	    READ_ONCE(inject->filter_dest_eid) != mh.dest)
		return false;
	if (!READ_ONCE(inject->filter_msg_type_enable))
		return true;

	/*
	 * The message-type byte exists only in the start-of-message packet.
	 * Do not mistake continuation payload data for a message type.
	 */
	if (!(mh.flags_seq_tag & MCTP_HDR_FLAG_SOM))
		return false;

	if (skb_copy_bits(skb, offset + sizeof(mh), &msg_type,
			  sizeof(msg_type)))
		return false;

	return READ_ONCE(inject->filter_msg_type) == (msg_type & 0x7f);
}

static bool
mctp_vdm_inject_now(struct mctp_pcie_vdm_error_inject *inject)
{
	switch (atomic_read(&inject->mode)) {
	case MCTP_PCIE_VDM_INJECT_ALWAYS:
		return true;
	case MCTP_PCIE_VDM_INJECT_RANDOM:
		return get_random_u32_below(100) <
		       READ_ONCE(inject->tx_error_rate);
	case MCTP_PCIE_VDM_INJECT_COUNT:
		return atomic_dec_if_positive(&inject->tx_inject_count) >= 0;
	default:
		return false;
	}
}

bool mctp_pcie_vdm_error_inject_tx(struct mctp_pcie_vdm_error_inject *inject,
				   struct sk_buff *skb)
{
	if (!inject || READ_ONCE(inject->dead) ||
	    !READ_ONCE(inject->enable_tx) ||
	    !READ_ONCE(inject->tx_error_code))
		return false;
	if (!mctp_pcie_vdm_error_inject_match(inject, skb,
					      skb_network_offset(skb)))
		return false;

	if (!mctp_vdm_inject_now(inject))
		return false;

	atomic64_inc(&inject->total_packets_processed);
	atomic64_inc(&inject->tx_errors_injected);
	atomic64_inc(&inject->total_errors_injected);
	return true;
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_error_inject_tx);

bool mctp_pcie_vdm_error_inject_rx(struct mctp_pcie_vdm_error_inject *inject,
				   struct sk_buff *skb)
{
	struct mctp_hdr *mh;
	u8 flags;
	u8 seq;

	if (!inject || READ_ONCE(inject->dead) ||
	    !READ_ONCE(inject->enable_rx) ||
	    (!READ_ONCE(inject->enable_fragment_drop) &&
	     !READ_ONCE(inject->enable_seq_corrupt) &&
	     !READ_ONCE(inject->enable_som_clear)))
		return false;
	if (!mctp_pcie_vdm_error_inject_match(inject, skb, 0))
		return false;

	if (skb_ensure_writable(skb, sizeof(*mh)))
		return false;

	mh = (struct mctp_hdr *)skb->data;
	flags = mh->flags_seq_tag &
		(MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM);
	if (flags == (MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM))
		return false;

	seq = (mh->flags_seq_tag >> MCTP_HDR_SEQ_SHIFT) &
	      MCTP_HDR_SEQ_MASK;
	if (READ_ONCE(inject->enable_som_clear) &&
	    (flags & MCTP_HDR_FLAG_SOM)) {
		mh->flags_seq_tag &= ~MCTP_HDR_FLAG_SOM;
		atomic64_inc(&inject->som_clears);
		atomic64_inc(&inject->total_errors_injected);
		return false;
	}

	if (READ_ONCE(inject->enable_seq_corrupt) &&
	    !(flags & MCTP_HDR_FLAG_SOM)) {
		seq = (seq + 1) & MCTP_HDR_SEQ_MASK;
		mh->flags_seq_tag &=
			~(MCTP_HDR_SEQ_MASK << MCTP_HDR_SEQ_SHIFT);
		mh->flags_seq_tag |= seq << MCTP_HDR_SEQ_SHIFT;
		atomic64_inc(&inject->seq_corruptions);
		atomic64_inc(&inject->total_errors_injected);
		return false;
	}

	if (READ_ONCE(inject->enable_fragment_drop) &&
	    !(flags & MCTP_HDR_FLAG_SOM)) {
		atomic64_inc(&inject->fragments_dropped);
		atomic64_inc(&inject->total_errors_injected);
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_error_inject_rx);

static int __init mctp_pcie_vdm_error_inject_module_init(void)
{
	if (!mctp_pcie_vdm_inject_get_root())
		pr_warn("MCTP PCIe VDM error injection debugfs unavailable\n");

	/* Diagnostics must never prevent the transport from probing. */
	return 0;
}

static void __exit mctp_pcie_vdm_error_inject_module_exit(void)
{
	mutex_lock(&mctp_pcie_vdm_inject_root_lock);
	debugfs_remove_recursive(mctp_pcie_vdm_inject_root);
	mctp_pcie_vdm_inject_root = NULL;
	mutex_unlock(&mctp_pcie_vdm_inject_root_lock);
}

module_init(mctp_pcie_vdm_error_inject_module_init);
module_exit(mctp_pcie_vdm_error_inject_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCTP PCIe VDM diagnostic error injection");
