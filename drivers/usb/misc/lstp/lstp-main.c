// SPDX-License-Identifier: GPL-2.0-only
/*
 * LSTP USB interface driver.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/err.h>

#include "lstp-main.h"

enum lstp_ch0_cmd {
	LSTP_CH0_CMD_READ_CONFIG = 0x08,
	LSTP_CH0_CMD_WRITE_CONFIG = 0x09,
	LSTP_CH0_CMD_LOCK = 0x0A,
};

struct lstp_ch0_config {
	u8 lstp_version;
	u8 num_channels; /* does not include ch0 */
} __packed;

/*******************************************************************************
 * Forward declarations
 ******************************************************************************/

static void lstp_usb_rx_callback(struct urb *urb);
static void lstp_rx_retry_work(struct work_struct *work);
static void lstp_teardown(struct lstp_usb *dev);
static void lstp_kobj_release(struct kobject *kobj);
static void lstp_channel_kobj_release(struct kobject *kobj);
static ssize_t lstp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lstp_channel_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf);
static ssize_t lstp_channel_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
					 const char *buf, size_t count);
static ssize_t lstp_channel_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

static struct kobj_attribute lstp_name_attr = __ATTR(name, 0444, lstp_name_show, NULL);

static struct attribute *lstp_attrs[] = {
	&lstp_name_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lstp);

static const struct kobj_type lstp_ktype = {
	.release = lstp_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = lstp_groups,
};

static struct kobj_attribute lstp_enable_attr =
	__ATTR(enable, 0644, lstp_channel_enable_show, lstp_channel_enable_store);

static struct kobj_attribute lstp_channel_name_attr =
	__ATTR(name, 0444, lstp_channel_name_show, NULL);

static struct attribute *lstp_channel_attrs[] = {
	&lstp_enable_attr.attr,
	&lstp_channel_name_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lstp_channel);

static const struct kobj_type lstp_channel_ktype = {
	.release = lstp_channel_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = lstp_channel_groups,
};

/* clang-format off */
static const int lstp_errno_map[] = {
	[LSTP_SUCCESS] = 0,
	[LSTP_ERROR] = -EIO,
	[LSTP_TIMEOUT] = -ETIMEDOUT,
	[LSTP_BUSY] = -EBUSY,
	[LSTP_NACK] = -ENXIO,
	[LSTP_ARB_LOST] = -EAGAIN,
	[LSTP_NOT_SUPP] = -EOPNOTSUPP,
	[LSTP_TOO_LARGE] = -EFBIG,
}; /* clang-format on */

/**
 * lstp_status_to_errno() - Convert LSTP protocol status to Linux errno.
 * @status: LSTP status code from device response
 *
 * Return: Negative errno, or 0 on LSTP_SUCCESS
 */
int lstp_status_to_errno(u8 status)
{
	if (status < ARRAY_SIZE(lstp_errno_map))
		return lstp_errno_map[status];

	return -EIO;
}

/*******************************************************************************
 * Management channel (Channel 0)
 ******************************************************************************/

/**
 * lstp_validate_rx_pkt() - Generic validation for received LSTP packets.
 * @dev:           LSTP USB device structure
 * @rx_pkt:        Received packet to validate
 * @actual_length: Actual number of bytes received from USB
 *
 * Checks header size, channel ID range, and payload length.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_validate_rx_pkt(struct lstp_usb *dev, struct lstp_packet *rx_pkt, size_t actual_length)
{
	struct lstp_header *rx_hdr = &rx_pkt->hdr;
	size_t claimed_payload_len;
	size_t max_payload_len;
	u8 ch_id;

	/* Validate header size */
	if (actual_length < sizeof(struct lstp_header)) {
		dev_err(&dev->intf->dev, "%s: Packet without header (got %zu, need >=%lu)\n",
			__func__, actual_length, sizeof(struct lstp_header));
		return -EIO;
	}

	/* Validate channel ID */
	ch_id = rx_hdr->ch_id;
	if (ch_id > dev->max_ch_id) {
		dev_err(&dev->intf->dev, "%s: Invalid channel ID (got %u, max %u)\n", __func__,
			ch_id, dev->max_ch_id);
		return -EIO;
	}

	claimed_payload_len = le16_to_cpu(rx_hdr->length);
	max_payload_len = dev->bulk_rx_size - sizeof(struct lstp_header);

	/* Overflow protection: validate payload length against buffer capacity */
	/* TODO: remove/change these checks when we implement the multi-packet support */
	if (claimed_payload_len > max_payload_len) {
		dev_err(&dev->intf->dev, "%s: ch_%u: Payload too large (claims %zu, max %zu)\n",
			__func__, ch_id, claimed_payload_len, max_payload_len);
		return -EIO;
	}

	/* Underflow protection: validate received data matches claimed payload */
	if (sizeof(struct lstp_header) + claimed_payload_len > actual_length) {
		dev_err(&dev->intf->dev, "%s: ch_%u: Payload too short (claims %zu, got %zu)\n",
			__func__, ch_id, claimed_payload_len,
			actual_length - sizeof(struct lstp_header));
		return -EIO;
	}

	return 0;
}

/**
 * lstp_validate_resp() - Validate LSTP response packets.
 * @dev:             Device structure for error reporting
 * @rx_pkt:          Received packet to validate
 * @min_payload_len: Minimum payload length, or LSTP_ANY_RX_LEN to accept any length
 *
 * Assumes the packet has already been validated with lstp_validate_rx_pkt().
 *
 * Accepts payloads at least @min_payload_len bytes long; longer payloads are
 * permitted to allow forward compatibility with newer firmware that may
 * append additional fields.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_validate_resp(struct lstp_usb *dev, struct lstp_packet *rx_pkt, size_t min_payload_len)
{
	int ret;
	struct lstp_header *rx_hdr = &rx_pkt->hdr;
	size_t payload_len;
	u8 ch_id = rx_hdr->ch_id;

	/* Validate LSTP status */
	ret = lstp_status_to_errno(GET_BIT_0_6(rx_hdr->status));
	if (ret) {
		dev_dbg(&dev->intf->dev, "%s: ch_%u: LSTP error status %d (errno=%d)\n", __func__,
			ch_id, GET_BIT_0_6(rx_hdr->status), ret);
		return ret;
	}

	/* Validate payload length meets minimum */
	payload_len = le16_to_cpu(rx_hdr->length);
	if (payload_len < min_payload_len) {
		dev_err(&dev->intf->dev,
			"%s: ch_%u: Payload too short (need at least %zu, got %zu)\n", __func__,
			ch_id, min_payload_len, payload_len);
		return -EIO;
	}

	/*
	 * Surface oversized payloads under dynamic debug so wire-format mismatches
	 * are visible during investigation without spamming the kernel log on the
	 * normal forward-compat path. Suppressed when no minimum was requested.
	 */
	if (min_payload_len && payload_len > min_payload_len)
		dev_dbg(&dev->intf->dev,
			"%s: ch_%u: Payload longer than minimum (got %zu, min %zu) -- forward-compat extension?\n",
			__func__, ch_id, payload_len, min_payload_len);

	return 0;
}

/**
 * lstp_ch0_read_helper() - Low-level READ_CONFIG via ch0 (blocking).
 * @dev:    LSTP USB device
 * @ch_id:  Channel ID to read
 * @offset: Offset into config data
 * @length: Bytes to read (or LSTP_READ_LEN_ALL)
 *
 * Caller must hold ch0->tx_mutex and release when done. resp_buf: on success
 * caller MUST lstp_unlock_resp_buffer(); on error lock released internally.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_ch0_read_helper(struct lstp_usb *dev, u8 ch_id, u16 offset, u16 length)
{
	int ret;
	struct lstp_channel *ch0;
	struct lstp_packet *tx_pkt;
	union lstp_ch0_req_payload *ch0_req;

	if (!dev || !dev->channels[0])
		return -ENODEV;

	ch0 = dev->channels[0];
	lockdep_assert_held(&ch0->tx_mutex);
	tx_pkt = (struct lstp_packet *)ch0->tx_buf;
	ch0_req = (union lstp_ch0_req_payload *)tx_pkt->payload;

	ch0_req->read.ch_id = ch_id;
	ch0_req->read.offset = cpu_to_le16(offset);
	ch0_req->read.length = cpu_to_le16(length);

	ret = lstp_recv_resp_helper(ch0, LSTP_CH0_CMD_READ_CONFIG, sizeof(ch0_req->read),
				    LSTP_ANY_RX_LEN);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: READ_CONFIG for ch_%d failed (%pe)\n", __func__,
			ch_id, ERR_PTR(ret));
		return ret;
	}

	/* Validate payload has at least the base lstp_ch0_resp_read config */
	if (!LSTP_GET_PAYLOAD((struct lstp_packet *)ch0->resp_buf, struct lstp_ch0_resp_read)) {
		dev_err(&dev->intf->dev, "%s: ch_%d: READ_CONFIG response too short\n", __func__,
			ch_id);
		lstp_unlock_resp_buffer(ch0);
		return -EINVAL;
	}

	/* Note: response in ch0->resp_buf */
	return 0;
}

/**
 * lstp_ch0_read_enabled_urb() - Read channel flags at runtime via URB.
 * @dev:          LSTP USB device structure
 * @ch_id:        Channel ID to query
 * @ch_flags_out: Output pointer for channel flags
 *
 * Reads the channel's enabled flag via READ_CONFIG command and copies to @ch_flags_out.
 * This function is intended specifically for runtime queries of the LSTP_CH_FLAG_ENABLE bit via
 * sysfs, not for general config access.
 *
 * Context: Takes ch0->tx_mutex, blocking USB I/O.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_ch0_read_enabled_urb(struct lstp_usb *dev, u8 ch_id, u8 *ch_flags_out)
{
	int ret;
	struct lstp_channel *ch0;
	struct lstp_packet *tx_pkt;
	struct lstp_packet *rx_pkt;
	union lstp_ch0_req_payload *ch0_req;
	struct lstp_ch0_resp_read *ch0_resp;

	if (!ch_flags_out)
		return -EINVAL;

	if (!dev || !dev->channels[0])
		return -ENODEV;

	ch0 = dev->channels[0];
	tx_pkt = (struct lstp_packet *)ch0->tx_buf;
	rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	ch0_req = (union lstp_ch0_req_payload *)tx_pkt->payload;

	mutex_lock(&ch0->tx_mutex);

	ch0_req->read.ch_id = ch_id;
	ch0_req->read.offset = 0;
	ch0_req->read.length = cpu_to_le16(LSTP_READ_LEN_ALL);

	ret = lstp_recv_resp_helper(ch0, LSTP_CH0_CMD_READ_CONFIG, sizeof(ch0_req->read),
				    LSTP_ANY_RX_LEN);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: READ_CONFIG for ch_%d failed (%pe)\n", __func__,
			ch_id, ERR_PTR(ret));
		goto out_mutex;
	}

	/* Get and validate payload pointer */
	ch0_resp = LSTP_GET_PAYLOAD(rx_pkt, struct lstp_ch0_resp_read);
	if (!ch0_resp) {
		dev_err(&dev->intf->dev,
			"%s: ch_0: Response too small for ch_%d (got %u, need >=%lu)\n", __func__,
			ch_id, le16_to_cpu(rx_pkt->hdr.length), sizeof(struct lstp_ch0_resp_read));
		ret = -EIO;
	}

	if (!ret)
		*ch_flags_out = ch0_resp->ch_flags;

	lstp_unlock_resp_buffer(ch0);
out_mutex:
	mutex_unlock(&ch0->tx_mutex);
	return ret;
}

/**
 * lstp_ch0_set_enable_urb() - Atomically set/clear channel enable flag.
 * @dev:     LSTP USB device structure
 * @ch_id:   Channel ID to modify
 * @ch_type: Channel type
 * @enable:  true to enable, false to disable
 *
 * Update only the enable bit (LSTP_CH_FLAG_ENABLE) while preserving all other flags.
 *
 * Context: Takes ch0->tx_mutex, blocking USB I/O.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_ch0_set_enable_urb(struct lstp_usb *dev, u8 ch_id, u8 ch_type, bool enable)
{
	int ret;
	struct lstp_channel *ch0;
	struct lstp_packet *tx_pkt;
	struct lstp_packet *rx_pkt;
	union lstp_ch0_req_payload *ch0_req;
	struct lstp_ch0_resp_read *ch0_resp;
	size_t write_len;
	u8 ch_flags;

	if (!dev || !dev->channels[0])
		return -ENODEV;

	ch0 = dev->channels[0];
	tx_pkt = (struct lstp_packet *)ch0->tx_buf;
	rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	ch0_req = (union lstp_ch0_req_payload *)tx_pkt->payload;

	mutex_lock(&ch0->tx_mutex);

	/* Step 1: Read current ch_flags */
	ch0_req->read.ch_id = ch_id;
	ch0_req->read.offset = 0;
	ch0_req->read.length = cpu_to_le16(LSTP_READ_LEN_ALL);

	ret = lstp_recv_resp_helper(ch0, LSTP_CH0_CMD_READ_CONFIG, sizeof(ch0_req->read),
				    LSTP_ANY_RX_LEN);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: READ_CONFIG for ch_%d failed (%pe)\n", __func__,
			ch_id, ERR_PTR(ret));
		goto out_mutex;
	}

	ch0_resp = LSTP_GET_PAYLOAD(rx_pkt, struct lstp_ch0_resp_read);
	if (!ch0_resp) {
		dev_err(&dev->intf->dev,
			"%s: ch_0: Response too small for ch_%d (got %u, need >=%lu)\n", __func__,
			ch_id, le16_to_cpu(rx_pkt->hdr.length), sizeof(struct lstp_ch0_resp_read));
		ret = -EIO;
		lstp_unlock_resp_buffer(ch0);
		goto out_mutex;
	}

	/* Step 2: Modify only the enable bit */
	ch_flags = ch0_resp->ch_flags;
	lstp_unlock_resp_buffer(ch0);
	if (enable)
		ch_flags |= LSTP_CH_FLAG_ENABLE;
	else
		ch_flags &= ~LSTP_CH_FLAG_ENABLE;

	/* Step 3: Write back modified flags (ch_id, offset, ch_type, ch_flags) */
	write_len = offsetof(typeof(ch0_req->write), ch_flags) + sizeof(ch0_req->write.ch_flags);

	/*
	 * Ensure structure layout matches our assumptions
	 *
	 * TODO: Remove this after we get support for writing arbitrary buffer
	 *       at specified offset
	 */
	BUILD_BUG_ON(offsetof(typeof(ch0_req->write), ch_id) != 0);
	BUILD_BUG_ON(offsetof(typeof(ch0_req->write), offset) != 1);
	BUILD_BUG_ON(offsetof(typeof(ch0_req->write), ch_type) != 3);
	BUILD_BUG_ON(offsetof(typeof(ch0_req->write), ch_flags) != 4);
	BUILD_BUG_ON(offsetof(typeof(ch0_req->write), ch_name) != 5);

	ch0_req->write.ch_id = ch_id;
	ch0_req->write.offset = 0;
	ch0_req->write.ch_type = ch_type;
	ch0_req->write.ch_flags = ch_flags;

	ret = lstp_recv_resp_helper(ch0, LSTP_CH0_CMD_WRITE_CONFIG, write_len, LSTP_ANY_RX_LEN);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: WRITE_CONFIG for ch_%d failed (%pe)\n",
			__func__, ch_id, ERR_PTR(ret));
		goto out_mutex;
	}

	lstp_unlock_resp_buffer(ch0);
out_mutex:
	mutex_unlock(&ch0->tx_mutex);
	return ret;
}

/**
 * lstp_ch0_lock() - Locks channel configuration (similar to fusing).
 * @dev: LSTP USB device structure
 *
 * Return: 0 on success, negative errno on failure
 */
static int __maybe_unused lstp_ch0_lock(struct lstp_usb *dev)
{
	int ret;
	struct lstp_channel *ch0;
	struct lstp_packet *rx_pkt;

	if (!dev || !dev->channels[0])
		return -ENODEV;

	ch0 = dev->channels[0];

	mutex_lock(&ch0->tx_mutex);

	ret = lstp_recv_resp_helper(ch0, LSTP_CH0_CMD_LOCK, 0, 0);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: LOCK failed (%pe)\n", __func__, ERR_PTR(ret));
		goto out_mutex;
	}

	rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	ret = lstp_validate_resp(dev, rx_pkt, 0);
	lstp_unlock_resp_buffer(ch0);
	if (ret)
		goto out_mutex;

out_mutex:
	mutex_unlock(&ch0->tx_mutex);
	return ret;
}

/*******************************************************************************
 * Devres action callbacks
 ******************************************************************************/

/**
 * lstp_put_usb_dev() - Devres callback to release USB device reference.
 * @data: Pointer to USB device
 */
static void lstp_put_usb_dev(void *data)
{
	usb_put_dev((struct usb_device *)data);
}

/**
 * lstp_kill_and_free_urb() - Devres callback to cancel and free URB.
 * @data: Pointer to URB
 */
static void lstp_kill_and_free_urb(void *data)
{
	usb_kill_urb((struct urb *)data);
	usb_free_urb((struct urb *)data);
}

/**
 * lstp_kobj_del_put() - Remove kobject from sysfs and release reference.
 * @data: Pointer to kobject to clean up
 */
static void lstp_kobj_del_put(void *data)
{
	struct kobject *kobj = data;

	kobject_del(kobj);
	kobject_put(kobj);
}

/**
 * lstp_kobj_release() - Release callback for the lstp kobject.
 * @kobj: Kobject being released
 */
static void lstp_kobj_release(struct kobject *kobj)
{
	/* Intentionally empty: memory is devm-managed in struct lstp_usb */
}

/**
 * lstp_channel_kobj_release() - Release callback for channel kobjects.
 * @kobj: Kobject being released
 */
static void lstp_channel_kobj_release(struct kobject *kobj)
{
	/* Intentionally empty: channel memory is devm-managed */
}

/**
 * lstp_put_fwnode() - Devres callback to release firmware node reference.
 * @data: Pointer to fwnode_handle
 */
static void lstp_put_fwnode(void *data)
{
	fwnode_handle_put((struct fwnode_handle *)data);
}

/*******************************************************************************
 * Sysfs attributes for channels
 ******************************************************************************/

/**
 * lstp_sysfs_get_channel() - Get channel from sysfs kobject.
 * @kobj:         Kobject passed to sysfs callback
 * @lstp_dev_out: Optional output pointer for LSTP USB device
 *
 * Return: Channel pointer, or ERR_PTR on failure
 */
static struct lstp_channel *lstp_sysfs_get_channel(struct kobject *kobj,
						   struct lstp_usb **lstp_dev_out)
{
	struct lstp_channel *ch;

	ch = container_of(kobj, struct lstp_channel, kobj);

	if (!ch || !ch->usb)
		return ERR_PTR(-ENODEV);

	if (lstp_dev_out)
		*lstp_dev_out = ch->usb;

	return ch;
}

/**
 * lstp_name_show() - Read handler for ``lstp/name``.
 * @kobj: Kobject for the ``lstp`` sysfs directory (embedded in lstp_usb)
 * @attr: Sysfs kobject attribute
 * @buf:  Output buffer for sysfs read
 *
 * Emits the LSTP interface name from the channel-0 discovery response.
 *
 * Return: Number of bytes written to buf, or negative errno on failure
 */
static ssize_t lstp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lstp_usb *lstp = container_of(kobj, struct lstp_usb, lstp_kobj);

	return sysfs_emit(buf, "%s\n", lstp->lstp_intf_name);
}

/**
 * lstp_channel_enable_show() - Show channel enable status via sysfs.
 * @kobj: Kobject for the channel (embedded in lstp_channel)
 * @attr: Sysfs kobject attribute
 * @buf:  Output buffer for sysfs read
 *
 * Queries the management channel (ch0) for this channel's configuration
 * and extracts the enable bit (bit 0 of ch_flags in READ_CONFIG response).
 *
 * Return: Number of bytes written to buf, or negative errno on failure
 */
static ssize_t lstp_channel_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	struct lstp_usb *lstp_dev;
	struct lstp_channel *ch;
	u8 ch_flags;
	u8 enable_bit;
	int ret;

	ch = lstp_sysfs_get_channel(kobj, &lstp_dev);
	if (IS_ERR(ch))
		return PTR_ERR(ch);

	ret = lstp_ch0_read_enabled_urb(lstp_dev, ch->ch_id, &ch_flags);
	if (ret)
		return ret;

	enable_bit = (ch_flags & LSTP_CH_FLAG_ENABLE) ? 1 : 0;

	return sysfs_emit(buf, "%u\n", enable_bit);
}

/**
 * lstp_channel_enable_store() - Set channel enable status via sysfs.
 * @kobj:  Kobject for the channel (embedded in lstp_channel)
 * @attr:  Sysfs kobject attribute
 * @buf:   Input buffer containing "0" or "1"
 * @count: Number of bytes in buf
 *
 * Return: Bytes consumed, or negative errno
 */
static ssize_t lstp_channel_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	struct lstp_usb *lstp_dev;
	struct lstp_channel *ch;
	unsigned int enable;
	int ret;

	ch = lstp_sysfs_get_channel(kobj, &lstp_dev);
	if (IS_ERR(ch))
		return PTR_ERR(ch);

	ret = kstrtouint(buf, 0, &enable);
	if (ret)
		return ret;

	if (enable > 1)
		return -EINVAL;

	ret = lstp_ch0_set_enable_urb(lstp_dev, ch->ch_id, ch->ch_type, enable != 0);
	if (ret)
		return ret;

	return count;
}

/**
 * lstp_channel_name_show() - Show channel display name via sysfs "name" attr.
 * @kobj: Kobject for the channel (embedded in lstp_channel)
 * @attr: Sysfs kobject attribute
 * @buf:  Output buffer for sysfs read
 *
 * Emits the channel's display_name.
 *
 * Return: Number of bytes written to buf, or negative errno on failure
 */
static ssize_t lstp_channel_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lstp_channel *ch;

	ch = lstp_sysfs_get_channel(kobj, NULL);
	if (IS_ERR(ch))
		return PTR_ERR(ch);

	return sysfs_emit(buf, "%s\n", ch->display_name);
}

/**
 * lstp_create_sysfs_hierarchy() - Create lstp sysfs ("name" + "channel/").
 * @dev: LSTP USB device structure
 *
 * Under the USB interface device sysfs directory, create:
 *     lstp/
 *     ├── name			(LSTP interface name, read-only)
 *     └── channel/		(per-channel sysfs parent directory)
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_create_sysfs_hierarchy(struct lstp_usb *dev)
{
	struct device *parent_dev = &dev->intf->dev;
	int ret;

	/* Create lstp directory (name attribute provided by lstp_ktype) */
	ret = kobject_init_and_add(&dev->lstp_kobj, &lstp_ktype, &parent_dev->kobj, "lstp");
	if (ret) {
		kobject_put(&dev->lstp_kobj);
		return ret;
	}

	ret = devm_add_action_or_reset(parent_dev, lstp_kobj_del_put, &dev->lstp_kobj);
	if (ret)
		return ret;

	/* Create channel directory under lstp */
	dev->channel_kobj = kobject_create_and_add("channel", &dev->lstp_kobj);
	if (!dev->channel_kobj)
		return -ENOMEM;

	return devm_add_action_or_reset(parent_dev, lstp_kobj_del_put, dev->channel_kobj);
}

/**
 * lstp_remove_device_link() - Devres callback to remove device symlink.
 * @data: Pointer to channel kobject
 */
static void lstp_remove_device_link(void *data)
{
	struct kobject *kobj = data;

	sysfs_remove_link(kobj, "device");
}

/**
 * lstp_channel_link_device() - Create symlink from channel to its child device.
 * @ch: LSTP channel (must have ch->child_dev set)
 *
 * Creates a symlink named "device" under the channel's sysfs directory
 * that points to the child device. This allows easy identification of
 * which subsystem device corresponds to each channel.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_channel_link_device(struct lstp_channel *ch)
{
	struct device *parent_dev = &ch->usb->intf->dev;
	int ret;

	if (WARN(!ch->child_dev, "lstp: ch_%d (%s): child_dev not set\n", ch->ch_id,
		 ch->subsys ? ch->subsys->name : "?"))
		return 0;

	ret = sysfs_create_link(&ch->kobj, &ch->child_dev->kobj, "device");
	if (ret) {
		dev_err(parent_dev, "%s: ch_%d: Failed to create device symlink (%pe)\n", __func__,
			ch->ch_id, ERR_PTR(ret));
		return ret;
	}

	return devm_add_action_or_reset(parent_dev, lstp_remove_device_link, &ch->kobj);
}

/**
 * lstp_create_channel_sysfs() - Create sysfs directory and attrs for channel.
 * @ch: LSTP channel
 *
 * Creates a directory "lstp/channel/N" (where N is the channel ID) with
 * "enable" and "name" attributes. "enable" is read-write and queries/updates
 * the device via the management channel. "name" is read-only and shows the
 * channel's display_name.
 *
 * Also creates a "device" symlink to the child device if ch->child_dev is set.
 *
 * The kobject cleanup is registered via devm_add_action_or_reset() so it
 * will be automatically handled when the USB interface is released.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_create_channel_sysfs(struct lstp_channel *ch)
{
	struct device *dev = &ch->usb->intf->dev;
	int ret;

	ret = kobject_init_and_add(&ch->kobj, &lstp_channel_ktype, ch->usb->channel_kobj, "%d",
				   ch->ch_id);
	if (ret) {
		dev_err(dev, "%s: Failed to create sysfs for ch_%d (%pe)\n", __func__, ch->ch_id,
			ERR_PTR(ret));
		kobject_put(&ch->kobj);
		return ret;
	}

	/* Register cleanup action to remove kobject from sysfs on disconnect */
	ret = devm_add_action_or_reset(dev, lstp_kobj_del_put, &ch->kobj);
	if (ret)
		return ret;

	/* Create symlink to child device if set */
	return lstp_channel_link_device(ch);
}

/*******************************************************************************
 * Device Tree functions
 ******************************************************************************/

/**
 * lstp_find_channel_fwnode() - Find channel firmware node (DT or ACPI).
 * @usb_dev_fwnode: USB device firmware node
 * @intf_num:       Interface number
 * @channel_id:     Channel ID
 * @compatible:     Compatible string
 *
 * Finds a channel firmware node that matches the given parameters.
 * Works transparently with Device Tree, ACPI (_DSD properties), and
 * software nodes.
 *
 * Expected firmware hierarchy::
 *
 *   usb-device@X {                    // USB device node (usb_dev_fwnode)
 *       compatible = "usbVVVV,PPPP";  // USB VID:PID (e.g., "usb0955,cf11")
 *
 *       interface@N {                 // USB interface node
 *           reg = <N>;                // bInterfaceNumber (intf_num)
 *
 *           channel@M {               // LSTP channel node (returned)
 *               reg = <M>;            // Channel ID (channel_id)
 *               compatible = "...";   // Must match 'compatible' parameter
 *               // Channel-specific properties and child devices...
 *           };
 *       };
 *   };
 *
 * Return: fwnode_handle with incremented refcount on success, NULL on failure
 */
static struct fwnode_handle *lstp_find_channel_fwnode(struct fwnode_handle *usb_dev_fwnode,
						      u8 intf_num, u8 channel_id,
						      const char *compatible)
{
	struct fwnode_handle *intf_node, *ch_node;
	u32 reg;

	if (!usb_dev_fwnode)
		return NULL;

	fwnode_for_each_child_node(usb_dev_fwnode, intf_node) {
		if (fwnode_property_read_u32(intf_node, "reg", &reg) || reg != intf_num)
			continue;

		fwnode_for_each_child_node(intf_node, ch_node) {
			if (fwnode_property_read_u32(ch_node, "reg", &reg) || reg != channel_id)
				continue;
			if (fwnode_property_match_string(ch_node, "compatible", compatible) < 0)
				continue;
			fwnode_handle_put(intf_node);
			return ch_node;
		}
		fwnode_handle_put(intf_node);
		break;
	}

	return NULL;
}

/**
 * lstp_init_channel_fwnode() - Initialize channel's firmware node (DT or ACPI).
 * @ch:         LSTP channel to initialize
 * @compatible: Compatible string to match in firmware description
 *
 * Finds and assigns the appropriate firmware node for this channel.
 * Works with Device Tree, ACPI, and software nodes.
 * Registers a devm action to automatically release the node reference.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_init_channel_fwnode(struct lstp_channel *ch, const char *compatible)
{
	if (!compatible)
		return 0;

	ch->fwnode = lstp_find_channel_fwnode(dev_fwnode(&ch->usb->udev->dev),
					      ch->usb->intf->cur_altsetting->desc.bInterfaceNumber,
					      ch->ch_id, compatible);
	if (!ch->fwnode)
		return 0;

	return devm_add_action_or_reset(&ch->usb->intf->dev, lstp_put_fwnode, ch->fwnode);
}

/**
 * lstp_create_channel() - Allocate and initialize an LSTP channel.
 * @dev:   Parent LSTP USB device
 * @ch_id: Channel ID to assign to the new channel
 *
 * Return: Pointer to the new channel on success, NULL on allocation failure
 */
static struct lstp_channel *lstp_create_channel(struct lstp_usb *dev, u8 ch_id)
{
	struct lstp_channel *ch;
	int ret;

	ch = devm_kzalloc(&dev->intf->dev, sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return NULL;

	ch->tx_buf = devm_kzalloc(&dev->intf->dev, dev->bulk_tx_size, GFP_KERNEL);
	if (!ch->tx_buf)
		return NULL;

	ch->resp_buf = NULL;
	ch->irq_buf = NULL;

	ch->bulk_tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ch->bulk_tx_urb)
		return NULL;

	ret = devm_add_action_or_reset(&dev->intf->dev, lstp_kill_and_free_urb, ch->bulk_tx_urb);
	if (ret)
		return NULL;

	ch->ch_id = ch_id;
	ch->usb = dev;
	mutex_init(&ch->tx_mutex);
	init_waitqueue_head(&ch->rx_wq);
	ch->resp_buffer_lock = 0;
	ch->irq_buffer_lock = 0;
	ch->irq_resp_buffer_lock = 0;
	ch->rx_ready = false;
	ch->disconnected = false;
	ch->priv = NULL;
	dev->channels[ch_id] = ch;
	return ch;
}

/**
 * lstp_ch0_init() - Create and initialize channel 0 (management).
 * @dev: LSTP USB device structure
 *
 * Creates ch0 via lstp_create_channel, allocates ch0->resp_buf, reads ch0 config
 * via lstp_ch0_read_helper, and parses version/count/label into @dev.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_ch0_init(struct lstp_usb *dev)
{
	int ret;
	struct lstp_packet *rx_pkt;
	union lstp_ch0_resp_payload *ch0_resp;
	struct lstp_ch0_config *config;
	struct lstp_channel *ch0;

	/* Create CH_0 and get READ request */
	ch0 = lstp_create_channel(dev, 0);
	if (!ch0) {
		dev_err(&dev->intf->dev, "%s: Could not allocate channel 0\n", __func__);
		return -ENOMEM;
	}
	ch0->ch_type = LSTP_CHANNEL_TYPE_MGMT;
	ch0->resp_buf = devm_kzalloc(&dev->intf->dev, dev->bulk_rx_size, GFP_KERNEL);
	if (!ch0->resp_buf)
		return -ENOMEM;

	mutex_lock(&ch0->tx_mutex);
	ret = lstp_ch0_read_helper(dev, 0, 0, LSTP_READ_LEN_ALL);
	if (ret) {
		mutex_unlock(&ch0->tx_mutex);
		return ret;
	}

	/* Validate expected CH0 config size */
	rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	ret = lstp_validate_resp(dev, rx_pkt,
				 sizeof(struct lstp_ch0_resp_read) +
					 sizeof(struct lstp_ch0_config));
	if (ret)
		goto out;
	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	config = (struct lstp_ch0_config *)ch0_resp->read.ch_config;

	/* Parse READ response for CH0 */
	if (ch0_resp->read.ch_type != LSTP_CHANNEL_TYPE_MGMT) {
		dev_err(&dev->intf->dev, "%s: ch_0: Received wrong channel 0 type (got %d)\n",
			__func__, ch0_resp->read.ch_type);
		ret = -EINVAL;
		goto out;
	}

	/* Validate interface name */
	if (ch0_resp->read.ch_name[0] == '\0') {
		dev_warn(&dev->intf->dev,
			 "%s: ch_0: Empty LSTP interface name, consider fixing it! Using default 'LSTP'\n",
			 __func__);
		strscpy(dev->lstp_intf_name, "LSTP", sizeof(dev->lstp_intf_name));
	} else {
		snprintf(dev->lstp_intf_name, sizeof(dev->lstp_intf_name), "%.*s", LSTP_CH_NAME_LEN,
			 ch0_resp->read.ch_name);
	}
	strscpy(ch0->display_name, dev->lstp_intf_name, sizeof(ch0->display_name));

	/* Validate LSTP version */
	if (config->lstp_version != LSTP_VERSION) {
		dev_err(&dev->intf->dev, "%s: ch_0: Invalid LSTP version\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	dev->lstp_version = config->lstp_version;

	/* Validate channel count */
	if (config->num_channels == 0 || config->num_channels >= LSTP_MAX_CHANNELS) {
		dev_err(&dev->intf->dev, "%s: ch_0: Invalid channel count %d (min 1, max %d)\n",
			__func__, config->num_channels, LSTP_MAX_CHANNELS - 1);
		ret = -EINVAL;
		goto out;
	}
	dev->max_ch_id = config->num_channels;

	dev_info(&dev->intf->dev, "%s: LSTP v%d: device %s discovered with %d channels\n", __func__,
		 dev->lstp_version, dev->lstp_intf_name, dev->max_ch_id);
	ret = 0;
out:
	lstp_unlock_resp_buffer(ch0);
	mutex_unlock(&ch0->tx_mutex);
	return ret;
}

/* clang-format off */
static const char * const lstp_ch_type_tags[] = {
	[LSTP_CHANNEL_TYPE_SPI] = "SPI",
	[LSTP_CHANNEL_TYPE_GPIO] = "GPIO",
	[LSTP_CHANNEL_TYPE_I2C] = "I2C",
	[LSTP_CHANNEL_TYPE_UART] = "UART",
	[LSTP_CHANNEL_TYPE_IPMI] = "IPMI",
}; /* clang-format on */

/**
 * lstp_set_display_name() - Set ch->display_name from firmware or synthesize a default.
 * @ch:          Channel (ch_id and ch_type must already be set)
 * @ch_name:     Raw firmware ch_name (may not be NUL-terminated)
 * @ch_name_len: Size of @ch_name buffer (typically LSTP_CH_NAME_LEN)
 *
 * Format: "<intf_name>_<ch_name>" or "<intf_name>_<TYPE>_CH<id>" (with a warning).
 */
static void lstp_set_display_name(struct lstp_channel *ch, const char *ch_name, size_t ch_name_len)
{
	size_t name_len = strnlen(ch_name, ch_name_len);
	const char *tag;

	if (name_len) {
		snprintf(ch->display_name, sizeof(ch->display_name), "%s_%.*s",
			 ch->usb->lstp_intf_name, (int)name_len, ch_name);
		return;
	}

	tag = (ch->ch_type < ARRAY_SIZE(lstp_ch_type_tags)) ? lstp_ch_type_tags[ch->ch_type] : NULL;
	if (!tag)
		tag = "UNKNOWN";

	snprintf(ch->display_name, sizeof(ch->display_name), "%s_%s_CH%u", ch->usb->lstp_intf_name,
		 tag, ch->ch_id);
	dev_warn(&ch->usb->intf->dev,
		 "%s: ch_%d: Empty channel name, consider fixing it! Using default '%s'\n",
		 __func__, ch->ch_id, ch->display_name);
}

/**
 * lstp_init_channels() - Discover and initialize all LSTP channels.
 * @dev: LSTP USB device structure
 *
 * Calls lstp_ch0_init() then creates and initializes channels 1...max_ch_id.
 * Does NOT register with Linux subsystems; call lstp_start_channels() after.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_init_channels(struct lstp_usb *dev)
{
	int ret;
	int ch_id;
	struct lstp_channel *ch0;
	struct lstp_packet *rx_pkt;
	union lstp_ch0_resp_payload *ch0_resp;

	ret = lstp_ch0_init(dev);
	if (ret)
		return ret;

	ch0 = dev->channels[0];
	for (ch_id = 1; ch_id <= dev->max_ch_id; ch_id++) {
		/* Create CH_i and get READ request */
		struct lstp_channel *ch = lstp_create_channel(dev, ch_id);

		if (!ch) {
			dev_err(&dev->intf->dev, "%s: Could not allocate channel %d\n", __func__,
				ch_id);
			return -ENOMEM;
		}

		mutex_lock(&ch0->tx_mutex);
		ret = lstp_ch0_read_helper(dev, ch_id, 0, LSTP_READ_LEN_ALL);
		if (ret) {
			mutex_unlock(&ch0->tx_mutex);
			return ret;
		}

		/* Parse READ response data and init type-specific structures */
		rx_pkt = (struct lstp_packet *)ch0->resp_buf;
		ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
		ch->ch_type = ch0_resp->read.ch_type;
		lstp_set_display_name(ch, ch0_resp->read.ch_name, LSTP_CH_NAME_LEN);

		ch->subsys = lstp_subsys_by_channel_type(ch->ch_type);
		if (!ch->subsys) {
			dev_warn(&dev->intf->dev, "%s: Channel %d has unsupported type %d\n",
				 __func__, ch_id, ch->ch_type);
			ret = 0;
		} else {
			ret = lstp_init_channel_fwnode(ch, ch->subsys->fwnode_compatible);
			if (!ret)
				ret = ch->subsys->channel_init(ch);
		}

		/* Done using ch0->resp_buf for this channel; release before next iteration. */
		lstp_unlock_resp_buffer(ch0);
		mutex_unlock(&ch0->tx_mutex);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * lstp_start_channels() - Register all channels with Linux subsystems.
 * @dev: LSTP USB device structure
 *
 * Call after lstp_init_channels() and RX URB submission.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_start_channels(struct lstp_usb *dev)
{
	int ret;
	int i;
	struct lstp_channel *ch;

	for (i = 1; i < LSTP_MAX_CHANNELS; i++) {
		ch = dev->channels[i];
		/* Skip unsupported channel types: nothing to expose. */
		if (!ch || !ch->subsys)
			continue;

		ret = ch->subsys->channel_start(ch);
		if (ret)
			return ret;
		ch->started = true;

		ret = lstp_create_channel_sysfs(ch);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * lstp_probe() - USB driver probe function.
 * @intf: USB interface being probed
 * @id:   USB device ID that matched
 *
 * Two-phase init: lstp_init_channels() discovers/allocates, then after
 * RX URB setup, lstp_start_channels() registers with Linux subsystems.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct lstp_usb *dev;
	int ret;

	dev = devm_kzalloc(&intf->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		usb_put_dev(udev);
		return -ENOMEM;
	}

	ret = devm_add_action_or_reset(&intf->dev, lstp_put_usb_dev, udev);
	if (ret)
		return ret;

	ret = usb_find_common_endpoints(intf->cur_altsetting, &ep_in, &ep_out, NULL, NULL);
	if (ret) {
		dev_err(&intf->dev, "%s: Could not find bulk endpoints\n", __func__);
		return ret;
	}

	dev->udev = udev;
	dev->intf = intf;
	dev->max_ch_id = 255; /* Maximum possible before device discovery */
	dev->bulk_in_ep = ep_in->bEndpointAddress;
	dev->bulk_out_ep = ep_out->bEndpointAddress;
	dev->bulk_tx_size = min_t(size_t, usb_endpoint_maxp(ep_out), LSTP_USB_EP_MAX_SIZE);
	dev->bulk_rx_size = min_t(size_t, usb_endpoint_maxp(ep_in), LSTP_USB_EP_MAX_SIZE);

	/* Validate minimum size */
	if (dev->bulk_tx_size < LSTP_USB_EP_MIN_SIZE || dev->bulk_rx_size < LSTP_USB_EP_MIN_SIZE) {
		dev_err(&intf->dev, "%s: Invalid endpoint sizes (tx=%zu, rx=%zu, min=%u)\n",
			__func__, dev->bulk_tx_size, dev->bulk_rx_size, LSTP_USB_EP_MIN_SIZE);
		return -EINVAL;
	}

	dev->rx_buf = devm_kzalloc(&intf->dev, dev->bulk_rx_size, GFP_KERNEL);
	if (!dev->rx_buf)
		return -ENOMEM;

	dev->bulk_rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bulk_rx_urb)
		return -ENOMEM;

	ret = devm_add_action_or_reset(&intf->dev, lstp_kill_and_free_urb, dev->bulk_rx_urb);
	if (ret)
		return ret;

	usb_set_intfdata(intf, dev);

	INIT_DELAYED_WORK(&dev->bulk_rx_retry.work, lstp_rx_retry_work);

	usb_fill_bulk_urb(dev->bulk_rx_urb, dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_ep),
			  dev->rx_buf, dev->bulk_rx_size, lstp_usb_rx_callback, dev);
	ret = usb_submit_urb(dev->bulk_rx_urb, GFP_KERNEL);
	if (ret) {
		dev_err(&intf->dev, "%s: Could not submit bulk RX URB (%pe)\n", __func__,
			ERR_PTR(ret));
		return ret;
	}

	ret = lstp_init_channels(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to initialize channels (%pe)\n", __func__,
			ERR_PTR(ret));
		goto err_teardown;
	}

	ret = lstp_create_sysfs_hierarchy(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to create sysfs hierarchy (%pe)\n", __func__,
			ERR_PTR(ret));
		goto err_teardown;
	}

	ret = lstp_start_channels(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to start channels (%pe)\n", __func__, ERR_PTR(ret));
		goto err_teardown;
	}

	dev_info(&intf->dev, "%s: LSTP device initialized successfully\n", __func__);
	return 0;

err_teardown:
	lstp_teardown(dev);
	return ret;
}

/*******************************************************************************
 * Channel State Helpers
 ******************************************************************************/

/**
 * lstp_signal_disconnect() - Signal all channels that the device is disconnected.
 * @dev: LSTP USB device structure
 */
static void lstp_signal_disconnect(struct lstp_usb *dev)
{
	for (int i = 0; i < LSTP_MAX_CHANNELS; i++) {
		if (dev->channels[i]) {
			/*
			 * Pairs with smp_load_acquire() in lstp_ch_disconnected().
			 * Ensures the flag is visible to the waiter before
			 * wake_up_all().
			 */
			smp_store_release(&dev->channels[i]->disconnected, true);
			wake_up_all(&dev->channels[i]->rx_wq);
		}
	}
}

/**
 * lstp_ch_signal_response() - Signal that a response has been received on a channel.
 * @ch: LSTP channel that received the response
 */
static void lstp_ch_signal_response(struct lstp_channel *ch)
{
	/*
	 * Pairs with smp_load_acquire() in lstp_ch_got_response().
	 * Release semantics ensure the memcpy into resp_buf is visible
	 * to readers before rx_ready becomes true.
	 */
	smp_store_release(&ch->rx_ready, true);
	wake_up_all(&ch->rx_wq);
}

/**
 * lstp_ch_got_response() - Check if a solicited response has been received.
 * @ch: LSTP channel to check
 *
 * Return: true if a response is available in resp_buf.
 */
static inline bool lstp_ch_got_response(struct lstp_channel *ch)
{
	/* Pairs with smp_store_release() in lstp_ch_signal_response() */
	return smp_load_acquire(&ch->rx_ready);
}

/**
 * lstp_ch_should_wake() - wait_event condition for lstp_recv_resp_helper().
 * @ch: LSTP channel to check
 *
 * Return: true if the waiter should wake (response received or device disconnected).
 */
static inline bool lstp_ch_should_wake(struct lstp_channel *ch)
{
	return lstp_ch_got_response(ch) || lstp_ch_disconnected(ch);
}

/**
 * lstp_stop_rx() - Quiesce the RX path.
 * @dev: LSTP USB device structure
 *
 * Poison first so any racing resubmit (callback or worker) gets -EPERM,
 * then drain the worker.
 */
static void lstp_stop_rx(struct lstp_usb *dev)
{
	usb_poison_urb(dev->bulk_rx_urb);
	cancel_delayed_work_sync(&dev->bulk_rx_retry.work);
}

/**
 * lstp_stop_channels() - Stop all channels in reverse start order.
 * @dev: LSTP USB device structure
 *
 * Mirrors lstp_start_channels() so subsystems are torn down in the
 * opposite order they were brought up. Skips channels that never started.
 */
static void lstp_stop_channels(struct lstp_usb *dev)
{
	for (int i = LSTP_MAX_CHANNELS - 1; i >= 1; i--) {
		struct lstp_channel *ch = dev->channels[i];

		if (ch && ch->started && ch->subsys->channel_stop)
			ch->subsys->channel_stop(ch);
	}
}

/**
 * lstp_stop_tx() - Quiesce all per-channel TX URBs.
 * @dev: LSTP USB device structure
 *
 * Poison the request and IRQ-response URBs on every initialised channel
 * so any racing usb_submit_urb() returns -EPERM. Safe on never-submitted
 * URBs.
 */
static void lstp_stop_tx(struct lstp_usb *dev)
{
	for (int i = 0; i < LSTP_MAX_CHANNELS; i++) {
		struct lstp_channel *ch = dev->channels[i];

		if (!ch)
			continue;

		usb_poison_urb(ch->bulk_tx_urb);
		usb_poison_urb(ch->bulk_tx_resp_urb);
	}
}

/**
 * lstp_teardown() - Ordered driver-level teardown of the LSTP device.
 * @dev: LSTP USB device structure
 *
 * Shared by lstp_disconnect() and the lstp_probe() error path (USB core
 * does not call .disconnect() on probe failure). RX must stop before
 * channel_stop, or the dispatcher can race teardown.
 */
static void lstp_teardown(struct lstp_usb *dev)
{
	/* RX must stop before channel_stop, or the dispatcher can race teardown. */
	lstp_stop_rx(dev);

	lstp_signal_disconnect(dev);
	lstp_stop_channels(dev);
	lstp_stop_tx(dev);
}

/**
 * lstp_disconnect() - USB driver disconnect function.
 * @intf: USB interface being disconnected
 *
 * Ordered teardown; remaining cleanup runs via devres on interface unbind.
 */
static void lstp_disconnect(struct usb_interface *intf)
{
	struct lstp_usb *dev = usb_get_intfdata(intf);

	dev_info(&intf->dev, "%s: LSTP device disconnected\n", __func__);

	if (!dev)
		return;

	lstp_teardown(dev);
}

/*******************************************************************************
 * USB Helper Functions
 ******************************************************************************/

/**
 * lstp_usb_tx_callback() - USB TX URB completion callback.
 * @urb: Completed transmit URB
 */
static void lstp_usb_tx_callback(struct urb *urb)
{
	struct lstp_channel *ch = urb->context;

	if (urb->status != 0 && urb->status != -ENOENT && urb->status != -ECONNRESET &&
	    urb->status != -ESHUTDOWN) {
		dev_warn(&ch->usb->intf->dev, "%s: ch_%d: TX URB error (%pe)\n", __func__,
			 ch->ch_id, ERR_PTR(urb->status));
	}
}

/**
 * lstp_usb_resp_tx_callback() - Completion callback for @bulk_tx_resp_urb.
 * @urb: Completed transmit URB
 *
 * Clears @ch->irq_resp_buffer_lock on every completion path (including kill
 * paths -ENOENT / -ECONNRESET / -ESHUTDOWN) so the bit never gets stuck
 * across disconnect/re-probe.
 */
static void lstp_usb_resp_tx_callback(struct urb *urb)
{
	struct lstp_channel *ch = urb->context;

	if (urb->status != 0 && urb->status != -ENOENT && urb->status != -ECONNRESET &&
	    urb->status != -ESHUTDOWN) {
		dev_warn(&ch->usb->intf->dev, "%s: ch_%d: RX response TX URB error (%pe)\n",
			 __func__, ch->ch_id, ERR_PTR(urb->status));
	}

	clear_bit(LSTP_BUFFER_LOCK_BIT, &ch->irq_resp_buffer_lock);
}

/**
 * lstp_alloc_irq_resp() - Allocate the response-TX buffer/URB used by lstp_send_irq_resp().
 * @ch: LSTP channel that will reply to unsolicited requests
 *
 * Subsystems whose @irq_callback ACKs unsolicited requests must call this
 * from their channel_init(); channels that never invoke lstp_send_irq_resp()
 * should not allocate these resources. Both the buffer and the URB are
 * managed via devres and freed when the USB interface goes away.
 *
 * Return: 0 on success, negative errno on failure.
 */
int lstp_alloc_irq_resp(struct lstp_channel *ch)
{
	struct device *dev = &ch->usb->intf->dev;
	int ret;

	ch->tx_resp_buf = devm_kzalloc(dev, sizeof(struct lstp_header), GFP_KERNEL);
	if (!ch->tx_resp_buf)
		return -ENOMEM;

	ch->bulk_tx_resp_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ch->bulk_tx_resp_urb)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, lstp_kill_and_free_urb, ch->bulk_tx_resp_urb);
	if (ret)
		return ret;

	return 0;
}

/**
 * lstp_send_irq_resp() - Post a zero-payload response for an unsolicited RX packet.
 * @ch:     LSTP channel that received the IRQ packet
 * @status: Status byte to return to firmware (bit 7 is forced to 1 = response)
 *
 * Builds a header-only response packet in @ch->tx_resp_buf and submits it on
 * @ch->bulk_tx_resp_urb with GFP_ATOMIC. Intended to be called from an
 * ``irq_callback`` handler (atomic context, USB RX completion path), where
 * the caller has no meaningful recovery for a submission failure — errors
 * are logged here and the function returns void (fire-and-forget).
 *
 * @ch->irq_resp_buffer_lock guards @tx_resp_buf / @bulk_tx_resp_urb against
 * concurrent reuse while a previous response is still being DMA'd. Under a
 * strictly 1-in-flight firmware contract the guard should never trip; any
 * hit is logged (ratelimited) as an actionable anomaly.
 */
void lstp_send_irq_resp(struct lstp_channel *ch, u8 status)
{
	struct lstp_packet *tx_pkt;
	int ret;

	if (!ch || !ch->usb || !ch->usb->udev || !ch->tx_resp_buf || !ch->bulk_tx_resp_urb)
		return;

	if (test_and_set_bit(LSTP_BUFFER_LOCK_BIT, &ch->irq_resp_buffer_lock)) {
		dev_warn_ratelimited(&ch->usb->intf->dev,
				     "%s: ch_%d: Dropping response - prior URB in flight\n",
				     __func__, ch->ch_id);
		return;
	}

	tx_pkt = (struct lstp_packet *)ch->tx_resp_buf;
	tx_pkt->hdr.ch_id = ch->ch_id;
	tx_pkt->hdr.length = 0;
	tx_pkt->hdr.status = SET_U8_BYTE(status, 1);

	usb_fill_bulk_urb(ch->bulk_tx_resp_urb, ch->usb->udev,
			  usb_sndbulkpipe(ch->usb->udev, ch->usb->bulk_out_ep), tx_pkt,
			  sizeof(struct lstp_header), lstp_usb_resp_tx_callback, ch);

	ret = usb_submit_urb(ch->bulk_tx_resp_urb, GFP_ATOMIC);
	if (ret) {
		clear_bit(LSTP_BUFFER_LOCK_BIT, &ch->irq_resp_buffer_lock);
		dev_err_ratelimited(&ch->usb->intf->dev,
				    "%s: ch_%d: Failed to submit TX URB (%pe)\n", __func__,
				    ch->ch_id, ERR_PTR(ret));
	}
}

/*
 * Retry indefinitely with bounded exponential backoff. No attempt budget:
 * giving up on transient pressure (-ENOMEM, etc.) would permanently kill
 * RX even when the underlying condition would self-heal.
 *
 * BASE_DELAY: minimum gap before the worker reposts the RX URB. Used both
 *             as the entry delay after a usb_submit_urb() failure and as a
 *             throttle for URB completion errors (e.g. -EPROTO from a
 *             babbling / mid-disconnect device) so we don't tight-loop the
 *             HCD from the completion callback. Kept well under
 *             LSTP_USB_RESPONSE_TIMEOUT_MS so a single transient blip does
 *             not burn the protocol's response budget.
 * MAX_DELAY:  ceiling for repeated submit failures' exponential backoff.
 */
#define LSTP_RX_RETRY_BASE_DELAY_MS 50
#define LSTP_RX_RETRY_MAX_DELAY_MS 1000
#define LSTP_RX_RETRY_MAX_BACKOFF_SHIFT 5

/**
 * lstp_rx_retry_backoff_ms() - Compute next RX retry delay.
 * @failures: Consecutive submit failure count (0 == first attempt)
 *
 * Doubles per attempt up to LSTP_RX_RETRY_MAX_DELAY_MS.
 *
 * Return: Delay in milliseconds before the next retry attempt.
 */
static unsigned long lstp_rx_retry_backoff_ms(unsigned int failures)
{
	unsigned int shift = failures ? min(failures - 1, LSTP_RX_RETRY_MAX_BACKOFF_SHIFT) : 0;
	unsigned long ms = (unsigned long)LSTP_RX_RETRY_BASE_DELAY_MS << shift;

	return min_t(unsigned long, ms, LSTP_RX_RETRY_MAX_DELAY_MS);
}

/**
 * lstp_rx_handle_err() - Defer or terminate RX after a URB error.
 * @dev:      LSTP USB device structure
 * @ret:      Errno from a usb_submit_urb() failure or a URB completion
 * @delay_ms: Backoff delay if scheduling a retry
 *
 * Records @ret in @bulk_rx_retry.last_err so the worker can act on it
 * (e.g. clear the endpoint halt for -EPIPE). Terminal teardown errnos
 * signal disconnect silently; everything else schedules the worker.
 */
static void lstp_rx_handle_err(struct lstp_usb *dev, int ret, unsigned long delay_ms)
{
	struct lstp_rx_retry *rxr = &dev->bulk_rx_retry;

	WRITE_ONCE(rxr->last_err, ret);

	/*
	 * -EPERM is the usb_poison_urb() fence set in lstp_stop_rx();
	 * -ENODEV/-ESHUTDOWN mean the device or HCD is gone. All three are
	 * the expected outcome of disconnect, not failures: signal the
	 * disconnect path and stop reposting. lstp_disconnect() already
	 * logs the device removal, so no message here.
	 */
	if (ret == -ENODEV || ret == -ESHUTDOWN || ret == -EPERM) {
		lstp_signal_disconnect(dev);
		return;
	}

	/*
	 * Wire-side completion errors (-EPROTO/-EOVERFLOW/-EILSEQ) come
	 * from a babbling or otherwise misbehaving device and can fire
	 * back-to-back, so ratelimit them. Everything else (notably
	 * submit-side -ENOMEM/-EAGAIN/-EINVAL from the worker) is rare and
	 * per-event diagnostic, so log unconditionally; the worker's
	 * exponential backoff already throttles repeats.
	 */
	if (ret == -EPROTO || ret == -EOVERFLOW || ret == -EILSEQ)
		dev_warn_ratelimited(&dev->intf->dev,
				     "%s: RX URB transient error (%pe), retry in %lums\n",
				     __func__, ERR_PTR(ret), delay_ms);
	else
		dev_warn(&dev->intf->dev, "%s: RX URB transient error (%pe), retry in %lums\n",
			 __func__, ERR_PTR(ret), delay_ms);

	schedule_delayed_work(&rxr->work, msecs_to_jiffies(delay_ms));
}

/**
 * lstp_rx_retry_work() - Process-context fallback for the RX completion
 *                           callback's atomic-context resubmit.
 * @work: &struct lstp_rx_retry.work embedded in &struct lstp_usb
 *
 * Clears endpoint halts on -EPIPE, then retries the RX URB with GFP_KERNEL.
 * Failures are handed to lstp_rx_handle_err() with an exponential
 * backoff.
 */
static void lstp_rx_retry_work(struct work_struct *work)
{
	struct lstp_rx_retry *rxr =
		container_of(to_delayed_work(work), struct lstp_rx_retry, work);
	struct lstp_usb *dev = container_of(rxr, struct lstp_usb, bulk_rx_retry);
	int last_err = READ_ONCE(rxr->last_err);
	int ret;

	if (last_err == -EPIPE) {
		ret = usb_clear_halt(dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_ep));
		if (ret)
			dev_warn(&dev->intf->dev, "%s: usb_clear_halt failed (%pe)\n", __func__,
				 ERR_PTR(ret));
	}

	ret = usb_submit_urb(dev->bulk_rx_urb, GFP_KERNEL);
	if (ret == 0) {
		if (rxr->failures)
			dev_info(&dev->intf->dev, "%s: RX URB resubmitted after %u failure(s)\n",
				 __func__, rxr->failures);
		rxr->failures = 0;
		return;
	}

	rxr->failures++;
	lstp_rx_handle_err(dev, ret, lstp_rx_retry_backoff_ms(rxr->failures));
}

/**
 * lstp_usb_rx_callback() - USB RX URB completion callback.
 * @urb: Completed receive URB
 *
 * Routes requests (bit7=0) to irq_callback, responses (bit7=1) to waiters.
 */
static void lstp_usb_rx_callback(struct urb *urb)
{
	struct lstp_usb *dev = urb->context;
	int ret;

	/* URB killed during disconnect */
	if (urb->status == -ENOENT || urb->status == -ECONNRESET || urb->status == -ESHUTDOWN) {
		lstp_signal_disconnect(dev);
		return;
	}

	/*
	 * Persistent HCD errors (e.g. -EPROTO from a babbling or mid-disconnect
	 * device) can fire back-to-back. Defer resubmit to the worker instead
	 * of tight-looping the HCD from atomic context.
	 */
	if (urb->status) {
		lstp_rx_handle_err(dev, urb->status, LSTP_RX_RETRY_BASE_DELAY_MS);
		return;
	}

	struct lstp_packet *rx_pkt = (struct lstp_packet *)dev->rx_buf;

	if (lstp_validate_rx_pkt(dev, rx_pkt, urb->actual_length))
		goto resubmit;

	/* Get channel - ch_id already bound checked by lstp_validate_rx_pkt() */
	u8 ch_id = rx_pkt->hdr.ch_id;
	struct lstp_channel *ch = dev->channels[ch_id];

	if (!ch) {
		dev_err(&dev->intf->dev, "%s: ch_%d: Channel not registered\n", __func__, ch_id);
		goto resubmit;
	}

	if (GET_BIT_7(rx_pkt->hdr.status) == 0 && ch->irq_buf && ch->irq_callback) {
		/* Unsolicited request -> irq_buf + callback */
		if (test_and_set_bit(LSTP_BUFFER_LOCK_BIT, &ch->irq_buffer_lock)) {
			dev_warn(&dev->intf->dev, "%s: ch_%d: Dropping request - irq buffer busy\n",
				 __func__, ch_id);
		} else {
			dev_dbg(&dev->intf->dev,
				"%s: ch_%d: Received unsolicited request (IRQ event)\n", __func__,
				ch_id);
			memcpy(ch->irq_buf, dev->rx_buf, urb->actual_length);
			ch->irq_callback(ch);
			clear_bit(LSTP_BUFFER_LOCK_BIT, &ch->irq_buffer_lock);
		}
	} else if (GET_BIT_7(rx_pkt->hdr.status) == 1 && ch->resp_buf) {
		/* Solicited response -> rx_buf + wake helper */
		if (!test_bit(LSTP_BUFFER_LOCK_BIT, &ch->resp_buffer_lock)) {
			dev_warn(&dev->intf->dev, "%s: ch_%d: Dropping response - no one waiting\n",
				 __func__, ch_id);
		} else {
			memcpy(ch->resp_buf, dev->rx_buf, urb->actual_length);
			lstp_ch_signal_response(ch);
		}
	} else {
		dev_err_ratelimited(&dev->intf->dev,
				    "%s: ch_%d: Dropping packet - channel misconfigured\n",
				    __func__, ch_id);
	}

resubmit:
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		lstp_rx_handle_err(dev, ret, LSTP_RX_RETRY_BASE_DELAY_MS);
}

/**
 * lstp_recv_resp_helper() - Send request and wait for response.
 * @ch:               LSTP channel
 * @cmd:              Command byte (bit 7 set automatically)
 * @request_len:      Payload length in tx_buf
 * @min_response_len: Minimum response payload length, or LSTP_ANY_RX_LEN
 *
 * Caller prepares tx_buf->payload, this sends and waits. Response in rx_buf.
 * Will return after a timeout. Responses larger than @min_response_len are
 * accepted to allow forward compatibility with newer firmware.
 *
 * Lock contract:
 *
 * * On success (return 0): caller MUST call lstp_unlock_resp_buffer()
 * * On error (return < 0): lock is released internally, caller must NOT unlock
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_recv_resp_helper(struct lstp_channel *ch, u8 cmd, u16 request_len, u16 min_response_len)
{
	int ret = 0;

	/* Validate channel and device */
	if (!ch || !ch->usb || !ch->usb->udev || !ch->tx_buf || !ch->resp_buf)
		return -ENODEV;

	/* Validate Request and Response packet sizes */
	if (sizeof(struct lstp_header) + request_len > ch->usb->bulk_tx_size ||
	    sizeof(struct lstp_header) + min_response_len > ch->usb->bulk_rx_size)
		return -EINVAL;

	/* Try to claim rx buffer - if already in use, that's a bug */
	if (test_and_set_bit(LSTP_BUFFER_LOCK_BIT, &ch->resp_buffer_lock)) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: BUG - rx buffer already in use\n",
			__func__, ch->ch_id);
		return -EBUSY;
	}

	struct lstp_packet *request_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *response_pkt = (struct lstp_packet *)ch->resp_buf;

	request_pkt->hdr.ch_id = ch->ch_id;
	request_pkt->hdr.cmd = SET_U8_BYTE(cmd, 0); /* bit 7 = 0 for request */
	request_pkt->hdr.length = cpu_to_le16(request_len);

	if (request_len > 0) {
		dev_dbg(&ch->usb->intf->dev,
			"%s: ch_%d: TX cmd=0x%02x, tx_len=%d, rx_len=%d, payload=0x%*ph\n",
			__func__, ch->ch_id, cmd, request_len, min_response_len, request_len,
			request_pkt->payload);
	} else {
		dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: TX cmd=0x%02x, tx_len=%d, rx_len=%d\n",
			__func__, ch->ch_id, cmd, request_len, min_response_len);
	}

	usb_fill_bulk_urb(ch->bulk_tx_urb, ch->usb->udev,
			  usb_sndbulkpipe(ch->usb->udev, ch->usb->bulk_out_ep), ch->tx_buf,
			  sizeof(struct lstp_header) + request_len, lstp_usb_tx_callback, ch);

	/* Pairs with smp_store_release()/smp_load_acquire() on rx_ready. */
	WRITE_ONCE(ch->rx_ready, false);

	ret = usb_submit_urb(ch->bulk_tx_urb, GFP_KERNEL);

	if (ret) {
		/*
		 * If teardown raced our submit, return -ENODEV (same as the
		 * lstp_ch_disconnected() branch below) instead of leaking
		 * -EPERM/-ESHUTDOWN to userspace.
		 */
		if (ret == -EPERM || ret == -ESHUTDOWN)
			ret = -ENODEV;
		goto out_unlock;
	}

	wait_event_timeout(ch->rx_wq, lstp_ch_should_wake(ch),
			   msecs_to_jiffies(LSTP_USB_RESPONSE_TIMEOUT_MS));

	if (!lstp_ch_got_response(ch)) {
		/* Safe to call even if URB already killed during disconnect */
		usb_kill_urb(ch->bulk_tx_urb);

		if (lstp_ch_disconnected(ch)) {
			ret = -ENODEV;
		} else {
			/* Timeout - indicates protocol violation and device hang */
			ret = -ETIMEDOUT;
		}

		dev_err(&ch->usb->intf->dev, "%s: ch_%d: No response (%pe)\n", __func__, ch->ch_id,
			ERR_PTR(ret));
		goto out_unlock;
	}

	ret = lstp_validate_resp(ch->usb, response_pkt, min_response_len);
	if (ret)
		goto out_unlock;

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: RX rx_len=%d (success)\n", __func__, ch->ch_id,
		le16_to_cpu(response_pkt->hdr.length));

	return 0;

out_unlock:
	clear_bit(LSTP_BUFFER_LOCK_BIT, &ch->resp_buffer_lock);
	return ret;
}

/**
 * lstp_unlock_resp_buffer() - Release rx buffer lock after successful lstp_recv_resp_helper().
 * @ch: LSTP channel
 *
 * Only call this after lstp_recv_resp_helper() returns 0 (success).
 * On error, the lock is already released internally.
 */
void lstp_unlock_resp_buffer(struct lstp_channel *ch)
{
	clear_bit(LSTP_BUFFER_LOCK_BIT, &ch->resp_buffer_lock);
}

/*******************************************************************************
 * Driver Registration
 ******************************************************************************/

static const struct usb_device_id lstp_id_table[] = {
	{ USB_VENDOR_AND_INTERFACE_INFO(0x0955, 0xFF, 0x3F, LSTP_VERSION) },
	{}
};
MODULE_DEVICE_TABLE(usb, lstp_id_table);

static struct usb_driver lstp_usb_driver = {
	.name = "lstp",
	.probe = lstp_probe,
	.disconnect = lstp_disconnect,
	.id_table = lstp_id_table,
};

LSTP_SUBSYS_DECLARE(spi);
LSTP_SUBSYS_DECLARE(gpio);
LSTP_SUBSYS_DECLARE(i2c);
LSTP_SUBSYS_DECLARE(uart);
LSTP_SUBSYS_DECLARE(ipmi);

/* clang-format off */
static const struct lstp_subsys *const lstp_subsystems[] = {
	LSTP_SUBSYS_REF(spi),
	LSTP_SUBSYS_REF(gpio),
	LSTP_SUBSYS_REF(i2c),
	LSTP_SUBSYS_REF(uart),
	LSTP_SUBSYS_REF(ipmi),
};

/* clang-format on */

const struct lstp_subsys *lstp_subsys_by_channel_type(u8 channel_type)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lstp_subsystems); i++)
		if (lstp_subsystems[i]->channel_type == channel_type)
			return lstp_subsystems[i];
	return NULL;
}

static int __init lstp_module_init(void)
{
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(lstp_subsystems); i++) {
		ret = lstp_subsystems[i]->init ? lstp_subsystems[i]->init() : 0;
		if (ret) {
			pr_err("lstp: subsys %s init failed: %pe\n", lstp_subsystems[i]->name,
			       ERR_PTR(ret));
			goto unwind_subsys;
		}
	}

	ret = usb_register(&lstp_usb_driver);
	if (ret)
		goto unwind_subsys;

	return 0;

unwind_subsys:
	while (i-- > 0)
		if (lstp_subsystems[i]->exit)
			lstp_subsystems[i]->exit();
	return ret;
}

static void __exit lstp_module_exit(void)
{
	size_t i = ARRAY_SIZE(lstp_subsystems);

	usb_deregister(&lstp_usb_driver);

	while (i-- > 0)
		if (lstp_subsystems[i]->exit)
			lstp_subsystems[i]->exit();
}

module_init(lstp_module_init);
module_exit(lstp_module_exit);

bool lstp_auto_bind_spidev = IS_ENABLED(CONFIG_USB_LSTP_SPI_SPIDEV);
module_param_named(auto_bind_spidev, lstp_auto_bind_spidev, bool, 0444);
/* clang-format off */
MODULE_PARM_DESC(auto_bind_spidev, "Auto-create spidev devices on SPI channels without firmware nodes (default: CONFIG_USB_LSTP_SPI_SPIDEV)");
/* clang-format on */

MODULE_DESCRIPTION("LSTP USB device driver");
MODULE_LICENSE("GPL");
