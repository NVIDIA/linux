// SPDX-License-Identifier: GPL-2.0-only
/*
 * LSTP USB interface driver.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/module.h>

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
static void lstp_channel_kobj_release(struct kobject *kobj);
static ssize_t lstp_channel_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf);
static ssize_t lstp_channel_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
					 const char *buf, size_t count);

static struct kobj_attribute lstp_enable_attr =
	__ATTR(enable, 0644, lstp_channel_enable_show, lstp_channel_enable_store);

static struct attribute *lstp_channel_attrs[] = {
	&lstp_enable_attr.attr,
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
	[LSTP_NOT_SUPP] = -ENOTSUPP,
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
 * @dev:                  Device structure for error reporting
 * @rx_pkt:               Received packet to validate
 * @expected_payload_len: Expected payload length, or LSTP_ANY_RX_LEN to accept any length
 *
 * Assumes the packet has already been validated with lstp_validate_rx_pkt().
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_validate_resp(struct lstp_usb *dev, struct lstp_packet *rx_pkt,
		       size_t expected_payload_len)
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

	/* Validate payload length matches expectation (if specified) */
	payload_len = le16_to_cpu(rx_hdr->length);
	if (expected_payload_len != LSTP_ANY_RX_LEN && payload_len != expected_payload_len) {
		dev_err(&dev->intf->dev,
			"%s: ch_%u: Unexpected payload length (expected %zu, got %zu)\n", __func__,
			ch_id, expected_payload_len, payload_len);
		return -EIO;
	}

	return 0;
}

/**
 * lstp_ch0_read() - Read channel configuration from management channel. (blocking)
 * @dev:    LSTP USB device structure
 * @ch_id:  Channel ID to read configuration for
 * @offset: Offset into the configuration data
 * @length: Number of bytes to read (or LSTP_READ_LEN_ALL)
 *
 * Sends a READ_CONFIG command to the Management channel (ch0) to retrieve
 * the configuration data for the specified channel.
 *
 * The response data is stored in dev->rx_buf. Caller must ensure exclusive access.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_ch0_read(struct lstp_usb *dev, u8 ch_id, u16 offset, u16 length)
{
	int ret;
	int actual_length;
	struct lstp_packet *tx_pkt;
	struct lstp_packet *rx_pkt;
	union lstp_ch0_req_payload *ch0_req;

	/* Allocate request packet */
	tx_pkt = kzalloc(sizeof(*tx_pkt) + sizeof(ch0_req->read), GFP_KERNEL);
	if (!tx_pkt)
		return -ENOMEM;

	ch0_req = (union lstp_ch0_req_payload *)tx_pkt->payload;

	/* Build request */
	tx_pkt->hdr.ch_id = LSTP_CHANNEL_TYPE_MGMT;
	tx_pkt->hdr.cmd = SET_U8_BYTE(LSTP_CH0_CMD_READ_CONFIG, 0);
	tx_pkt->hdr.length = cpu_to_le16(sizeof(ch0_req->read));
	ch0_req->read.ch_id = ch_id;
	ch0_req->read.offset = cpu_to_le16(offset);
	ch0_req->read.length = cpu_to_le16(length);

	/* Send request */
	ret = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, dev->bulk_out_ep), tx_pkt,
			   sizeof(struct lstp_header) + sizeof(ch0_req->read), NULL,
			   LSTP_USB_REQUEST_TIMEOUT_MS);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: Could not send request for ch_%d (%d)\n",
			__func__, ch_id, ret);
		goto out_free;
	}

	/* Receive response */
	ret = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_ep), dev->rx_buf,
			   dev->bulk_rx_size, &actual_length, LSTP_USB_RESPONSE_TIMEOUT_MS);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: Could not receive response for ch_%d (%d)\n",
			__func__, ch_id, ret);
		goto out_free;
	}

	/* Validate response */
	rx_pkt = (struct lstp_packet *)dev->rx_buf;
	ret = lstp_validate_rx_pkt(dev, rx_pkt, actual_length);
	if (ret)
		goto out_free;

	if (rx_pkt->hdr.ch_id != LSTP_CHANNEL_TYPE_MGMT) {
		dev_err(&dev->intf->dev, "%s: ch_0: Wrong channel ID (expected %u, got %u)\n",
			__func__, LSTP_CHANNEL_TYPE_MGMT, rx_pkt->hdr.ch_id);
		ret = -EIO;
		goto out_free;
	}

	if (GET_BIT_7(rx_pkt->hdr.status) != 1) {
		dev_err(&dev->intf->dev, "%s: ch_0: Not a response packet (bit 7 = 0)\n", __func__);
		ret = -EIO;
		goto out_free;
	}

	/* Note: data is stored in dev->rx_buf */
out_free:
	kfree(tx_pkt);
	return ret;
}

/**
 * lstp_ch0_read_enabled_urb() - Read channel flags at runtime via URB.
 * @dev:          LSTP USB device structure
 * @ch_id:        Channel ID to query
 * @ch_flags_out: Output pointer for channel flags
 *
 * Reads the channel's enabled flag via READ_CONFIG command and copies to @ch_flags_out).
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
		dev_err(&dev->intf->dev, "%s: ch_0: READ_CONFIG for ch_%d failed (%d)\n", __func__,
			ch_id, ret);
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
		dev_err(&dev->intf->dev, "%s: ch_0: READ_CONFIG for ch_%d failed (%d)\n", __func__,
			ch_id, ret);
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
		dev_err(&dev->intf->dev, "%s: ch_0: WRITE_CONFIG for ch_%d failed (%d)\n", __func__,
			ch_id, ret);
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
	int actual_length;
	struct lstp_packet *tx_pkt;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)dev->rx_buf;

	tx_pkt = kzalloc(sizeof(*tx_pkt), GFP_KERNEL);
	if (!tx_pkt)
		return -ENOMEM;

	tx_pkt->hdr.ch_id = 0;
	tx_pkt->hdr.cmd = SET_U8_BYTE(LSTP_CH0_CMD_LOCK, 0);
	tx_pkt->hdr.length = cpu_to_le16(0);

	ret = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, dev->bulk_out_ep), tx_pkt,
			   sizeof(struct lstp_header), NULL, LSTP_USB_REQUEST_TIMEOUT_MS);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: Could not send lock request (%d)\n", __func__,
			ret);
		goto out_free;
	}

	ret = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_ep), dev->rx_buf,
			   dev->bulk_rx_size, &actual_length, LSTP_USB_RESPONSE_TIMEOUT_MS);
	if (ret) {
		dev_err(&dev->intf->dev, "%s: ch_0: Could not receive lock response (%d)\n",
			__func__, ret);
		goto out_free;
	}

	/* Validate response */
	ret = lstp_validate_rx_pkt(dev, rx_pkt, actual_length);
	if (ret)
		goto out_free;

	if (rx_pkt->hdr.ch_id != LSTP_CHANNEL_TYPE_MGMT) {
		dev_err(&dev->intf->dev, "%s: ch_0: Wrong channel ID (expected %u, got %u)\n",
			__func__, LSTP_CHANNEL_TYPE_MGMT, rx_pkt->hdr.ch_id);
		ret = -EIO;
		goto out_free;
	}

	if (GET_BIT_7(rx_pkt->hdr.status) != 1) {
		dev_err(&dev->intf->dev, "%s: ch_0: Not a response packet (bit 7 = 0)\n", __func__);
		ret = -EIO;
		goto out_free;
	}

	ret = lstp_validate_resp(dev, rx_pkt, 0);
	if (ret)
		goto out_free;

out_free:
	kfree(tx_pkt);
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
 * lstp_channel_kobj_release() - Release callback for channel kobjects.
 * @kobj: Kobject being released
 */
static void lstp_channel_kobj_release(struct kobject *kobj)
{
	/* Intentionally empty: channel memory is devm-managed */
}

/**
 * lstp_put_of_node() - Devres callback to release device tree node.
 * @data: Pointer to device_node
 */
static void lstp_put_of_node(void *data)
{
	of_node_put((struct device_node *)data);
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
 * lstp_create_sysfs_hierarchy() - Create lstp/channel sysfs directories.
 * @dev: LSTP USB device structure
 *
 * Creates the "lstp" and "lstp/channel" directories under the USB interface
 * device's sysfs directory. These serve as parent directories for per-channel
 * sysfs entries.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_create_sysfs_hierarchy(struct lstp_usb *dev)
{
	struct device *parent_dev = &dev->intf->dev;
	int ret;

	/* Create lstp directory */
	dev->lstp_kobj = kobject_create_and_add("lstp", &parent_dev->kobj);
	if (!dev->lstp_kobj)
		return -ENOMEM;

	ret = devm_add_action_or_reset(parent_dev, lstp_kobj_del_put, dev->lstp_kobj);
	if (ret)
		return ret;

	/* Create channel directory under lstp */
	dev->channel_kobj = kobject_create_and_add("channel", dev->lstp_kobj);
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

	if (!ch->child_dev)
		return 0;

	ret = sysfs_create_link(&ch->kobj, &ch->child_dev->kobj, "device");
	if (ret) {
		dev_err(parent_dev, "%s: ch_%d: Failed to create device symlink (%d)\n", __func__,
			ch->ch_id, ret);
		return ret;
	}

	return devm_add_action_or_reset(parent_dev, lstp_remove_device_link, &ch->kobj);
}

/**
 * lstp_create_channel_sysfs() - Create sysfs directory and attrs for channel.
 * @ch: LSTP channel
 *
 * Creates a directory "lstp/channel/N" (where N is the channel ID) with an
 * "enable" attribute for runtime access to the channel's enable status.
 * The attribute is read-write and queries/updates the device via the
 * management channel.
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
		dev_err(dev, "%s: Failed to create sysfs for ch_%d (%d)\n", __func__, ch->ch_id,
			ret);
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
 * lstp_find_channel_node() - Find channel node in device tree.
 * @usb_dev_node: USB device node
 * @intf_num:     Interface number
 * @channel_id:   Channel ID
 * @compatible:   Compatible string
 *
 * Finds a channel node in the device tree that matches the given parameters.
 * Traverses the device tree hierarchy to locate LSTP channel nodes.
 *
 * Expected DTS structure::
 *
 *   usb-device@X {                    // USB device node (usb_dev_node)
 *       compatible = "usbVVVV,PPPP";  // USB VID:PID (e.g., "usb0955,cf11")
 *       #address-cells = <1>;
 *       #size-cells = <0>;
 *
 *       interface@N {                 // USB interface node
 *           reg = <N>;                // bInterfaceNumber (intf_num)
 *           #address-cells = <1>;
 *           #size-cells = <0>;
 *
 *           channel@M {               // LSTP channel node (returned)
 *               reg = <M>;            // Channel ID (channel_id)
 *               compatible = "...";   // Must match 'compatible' parameter
 *                                     // e.g., "nv,lstp-spi", "nv,lstp-ipmi"
 *               // Channel-specific properties and child devices...
 *           };
 *       };
 *   };
 *
 * Note: DTS node addresses map to USB hierarchy:
 *
 * * device@X: USB device address/port number within the parent hub
 * * interface@N: USB interface number (bInterfaceNumber from USB descriptor)
 * * channel@M: LSTP-specific channel ID reported by device firmware
 *
 * Return: Device node on success, NULL on failure
 */
static struct device_node *lstp_find_channel_node(struct device_node *usb_dev_node, u8 intf_num,
						  u8 channel_id, const char *compatible)
{
	u32 reg;

	for_each_child_of_node_scoped(usb_dev_node, intf_node) {
		if (of_property_read_u32(intf_node, "reg", &reg) || reg != intf_num)
			continue;

		for_each_child_of_node_scoped(intf_node, ch_node) {
			if (of_property_read_u32(ch_node, "reg", &reg) || reg != channel_id)
				continue;
			if (!of_device_is_compatible(ch_node, compatible))
				continue;
			return of_node_get(ch_node);
		}
		break;
	}

	return NULL;
}

/**
 * lstp_init_channel_of_node() - Initialize channel's device tree node.
 * @ch:         LSTP channel to initialize
 * @compatible: Compatible string to match in device tree
 *
 * Finds and assigns the appropriate device tree node for this channel.
 * Registers a devm action to automatically release the node reference.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_init_channel_of_node(struct lstp_channel *ch, const char *compatible)
{
	ch->of_node = lstp_find_channel_node(ch->usb->udev->dev.of_node,
					     ch->usb->intf->cur_altsetting->desc.bInterfaceNumber,
					     ch->ch_id, compatible);
	if (!ch->of_node)
		return 0;

	return devm_add_action_or_reset(&ch->usb->intf->dev, lstp_put_of_node, ch->of_node);
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
	ch->rx_ready = false;
	ch->priv = NULL;
	dev->channels[ch_id] = ch;
	return ch;
}

/**
 * lstp_init_channels() - Discover and initialize all LSTP channels.
 * @dev: LSTP USB device structure
 *
 * Queries ch0 for protocol version and channel count, then initializes each
 * channel's type-specific structures. Does NOT register with Linux subsystems;
 * call lstp_start_channels() after RX URB setup.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_init_channels(struct lstp_usb *dev)
{
	int ret;
	int ch_id;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)dev->rx_buf;
	union lstp_ch0_resp_payload *ch0_resp;
	struct lstp_ch0_config *config;
	struct lstp_channel *ch0;
	u8 max_ch_id = 0;

	/* Create channel 0 (management channel) for URB-based runtime access */
	ch0 = lstp_create_channel(dev, 0);
	if (!ch0) {
		dev_err(&dev->intf->dev, "%s: Could not allocate channel 0\n", __func__);
		return -ENOMEM;
	}
	ch0->ch_type = LSTP_CHANNEL_TYPE_MGMT;

	/* Allocate rx_buf for management channel */
	ch0->resp_buf = devm_kzalloc(&dev->intf->dev, dev->bulk_rx_size, GFP_KERNEL);
	if (!ch0->resp_buf)
		return -ENOMEM;

	/* Get READ request for CH0 (using sync I/O since RX URB not active yet) */
	ret = lstp_ch0_read(dev, 0, 0, LSTP_READ_LEN_ALL);
	if (ret)
		return ret;

	/* Validate expected CH0 config size */
	ret = lstp_validate_resp(dev, rx_pkt,
				 sizeof(struct lstp_ch0_resp_read) +
					 sizeof(struct lstp_ch0_config));
	if (ret)
		return ret;

	/* Get config - already validated by lstp_validate_resp() */
	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	config = (struct lstp_ch0_config *)ch0_resp->read.ch_config;

	/* Parse READ response for CH0 */
	if (ch0_resp->read.ch_type != LSTP_CHANNEL_TYPE_MGMT) {
		dev_err(&dev->intf->dev, "%s: ch_0: Received wrong channel 0 type (got %d)\n",
			__func__, ch0_resp->read.ch_type);
		return -EINVAL;
	}

	/* Validate channel name */
	if (ch0_resp->read.ch_name[0] == '\0') {
		dev_err(&dev->intf->dev, "%s: ch_0: Invalid LSTP interface name\n", __func__);
		return -EINVAL;
	}
	strscpy(dev->lstp_intf_name, ch0_resp->read.ch_name, LSTP_CH_NAME_LEN);

	/* Validate LSTP version */
	if (config->lstp_version != LSTP_VERSION) {
		dev_err(&dev->intf->dev, "%s: ch_0: Invalid LSTP version\n", __func__);
		return -EINVAL;
	}
	dev->lstp_version = config->lstp_version;

	/* Validate channel count */
	max_ch_id = config->num_channels;
	if (max_ch_id == 0) {
		dev_err(&dev->intf->dev, "%s: ch_0: Invalid channel count %d (min 1, max 255)\n",
			__func__, max_ch_id);
		return -EINVAL;
	}
	dev->max_ch_id = max_ch_id;

	dev_info(&dev->intf->dev, "%s: LSTP v%d: device %s discovered with %d channels\n", __func__,
		 dev->lstp_version, ch0_resp->read.ch_name, max_ch_id);

	for (ch_id = 1; ch_id <= max_ch_id; ch_id++) {
		/* Create CH_i and get READ request */
		struct lstp_channel *ch = lstp_create_channel(dev, ch_id);

		if (!ch) {
			dev_err(&dev->intf->dev, "%s: Could not allocate channel %d\n", __func__,
				ch_id);
			return -ENOMEM;
		}
		ret = lstp_ch0_read(dev, ch_id, 0, LSTP_READ_LEN_ALL);
		if (ret)
			return ret;

		/* Parse READ response data and init type-specific structures */
		ch->ch_type = ch0_resp->read.ch_type;
		switch (ch0_resp->read.ch_type) {
		case LSTP_CHANNEL_TYPE_SPI:
			ret = lstp_init_channel_of_node(ch, "nv,lstp-spi");
			if (ret)
				return ret;
			ret = lstp_spi_init(ch);
			if (ret)
				return ret;
			break;
		case LSTP_CHANNEL_TYPE_I2C:
			ret = lstp_i2c_init(ch);
			if (ret)
				return ret;
			break;
		case LSTP_CHANNEL_TYPE_IPMI:
			ret = lstp_init_channel_of_node(ch, "nv,lstp-ipmi");
			if (ret)
				return ret;
			ret = lstp_ipmi_init(ch);
			if (ret)
				return ret;
			break;
		default:
			dev_warn(&dev->intf->dev, "%s: Channel %d has unsupported type %d\n",
				 __func__, ch_id, ch0_resp->read.ch_type);
			break;
		}
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
		if (!ch)
			continue;

		switch (ch->ch_type) {
		case LSTP_CHANNEL_TYPE_SPI:
			ret = lstp_spi_start(ch);
			if (ret)
				return ret;
			break;
		case LSTP_CHANNEL_TYPE_I2C:
			ret = lstp_i2c_start(ch);
			if (ret)
				return ret;
			break;
		case LSTP_CHANNEL_TYPE_IPMI:
			ret = lstp_ipmi_start(ch);
			if (ret)
				return ret;
			break;
		default:
			break;
		}
	}

	for (i = 1; i < LSTP_MAX_CHANNELS; i++) {
		ch = dev->channels[i];
		if (!ch)
			continue;

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
	dev->bulk_tx_size = usb_endpoint_maxp(ep_out);
	dev->bulk_rx_size = usb_endpoint_maxp(ep_in);

	/* Clamp to max size */
	if (dev->bulk_tx_size > LSTP_USB_EP_MAX_SIZE)
		dev->bulk_tx_size = LSTP_USB_EP_MAX_SIZE;
	if (dev->bulk_rx_size > LSTP_USB_EP_MAX_SIZE)
		dev->bulk_rx_size = LSTP_USB_EP_MAX_SIZE;

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

	ret = lstp_init_channels(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to initialize channels (%d)\n", __func__, ret);
		return ret;
	}

	ret = lstp_create_sysfs_hierarchy(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to create sysfs hierarchy (%d)\n", __func__, ret);
		return ret;
	}

	usb_fill_bulk_urb(dev->bulk_rx_urb, dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_ep),
			  dev->rx_buf, dev->bulk_rx_size, lstp_usb_rx_callback, dev);

	ret = usb_submit_urb(dev->bulk_rx_urb, GFP_KERNEL);
	if (ret) {
		dev_err(&intf->dev, "%s: Could not submit bulk RX URB (%d)\n", __func__, ret);
		return ret;
	}

	ret = lstp_start_channels(dev);
	if (ret) {
		dev_err(&intf->dev, "%s: Failed to start channels (%d)\n", __func__, ret);
		return ret;
	}

	dev_info(&intf->dev, "%s: LSTP device initialized successfully\n", __func__);
	return 0;
}

/**
 * lstp_disconnect() - USB driver disconnect function.
 * @intf: USB interface being disconnected
 */
static void lstp_disconnect(struct usb_interface *intf)
{
	dev_info(&intf->dev, "%s: LSTP device disconnected\n", __func__);
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
		dev_warn(&ch->usb->intf->dev, "%s: ch_%d: TX URB error (%d)\n", __func__, ch->ch_id,
			 urb->status);
	}
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

	/* URB killed during disconnect */
	if (urb->status == -ENOENT || urb->status == -ECONNRESET || urb->status == -ESHUTDOWN) {
		for (int i = 0; i < LSTP_MAX_CHANNELS; i++) {
			if (dev->channels[i])
				wake_up_all(&dev->channels[i]->rx_wq);
		}
		return;
	}

	if (urb->status) {
		dev_err(&dev->intf->dev, "%s: RX URB error (%d)\n", __func__, urb->status);
		goto resubmit;
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
			smp_store_release(&ch->rx_ready, true); /* Unblocks recv_resp_helper() */
			wake_up_all(&ch->rx_wq);
		}
	} else {
		dev_err_ratelimited(&dev->intf->dev,
				    "%s: ch_%d: Dropping packet - channel misconfigured\n",
				    __func__, ch_id);
	}

resubmit:
	/* Resubmit URB for continuous reception */
	usb_submit_urb(urb, GFP_ATOMIC);
}

/**
 * lstp_recv_resp_helper() - Send request and wait for response.
 * @ch:           LSTP channel
 * @cmd:          Command byte (bit 7 set automatically)
 * @request_len:  Payload length in tx_buf
 * @response_len: Expected response length, or LSTP_ANY_RX_LEN
 *
 * Caller prepares tx_buf->payload, this sends and waits. Response in rx_buf.
 * Will return after a timeout.
 *
 * Lock contract:
 *
 * * On success (return 0): caller MUST call lstp_unlock_resp_buffer()
 * * On error (return < 0): lock is released internally, caller must NOT unlock
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_recv_resp_helper(struct lstp_channel *ch, u8 cmd, u16 request_len, u16 response_len)
{
	int ret = 0;

	/* Validate channel and device */
	if (!ch || !ch->usb || !ch->usb->udev || !ch->tx_buf || !ch->resp_buf)
		return -ENODEV;

	/* Validate Request and Response packet sizes */
	if (sizeof(struct lstp_header) + request_len > ch->usb->bulk_tx_size ||
	    (response_len != LSTP_ANY_RX_LEN &&
	     sizeof(struct lstp_header) + response_len > ch->usb->bulk_rx_size))
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
			__func__, ch->ch_id, cmd, request_len, response_len, request_len,
			request_pkt->payload);
	} else {
		dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: TX cmd=0x%02x, tx_len=%d, rx_len=%d\n",
			__func__, ch->ch_id, cmd, request_len, response_len);
	}

	usb_fill_bulk_urb(ch->bulk_tx_urb, ch->usb->udev,
			  usb_sndbulkpipe(ch->usb->udev, ch->usb->bulk_out_ep), ch->tx_buf,
			  sizeof(struct lstp_header) + request_len, lstp_usb_tx_callback, ch);

	ch->rx_ready = false;

	ret = usb_submit_urb(ch->bulk_tx_urb, GFP_KERNEL);

	if (ret)
		goto out_unlock;
	/* Unblocks recv_resp_helper() */
	bool got_response = wait_event_timeout(ch->rx_wq, smp_load_acquire(&ch->rx_ready),
					       msecs_to_jiffies(LSTP_USB_RESPONSE_TIMEOUT_MS));
	if (!got_response) {
		usb_kill_urb(ch->bulk_tx_urb);
		if (!usb_get_intfdata(ch->usb->intf)) {
			ret = -ENODEV;
		} else {
			/*
			 * Device is not allowed to take longer than LSTP_USB_RESPONSE_TIMEOUT_MS to
			 * respond. Timeout indicates device hang.
			 */
			dev_err(&ch->usb->intf->dev,
				"%s: ch_%d: Response timeout - channel disabled\n", __func__,
				ch->ch_id);
			ret = -ETIMEDOUT;
		}
		goto out_unlock;
	}

	ret = lstp_validate_resp(ch->usb, response_pkt, response_len);
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
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0955, 0xcf11, 0xFF, 0x3F, LSTP_VERSION) },
	{}
};
MODULE_DEVICE_TABLE(usb, lstp_id_table);

static struct usb_driver lstp_usb_driver = {
	.name = "lstp",
	.probe = lstp_probe,
	.disconnect = lstp_disconnect,
	.id_table = lstp_id_table,
};

module_usb_driver(lstp_usb_driver);

MODULE_LICENSE("GPL");
