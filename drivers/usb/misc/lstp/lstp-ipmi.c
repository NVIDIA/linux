// SPDX-License-Identifier: GPL-2.0-only
/*
 * IPMI driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/idr.h>

#include "lstp-main.h"
#include "lstp-ipmi-postcodes.h"

#define BUFFER_SIZE 1024

static DEFINE_IDA(lstp_ipmi_ida);

/* IPMI message constants and structures */

#define IPMI_LSTP_PAYLOAD_MAX 254
#define IPMI_MIN_MSG_LEN 2 /* netfn/lun + cmd */

struct ipmi_lstp_msg_header {
	unsigned int len;
	u8 msg_num;
} __packed;

struct ipmi_lstp_msg {
	struct ipmi_lstp_msg_header header;
	__u8 payload[IPMI_LSTP_PAYLOAD_MAX];
};

struct lstp_ipmi_ctx {
	struct lstp_channel *ch;
	struct miscdevice miscdev;
	int ida_id;
	bool running;
	spinlock_t lock; /* Protects running, fifo, msg_count */
	wait_queue_head_t req_wq;
	struct ipmi_lstp_msg request;
	struct kfifo fifo;
	u8 msg_count;
	struct lstp_ipmi_postcodes postcodes;
};

enum lstp_ipmi_cmd {
	LSTP_IPMI_CMD_MESSAGE = 0x00,
};

/*******************************************************************************
 * Helper functions
 ******************************************************************************/

/**
 * to_ctx() - Get IPMI context from file pointer.
 * @file: File pointer from file operations
 *
 * Retrieves the lstp_ipmi_ctx structure from the file's private_data
 * using container_of on the embedded miscdevice.
 *
 * Context: Any context.
 *
 * Return: Pointer to the lstp_ipmi_ctx structure
 */
static inline struct lstp_ipmi_ctx *to_ctx(struct file *file)
{
	return container_of(file->private_data, struct lstp_ipmi_ctx, miscdev);
}

/*******************************************************************************
 * Character device file operations
 ******************************************************************************/

/**
 * lstp_ipmi_open() - Open the IPMI character device.
 * @inode: Inode structure for the device
 * @file:  File pointer for the opened device
 *
 * Opens the IPMI character device for exclusive access. Only one process
 * can have the device open at a time. If already open, returns -EBUSY.
 *
 * Context: Process context. Acquires ctx->lock.
 *
 * Return: 0 on success, -EBUSY if device is already open
 */
static int lstp_ipmi_open(struct inode *inode, struct file *file)
{
	struct lstp_ipmi_ctx *ctx = to_ctx(file);
	int ret = 0;

	spin_lock_irq(&ctx->lock);
	if (!ctx->running)
		ctx->running = true;
	else
		ret = -EBUSY;
	spin_unlock_irq(&ctx->lock);

	return ret;
}

/**
 * lstp_ipmi_read() - Read IPMI request from the device.
 * @file:  File pointer for the device
 * @buf:   User-space buffer to copy data to
 * @count: Maximum number of bytes to read
 * @ppos:  File position (unused)
 *
 * Reads incoming IPMI requests from the FIFO buffer. If no data is available
 * and O_NONBLOCK is not set, blocks until data arrives. The data format
 * consists of an ipmi_lstp_msg_header followed by the IPMI payload.
 *
 * Context: Process context. May sleep waiting for data. Acquires ctx->lock.
 *
 * Return: Number of bytes read on success, negative errno on failure.
 *         Returns -EAGAIN if O_NONBLOCK is set and no data is available.
 *         Returns -ERESTARTSYS if interrupted by a signal.
 */
static ssize_t lstp_ipmi_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct lstp_ipmi_ctx *ctx = to_ctx(file);
	unsigned long flags;
	unsigned int count_out;
	ssize_t ret = 0;

	if (kfifo_is_empty(&ctx->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(ctx->req_wq, !kfifo_is_empty(&ctx->fifo));
		if (ret == -ERESTARTSYS)
			return ret;
	}
	spin_lock_irqsave(&ctx->lock, flags);
	ret = kfifo_to_user(&ctx->fifo, buf, count, &count_out);
	spin_unlock_irqrestore(&ctx->lock, flags);
	return (ret < 0) ? ret : count_out;
}

/**
 * lstp_ipmi_write() - Write IPMI response to the device.
 * @file:  File pointer for the device
 * @buf:   User-space buffer containing the response
 * @count: Number of bytes to write
 * @ppos:  File position (unused)
 *
 * Sends an IPMI response message to the remote endpoint via the LSTP channel.
 * The message must include an ipmi_lstp_msg_header followed by the payload.
 * The msg_num in the header must match the current expected message number;
 * if it doesn't match, the write is silently ignored (returns count).
 *
 * Context: Process context. Takes ch->tx_mutex. Performs blocking USB I/O.
 *
 * Return: Number of bytes written on success, negative errno on failure.
 *         Returns -EINVAL if message size is invalid.
 *         Returns -EFAULT if copy from user fails.
 */
static ssize_t lstp_ipmi_write(struct file *file, const char __user *buf, size_t count,
			       loff_t *ppos)
{
	struct lstp_ipmi_ctx *ctx = to_ctx(file);
	struct lstp_channel *ch = ctx->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct ipmi_lstp_msg msg;
	ssize_t ret;

	if (count < sizeof(struct ipmi_lstp_msg_header) || count > sizeof(struct ipmi_lstp_msg))
		return -EINVAL;

	if (copy_from_user(&msg, buf, count))
		return -EFAULT;

	if (msg.header.msg_num != ctx->msg_count)
		return count;

	if (!msg.header.len || msg.header.len > IPMI_LSTP_PAYLOAD_MAX ||
	    count < sizeof(struct ipmi_lstp_msg_header) + msg.header.len)
		return -EINVAL;

	mutex_lock(&ch->tx_mutex);

	tx_pkt->hdr.ch_id = ch->ch_id;
	tx_pkt->hdr.cmd = LSTP_IPMI_CMD_MESSAGE;
	tx_pkt->hdr.length = cpu_to_le16(msg.header.len);
	memcpy(tx_pkt->payload, msg.payload, msg.header.len);

	ret = usb_bulk_msg(ch->usb->udev, usb_sndbulkpipe(ch->usb->udev, ch->usb->bulk_out_ep),
			   ch->tx_buf, sizeof(tx_pkt->hdr) + msg.header.len, NULL,
			   LSTP_USB_REQUEST_TIMEOUT_MS);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not forward response (%zd)\n",
			__func__, ch->ch_id, ret);
	}

	mutex_unlock(&ch->tx_mutex);

	return (ret < 0) ? ret : count;
}

/**
 * lstp_ipmi_release() - Release the IPMI character device.
 * @inode: Inode structure for the device
 * @file:  File pointer for the device
 *
 * Releases the IPMI character device, allowing another process to open it.
 * Clears the running flag to indicate the device is no longer in use.
 *
 * Context: Process context. Acquires ctx->lock.
 *
 * Return: Always returns 0
 */
static int lstp_ipmi_release(struct inode *inode, struct file *file)
{
	struct lstp_ipmi_ctx *ctx = to_ctx(file);

	spin_lock_irq(&ctx->lock);
	ctx->running = false;
	spin_unlock_irq(&ctx->lock);

	return 0;
}

/**
 * lstp_ipmi_poll() - Poll for readable data on the device.
 * @file: File pointer for the device
 * @wait: Poll table to register wait queue with
 *
 * Implements poll/select/epoll support for the IPMI device. Returns
 * POLLIN | POLLRDNORM when data is available to read from the FIFO.
 *
 * Context: Process context. May be called with interrupts disabled.
 *
 * Return: Poll mask indicating readable state
 */
static __poll_t lstp_ipmi_poll(struct file *file, poll_table *wait)
{
	struct lstp_ipmi_ctx *ctx = to_ctx(file);

	poll_wait(file, &ctx->req_wq, wait);
	if (!kfifo_is_empty(&ctx->fifo))
		return POLLIN | POLLRDNORM;

	return 0;
}

static const struct file_operations lstp_ipmi_fops = {
	.owner = THIS_MODULE,
	.open = lstp_ipmi_open,
	.read = lstp_ipmi_read,
	.write = lstp_ipmi_write,
	.release = lstp_ipmi_release,
	.poll = lstp_ipmi_poll,
};

/*******************************************************************************
 * Devres action callbacks
 ******************************************************************************/

/**
 * lstp_ipmi_free_fifo() - Free the IPMI FIFO buffer.
 * @data: Pointer to lstp_ipmi_ctx structure
 *
 * Devres action callback to free the kfifo buffer when the device is removed.
 *
 * Context: Process context.
 */
static void lstp_ipmi_free_fifo(void *data)
{
	struct lstp_ipmi_ctx *ctx = (struct lstp_ipmi_ctx *)data;

	kfifo_free(&ctx->fifo);
}

/**
 * lstp_ipmi_free_ida() - Free the IDA-allocated device ID.
 * @data: Pointer to the device ID integer
 *
 * Devres action callback to release the IDA-allocated device ID when the
 * device is removed.
 *
 * Context: Process context.
 */
static void lstp_ipmi_free_ida(void *data)
{
	int *dev_id = data;

	ida_free(&lstp_ipmi_ida, *dev_id);
}

/**
 * lstp_ipmi_deregister() - Deregister the misc device.
 * @data: Pointer to lstp_ipmi_ctx structure
 *
 * Devres action callback to deregister the miscdevice when the LSTP device
 * is removed.
 *
 * Context: Process context.
 */
static void lstp_ipmi_deregister(void *data)
{
	struct lstp_ipmi_ctx *ctx = (struct lstp_ipmi_ctx *)data;

	misc_deregister(&ctx->miscdev);
}

/*******************************************************************************
 * RX callback for unsolicited IPMI requests
 ******************************************************************************/

/**
 * lstp_ipmi_irq_callback() - Handle incoming IPMI requests from the device.
 * @ch: LSTP channel that received the data
 *
 * Callback invoked when an unsolicited IPMI request is received from the
 * remote endpoint. Reads the packet from ch->irq_buf, validates the
 * message length, filters out postcode messages (netfn=0x2c, cmd=0x02,
 * group=0xAE), and queues valid requests to the FIFO for userspace to read.
 *
 * If the FIFO is full, it is reset to make room for the new message. This
 * ensures that stale messages don't block new incoming requests.
 *
 * Context: Interrupt context (called from USB RX completion). Acquires
 *          ctx->lock with irqsave. Must not sleep.
 */
static void lstp_ipmi_irq_callback(struct lstp_channel *ch)
{
	struct lstp_ipmi_ctx *ctx = (struct lstp_ipmi_ctx *)ch->priv;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->irq_buf;
	u16 rx_len = le16_to_cpu(rx_pkt->hdr.length);
	unsigned long flags = 0;
	int retval = 0;

	if (rx_len < IPMI_MIN_MSG_LEN || rx_len > IPMI_LSTP_PAYLOAD_MAX) {
		dev_warn(&ch->usb->intf->dev, "%s: ch_%d: Invalid IPMI request length %u\n",
			 __func__, ch->ch_id, rx_len);
		return;
	}

	if (lstp_ipmi_postcodes_is_postcode(rx_pkt->payload, rx_len)) {
		lstp_ipmi_postcodes_send(&ctx->postcodes, rx_pkt->payload, rx_len);
	} else if (kfifo_initialized(&ctx->fifo)) {
		ssize_t to_send = sizeof(struct ipmi_lstp_msg_header) + rx_len;

		if (to_send > BUFFER_SIZE)
			return;

		spin_lock_irqsave(&ctx->lock, flags);

		/* Reset FIFO if there's not enough space for the new message */
		if ((BUFFER_SIZE - kfifo_len(&ctx->fifo)) < to_send)
			kfifo_reset(&ctx->fifo);

		ctx->request.header.len = rx_len;
		ctx->request.header.msg_num = (++ctx->msg_count);

		retval = kfifo_in(&ctx->fifo, &ctx->request.header,
				  sizeof(struct ipmi_lstp_msg_header));
		if (retval != sizeof(struct ipmi_lstp_msg_header)) {
			kfifo_reset(&ctx->fifo);
			spin_unlock_irqrestore(&ctx->lock, flags);
			return;
		}
		to_send = min_t(ssize_t, rx_len, IPMI_LSTP_PAYLOAD_MAX);
		retval = kfifo_in(&ctx->fifo, rx_pkt->payload, to_send);
		if (retval != to_send) {
			kfifo_reset(&ctx->fifo);
			spin_unlock_irqrestore(&ctx->lock, flags);
			return;
		}

		spin_unlock_irqrestore(&ctx->lock, flags);
		wake_up_all(&ctx->req_wq);
	}
}

/*******************************************************************************
 * IPMI channel initialization and start
 ******************************************************************************/

/**
 * lstp_ipmi_init() - Initialize an LSTP IPMI channel.
 * @ch: LSTP channel to initialize as IPMI
 *
 * Allocates and initializes the IPMI context structure, FIFO buffer, and
 * prepares the miscdevice for the channel. The device name is either taken
 * from the device tree "label" property or auto-generated as "ipmi-lstpN".
 *
 * Expected device tree node structure (optional)::
 *
 *   channel@M {
 *       compatible = "nv,lstp-ipmi";
 *       reg = <M>;                  // Channel ID
 *       label = "ipmi-custom-name"; // Device name (optional)
 *                                   // If omitted, uses "ipmi-lstpN"
 *   };
 *
 * This function only allocates resources and sets up the RX callback.
 * Call lstp_ipmi_start() after the RX URB is active to register the
 * miscdevice and make it available to userspace.
 *
 * Context: Process context. Called during probe before RX URB is active.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_ipmi_init(struct lstp_channel *ch)
{
	int ret;
	struct lstp_ipmi_ctx *ctx;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->usb->rx_buf;
	const char *label;
	int dev_id;

	/* Validate expected IPMI config size */
	ret = lstp_validate_resp(ch->usb, rx_pkt, sizeof(struct lstp_ch0_resp_read));
	if (ret)
		return ret;

	/* Allocate memory */
	ch->irq_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->irq_buf)
		return -ENOMEM;

	ctx = devm_kzalloc(&ch->usb->intf->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ch->priv = ctx;

	ret = kfifo_alloc(&ctx->fifo, BUFFER_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&ch->usb->intf->dev, lstp_ipmi_free_fifo, ctx);
	if (ret)
		return ret;

	spin_lock_init(&ctx->lock);
	init_waitqueue_head(&ctx->req_wq);

	ctx->ch = ch;
	ctx->running = false;
	ctx->msg_count = 0;
	ctx->ida_id = -1;

	/* Get device name from device tree label or generate one */
	if (ctx->ch->of_node && !of_property_read_string(ctx->ch->of_node, "label", &label)) {
		ctx->miscdev.name = devm_kstrdup(&ch->usb->intf->dev, label, GFP_KERNEL);
	} else {
		dev_id = ida_alloc(&lstp_ipmi_ida, GFP_KERNEL);
		if (dev_id < 0)
			return dev_id;
		ctx->ida_id = dev_id;
		ret = devm_add_action_or_reset(&ch->usb->intf->dev, lstp_ipmi_free_ida,
					       &ctx->ida_id);
		if (ret)
			return ret;

		ctx->miscdev.name =
			devm_kasprintf(&ch->usb->intf->dev, GFP_KERNEL, "ipmi-lstp%d", dev_id);
	}
	if (!ctx->miscdev.name)
		return -ENOMEM;

	ctx->miscdev.minor = MISC_DYNAMIC_MINOR;
	ctx->miscdev.fops = &lstp_ipmi_fops;
	ctx->miscdev.parent = &ch->usb->intf->dev;

	/* Initialize postcodes support (name derived from miscdev.name) */
	ret = lstp_ipmi_postcodes_init(&ctx->postcodes, &ch->usb->intf->dev, ctx->miscdev.name);
	if (ret)
		return ret;

	ch->irq_callback = lstp_ipmi_irq_callback;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Initialized\n", __func__, ch->ch_id);
	return 0;
}

/**
 * lstp_ipmi_start() - Start an LSTP IPMI channel.
 * @ch: LSTP channel to start
 *
 * Registers the miscdevice to make the IPMI interface available to userspace.
 * This should be called after the RX URB is active so the channel can properly
 * receive unsolicited IPMI requests.
 *
 * Context: Process context. Called during probe after RX URB is active.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_ipmi_start(struct lstp_channel *ch)
{
	struct lstp_ipmi_ctx *ctx = (struct lstp_ipmi_ctx *)ch->priv;
	int ret;

	if (!ctx) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: IPMI context not initialized\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	ret = misc_register(&ctx->miscdev);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register miscdevice (%d)\n",
			__func__, ch->ch_id, ret);
		return ret;
	}

	ret = devm_add_action_or_reset(&ch->usb->intf->dev, lstp_ipmi_deregister, ctx);
	if (ret)
		return ret;

	/* Start postcodes device */
	ret = lstp_ipmi_postcodes_start(&ctx->postcodes, &ch->usb->intf->dev);
	if (ret)
		return ret;

	ch->child_dev = ctx->miscdev.this_device;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Started\n", __func__, ch->ch_id);
	return 0;
}
