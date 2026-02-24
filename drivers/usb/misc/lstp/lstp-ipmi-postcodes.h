/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IPMI postcodes support for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef __LSTP_IPMI_POSTCODES_H
#define __LSTP_IPMI_POSTCODES_H

#include <linux/types.h>

#ifdef CONFIG_SEPARATE_LSTP_IPMI_POSTCODES

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/uaccess.h>

/*
 * Postcode message format (OEM Group Extension):
 * Byte 0: netfn/lun (netfn 0x2C = Group Extension)
 * Byte 1: cmd (0x02 = OEM command)
 * Byte 2: group (0xAE = OEM group identifier)
 * Bytes 3-11: 9 bytes of postcode data
 */
#define LSTP_IPMI_POST_NETFN 0x2C
#define LSTP_IPMI_POST_CMD 0x02
#define LSTP_IPMI_POST_GROUP 0xAE
#define LSTP_IPMI_POST_MIN_LEN 3 /* netfn + cmd + group */
#define LSTP_IPMI_POST_CODE_SIZE 9 /* postcode data length */
#define LSTP_IPMI_POST_CODE_OFFSET 3 /* offset to postcode data */
#define LSTP_IPMI_POST_BUFFER_SIZE 1024

/**
 * struct lstp_ipmi_postcodes - Postcodes device context
 * @miscdev: Misc device for userspace access
 * @lock: Spinlock protecting FIFO and running flag
 * @wait_queue: Wait queue for readers
 * @fifo: FIFO buffer for postcode data
 * @running: Exclusive access flag
 * @initialized: Set to true after successful initialization
 */
struct lstp_ipmi_postcodes {
	struct miscdevice miscdev;
	spinlock_t lock; /* Protects FIFO and running flag */
	wait_queue_head_t wait_queue;
	struct kfifo fifo;
	bool running;
	bool initialized;
};

/* Helper to get postcodes context from file */
static inline struct lstp_ipmi_postcodes *lstp_ipmi_postcodes_to_ctx(struct file *file)
{
	return container_of(file->private_data, struct lstp_ipmi_postcodes, miscdev);
}

static int lstp_ipmi_postcodes_open(struct inode *inode, struct file *file)
{
	struct lstp_ipmi_postcodes *post = lstp_ipmi_postcodes_to_ctx(file);
	int ret = 0;

	spin_lock_irq(&post->lock);
	if (!post->running)
		post->running = true;
	else
		ret = -EBUSY;
	spin_unlock_irq(&post->lock);

	return ret;
}

static ssize_t lstp_ipmi_postcodes_read(struct file *file, char __user *buf, size_t count,
					loff_t *ppos)
{
	struct lstp_ipmi_postcodes *post = lstp_ipmi_postcodes_to_ctx(file);
	u8 tmp_buf[LSTP_IPMI_POST_CODE_SIZE];
	unsigned long flags;
	unsigned int copied;
	ssize_t ret;

	if (kfifo_is_empty(&post->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(post->wait_queue, !kfifo_is_empty(&post->fifo));
		if (ret == -ERESTARTSYS)
			return ret;
	}

	/*
	 * Copy to kernel buffer under spinlock, then copy to user without lock.
	 * This avoids calling copy_to_user() with spinlock held (which can sleep
	 * on page fault).
	 */
	spin_lock_irqsave(&post->lock, flags);
	copied = kfifo_out(&post->fifo, tmp_buf, min_t(size_t, count, LSTP_IPMI_POST_CODE_SIZE));
	spin_unlock_irqrestore(&post->lock, flags);

	if (copied == 0)
		return -EAGAIN;

	if (copy_to_user(buf, tmp_buf, copied))
		return -EFAULT;

	return copied;
}

static int lstp_ipmi_postcodes_release(struct inode *inode, struct file *file)
{
	struct lstp_ipmi_postcodes *post = lstp_ipmi_postcodes_to_ctx(file);

	spin_lock_irq(&post->lock);
	post->running = false;
	spin_unlock_irq(&post->lock);

	return 0;
}

static __poll_t lstp_ipmi_postcodes_poll(struct file *file, poll_table *wait)
{
	struct lstp_ipmi_postcodes *post = lstp_ipmi_postcodes_to_ctx(file);

	poll_wait(file, &post->wait_queue, wait);
	if (!kfifo_is_empty(&post->fifo))
		return POLLIN | POLLRDNORM;

	return 0;
}

static const struct file_operations lstp_ipmi_postcodes_fops = {
	.owner = THIS_MODULE,
	.open = lstp_ipmi_postcodes_open,
	.read = lstp_ipmi_postcodes_read,
	.release = lstp_ipmi_postcodes_release,
	.poll = lstp_ipmi_postcodes_poll,
};

/* Devres cleanup callbacks */
static void lstp_ipmi_postcodes_free_fifo(void *data)
{
	struct lstp_ipmi_postcodes *post = data;

	kfifo_free(&post->fifo);
}

static void lstp_ipmi_postcodes_deregister(void *data)
{
	struct lstp_ipmi_postcodes *post = data;

	misc_deregister(&post->miscdev);
}

/**
 * lstp_ipmi_postcodes_init - Initialize postcodes support
 * @post: Postcodes context to initialize
 * @dev: Device for devres management and naming
 * @base_name: Base device name (will be suffixed with "-postcodes")
 *
 * Initializes the postcodes context with FIFO, spinlock, and wait queue.
 * Must be called after the base device name is known.
 *
 * Context: Process context.
 *
 * Return: 0 on success, negative errno on failure.
 */
static inline int lstp_ipmi_postcodes_init(struct lstp_ipmi_postcodes *post, struct device *dev,
					   const char *base_name)
{
	int ret;

	memset(post, 0, sizeof(*post));

	ret = kfifo_alloc(&post->fifo, LSTP_IPMI_POST_BUFFER_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, lstp_ipmi_postcodes_free_fifo, post);
	if (ret)
		return ret;

	spin_lock_init(&post->lock);
	init_waitqueue_head(&post->wait_queue);
	post->running = false;

	/* Generate postcodes device name from base name */
	post->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-postcodes", base_name);
	if (!post->miscdev.name)
		return -ENOMEM;

	post->miscdev.minor = MISC_DYNAMIC_MINOR;
	post->miscdev.fops = &lstp_ipmi_postcodes_fops;
	post->miscdev.parent = dev;

	post->initialized = true;

	return 0;
}

/**
 * lstp_ipmi_postcodes_start - Register the postcodes misc device
 * @post: Postcodes context
 * @dev: Device for error reporting
 *
 * Registers the postcodes misc device to make it available to userspace.
 *
 * Context: Process context.
 *
 * Return: 0 on success, negative errno on failure.
 */
static inline int lstp_ipmi_postcodes_start(struct lstp_ipmi_postcodes *post, struct device *dev)
{
	int ret;

	if (!post->initialized)
		return -EINVAL;

	ret = misc_register(&post->miscdev);
	if (ret) {
		dev_err(dev, "Could not register postcodes device (%d)\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, lstp_ipmi_postcodes_deregister, post);
	if (ret)
		return ret;

	return 0;
}

/**
 * lstp_ipmi_postcodes_is_postcode - Check if IPMI message is a postcode
 * @payload: IPMI message payload
 * @len: Payload length
 *
 * Checks if the IPMI message matches the OEM Group Extension postcode
 * signature (netfn=0x2C, cmd=0x02, group=0xAE).
 *
 * Context: Any context (lock-free, no side effects).
 *
 * Return: true if message is a postcode, false otherwise.
 */
static inline bool lstp_ipmi_postcodes_is_postcode(const u8 *payload, u16 len)
{
	if (len < LSTP_IPMI_POST_MIN_LEN)
		return false;

	return ((payload[0] >> 2) == LSTP_IPMI_POST_NETFN && payload[1] == LSTP_IPMI_POST_CMD &&
		payload[2] == LSTP_IPMI_POST_GROUP);
}

/**
 * lstp_ipmi_postcodes_send - Queue a postcode message
 * @post: Postcodes context
 * @payload: IPMI message payload containing postcode data
 * @len: Payload length
 *
 * Extracts postcode data from the IPMI payload and queues it to the FIFO.
 * The postcode is POST_CODE_SIZE bytes starting at POST_CODE_OFFSET.
 *
 * Context: Interrupt context safe. Acquires post->lock with irqsave.
 */
static inline void lstp_ipmi_postcodes_send(struct lstp_ipmi_postcodes *post, const u8 *payload,
					    u16 len)
{
	unsigned long flags;
	unsigned int rc;

	if (!post->initialized)
		return;

	if (len < (LSTP_IPMI_POST_CODE_OFFSET + LSTP_IPMI_POST_CODE_SIZE))
		return;

	spin_lock_irqsave(&post->lock, flags);

	/* Reset FIFO if there's not enough space */
	if ((LSTP_IPMI_POST_BUFFER_SIZE - kfifo_len(&post->fifo)) < LSTP_IPMI_POST_CODE_SIZE)
		kfifo_reset(&post->fifo);

	rc = kfifo_in(&post->fifo, &payload[LSTP_IPMI_POST_CODE_OFFSET], LSTP_IPMI_POST_CODE_SIZE);
	if (rc != LSTP_IPMI_POST_CODE_SIZE) {
		kfifo_reset(&post->fifo);
		spin_unlock_irqrestore(&post->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&post->lock, flags);

	wake_up_all(&post->wait_queue);
}

#else /* !CONFIG_SEPARATE_LSTP_IPMI_POSTCODES */

/* Stub implementation when CONFIG_SEPARATE_LSTP_IPMI_POSTCODES is not defined */

struct lstp_ipmi_postcodes {
	char __dummy; /* Avoid empty struct warnings */
};

static inline int lstp_ipmi_postcodes_init(struct lstp_ipmi_postcodes *, struct device *,
					   const char *)
{
	return 0;
}

static inline int lstp_ipmi_postcodes_start(struct lstp_ipmi_postcodes *, struct device *)
{
	return 0;
}

static inline bool lstp_ipmi_postcodes_is_postcode(const u8 *, u16)
{
	return false;
}

static inline void lstp_ipmi_postcodes_send(struct lstp_ipmi_postcodes *, const u8 *, u16)
{
}

#endif /* CONFIG_SEPARATE_LSTP_IPMI_POSTCODES */

#endif /* __LSTP_IPMI_POSTCODES_H */
