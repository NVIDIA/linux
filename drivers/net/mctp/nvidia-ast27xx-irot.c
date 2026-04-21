// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * MCTP binding driver for NVIDIA IRoT on ASPEED AST27xx BMC.
 *
 * Communicates with the IRoT over a mailbox + shared-memory IPC channel
 * and exposes an MCTP netdev for the kernel MCTP stack.
 *
 * Device tree example:
 *
 *   / {
 *       mctpirot0 {
 *           compatible = "nvidia,ast27xx,irot";
 *           mboxes = <&mbox0 0>;
 *           mbox-names = "irot";
 *           status = "okay";
 *       };
 *   };
 *
 *   &mbox0 {
 *       status = "okay";
 *   };
 */

#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <uapi/linux/if_arp.h>

#include "nvidia_irot_ast27xx_msg_ns.h"

/*
 * Maximum number of MCTP packets queued between the netdev TX path and the
 * TX worker thread.  20 is intentionally oversized — each slot costs only
 * one sk_buff pointer — to avoid back-pressure under burst traffic.
 */
#define NVIDIA_IROT_QUEUE_SIZE 20

/* State shared between the mailbox callback and worker threads */
struct nvidia_irot_driver_shared_state {
	/* Shared memory (sizes are zero if not initialized; write-once) */
	struct nvidia_irot_message_data_buffers shmem;

	/* Pending MCTP read (command == 0 if none pending) */
	struct nvidia_irot_message pending_read;

	/* Pending MCTP write (command == 0 if none pending) */
	struct nvidia_irot_message pending_write;

	/* Received ping awaiting response (command == 0 if none) */
	struct nvidia_irot_message rx_ping;

	/*
	 * Duplicate command response.  When a response is dropped and the
	 * same command arrives again, the mailbox callback stages a response
	 * here instead of re-processing the command.  command == 0 if none.
	 */
	struct nvidia_irot_message duplicate_response;
};

/* State private to worker threads */
struct nvidia_irot_driver_worker_state {
	/* Uncacheable RAM from memremap for reading MCTP packets (write-once) */
	const void *rx_addr;

	/* Uncacheable RAM from memremap for writing MCTP packets (write-once) */
	void *tx_addr;

	/* Next counter for mailbox command output */
	u32 next_tx_counter;
};

/* State private to the mailbox callback (no locking needed) */
struct nvidia_irot_driver_mailbox_state {
	u32 prev_rx_counter;
	bool has_prev_rx_counter;
};

/**
 * struct nvidia_irot_driver - main driver state
 * @dev: back-pointer to the platform device (set once during probe)
 * @shared_spin: spinlock protecting @shared_state (must not sleep while held)
 * @shared_state: state shared between mailbox callback and workers
 * @worker_mutex: mutex protecting @worker_state
 * @worker_state: state used only by worker functions
 * @mailbox_state: state private to the mailbox RX callback
 * @probe_sem: signals probe that shared memory info has arrived
 * @rx_worker_wq: RX worker wait queue
 * @rx_work_ready: set to 1 when the RX worker has work
 * @tx_queue_ready_wq: woken when tx_queue is pushed to
 * @tx_shmem_ready_wq: woken when shared memory becomes available
 * @tx_shmem_ready: set to 1 when shared memory is ready
 * @mbox_client: mailbox client
 * @mbox_chan: mailbox channel
 * @rx_worker: RX worker thread
 * @tx_worker: TX worker thread
 * @netdev: MCTP network device
 * @tx_queue: packet queue from netdev to TX worker
 */
struct nvidia_irot_driver {
	struct device *dev;

	spinlock_t shared_spin;
	struct nvidia_irot_driver_shared_state shared_state;

	struct mutex worker_mutex;
	struct nvidia_irot_driver_worker_state worker_state;

	struct nvidia_irot_driver_mailbox_state mailbox_state;

	struct semaphore probe_sem;

	wait_queue_head_t rx_worker_wq;
	atomic_t rx_work_ready;

	wait_queue_head_t tx_queue_ready_wq;

	wait_queue_head_t tx_shmem_ready_wq;
	atomic_t tx_shmem_ready;

	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	struct task_struct *rx_worker;
	struct task_struct *tx_worker;

	struct net_device *netdev;

	struct sk_buff_head tx_queue;
};

static int copy_to_shmem(struct nvidia_irot_message_span dst_phy,
			 void *dst_virt, const struct sk_buff *src)
{
	int ret;

	if (!src)
		return -EINVAL;
	if (dst_phy.size < src->len)
		return -EINVAL;
	if (!dst_virt)
		return -EFAULT;

	ret = skb_copy_bits(src, 0, dst_virt, src->len);
	dma_wmb();
	return ret;
}

static int copy_from_shmem(struct sk_buff *dst,
			   struct nvidia_irot_message_span src_phy,
			   const void *src_virt)
{
	if (!src_virt)
		return -EFAULT;
	if (src_phy.size > skb_tailroom(dst))
		return -EINVAL;
	dma_rmb();
	skb_put_data(dst, src_virt, src_phy.size);
	return 0;
}

/**
 * send_message() - send a 32-byte message over mailbox
 * @driver: driver managing the mailbox
 * @msg: message to send
 *
 * Return: 0 on success, negative errno on error.
 */
static int send_message(struct nvidia_irot_driver *driver,
			const struct nvidia_irot_message *msg)
{
	int ret;

	if (!driver->mbox_chan)
		return -EINVAL;

	ret = mbox_send_message(driver->mbox_chan, (void *)msg);
	if (ret < 0) {
		dev_err(driver->dev, "mbox_send_message failed, ret: %d\n",
			ret);
		return ret;
	}
	return 0;
}

static resource_size_t addr_from_span(struct nvidia_irot_message_span shmem)
{
	return (((resource_size_t)shmem.address.high) << 32U) |
	       shmem.address.low;
}

/*
 * Copy shared_state under spinlock and handle generic housekeeping:
 * init shared memory regions, respond to pings, and send duplicate
 * command responses.
 *
 * Caller must hold driver->worker_mutex.  This function never drops it.
 */
static void worker_locked_handle_shared_state(
	struct nvidia_irot_driver *driver,
	struct nvidia_irot_driver_shared_state *shared_state_copy)
{
	spin_lock_irq(&driver->shared_spin);
	memcpy(shared_state_copy, &driver->shared_state,
	       sizeof(*shared_state_copy));
	spin_unlock_irq(&driver->shared_spin);

	/* Init shared memory regions if needed (shmem is write-once) */
	if (!driver->worker_state.rx_addr &&
	    shared_state_copy->shmem.a35_read.size != 0) {
		driver->worker_state.rx_addr = memremap(
			addr_from_span(shared_state_copy->shmem.a35_read),
			shared_state_copy->shmem.a35_read.size, MEMREMAP_WC);
		if (!driver->worker_state.rx_addr)
			dev_err(driver->dev,
				"memremap failed for rx at %pa size %u\n",
				&(resource_size_t){ addr_from_span(
					shared_state_copy->shmem.a35_read) },
				shared_state_copy->shmem.a35_read.size);
	}
	if (!driver->worker_state.tx_addr &&
	    shared_state_copy->shmem.a35_write.size != 0) {
		driver->worker_state.tx_addr = memremap(
			addr_from_span(shared_state_copy->shmem.a35_write),
			shared_state_copy->shmem.a35_write.size, MEMREMAP_WC);
		if (!driver->worker_state.tx_addr)
			dev_err(driver->dev,
				"memremap failed for tx at %pa size %u\n",
				&(resource_size_t){ addr_from_span(
					shared_state_copy->shmem.a35_write) },
				shared_state_copy->shmem.a35_write.size);
	}

	/* Handle pings */
	if (shared_state_copy->rx_ping.command == nvidia_irot_cc_ping) {
		struct nvidia_irot_message response = NVIDIA_IROT_MESSAGE_INIT;

		response.command = nvidia_irot_cc_ping_rsp;
		response.data.value = shared_state_copy->rx_ping.data.value;
		if (send_message(driver, &response) == 0) {
			/*
			 * Clear the in-flight ping only if the counter still
			 * matches.  A new ping may have arrived in the meantime.
			 */
			spin_lock_irq(&driver->shared_spin);
			if (driver->shared_state.rx_ping.data.value ==
			    shared_state_copy->rx_ping.data.value)
				driver->shared_state.rx_ping.command = 0;
			spin_unlock_irq(&driver->shared_spin);
		}
	}

	/*
	 * Handle duplicate command responses.
	 * duplicate_response is only written by the mailbox callback when
	 * cleared, and only cleared by the worker_mutex owner, so we have
	 * exclusive write access through our local copy.
	 */
	if (shared_state_copy->duplicate_response.command != 0) {
		if (send_message(driver,
				 &shared_state_copy->duplicate_response) == 0) {
			shared_state_copy->duplicate_response.command = 0;
			spin_lock_irq(&driver->shared_spin);
			driver->shared_state.duplicate_response.command = 0;
			spin_unlock_irq(&driver->shared_spin);
		}
	}
}

/*
 * Get the byte offset of @allocation within @region.
 *
 * Return: offset on success, negative errno on failure.
 */
static ssize_t get_start_offset(struct nvidia_irot_message_span region,
				struct nvidia_irot_message_span allocation)
{
	resource_size_t region_start = addr_from_span(region);
	resource_size_t region_end = region_start + region.size;
	resource_size_t alloc_start = addr_from_span(allocation);
	resource_size_t alloc_end = alloc_start + allocation.size;

	if (region_end < region_start)
		return -EFAULT; /* region wrap around */
	if (alloc_end < alloc_start)
		return -EFAULT; /* allocation wrap around */
	if (alloc_start < region_start)
		return -EFAULT; /* allocation starts before region */
	if (alloc_end > region_end)
		return -EFAULT; /* allocation exceeds region */

	return alloc_start - region_start;
}

/*
 * Finalize a read operation.  On success, clears the pending read and sends
 * an ACK to the IRoT.  Always releases worker_mutex before returning.
 */
static ssize_t finish_read_and_unlock(struct nvidia_irot_driver *driver,
				      ssize_t result,
				      const struct nvidia_irot_message *message)
{
	if (result >= 0 && result != message->data.mctp.packet.size)
		result = -EFAULT;

	if (result >= 0) {
		struct nvidia_irot_message response = NVIDIA_IROT_MESSAGE_INIT;

		spin_lock_irq(&driver->shared_spin);
		driver->shared_state.pending_read.command = 0;
		spin_unlock_irq(&driver->shared_spin);

		response.command = nvidia_irot_cc_mctp_done;
		response.data.value = message->data.mctp.counter;
		/* Dropped ACKs are handled by IRoT retransmission */
		(void)send_message(driver, &response);

		mutex_unlock(&driver->worker_mutex);
		return result;
	}

	mutex_unlock(&driver->worker_mutex);
	return result;
}

struct nvidia_irot_netdev_priv {
	struct nvidia_irot_driver *driver;
};

static netdev_tx_t nvidia_irot_start_xmit(struct sk_buff *skb,
					  struct net_device *dev)
{
	struct nvidia_irot_netdev_priv *priv = netdev_priv(dev);
	struct nvidia_irot_driver *driver = priv->driver;
	netdev_tx_t status = NETDEV_TX_BUSY;
	unsigned long flags;

	dev_dbg(driver->dev, "tx: %u bytes\n", skb->len);

	/*
	 * Lock the queue for the entire push so we can atomically inspect
	 * the length to manage the network interface queue state.
	 */
	spin_lock_irqsave(&driver->tx_queue.lock, flags);
	if (skb_queue_len(&driver->tx_queue) >= NVIDIA_IROT_QUEUE_SIZE) {
		status = NETDEV_TX_BUSY;
		dev_dbg(driver->dev, "tx queue overflow\n");
		dev->stats.tx_dropped++;
		netif_stop_queue(dev);
	} else {
		status = NETDEV_TX_OK;
		__skb_queue_tail(&driver->tx_queue, skb);
		if (skb_queue_len(&driver->tx_queue) == NVIDIA_IROT_QUEUE_SIZE)
			netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(&driver->tx_queue.lock, flags);

	if (status == NETDEV_TX_OK)
		wake_up(&driver->tx_queue_ready_wq);

	return status;
}

static int nvidia_irot_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int nvidia_irot_stop(struct net_device *dev)
{
	struct nvidia_irot_netdev_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	skb_queue_purge(&priv->driver->tx_queue);
	return 0;
}

static const struct net_device_ops nvidia_irot_nops = {
	.ndo_start_xmit = nvidia_irot_start_xmit,
	.ndo_open = nvidia_irot_open,
	.ndo_stop = nvidia_irot_stop,
};

static void nvidia_irot_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	/* MTU is replaced after shared memory negotiation */
	dev->min_mtu = 68;
	dev->max_mtu = 68;
	dev->mtu = 68;

	dev->hard_header_len = 0;
	dev->tx_queue_len = NVIDIA_IROT_QUEUE_SIZE;
	dev->netdev_ops = &nvidia_irot_nops;
	dev->addr_len = 0;
}

static int nvidia_irot_create_mctp_dev(struct nvidia_irot_driver *driver,
				       const char *name)
{
	struct nvidia_irot_netdev_priv *priv;
	int ret;
	u32 mtu;

	driver->netdev = alloc_netdev(sizeof(struct nvidia_irot_netdev_priv),
				      name, NET_NAME_PREDICTABLE,
				      nvidia_irot_netdev_setup);
	if (!driver->netdev)
		return -ENOMEM;

	spin_lock_irq(&driver->shared_spin);
	mtu = driver->shared_state.shmem.mtu_limit;
	spin_unlock_irq(&driver->shared_spin);
	if (mtu >= 68) {
		driver->netdev->min_mtu = mtu;
		driver->netdev->max_mtu = mtu;
		driver->netdev->mtu = mtu;
	}

	priv = netdev_priv(driver->netdev);
	priv->driver = driver;

	ret = mctp_register_netdev(driver->netdev, NULL,
				   MCTP_PHYS_BINDING_VENDOR);
	if (ret) {
		free_netdev(driver->netdev);
		driver->netdev = NULL;
		return ret;
	}
	return 0;
}

/**
 * nvidia_irot_driver_of_mbox() - get driver from mailbox client
 * @client: the mailbox client
 *
 * Resolves the driver via container_of and cross-checks against
 * dev_get_drvdata() as defense-in-depth for this safety-critical path.
 *
 * Return: driver pointer, or NULL on error.
 */
static struct nvidia_irot_driver *
nvidia_irot_driver_of_mbox(struct mbox_client *client)
{
	struct nvidia_irot_driver *driver;
	void *driver_from_devdata;

	if (!client) {
		pr_err("nvidia-ast27xx-irot: null client in %s\n", __func__);
		return NULL;
	}

	driver = container_of(client, struct nvidia_irot_driver, mbox_client);

	if (!client->dev) {
		dev_err(driver->dev, "null client->dev in %s\n", __func__);
		return NULL;
	}

	driver_from_devdata = dev_get_drvdata(client->dev);
	if (driver != driver_from_devdata) {
		dev_err(driver->dev,
			"container_of and dev_get_drvdata disagree in %s\n",
			__func__);
		return NULL;
	}
	return driver;
}

static void wake_rx_worker(struct nvidia_irot_driver *driver)
{
	atomic_xchg(&driver->rx_work_ready, 1);
	wake_up(&driver->rx_worker_wq);
}

/*
 * Schedule a response to a duplicate command without re-processing it.
 * Called from mailbox callback — must not block.
 * Caller must hold driver->shared_spin.
 */
static void
locked_handle_duplicate_rx_command(struct nvidia_irot_driver *driver,
				   const struct nvidia_irot_message *command)
{
	struct nvidia_irot_message *staged =
		&driver->shared_state.duplicate_response;
	struct nvidia_irot_message response = NVIDIA_IROT_MESSAGE_INIT;

	if (staged->command != 0)
		return;
	if (command->command != nvidia_irot_cc_mctp)
		return;

	response.command = nvidia_irot_cc_mctp_done;
	response.data.value = command->data.mctp.counter;
	*staged = response;

	wake_rx_worker(driver);
}

/*
 * Process an incoming mailbox message.
 * Called from mailbox callback — must not block.
 * Caller must hold driver->shared_spin.
 */
static void locked_handle_rx_message(struct nvidia_irot_driver *driver,
				     const struct nvidia_irot_message *message)
{
	switch (message->command) {
	case nvidia_irot_cc_ping:
		driver->shared_state.rx_ping = *message;
		wake_rx_worker(driver);
		return;

	case nvidia_irot_cc_mctp_buffer:
		/* Duplicate buffer info — ignore if already initialized */
		if (driver->shared_state.shmem.a35_read.size != 0)
			return;
		if (driver->shared_state.shmem.a35_write.size != 0)
			return;
		driver->shared_state.shmem = message->data.buffers;
		up(&driver->probe_sem);
		return;

	case nvidia_irot_cc_mctp:
		if (driver->shared_state.pending_read.command != 0)
			return; /* read already pending, drop */
		if (driver->mailbox_state.has_prev_rx_counter &&
		    driver->mailbox_state.prev_rx_counter ==
			    message->data.mctp.counter) {
			/* duplicate command — send response without re-processing */
			locked_handle_duplicate_rx_command(driver, message);
			return;
		}
		driver->shared_state.pending_read = *message;
		driver->mailbox_state.has_prev_rx_counter = true;
		driver->mailbox_state.prev_rx_counter =
			message->data.mctp.counter;
		wake_rx_worker(driver);
		return;

	case nvidia_irot_cc_mctp_done:
		/* TX buffer acknowledged by IRoT */
		if (driver->shared_state.pending_write.command == 0)
			return; /* no write was pending */
		if (driver->shared_state.pending_write.data.mctp.counter !=
		    message->data.value)
			return; /* counter mismatch */
		driver->shared_state.pending_write.command = 0;
		atomic_xchg(&driver->tx_shmem_ready, 1);
		wake_up(&driver->tx_shmem_ready_wq);
		return;

	default:
		return;
	}
}

/*
 * Mailbox RX callback — called when the IRoT sends us an IPC notification.
 */
static void nvidia_irot_on_notification(struct mbox_client *client, void *data)
{
	struct nvidia_irot_driver *driver = nvidia_irot_driver_of_mbox(client);
	struct nvidia_irot_message message;
	unsigned long flags;

	if (!driver)
		return;

	memcpy(&message, data, sizeof(message));
	spin_lock_irqsave(&driver->shared_spin, flags);
	locked_handle_rx_message(driver, &message);
	spin_unlock_irqrestore(&driver->shared_spin, flags);
}

static int nvidia_irot_load_mailbox(struct device *dev,
				    struct nvidia_irot_driver *driver)
{
	struct mbox_client *client = &driver->mbox_client;
	struct mbox_chan *chan;

	client->dev = dev;
	client->rx_callback = nvidia_irot_on_notification;
	client->tx_done = NULL;
	client->tx_block = true;
	client->tx_tout = 250;

	chan = mbox_request_channel_byname(client, "irot");
	if (IS_ERR(chan)) {
		dev_err(dev, "Failed to request mailbox channel\n");
		return PTR_ERR(chan);
	}

	driver->mbox_chan = chan;
	dev_dbg(dev, "Mailbox channel configured successfully\n");
	return 0;
}

/*
 * Read an MCTP packet from shared memory into an skb and deliver it.
 *
 * Return: packet length on success, negative errno on failure.
 */
static ssize_t worker_locked_handle_read(
	struct nvidia_irot_driver *driver,
	struct nvidia_irot_driver_shared_state *shared_state_copy)
{
	struct sk_buff *skb;
	struct mctp_skb_cb *cb;
	ssize_t offset;
	size_t length;
	int ret;

	if (shared_state_copy->pending_read.command != nvidia_irot_cc_mctp ||
	    shared_state_copy->pending_read.data.mctp.packet.size == 0)
		return -EINVAL;

	length = shared_state_copy->pending_read.data.mctp.packet.size;
	offset = get_start_offset(
		shared_state_copy->shmem.a35_read,
		shared_state_copy->pending_read.data.mctp.packet);
	if (offset < 0) {
		/*
		 * Error path: clear the pending read so the driver doesn't
		 * get stuck.  The IRoT will retransmit if no ACK arrives.
		 */
		spin_lock_irq(&driver->shared_spin);
		driver->shared_state.pending_read.command = 0;
		spin_unlock_irq(&driver->shared_spin);
		dev_err(driver->dev, "bad read from shared memory\n");
		driver->netdev->stats.rx_errors++;
		return -EINVAL;
	}

	skb = netdev_alloc_skb(driver->netdev, length);
	if (!skb) {
		driver->netdev->stats.rx_dropped++;
		return -ENOMEM;
	}
	skb->protocol = htons(ETH_P_MCTP);
	ret = copy_from_shmem(skb,
			      shared_state_copy->pending_read.data.mctp.packet,
			      driver->worker_state.rx_addr + offset);
	if (ret != 0) {
		dev_err(driver->dev, "failed to copy rx from shared memory\n");
		driver->netdev->stats.rx_errors++;
		kfree_skb(skb);
		return -EINVAL;
	}
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);

	cb = __mctp_cb(skb);
	cb->halen = 0;

	dev_dbg(driver->dev, "rx: %zu bytes\n", length);

	driver->netdev->stats.rx_packets++;
	driver->netdev->stats.rx_bytes += length;

	ret = netif_receive_skb(skb);
	if (ret != NET_RX_SUCCESS) {
		/* skb is still freed by a failed netif_receive_skb */
		dev_err(driver->dev, "netif_receive_skb failed: %d\n", ret);
		driver->netdev->stats.rx_errors++;
		return -EINVAL;
	}
	return length;
}

static int nvidia_irot_rx_worker(void *data)
{
	struct nvidia_irot_driver *driver = data;
	struct nvidia_irot_driver_shared_state state;

	for (;;) {
		wait_event_interruptible(
			driver->rx_worker_wq,
			kthread_should_stop() ||
				atomic_read(&driver->rx_work_ready));
		if (kthread_should_stop())
			return 0;

		atomic_xchg(&driver->rx_work_ready, 0);
		mutex_lock(&driver->worker_mutex);
		worker_locked_handle_shared_state(driver, &state);
		finish_read_and_unlock(
			driver, worker_locked_handle_read(driver, &state),
			&state.pending_read);
	}
}

static ssize_t worker_locked_handle_write(
	struct nvidia_irot_driver *driver,
	const struct nvidia_irot_driver_shared_state *shared_state_copy,
	struct sk_buff *skb)
{
	struct nvidia_irot_message message = NVIDIA_IROT_MESSAGE_INIT;
	size_t count = skb->len;
	int ret;

	if (count == 0)
		return -EINVAL;

	if (count > shared_state_copy->shmem.mtu_limit) {
		dev_err(driver->dev,
			"tx packet length %zu exceeds mtu limit %u\n", count,
			shared_state_copy->shmem.mtu_limit);
		driver->netdev->stats.tx_dropped++;
		return -EINVAL;
	}

	ret = copy_to_shmem(shared_state_copy->shmem.a35_write,
			    driver->worker_state.tx_addr, skb);
	if (ret < 0) {
		dev_err(driver->dev, "copy_to_shmem failed\n");
		driver->netdev->stats.tx_dropped++;
		return ret;
	}

	message.command = nvidia_irot_cc_mctp;
	message.data.mctp.counter = driver->worker_state.next_tx_counter++;
	message.data.mctp.packet.address =
		shared_state_copy->shmem.a35_write.address;
	message.data.mctp.packet.size = count;

	spin_lock_irq(&driver->shared_spin);
	driver->shared_state.pending_write = message;
	spin_unlock_irq(&driver->shared_spin);

	ret = send_message(driver, &message);
	if (ret < 0) {
		spin_lock_irq(&driver->shared_spin);
		driver->shared_state.pending_write.command = 0;
		spin_unlock_irq(&driver->shared_spin);
		driver->netdev->stats.tx_errors++;
		return ret;
	}

	driver->netdev->stats.tx_packets++;
	driver->netdev->stats.tx_bytes += count;

	return count;
}

static int nvidia_irot_tx_worker(void *data)
{
	struct nvidia_irot_driver *driver = data;
	struct nvidia_irot_driver_shared_state ssc;
	struct sk_buff *skb = NULL;
	ssize_t ret;

	for (;;) {
		while (!skb) {
			wait_event_interruptible(
				driver->tx_queue_ready_wq,
				!skb_queue_empty(&driver->tx_queue) ||
					kthread_should_stop());
			if (kthread_should_stop())
				return 0;

			skb = skb_dequeue(&driver->tx_queue);
			if (skb)
				break;
		}

		for (;;) {
			wait_event_interruptible(
				driver->tx_shmem_ready_wq,
				kthread_should_stop() ||
					atomic_read(&driver->tx_shmem_ready));
			if (kthread_should_stop()) {
				kfree_skb(skb);
				return 0;
			}
			atomic_xchg(&driver->tx_shmem_ready, 0);

			mutex_lock(&driver->worker_mutex);
			worker_locked_handle_shared_state(driver, &ssc);
			/*
			 * pending_write is only cleared by the
			 * mailbox callback, so while we hold
			 * worker_mutex we have exclusive write
			 * access.
			 */
			if (driver->worker_state.tx_addr &&
			    ssc.pending_write.command == 0)
				break;
			mutex_unlock(&driver->worker_mutex);
		}

		ret = worker_locked_handle_write(driver, &ssc, skb);
		if (ret < 0)
			dev_err(driver->dev, "failed to send MCTP packet\n");
		mutex_unlock(&driver->worker_mutex);

		/*
		 * Error path: drop is intentional — the IRoT protocol
		 * handles retransmission at the peer level, so driver-level
		 * retry adds complexity for marginal benefit.
		 */
		if (ret >= 0)
			consume_skb(skb);
		else
			kfree_skb(skb);
		skb = NULL;

		/*
		 * Wake the queue after freeing the skb so the xmit path
		 * can enqueue new packets.  netif_wake_queue both sets
		 * the queue-running flag and reschedules the qdisc.
		 */
		if (skb_queue_len(&driver->tx_queue) < NVIDIA_IROT_QUEUE_SIZE)
			netif_wake_queue(driver->netdev);
	}
}

/*
 * Request shared memory buffer information from the IRoT and wait for
 * the response.  Called during probe before workers are started, so
 * worker_mutex need not be held.
 */
static int nvidia_irot_init_shmem(struct nvidia_irot_driver *driver)
{
	struct nvidia_irot_driver_shared_state ssc;
	struct nvidia_irot_message message;
	int i, ret;

	ssc.shmem.mtu_limit = 0;

	for (i = 0; i < 3; ++i) {
		u64 end_jiffies64;

		message = (struct nvidia_irot_message)NVIDIA_IROT_MESSAGE_INIT;
		message.command = nvidia_irot_cc_get_mctp_buffers;
		ret = send_message(driver, &message);
		if (ret < 0) {
			dev_err(driver->dev,
				"failed to send startup IPC message: %d\n",
				ret);
			return ret;
		}

		end_jiffies64 = get_jiffies_64() + (5 * HZ);
		while (time_is_after_jiffies64(end_jiffies64)) {
			/* return value intentionally ignored — we poll state */
			(void)down_timeout(&driver->probe_sem, HZ);
			worker_locked_handle_shared_state(driver, &ssc);
			if (ssc.shmem.mtu_limit != 0)
				break;
		}

		if (ssc.shmem.mtu_limit != 0)
			break;
		dev_warn(driver->dev,
			 "timed out waiting for IRoT buffer info\n");
	}

	if (ssc.shmem.mtu_limit == 0) {
		dev_err(driver->dev,
			"timeout waiting for IRoT shared memory buffers\n");
		return -ETIMEDOUT;
	}
	return 0;
}

/*
 * Tear down a driver that may be in any stage of initialization.
 * Every field is NULL-checked, so this is safe to call after partial
 * construction (devm_kzalloc zero-initializes all pointers).
 * The driver struct itself is devm-managed and freed by the driver core.
 */
static void clean_driver(struct platform_device *pdev)
{
	struct nvidia_irot_driver *driver;

	if (!pdev)
		return;

	driver = platform_get_drvdata(pdev);
	if (!driver)
		return;

	if (driver->rx_worker) {
		int rv = kthread_stop(driver->rx_worker);

		if (rv)
			dev_err(driver->dev,
				"rx worker exited with error: %d\n", rv);
		driver->rx_worker = NULL;
	}
	if (driver->tx_worker) {
		int rv = kthread_stop(driver->tx_worker);

		if (rv)
			dev_err(driver->dev,
				"tx worker exited with error: %d\n", rv);
		driver->tx_worker = NULL;
	}

	if (driver->netdev) {
		netif_tx_disable(driver->netdev);
		skb_queue_purge(&driver->tx_queue);

		rtnl_lock();
		if (netif_running(driver->netdev))
			dev_close(driver->netdev);
		rtnl_unlock();

		mctp_unregister_netdev(driver->netdev);
		free_netdev(driver->netdev);
		driver->netdev = NULL;
	}

	/* Workers are stopped — worker_state can be accessed freely */
	if (driver->worker_state.rx_addr) {
		memunmap((void *)driver->worker_state.rx_addr);
		driver->worker_state.rx_addr = NULL;
	}
	if (driver->worker_state.tx_addr) {
		memunmap(driver->worker_state.tx_addr);
		driver->worker_state.tx_addr = NULL;
	}

	if (driver->mbox_chan) {
		mbox_free_channel(driver->mbox_chan);
		driver->mbox_chan = NULL;
	}

	platform_set_drvdata(pdev, NULL);
	/* driver is devm-managed — freed automatically by the driver core */
}

static int nvidia_irot_probe(struct platform_device *pdev)
{
	struct nvidia_irot_driver *driver;
	struct device *dev = &pdev->dev;
	int ret;

	dev_info(dev, "probe starting\n");

	driver = devm_kzalloc(dev, sizeof(*driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	driver->dev = dev;
	platform_set_drvdata(pdev, driver);
	spin_lock_init(&driver->shared_spin);
	mutex_init(&driver->worker_mutex);
	sema_init(&driver->probe_sem, 0);
	init_waitqueue_head(&driver->rx_worker_wq);
	atomic_set(&driver->rx_work_ready, 0);
	init_waitqueue_head(&driver->tx_queue_ready_wq);
	init_waitqueue_head(&driver->tx_shmem_ready_wq);
	skb_queue_head_init(&driver->tx_queue);

	ret = nvidia_irot_load_mailbox(dev, driver);
	if (ret)
		goto err_cleanup;

	ret = nvidia_irot_init_shmem(driver);
	if (ret < 0)
		goto err_cleanup;

	atomic_set(&driver->tx_shmem_ready, 1);
	wake_up(&driver->tx_shmem_ready_wq);

	ret = nvidia_irot_create_mctp_dev(driver, dev_name(dev));
	if (ret < 0) {
		dev_err(dev, "failed to create mctp device: %d\n", ret);
		goto err_cleanup;
	}

	driver->rx_worker = kthread_run(nvidia_irot_rx_worker, driver,
					"%s-rx-worker", dev_name(dev));
	if (IS_ERR(driver->rx_worker)) {
		ret = PTR_ERR(driver->rx_worker);
		driver->rx_worker = NULL;
		dev_err(dev, "failed to create rx worker thread\n");
		goto err_cleanup;
	}

	driver->tx_worker = kthread_run(nvidia_irot_tx_worker, driver,
					"%s-tx-worker", dev_name(dev));
	if (IS_ERR(driver->tx_worker)) {
		ret = PTR_ERR(driver->tx_worker);
		driver->tx_worker = NULL;
		dev_err(dev, "failed to create tx worker thread\n");
		goto err_cleanup;
	}

	dev_info(dev, "probe successful\n");
	return 0;

err_cleanup:
	dev_err(dev, "probe failed: %d\n", ret);
	/* clean_driver handles partial construction (devm_kzalloc zero-init) */
	clean_driver(pdev);
	return ret;
}

static void nvidia_irot_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "removing device\n");
	clean_driver(pdev);
}

static const struct of_device_id nvidia_irot_match[] = {
	{ .compatible = "nvidia,ast27xx,irot" },
	{},
};
MODULE_DEVICE_TABLE(of, nvidia_irot_match);

static struct platform_driver nvidia_irot_pdriver = {
	.probe		= nvidia_irot_probe,
	.remove		= nvidia_irot_remove,
	.driver		= {
		.name		= "nvidia-ast27xx-irot",
		.of_match_table	= nvidia_irot_match,
	},
};

module_platform_driver(nvidia_irot_pdriver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVIDIA IRoT MCTP Driver for ASPEED AST27xx BMC");
