// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * MCTP binding driver for NVIDIA OP-TEE VRoT trusted applications.
 * Each device-tree node creates an MCTP endpoint. The TA is opened on the
 * first packet so tee-supplicant can start after this module is loaded.
 */

#include <linux/build_bug.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/spinlock.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>
#include <linux/wait.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <uapi/linux/if_arp.h>

#define HEARTBEAT_COMMAND_INTERVAL_MS 30000
/*
 * Calling the TA before its requested deadline is safe. Cap call-again delays
 * at five minutes to keep buggy TA values well away from MAX_JIFFY_OFFSET and
 * bound the wait before pending non-atomic work is serviced.
 */
#define NVIDIA_VROT_MAX_CALL_AGAIN_DELAY_MS (5 * 60 * 1000)
#define TA_COMMAND_ID_WRITE_PACKET 0
#define TA_COMMAND_ID_HANDLE_NON_ATOMIC_OPERATION 1
#define TA_COMMAND_ID_HEARTBEAT 3

#define NVIDIA_VROT_QUEUE_SIZE 20

#define NVIDIA_VROT_MAX_PROTOCOLS 10

/*
 * Entries that are encoded in the nvidia,mctp-protocol-versions property.
 * The property is an array of u32.
 * Odd indexes represent protocol IDs and even indexes are the version of the protocol.
 * Is needed because the VRoTs do not implement the control MCTP protocol so we
 * fake it in the kernel module.
 * Entries should not be provided for the control protocol or base MCTP version
 * as that is controlled by this module.
 */
struct nvidia_ta_mctp_protocol_version {
	/*
	 * The MCTP protocol ID.
	 * 1 is PLDM, 5 is SPDM.
	 */
	u8 protocol;
	/*
	 * The version number.
	 * Use 0 if you do not which to respond to a
	 * Get MCTP Version Support command for this protocol.
	 * This is a u32 in native endianness, using a u8[4] to reduce padding.
	 * 0xF1F0F000 is 1.0.0.
	 */
	u8 version[4];
};

/* Immutable config taken from device tree. */
struct nvidia_ta_mctp_config {
	/*
	 * nvidia,ta-uuid property.
	 * The UUID of the trusted application that acts as a VRoT.
	 */
	uuid_t ta_uuid;
	/*
	 * nvidia,mctp-uuid property.
	 * The UUID to use in response to a Get Endpoint UUID MCTP Control Message.
	 */
	uuid_t mctp_uuid;
	/*
	 * nvidia,mctp-mtu property.
	 * The max allowed MTU for the VRoT trusted application.
	 * Is not discovered at startup via TEE function calls because we want to delay
	 * TA initialization to as late as possible to allow for tee-supplicant to be started
	 */
	/*  which may occur after kernel module loading. */
	u32 mtu;
	/* The number of valid entries in protocols. */
	u32 protocols_size;
	/* nvidia,mctp-protocol-versions property. */
	struct nvidia_ta_mctp_protocol_version
		protocols[NVIDIA_VROT_MAX_PROTOCOLS];
};

/* State used by callbacks. */
struct nvidia_ta_mctp_driver {
	struct device *dev;
	/* Info from device tree. */
	struct nvidia_ta_mctp_config config;

	/* Trusted execution context. */
	struct tee_context *tee_ctx;
	/* Shared Memory with the trusted environment. */
	struct tee_shm *tee_pool;
	/* Trusted execution session id. */
	u32 tee_session_id;

	/* Network device used by the linux kernel for this driver. */
	struct net_device *netdev;

	/* Worker task for handling MCTP packets. */
	struct task_struct *worker_task;

	/* wait queue that is woken when tx_queue is pushed to */
	wait_queue_head_t wq;

	/* Packet queue from the network device to the tx worker thread. */
	struct sk_buff_head tx_queue;

	/*
	 * The timestamp when we should service non-atomic work.
	 * Is only valid if non_atomic is true.
	 */
	u64 non_atomic_work_jiffies_64;

	/*
	 * The timestamp when we should next send a heartbeat.
	 * Is only valid if tee_initialized is true.
	 */
	u64 next_heartbeat_jiffies_64;

	/* The TA is initialized on the first write, after tee-supplicant starts. */
	u32 tee_initialized : 1;

	/* Non-atomic operation flag */
	u32 non_atomic : 1;
};

/* Private state for the mctp network device. */
struct nvidia_vrot_netdev_priv {
	struct nvidia_ta_mctp_driver *driver;
};

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	return ver->impl_id == TEE_IMPL_ID_OPTEE;
}

static int open_ta_session(struct nvidia_ta_mctp_driver *driver)
{
	struct tee_ioctl_open_session_arg sess_arg = { };
	int ret;

	export_uuid(sess_arg.uuid, &driver->config.ta_uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;

	ret = tee_client_open_session(driver->tee_ctx, &sess_arg, NULL);
	if (ret < 0) {
		dev_err(driver->dev, "failed to open OP-TEE session: %d\n", ret);
		return ret;
	}
	if (sess_arg.ret) {
		dev_err(driver->dev, "TA session open failed: 0x%x\n", sess_arg.ret);
		return -EIO;
	}

	driver->tee_session_id = sess_arg.session;
	dev_dbg(driver->dev, "OP-TEE session opened: %u\n",
		driver->tee_session_id);
	return 0;
}

static int send_heartbeat_command_to_tee(struct nvidia_ta_mctp_driver *driver)
{
	struct tee_ioctl_invoke_arg invoke_arg = {
		.func = TA_COMMAND_ID_HEARTBEAT,
		.session = driver->tee_session_id,
		.num_params = 4,
	};
	struct tee_param params[4] = { };
	void *va;
	int ret;

	/* Schedule the next heartbeat even if this invocation fails. */
	driver->next_heartbeat_jiffies_64 =
		get_jiffies_64() + msecs_to_jiffies(HEARTBEAT_COMMAND_INTERVAL_MS);

	va = tee_shm_get_va(driver->tee_pool, 0);
	if (IS_ERR(va))
		return PTR_ERR(va);

	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	params[0].u.memref.shm = driver->tee_pool;
	params[0].u.memref.size = driver->config.mtu;

	ret = tee_client_invoke_func(driver->tee_ctx, &invoke_arg, params);
	if (ret < 0) {
		dev_err(driver->dev, "heartbeat invocation failed: %d\n", ret);
		return ret;
	}
	if (invoke_arg.ret) {
		dev_err(driver->dev, "heartbeat TA error: 0x%x\n",
			invoke_arg.ret);
		return -EIO;
	}
	if (params[0].u.memref.size > driver->config.mtu) {
		dev_err(driver->dev, "heartbeat response exceeds shared memory\n");
		return -EMSGSIZE;
	}

	dev_dbg(driver->dev, "heartbeat: %zu bytes\n",
		params[0].u.memref.size);
	return 0;
}

static int init_tee(struct nvidia_ta_mctp_driver *driver)
{
	int ret;

	if (driver->tee_initialized)
		return 0;

	driver->tee_ctx = tee_client_open_context(NULL, optee_ctx_match,
						  NULL, NULL);
	if (IS_ERR_OR_NULL(driver->tee_ctx)) {
		ret = IS_ERR(driver->tee_ctx) ? PTR_ERR(driver->tee_ctx) : -ENODEV;
		driver->tee_ctx = NULL;
		dev_err(driver->dev, "failed to open TEE context: %d\n", ret);
		return ret;
	}

	ret = open_ta_session(driver);
	if (ret)
		goto close_context;

	driver->tee_pool = tee_shm_alloc_kernel_buf(driver->tee_ctx,
						    driver->config.mtu);
	if (IS_ERR_OR_NULL(driver->tee_pool)) {
		ret = IS_ERR(driver->tee_pool) ? PTR_ERR(driver->tee_pool) : -ENOMEM;
		driver->tee_pool = NULL;
		dev_err(driver->dev, "failed to allocate shared memory: %d\n", ret);
		goto close_session;
	}

	driver->tee_initialized = true;
	driver->next_heartbeat_jiffies_64 = get_jiffies_64();
	return 0;

close_session:
	tee_client_close_session(driver->tee_ctx, driver->tee_session_id);
	driver->tee_session_id = 0;
close_context:
	tee_client_close_context(driver->tee_ctx);
	driver->tee_ctx = NULL;
	return ret;
}

static int invoke_ta_function(struct nvidia_ta_mctp_driver *driver,
			      const struct sk_buff *skb, int command_id,
			      size_t *out_size)
{
	struct tee_ioctl_invoke_arg invoke_arg = {
		.func = command_id,
		.session = driver->tee_session_id,
		.num_params = 4,
	};
	struct tee_param params[4] = { };
	size_t count = skb ? skb->len : 0;
	u32 delay_ms;
	void *va;
	int ret;

	*out_size = 0;
	if (count > driver->config.mtu)
		return -EMSGSIZE;

	va = tee_shm_get_va(driver->tee_pool, 0);
	if (IS_ERR(va))
		return PTR_ERR(va);
	if (count) {
		ret = skb_copy_bits(skb, 0, va, count);
		if (ret)
			return ret;
		dev_dbg(driver->dev, "tx: %zu bytes\n", count);
	}

	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	params[0].u.memref.shm = driver->tee_pool;
	params[0].u.memref.size = driver->config.mtu;
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	params[2].u.value.a = count;

	ret = tee_client_invoke_func(driver->tee_ctx, &invoke_arg, params);
	if (ret < 0 || invoke_arg.ret) {
		driver->non_atomic = false;
		driver->non_atomic_work_jiffies_64 = 0;
		if (ret < 0) {
			dev_err(driver->dev, "TA invocation failed: %d\n", ret);
			return ret;
		}
		dev_err(driver->dev, "TA invocation error: 0x%x\n",
			invoke_arg.ret);
		return -EIO;
	}

	*out_size = params[2].u.value.b;
	driver->non_atomic = !!params[1].u.value.a;
	if (driver->non_atomic) {
		/* The TA reports the call-again delay in milliseconds. */
		delay_ms = min_t(u64, params[1].u.value.b,
				 NVIDIA_VROT_MAX_CALL_AGAIN_DELAY_MS);
		driver->non_atomic_work_jiffies_64 =
			get_jiffies_64() + msecs_to_jiffies(delay_ms);
		dev_dbg(driver->dev, "TA call again in %u ms\n", delay_ms);
	} else {
		driver->non_atomic_work_jiffies_64 = 0;
	}

	return 0;
}

static int handle_data_from_ta(struct nvidia_ta_mctp_driver *driver,
			       size_t size)
{
	const void *shared_mem_data;
	struct mctp_skb_cb *cb;
	struct sk_buff *skb;
	int ret;

	shared_mem_data = tee_shm_get_va(driver->tee_pool, 0);
	if (IS_ERR(shared_mem_data))
		return PTR_ERR(shared_mem_data);
	if (size <= 4 || size > driver->config.mtu) {
		dev_err(driver->dev, "invalid MCTP response size: %zu\n", size);
		return -EINVAL;
	}

	skb = netdev_alloc_skb(driver->netdev, size);
	if (!skb)
		return -ENOMEM;
	skb->protocol = htons(ETH_P_MCTP);
	skb_put_data(skb, shared_mem_data, size);
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	cb = __mctp_cb(skb);
	cb->halen = 0;

	ret = netif_receive_skb(skb);
	if (ret != NET_RX_SUCCESS)
		return -EIO;
	driver->netdev->stats.rx_packets++;
	driver->netdev->stats.rx_bytes += size;
	dev_dbg(driver->dev, "rx: %zu bytes\n", size);
	return 0;
}

/*
 * Negative returns mean the request was not accepted. Both nonnegative
 * statuses mean TX succeeded; response failures have already been counted
 * and logged as RX errors. Never retry a request solely because RX failed.
 */
enum nvidia_vrot_tx_status {
	NVIDIA_VROT_TX_OK,
	NVIDIA_VROT_TX_OK_RX_FAILED,
};

static int nvidia_vrot_response_error(struct nvidia_ta_mctp_driver *driver,
				      int error)
{
	driver->netdev->stats.rx_errors++;
	driver->netdev->stats.rx_dropped++;
	dev_err(driver->dev, "failed to deliver MCTP response: %d\n", error);
	return NVIDIA_VROT_TX_OK_RX_FAILED;
}

static int nvidia_ta_write_to_tee(struct nvidia_ta_mctp_driver *driver,
				  const struct sk_buff *skb, int command_id)
{
	size_t out_size;
	int ret;

	ret = init_tee(driver);
	if (ret)
		return ret;

	ret = invoke_ta_function(driver, skb, command_id, &out_size);
	if (ret)
		return ret;

	if (out_size) {
		ret = handle_data_from_ta(driver, out_size);
		if (ret)
			return nvidia_vrot_response_error(driver, ret);
	}
	return NVIDIA_VROT_TX_OK;
}

/* Release resources in reverse order, including partially probed devices. */
static void clean_driver(struct platform_device *pdev)
{
	struct nvidia_ta_mctp_driver *driver = platform_get_drvdata(pdev);

	if (!driver)
		return;

	if (driver->worker_task) {
		kthread_stop(driver->worker_task);
		driver->worker_task = NULL;
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

	if (driver->tee_pool) {
		tee_shm_free(driver->tee_pool);
		driver->tee_pool = NULL;
	}
	if (driver->tee_initialized) {
		tee_client_close_session(driver->tee_ctx,
					 driver->tee_session_id);
		driver->tee_initialized = false;
	}
	if (driver->tee_ctx) {
		tee_client_close_context(driver->tee_ctx);
		driver->tee_ctx = NULL;
	}
	platform_set_drvdata(pdev, NULL);
}

static netdev_tx_t nvidia_vrot_start_xmit(struct sk_buff *skb,
					  struct net_device *dev)
{
	struct nvidia_vrot_netdev_priv *priv = netdev_priv(dev);
	struct nvidia_ta_mctp_driver *driver = priv->driver;
	unsigned long flags;

	spin_lock_irqsave(&driver->tx_queue.lock, flags);
	if (skb_queue_len(&driver->tx_queue) >= NVIDIA_VROT_QUEUE_SIZE) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&driver->tx_queue.lock, flags);
		dev_dbg(driver->dev, "tx queue full\n");
		return NETDEV_TX_BUSY;
	}

	__skb_queue_tail(&driver->tx_queue, skb);
	if (skb_queue_len(&driver->tx_queue) == NVIDIA_VROT_QUEUE_SIZE)
		netif_stop_queue(dev);
	spin_unlock_irqrestore(&driver->tx_queue.lock, flags);

	wake_up(&driver->wq);
	return NETDEV_TX_OK;
}

static int nvidia_vrot_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int nvidia_vrot_stop(struct net_device *dev)
{
	struct nvidia_vrot_netdev_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	skb_queue_purge(&priv->driver->tx_queue);
	return 0;
}

static const struct net_device_ops nvidia_vrot_nops = {
	.ndo_start_xmit = nvidia_vrot_start_xmit,
	.ndo_open = nvidia_vrot_open,
	.ndo_stop = nvidia_vrot_stop,
};

/* mctp header byte indexes */
static const size_t nv_mctp_ver_index = 0;
static const size_t nv_mctp_dst_index = 1;
static const size_t nv_mctp_src_index = 2;
static const size_t nv_mctp_flags_index = 3;

/* only support version 1 packets with no reserved bits set */
static const u8 nv_mctp_expected_version_byte = 1;

/* flag bits of interest */
static const u8 nv_mctp_som_bit = 1 << 7;
static const u8 nv_mctp_eom_bit = 1 << 6;
static const u8 nv_mctp_to_bit = 1 << 3;

/* control header bytes */
static const size_t nv_mctp_ctrl_msg_type_index = 4;
static const size_t nv_mctp_ctrl_flags_index = 5;
static const size_t nv_mctp_ctrl_cc_index = 6;
static const size_t nv_mctp_ctrl_rsp_index = 7;

/* control header values */
static const u8 nv_mctp_ctrl_msg_type = 0;

/* ctrl flags bits of interest */
static const u8 nv_mctp_ctrl_req_bit = 1 << 7;
static const u8 nv_mctp_ctrl_datagram_bit = 1 << 6;

/* used ctrl response codes */
static const u8 nv_mctp_ctrl_rsp_ok = 0;
static const u8 nv_mctp_ctrl_rsp_error = 1;
static const u8 nv_mctp_ctrl_rsp_bad_data = 2;
static const u8 nv_mctp_ctrl_rsp_bad_cmd = 5;

/* control header sizes */
static const size_t nv_mctp_ctrl_cmd_data_start = 7;
static const size_t nv_mctp_ctrl_rsp_data_start = 8;

/* must have entire header */
static const size_t nv_min_ctrl_req_size = nv_mctp_ctrl_cmd_data_start;
/* must always fit in a min MTU packet */
#define NV_MAX_CTRL_MSG_SIZE 68

enum nvidia_vrot_packet_type {
	NVIDIA_VROT_PACKET_TO_TA,
	NVIDIA_VROT_PACKET_CONTROL_REQUEST,
	NVIDIA_VROT_PACKET_DROP,
};

static enum nvidia_vrot_packet_type
nvidia_ta_classify_packet(struct nvidia_ta_mctp_driver *driver,
			  const struct sk_buff *skb)
{
	const u8 required_bits = nv_mctp_som_bit | nv_mctp_eom_bit |
				 nv_mctp_to_bit;
	u8 data[7];

	if (skb->len < nv_min_ctrl_req_size)
		return NVIDIA_VROT_PACKET_TO_TA;

	if (skb_copy_bits(skb, 0, data, sizeof(data))) {
		dev_warn(driver->dev, "skb_copy_bits failed when detecting packet type\n");
		return NVIDIA_VROT_PACKET_DROP;
	}
	if (data[nv_mctp_ver_index] != nv_mctp_expected_version_byte ||
	    data[nv_mctp_ctrl_msg_type_index] != nv_mctp_ctrl_msg_type)
		return NVIDIA_VROT_PACKET_TO_TA;

	/* Invalid control flags must not reach the TA or elicit a response. */
	if (skb->len > NV_MAX_CTRL_MSG_SIZE ||
	    (data[nv_mctp_flags_index] & required_bits) != required_bits ||
	    !(data[nv_mctp_ctrl_flags_index] & nv_mctp_ctrl_req_bit) ||
	    (data[nv_mctp_ctrl_flags_index] & nv_mctp_ctrl_datagram_bit)) {
		dev_err(driver->dev,
			"dropping unsupported MCTP control frame: len=%u transport_flags=0x%02x control_flags=0x%02x command=0x%02x\n",
			skb->len, data[nv_mctp_flags_index],
			data[nv_mctp_ctrl_flags_index],
			data[nv_mctp_ctrl_cc_index]);
		return NVIDIA_VROT_PACKET_DROP;
	}

	return NVIDIA_VROT_PACKET_CONTROL_REQUEST;
}

static int
nvidia_ta_handle_control_request(struct nvidia_ta_mctp_driver *driver,
				 const struct sk_buff *skb)
{
	/* Ensure buffer is large enough for maximum control responses */
	BUILD_BUG_ON(nv_mctp_ctrl_rsp_data_start + 16 >
		     NV_MAX_CTRL_MSG_SIZE); // Get Endpoint UUID
	BUILD_BUG_ON(nv_mctp_ctrl_rsp_data_start + 1 +
			     NVIDIA_VROT_MAX_PROTOCOLS >
		     NV_MAX_CTRL_MSG_SIZE); // Get Message Type Support

	dev_dbg(driver->dev, "handling control message...\n");

	unsigned char cmd[NV_MAX_CTRL_MSG_SIZE] = { 0 };
	if (skb_copy_bits(skb, 0, cmd, skb->len) != 0) {
		return -EFAULT;
	}
	dev_dbg(driver->dev, "control tx: %u bytes\n", skb->len);

	unsigned char rsp[NV_MAX_CTRL_MSG_SIZE] = { 0 };

	/* fill MCTP header */
	rsp[nv_mctp_ver_index] = cmd[nv_mctp_ver_index];
	rsp[nv_mctp_dst_index] = cmd[nv_mctp_src_index];
	rsp[nv_mctp_src_index] = cmd[nv_mctp_dst_index];
	rsp[nv_mctp_flags_index] = cmd[nv_mctp_flags_index] & ~nv_mctp_to_bit;

	/* fill control header */
	rsp[nv_mctp_ctrl_msg_type_index] = cmd[nv_mctp_ctrl_msg_type_index];
	rsp[nv_mctp_ctrl_flags_index] = cmd[nv_mctp_ctrl_flags_index] &
					~nv_mctp_ctrl_req_bit;
	rsp[nv_mctp_ctrl_cc_index] = cmd[nv_mctp_ctrl_cc_index];
	rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_error;

	/* payload-less length */
	size_t rsp_len = nv_mctp_ctrl_rsp_data_start;

	/* supporting only mandatory and actually used commands */
	switch (cmd[nv_mctp_ctrl_cc_index]) {
	case 1:
		if (skb->len < nv_mctp_ctrl_cmd_data_start + 2) {
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_data;
			break;
		}
		dev_dbg(driver->dev, "Set Endpoint ID\n");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		/* accepted + no pool */
		rsp[nv_mctp_ctrl_rsp_data_start + 0] = 0;
		/* echo back EID, is 2nd data byte in command */
		rsp[nv_mctp_ctrl_rsp_data_start + 1] =
			cmd[nv_mctp_ctrl_cmd_data_start + 1];
		/* no pool */
		rsp[nv_mctp_ctrl_rsp_data_start + 2] = 0;
		rsp_len = nv_mctp_ctrl_rsp_data_start + 3;
		break;

	case 2:
		dev_dbg(driver->dev, "Get Endpoint ID\n");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		/* echo back endpoint ID */
		rsp[nv_mctp_ctrl_rsp_data_start + 0] = cmd[nv_mctp_dst_index];
		/* simple endpoint + dynamic ID */
		rsp[nv_mctp_ctrl_rsp_data_start + 1] = 0;
		/* no transport specific info */
		rsp[nv_mctp_ctrl_rsp_data_start + 2] = 0;
		rsp_len = nv_mctp_ctrl_rsp_data_start + 3;
		break;

	case 3:
		dev_dbg(driver->dev, "Get Endpoint UUID\n");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		memcpy(&rsp[nv_mctp_ctrl_rsp_data_start],
		       &driver->config.mctp_uuid, 16);
		rsp_len = nv_mctp_ctrl_rsp_data_start + 16;
		break;

	case 4: {
		if (skb->len < nv_mctp_ctrl_cmd_data_start + 1) {
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_data;
			break;
		}
		dev_dbg(driver->dev, "Get Version Support\n");
		u32 version = 0;
		switch (cmd[nv_mctp_ctrl_cmd_data_start]) {
		case 0x00:
			/* control version 1.3.1 */
			version = 0xF1F3F100;
			break;

		case 0xFF:
			/* base version 1.3.1 */
			version = 0xF1F3F100;
			break;

		default:
			/* other protocols, use version set via device tree */
			for (u32 i = 0; i < driver->config.protocols_size;
			     ++i) {
				if (driver->config.protocols[i].protocol ==
				    cmd[nv_mctp_ctrl_cmd_data_start]) {
					memcpy(&version,
					       driver->config.protocols[i]
						       .version,
					       4);
					break;
				}
			}
			break;
		}

		if (!version) {
			/* version unknown, error */
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_data;
		} else {
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
			/* Device-tree versions are native endian; MCTP uses big endian. */
			version = cpu_to_be32(version);
			memcpy(&rsp[nv_mctp_ctrl_rsp_data_start], &version,
			       sizeof(version));
			rsp_len = nv_mctp_ctrl_rsp_data_start + sizeof(version);
		}
		break;
	}

	case 5:
		dev_dbg(driver->dev, "Get Message Type Support\n");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		rsp[nv_mctp_ctrl_rsp_data_start + 0] =
			driver->config.protocols_size;
		for (u32 i = 0; i < driver->config.protocols_size; ++i) {
			rsp[nv_mctp_ctrl_rsp_data_start + 1 + i] =
				driver->config.protocols[i].protocol;
		}
		rsp_len = nv_mctp_ctrl_rsp_data_start + 1 +
			  driver->config.protocols_size;
		break;

	default:
		dev_warn(driver->dev, "Unsupported control command: 0x%02X\n",
			 cmd[nv_mctp_ctrl_cc_index]);
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_cmd;
		break;
	}

	dev_dbg(driver->dev, "control rx: %zu bytes\n", rsp_len);

	struct sk_buff *rsp_skb;
	rsp_skb = netdev_alloc_skb(driver->netdev, rsp_len);
	if (!rsp_skb)
		return nvidia_vrot_response_error(driver, -ENOMEM);
	rsp_skb->protocol = htons(ETH_P_MCTP);
	(void)skb_put_data(rsp_skb, rsp, rsp_len);
	skb_reset_mac_header(rsp_skb);
	skb_reset_network_header(rsp_skb);

	struct mctp_skb_cb *const cb = __mctp_cb(rsp_skb);
	cb->halen = 0;

	const int ret = netif_receive_skb(rsp_skb);
	if (ret != NET_RX_SUCCESS)
		return nvidia_vrot_response_error(driver, -EIO);
	driver->netdev->stats.rx_packets++;
	driver->netdev->stats.rx_bytes += rsp_len;
	return NVIDIA_VROT_TX_OK;
}

enum nvidia_ta_timeout_cause {
	/* There is no reason to ever timeout. */
	NVIDIA_TA_TIMEOUT_NONE,
	/* Should timeout to process delayed work. */
	NVIDIA_TA_TIMEOUT_WORK_TO_DO,
	/* Should timeout to send a heartbeat. */
	NVIDIA_TA_TIMEOUT_HEARTBEAT,
};

struct nvidia_ta_timeout {
	enum nvidia_ta_timeout_cause cause;
	/* Will be MAX_SCHEDULE_TIMEOUT if cause is none. */
	long duration_jiffies;
};

static struct nvidia_ta_timeout
nvidia_ta_get_next_timeout(struct nvidia_ta_mctp_driver *driver)
{
	struct nvidia_ta_timeout timeout = {
		.cause = NVIDIA_TA_TIMEOUT_NONE,
		.duration_jiffies = MAX_SCHEDULE_TIMEOUT,
	};
	u64 now = get_jiffies_64();
	long work_delay, heartbeat_delay;

	if (!driver->tee_initialized)
		return timeout;

	heartbeat_delay = driver->next_heartbeat_jiffies_64 <= now ? 0 :
		min_t(u64, driver->next_heartbeat_jiffies_64 - now,
		      MAX_SCHEDULE_TIMEOUT);
	timeout.cause = NVIDIA_TA_TIMEOUT_HEARTBEAT;
	timeout.duration_jiffies = heartbeat_delay;

	if (driver->non_atomic) {
		work_delay = driver->non_atomic_work_jiffies_64 <= now ? 0 :
			min_t(u64, driver->non_atomic_work_jiffies_64 - now,
			      MAX_SCHEDULE_TIMEOUT);
		if (work_delay <= heartbeat_delay) {
			timeout.cause = NVIDIA_TA_TIMEOUT_WORK_TO_DO;
			timeout.duration_jiffies = work_delay;
		}
	}

	return timeout;
}

static int nvidia_ta_worker(void *data)
{
	struct nvidia_ta_mctp_driver *driver = data;

	struct sk_buff *skb = NULL;

	for (;;) {
		struct nvidia_ta_timeout timeout;

		/* Wait for a packet, a delayed TA call, or a heartbeat. */
		while (!skb) {
			timeout = nvidia_ta_get_next_timeout(driver);
			if (timeout.cause != NVIDIA_TA_TIMEOUT_NONE &&
			    timeout.duration_jiffies == 0) {
				/* already past a deadline, handle it without blocking */
				break;
			}
			bool timeout_expired = false;
			if (timeout.cause == NVIDIA_TA_TIMEOUT_NONE) {
				/* wait forever */
				wait_event_interruptible(
					driver->wq,
					!skb_queue_empty(&driver->tx_queue) ||
						kthread_should_stop());
			} else {
				/* wait with timeout */
				const int wait_status =
					wait_event_interruptible_timeout(
						driver->wq,
						!skb_queue_empty(
							&driver->tx_queue) ||
							kthread_should_stop(),
						timeout.duration_jiffies);
				if (wait_status == 0) {
					timeout_expired = true;
				}
			}
			if (kthread_should_stop()) {
				return 0;
			}
			if (timeout_expired) {
				/* hit timeout, break to handle delayed work or heartbeat */
				break;
			}

			/* got woken up before timeout, check for a packet */
			spin_lock_irq(&driver->tx_queue.lock);
			skb = __skb_dequeue(&driver->tx_queue);
			if (skb && netif_running(driver->netdev) &&
			    netif_queue_stopped(driver->netdev) &&
			    skb_queue_len(&driver->tx_queue) <
				NVIDIA_VROT_QUEUE_SIZE)
				netif_wake_queue(driver->netdev);
			spin_unlock_irq(&driver->tx_queue.lock);
			if (skb)
				break;
		}

		if (!skb) {
			/* handle the cause of the timeout */
			switch (timeout.cause) {
			case NVIDIA_TA_TIMEOUT_WORK_TO_DO:
				nvidia_ta_write_to_tee(
					driver, NULL,
					TA_COMMAND_ID_HANDLE_NON_ATOMIC_OPERATION);
				break;

			case NVIDIA_TA_TIMEOUT_HEARTBEAT:
				send_heartbeat_command_to_tee(driver);
				break;

			default:
				dev_warn(driver->dev, "unexpected timeout cause: %d\n",
					 timeout.cause);
				break;
			}
		} else {
			enum nvidia_vrot_packet_type type;
			int ret;

			type = nvidia_ta_classify_packet(driver, skb);
			if (type == NVIDIA_VROT_PACKET_DROP) {
				driver->netdev->stats.tx_dropped++;
				kfree_skb(skb);
				skb = NULL;
				continue;
			}
			if (type == NVIDIA_VROT_PACKET_CONTROL_REQUEST)
				ret = nvidia_ta_handle_control_request(driver, skb);
			else
				ret = nvidia_ta_write_to_tee(driver, skb,
							     TA_COMMAND_ID_WRITE_PACKET);

			if (ret < 0) {
				driver->netdev->stats.tx_errors++;
				driver->netdev->stats.tx_dropped++;
				dev_err(driver->dev, "failed to send MCTP packet: %d\n",
					ret);
				kfree_skb(skb);
			} else {
				driver->netdev->stats.tx_packets++;
				driver->netdev->stats.tx_bytes += skb->len;
				consume_skb(skb);
			}
			skb = NULL;
		}
	}

	return 0;
}

static void nvidia_vrot_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	/* NOTE: MTU reconfigured outside of setup */
	dev->min_mtu = 68;
	dev->max_mtu = 68;
	dev->mtu = 68;

	dev->hard_header_len = 0;
	dev->tx_queue_len = NVIDIA_VROT_QUEUE_SIZE;
	dev->netdev_ops = &nvidia_vrot_nops;

	dev->addr_len = 0;
}

/*
 * Initializes the endpoint driver.
 * [in,out] driver Endpoint driver. driver->config is already init.
 * [in] name The name to use for the network device and worker thread.
 * 0 on success, -error code on failure.
 */
static int nvidia_ta_mctp_init_endpoint(struct nvidia_ta_mctp_driver *driver,
					const char *name)
{
	struct nvidia_vrot_netdev_priv *priv;
	int ret;

	init_waitqueue_head(&driver->wq);
	skb_queue_head_init(&driver->tx_queue);

	driver->non_atomic = false;
	driver->non_atomic_work_jiffies_64 = 0;
	driver->next_heartbeat_jiffies_64 = 0;
	driver->tee_initialized = false;

	/* create the mctp network device */
	driver->netdev = alloc_netdev(sizeof(struct nvidia_vrot_netdev_priv),
				      name, NET_NAME_PREDICTABLE,
				      nvidia_vrot_netdev_setup);
	if (!driver->netdev) {
		dev_err(driver->dev, "failed to allocate netdev\n");
		return -ENOMEM;
	}
	driver->netdev->min_mtu = driver->config.mtu;
	driver->netdev->mtu = driver->config.mtu;
	driver->netdev->max_mtu = driver->config.mtu;
	priv = netdev_priv(driver->netdev);
	priv->driver = driver;
	ret = mctp_register_netdev(driver->netdev, NULL,
				   MCTP_PHYS_BINDING_VENDOR);
	if (ret) {
		free_netdev(driver->netdev);
		driver->netdev = NULL;
		dev_err(driver->dev, "failed to register netdev: %d\n", ret);
		return ret;
	}

	driver->worker_task =
		kthread_run(nvidia_ta_worker, driver, "%s-worker", name);
	if (IS_ERR(driver->worker_task)) {
		ret = PTR_ERR(driver->worker_task);
		driver->worker_task = NULL;
		dev_err(driver->dev, "failed to start worker thread: %d\n", ret);
		return ret;
	}

	return 0;
}

static int nvidia_vrot_load_devicetree(struct platform_device *pdev,
				       struct nvidia_ta_mctp_config *config)
{
	const struct device_node *np = pdev->dev.of_node;
	const char *ta_uuid_str, *mctp_uuid_str;
	const __be32 *prop;
	int len, num_entries, ret;
	u32 protocol, version;
	u32 i;

	if (!np || !config)
		return -EINVAL;

	/* Parse nvidia,ta-uuid. */
	ret = of_property_read_string(np, "nvidia,ta-uuid", &ta_uuid_str);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read nvidia,ta-uuid from device tree\n");
		return ret;
	}
	ret = uuid_parse(ta_uuid_str, &config->ta_uuid);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse ta_uuid string '%s'\n", ta_uuid_str);
		return -EINVAL;
	}

	/* Parse nvidia,mctp-uuid. */
	ret = of_property_read_string(np, "nvidia,mctp-uuid", &mctp_uuid_str);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read nvidia,mctp-uuid from device tree\n");
		return ret;
	}
	ret = uuid_parse(mctp_uuid_str, &config->mctp_uuid);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse mctp_uuid string '%s'\n", mctp_uuid_str);
		return -EINVAL;
	}

	/* Parse nvidia,mctp-mtu. */
	ret = of_property_read_u32(np, "nvidia,mctp-mtu", &config->mtu);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read nvidia,mctp-mtu from device tree\n");
		return ret;
	}
	if (config->mtu < 68 || config->mtu > 65536) {
		dev_err(&pdev->dev, "Invalid nvidia,mctp-mtu value %u (must be between 68 and 65536)\n",
			config->mtu);
		return -EINVAL;
	}

	/* Parse nvidia,mctp-protocol-versions as u32 pairs. */
	prop = of_get_property(np, "nvidia,mctp-protocol-versions", &len);
	if (!prop || len < 0) {
		dev_err(&pdev->dev, "Failed to get nvidia,mctp-protocol-versions from device tree\n");
		return -EINVAL;
	}
	num_entries = len / sizeof(u32);
	if (len % (2 * sizeof(u32))) {
		dev_err(&pdev->dev, "nvidia,mctp-protocol-versions must have even number of entries (protocol/version pairs), got %d\n",
			num_entries);
		return -EINVAL;
	}
	config->protocols_size = num_entries / 2;
	if (config->protocols_size > NVIDIA_VROT_MAX_PROTOCOLS) {
		dev_err(&pdev->dev, "protocols_size (%u) exceeds max supported %d\n",
			config->protocols_size, NVIDIA_VROT_MAX_PROTOCOLS);
		return -EINVAL;
	}

	for (i = 0; i < config->protocols_size; i++) {
		protocol = be32_to_cpu(prop[i * 2]);
		version = be32_to_cpu(prop[i * 2 + 1]);
		if (protocol > 0xFF) {
			dev_err(&pdev->dev, "protocol value 0x%x at index %u exceeds u8 range\n",
				protocol, i);
			return -EINVAL;
		}
		if (protocol == 0x00) {
			dev_err(&pdev->dev, "protocol value 0x%x at index %u is reserved (control protocol)\n",
				protocol, i);
			return -EINVAL;
		}
		if (protocol == 0xFF) {
			dev_err(&pdev->dev, "protocol value 0x%x at index %u is reserved (base version)\n",
				protocol, i);
			return -EINVAL;
		}
		config->protocols[i].protocol = (u8)protocol;
		memcpy(config->protocols[i].version, &version, sizeof(version));
	}

	return 0;
}

static int nvidia_vrot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvidia_ta_mctp_driver *driver;
	int ret;

	driver = devm_kzalloc(dev, sizeof(*driver), GFP_KERNEL);
	if (!driver) {
		dev_err(dev, "Failed to allocate memory for driver\n");
		return -ENOMEM;
	}
	driver->dev = dev;
	platform_set_drvdata(pdev, driver);

	ret = nvidia_vrot_load_devicetree(pdev, &driver->config);
	if (ret != 0) {
		dev_err(dev,
			"Failed to load device tree properties for device '%s' (devicetree path: %s), ret: %d\n",
			dev_name(dev), of_node_full_name(dev->of_node), ret);
		clean_driver(pdev);
		return ret;
	}

	ret = nvidia_ta_mctp_init_endpoint(driver, dev_name(dev));
	if (ret != 0) {
		dev_err(dev,
			"failed to init endpoint for device '%s' (devicetree path: %s), ret: %d\n",
			dev_name(dev), of_node_full_name(dev->of_node), ret);
		clean_driver(pdev);
		return ret;
	}

	dev_info(dev, "VRoT MCTP endpoint registered\n");
	return 0;
}

static void nvidia_vrot_remove(struct platform_device *pdev)
{
	clean_driver(pdev);
}

static const struct of_device_id nvidia_vrot_match[] = {
	{ .compatible = "nvidia,optee,vrot" },
	{},
};
MODULE_DEVICE_TABLE(of, nvidia_vrot_match);

static struct platform_driver nvidia_vrot_driver = {
	.probe = nvidia_vrot_probe,
	.remove = nvidia_vrot_remove,
	.driver = {
		.name = "nvidia-optee-vrot",
		.of_match_table = nvidia_vrot_match,
	},
};

module_platform_driver(nvidia_vrot_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVIDIA VRoT MCTP Trusted Application Driver");
