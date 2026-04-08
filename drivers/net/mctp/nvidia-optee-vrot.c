/// @file
/// @brief Kernel module for MCTP communication with NVIDIA's VRoT.
/// @details VRoTs are optee Trusted Applications that each manage a single endpoint.
/// Enable with the following device tree to create a mctp device called mctpvrot0.
/// @code{.dts}
/// {
/// 	mctpvrot0 {
/// 		compatible = "nvidia,optee,vrot";
/// 		nvidia,ta-uuid = "14fe76f6-6114-499c-9b81-f90ebde9c50c";
/// 		nvidia,mctp-uuid = "14fe76f6-6114-499c-9b81-f90ebde9c50c";
/// 		nvidia,mctp-mtu = <5000>;
/// 		nvidia,mctp-protocol-versions = <0x1 0xF1F0F000
/// 		                                 0x5 0x0
/// 		                                 0x7e 0xF1F0FF00>;
/// 		status = "okay";
/// 	};
/// };
/// @endcode
///
/// Device Tree Properties:
/// - nvidia,ta-uuid: The UUID of the trusted application.
/// - nvidia,mctp-uuid: The UUID used in the GET UUID MCTP control message.
/// - nvidia,mctp-mtu: The max transmission unit for MCTP communication.
/// - nvidia,mctp-protocol-versions: A mapping of supported MCTP procotols
///     to the version that should be reported in a Get MCTP Version Support command.
///     Use zero to not respond to version requests.
///     Non-zero values are formatted in the standard MCTP version format.
///     The example devicetree reports PLDM version 1.0.0 SPDM version unknown
///     and vendor defined 1.0.

#include <linux/build_bug.h>
#include <linux/device.h>
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
#define TEE_IOCTL_PARAM_ATTR_TYPE_NONE 0

#define LOG_IMPL(lvl, fmt, ...) \
	printk(lvl "NVIDIA-OPTEE-VROT: " fmt "\n", ##__VA_ARGS__)

/// @brief Helper logging macros.
/// @details Has the same API as printf().
/// @{
#ifdef DEBUG
#define LOG_INF(...) LOG_IMPL(KERN_INFO, __VA_ARGS__)
#else
#define LOG_INF(...) \
	do {         \
	} while (0)
#endif
#define LOG_WRN(...) LOG_IMPL(KERN_WARNING, __VA_ARGS__)
#define LOG_ERR(...) LOG_IMPL(KERN_ERR, __VA_ARGS__)
/// @}

#define TA_COMMAND_ID_WRITE_PACKET 0
#define TA_COMMAND_ID_HANDLE_NON_ATOMIC_OPERATION 1
#define TA_COMMAND_ID_HEARTBEAT 3

#define NVIDIA_VROT_QUEUE_SIZE 20

#define NVIDIA_VROT_MAX_PROTOCOLS 10

/// @brief Entries that are encoded in the nvidia,mctp-protocol-versions property.
/// @details The property is an array of u32.
/// Odd indexes represent protocol IDs and even indexes are the version of the protocol.
/// Is needed because the VRoTs do not implement the control MCTP protocol so we
/// fake it in the kernel module.
/// @note Entries should not be provided for the control protocol or base MCTP version
/// as that is controlled by this module.
struct nvidia_ta_mctp_protocol_version {
	/// @brief The MCTP protocol ID.
	/// @example 1 is PLDM, 5 is SPDM.
	u8 protocol;
	/// @brief The version number.
	/// @note Use 0 if you do not which to respond to a
	/// Get MCTP Version Support command for this protocol.
	/// @note This is a u32 in native endianness, using a u8[4] to reduce padding.
	/// @example 0xF1F0F000 is 1.0.0.
	u8 version[4];
};

/// @brief Immutable config taken from device tree.
struct nvidia_ta_mctp_config {
	/// @brief nvidia,ta-uuid property.
	/// @details The UUID of the trusted application that acts as a VRoT.
	uuid_t ta_uuid;
	/// @brief nvidia,mctp-uuid property.
	/// @details The UUID to use in response to a Get Endpoint UUID MCTP Control Message.
	uuid_t mctp_uuid;
	/// @brief nvidia,mctp-mtu property.
	/// @details The max allowed MTU for the VRoT trusted application.
	/// @note Is not discovered at startup via TEE function calls because we want to delay
	/// TA initialization to as late as possible to allow for tee-supplicant to be started
	//  which may occur after kernel module loading.
	u32 mtu;
	/// @brief The number of valid entries in protocols.
	u32 protocols_size;
	/// @brief nvidia,mctp-protocol-versions property.
	struct nvidia_ta_mctp_protocol_version
		protocols[NVIDIA_VROT_MAX_PROTOCOLS];
};

/// @brief State used by callbacks.
struct nvidia_ta_mctp_driver {
	/// @brief Info from device tree.
	struct nvidia_ta_mctp_config config;

	/// @brief Trusted execution context.
	struct tee_context *tee_ctx;
	/// @brief Shared Memory with the trusted environment.
	struct tee_shm *tee_pool;
	/// @brief Trusted execution session id.
	u32 tee_session_id;

	/// @brief Network device used by the linux kernel for this driver.
	struct net_device *netdev;

	/// @brief Worker task for handling MCTP packets.
	struct task_struct *worker_task;

	/// @brief wait queue that is woken when tx_queue is pushed to
	wait_queue_head_t wq;

	/// @brief Packet queue from the network device to the tx worker thread.
	struct sk_buff_head tx_queue;

	/// @brief The timestamp when we should service non-atomic work.
	/// @details Is only valid if non_atomic is true.
	u64 non_atomic_work_jiffies_64;

	/// @brief The timestamp when we should next send a heartbeat.
	/// @details Is only valid if tee_initialized is true.
	u64 next_heartbeat_jiffies_64;

	/// @brief If the tee is initialized. Used to deduplicate initialization since since we initialize once on first write (to ensure TEE-Supplicant is running)
	u32 tee_initialized : 1;

	/// @brief Non-atomic operation flag
	u32 non_atomic : 1;
};

/// @brief Private state for the mctp network device.
struct nvidia_vrot_netdev_priv {
	struct nvidia_ta_mctp_driver *driver;
};

static inline void log_buffer_hex_chunks(const char *prefix, const void *buf,
					 size_t count)
{
#ifndef DEBUG
	(void)prefix;
	(void)buf;
	(void)count;
#else
#define CHUNK_SIZE 16
	const unsigned char *ubuf = (const unsigned char *)buf;
	size_t i, j;
	char line[CHUNK_SIZE *
		  3]; // 2 chars per byte + 1 space or null terminator per byte
	for (i = 0; i < count; i += CHUNK_SIZE) {
		size_t chunk = (count - i > CHUNK_SIZE) ? CHUNK_SIZE :
							  (count - i);
		size_t pos = 0;
		for (j = 0; j < chunk; j++) {
			pos += sprintf(&line[pos], "%02x%s", ubuf[i + j],
				       (j < chunk - 1) ? " " : "");
		}
		line[pos] = '\0';
		LOG_INF("%s [%04zu-%04zu]: %s", prefix, i, i + chunk - 1, line);
	}
#undef CHUNK_SIZE
#endif
}

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	switch (ver->impl_id) {
	case TEE_IMPL_ID_OPTEE:
		return 1;
	default:
		return 0;
	}
}

static int open_ta_session(struct nvidia_ta_mctp_driver *driver)
{
	int ret;
	struct tee_ioctl_open_session_arg sess_arg;
	memset(&sess_arg, 0, sizeof(sess_arg));

	if (driver == NULL) {
		LOG_ERR("open_ta_session: driver is NULL");
		return -EIO;
	}

	export_uuid(sess_arg.uuid, &driver->config.ta_uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;
	/* Open the session */
	ret = tee_client_open_session(driver->tee_ctx, &sess_arg, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to open OP-TEE session");
		return -EINVAL;
	}

	if (sess_arg.ret != 0) {
		LOG_ERR("Session open failed with TA error: 0x%x",
			sess_arg.ret);
		return -EINVAL;
	}

	/* Store the session ID */
	driver->tee_session_id = sess_arg.session;
	LOG_INF("Kernel: OP-TEE session opened with ID: %u",
		driver->tee_session_id);

	return 0;
}

static int send_heartbeat_command_to_tee(struct nvidia_ta_mctp_driver *driver)
{
	driver->next_heartbeat_jiffies_64 =
		get_jiffies_64() +
		msecs_to_jiffies(HEARTBEAT_COMMAND_INTERVAL_MS);

	struct tee_ioctl_invoke_arg invoke_arg = {
		.func = TA_COMMAND_ID_HEARTBEAT,
		.session = driver->tee_session_id,
		.num_params = 4
	};

	struct tee_param params[4] = { { 0 } };
	void *va;
	int ret;

	// Get virtual address of shared memory
	va = tee_shm_get_va(driver->tee_pool, 0);
	if (IS_ERR(va)) {
		LOG_ERR("Failed to get virtual address: %ld", PTR_ERR(va));
		return -EINVAL;
	}
	// Setup parameter for TA
	// only parameter is the shared memory buffer to hold the vrot state
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	params[0].u.memref.shm = driver->tee_pool;
	params[0].u.memref.shm_offs = 0;
	params[0].u.memref.size = driver->config.mtu;

	// Setup parameters 1-3: TYPE_NONE
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	LOG_INF("Sending param types: p0=0x%llx, p1=0x%llx, p2=0x%llx, p3=0x%llx",
		params[0].attr, params[1].attr, params[2].attr, params[3].attr);

	// Invoke TA
	ret = tee_client_invoke_func(driver->tee_ctx, &invoke_arg, params);
	LOG_INF("TA invocation returned: ret=%d, invoke_arg.ret=0x%x", ret,
		invoke_arg.ret);
	if (ret < 0 || invoke_arg.ret != 0) {
		if (ret < 0) {
			LOG_ERR("TA invocation failed: ret=%d. Check if TA is available and initialized on the system.",
				ret);
			return -EIO;
		}
		if (invoke_arg.ret != 0) {
			LOG_ERR("TA invocation returned error: ta_ret=0x%x",
				invoke_arg.ret);
		}
		return -EIO;
	}

	LOG_INF("TA invocation completed");
	size_t out_size = params[0].u.memref.size;

	// at this point, we have the data in the shared memory from the tee invocation.
	char *vrot_state = tee_shm_get_va(driver->tee_pool, 0);
	if (vrot_state == NULL) {
		LOG_ERR("Failed to get virtual address of shared memory");
		return -EFAULT;
	}

	log_buffer_hex_chunks("VRoT state", vrot_state, out_size);
	return 0;
}

static int init_tee(struct nvidia_ta_mctp_driver *driver)
{
	int ret;
	if (driver == NULL) {
		LOG_ERR("init_tee: driver is NULL");
		return -EIO;
	}

	if (driver->tee_initialized) {
		return 0; // Already initialized
	}

	driver->tee_ctx =
		tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
	if (IS_ERR_OR_NULL(driver->tee_ctx)) {
		LOG_ERR("Failed to open TEE context: %ld",
			PTR_ERR(driver->tee_ctx));
		return -ENODEV;
	}

	ret = open_ta_session(driver);
	if (ret < 0) {
		tee_client_close_context(driver->tee_ctx);
		driver->tee_ctx = NULL;
		return ret;
	}

	driver->tee_pool =
		tee_shm_alloc_kernel_buf(driver->tee_ctx, driver->config.mtu);
	if (IS_ERR_OR_NULL(driver->tee_pool)) {
		LOG_ERR("Failed to allocate shared memory: %ld",
			PTR_ERR(driver->tee_pool));
		tee_client_close_session(driver->tee_ctx,
					 driver->tee_session_id);
		tee_client_close_context(driver->tee_ctx);
		driver->tee_ctx = NULL;
		driver->tee_session_id = 0;
		return -ENOMEM;
	}

	LOG_INF("Shared memory allocated: size=%zu", driver->tee_pool->size);
	driver->tee_initialized = true;
	// want an immediate heartbeat
	driver->next_heartbeat_jiffies_64 = get_jiffies_64();
	return 0;
}

static int invoke_ta_function(struct nvidia_ta_mctp_driver *driver,
			      struct sk_buff *skb, int command_id,
			      size_t *out_size)
{
	if (driver == NULL) {
		LOG_ERR("invoke_ta_function: driver is NULL");
		return -EIO;
	}

	size_t count = 0;
	if (skb) {
		count = skb->len;
	}

	struct tee_ioctl_invoke_arg invoke_arg = {
		.func = command_id,
		.session = driver->tee_session_id,
		.num_params = 4
	};

	struct tee_param params[4] = { { 0 } };
	void *va;
	int ret;

	if (!driver->tee_pool || !driver->tee_ctx) {
		LOG_ERR("Shared memory or context not initialized");
		return -EINVAL;
	}

	// Get virtual address of shared memory
	va = tee_shm_get_va(driver->tee_pool, 0);
	if (IS_ERR(va)) {
		LOG_ERR("Failed to get virtual address: %ld", PTR_ERR(va));
		return -EINVAL;
	}

	// Copy input data to shared memory
	if (count > driver->tee_pool->size) {
		LOG_WRN("Input size (%zu) exceeds shared memory size (%zu)",
			count, driver->tee_pool->size);
		count = driver->tee_pool->size;
	}

	// copy input data to shared memory
	if (count != 0) {
		skb_copy_bits(skb, 0, va, count);
		log_buffer_hex_chunks("Tx to TA", va, count);
	}

	// Setup parameter for TA
	// first parameter is the shared memory buffer
	params[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	params[0].u.memref.shm = driver->tee_pool;
	params[0].u.memref.shm_offs = 0;
	params[0].u.memref.size = count;

	// second parameter will be used to get bool indicating whether the TA is handling a non-atomic operation, and a delay for invoking the TA again
	params[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	params[1].u.value.a = 0;
	params[1].u.value.b = 0;

	// Setup parameters 2-3: TYPE_NONE for now
	params[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
	params[3].attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;

	LOG_INF("Sending param types: p0=0x%llx, p1=0x%llx, p2=0x%llx, p3=0x%llx",
		params[0].attr, params[1].attr, params[2].attr, params[3].attr);
	LOG_INF("Sending size: %zu", count);

	// Invoke TA
	ret = tee_client_invoke_func(driver->tee_ctx, &invoke_arg, params);
	LOG_INF("TA invocation returned: ret=%d, invoke_arg.ret=0x%x", ret,
		invoke_arg.ret);
	if (ret < 0 || invoke_arg.ret != 0) {
		*out_size = 0;
		driver->non_atomic = false;
		driver->non_atomic_work_jiffies_64 = 0;
		if (ret < 0) {
			LOG_ERR("TA invocation failed: ret=%d. Check if TA is available and initialized on the system.",
				ret);
			return -EIO;
		}
		if (invoke_arg.ret != 0) {
			LOG_ERR("TA invocation returned error: ta_ret=0x%x",
				invoke_arg.ret);
			return -EIO;
		}
	}

	LOG_INF("TA invocation completed");
	*out_size = params[0].u.memref.size;
	driver->non_atomic = (bool)params[1].u.value.a;
	const u32 delay = params[1].u.value.b;
	if (driver->non_atomic) {
		driver->non_atomic_work_jiffies_64 = get_jiffies_64() + delay;
		LOG_INF("TA is handling a non-atomic operation, delay: %u",
			delay);
	} else {
		driver->non_atomic_work_jiffies_64 = 0;
	}
	return count;
}

static int handle_data_from_ta(struct nvidia_ta_mctp_driver *driver,
			       size_t size)
{
	int ret = -EINVAL;

	const char *const shared_mem_data = tee_shm_get_va(driver->tee_pool, 0);
	if (shared_mem_data == NULL) {
		LOG_ERR("Failed to get virtual address of shared memory");
		return -EFAULT;
	}

	if (size <= 4) {
		LOG_ERR("Response size too small for MCTP packet: %u",
			(unsigned)size);
		return -EINVAL;
	}

	log_buffer_hex_chunks("Rx from TA", shared_mem_data, size);

	struct sk_buff *skb;
	skb = netdev_alloc_skb(driver->netdev, size);
	if (!skb) {
		LOG_ERR("failed to allocate skb for read packet");
		return -ENOMEM;
	}
	skb->protocol = htons(ETH_P_MCTP);
	(void)skb_put_data(skb, shared_mem_data, size);
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);

	struct mctp_skb_cb *const cb = __mctp_cb(skb);
	cb->halen = 0;

	ret = netif_receive_skb(skb);
	if (ret != NET_RX_SUCCESS) {
		LOG_ERR("netif_rx failed with %d", (int)ret);
		// NOTE: skb is already freed by netif_receive_skb on error
		return -EINVAL;
	}
	return 0;
}

static void nvidia_ta_write_to_tee(struct nvidia_ta_mctp_driver *driver,
				   struct sk_buff *skb, int command_id)
{
	int ret = -EINVAL;
	LOG_INF("nvidia_ta_write_to_tee: Writing to TEE");
	ret = init_tee(driver); // Initialize on first write
	if (ret < 0) {
		LOG_ERR("Kernel: TEE initialization failed: %d", ret);
		return;
	}

	size_t out_size = 0;
	ret = invoke_ta_function(driver, skb, command_id, &out_size);
	if (ret < 0) {
		LOG_ERR("Kernel: Failed to invoke TA function: %d", ret);
		return;
	}

	if (out_size == 0) {
		return;
	}

	ret = handle_data_from_ta(driver, out_size);
	if (ret < 0) {
		LOG_ERR("Kernel: Failed to handle data read from TA: %d", ret);
		return;
	}
}

/// @brief Performs cleanup for our custom driver bound to the given platform_device.
/// @param pdev The device having its driver cleaned up.
/// @note The driver can be in any stage of initialization including completely unbound.
static void clean_driver(struct platform_device *pdev)
{
	if (pdev == NULL) {
		return;
	}

	struct nvidia_ta_mctp_driver *const driver =
		(struct nvidia_ta_mctp_driver *)platform_get_drvdata(pdev);
	if (driver == NULL) {
		return;
	}

	// perform any needed cleanup
	// Stop the worker thread first
	if (driver->worker_task != NULL) {
		kthread_stop(driver->worker_task);
		driver->worker_task = NULL;
	}

	// Ensure network device is down and purge any pending packets
	if (driver->netdev != NULL) {
		// Stop the queue to prevent new packets
		netif_tx_disable(driver->netdev);

		// Purge any remaining packets in the queue
		skb_queue_purge(&driver->tx_queue);

		// Explicitly bring the device down if it's still running
		rtnl_lock();
		if (netif_running(driver->netdev)) {
			dev_close(driver->netdev);
		}
		rtnl_unlock();

		// Unregister the netdev - this should handle final cleanup
		mctp_unregister_netdev(driver->netdev);
		free_netdev(driver->netdev);
		driver->netdev = NULL;
	}

	if (driver->tee_initialized) {
		tee_client_close_session(driver->tee_ctx,
					 driver->tee_session_id);
		driver->tee_initialized = false;
	}
	if (driver->tee_pool != NULL) {
		tee_shm_free(driver->tee_pool);
		driver->tee_pool = NULL;
	}
	if (driver->tee_ctx != NULL) {
		tee_client_close_context(driver->tee_ctx);
		driver->tee_ctx = NULL;
	}

	// the driver is always heap allocated and we have exclusive ownership of it, free it.
	platform_set_drvdata(pdev, NULL);
	kfree(driver);
}

netdev_tx_t nvidia_vrot_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	netdev_tx_t status = NETDEV_TX_BUSY;

	struct nvidia_vrot_netdev_priv *const priv = netdev_priv(dev);
	struct nvidia_ta_mctp_driver *const driver = priv->driver;

	unsigned long flags;

	// locking the queue for the entire push so we can atomically inspect
	// the length to manage the network interface queue enable state.
	spin_lock_irqsave(&driver->tx_queue.lock, flags);
	if (skb_queue_len(&driver->tx_queue) >= NVIDIA_VROT_QUEUE_SIZE) {
		// error: queue already full
		status = NETDEV_TX_BUSY;
		LOG_WRN("mctp tx queue overflow");
		netif_stop_queue(dev);
	} else {
		// push the packet
		status = NETDEV_TX_OK;
		__skb_queue_tail(&driver->tx_queue, skb);
		if (skb_queue_len(&driver->tx_queue) ==
		    NVIDIA_VROT_QUEUE_SIZE) {
			// stop the queue to prevent excessive RAM usage
			netif_stop_queue(dev);
		}
	}
	spin_unlock_irqrestore(&driver->tx_queue.lock, flags);

	if (status == NETDEV_TX_OK) {
		wake_up(&driver->wq);
	}
	return status;
}

int nvidia_vrot_open(struct net_device *dev)
{
	// TODO: could start/resume worker threads
	netif_start_queue(dev);
	return 0;
}

int nvidia_vrot_stop(struct net_device *dev)
{
	// TODO: could stop worker threads
	netif_stop_queue(dev);
	return 0;
}

static struct net_device_ops nvidia_vrot_nops = {
	.ndo_start_xmit = nvidia_vrot_start_xmit,
	.ndo_open = nvidia_vrot_open,
	.ndo_stop = nvidia_vrot_stop,
};

// mctp header byte indexes
static const size_t nv_mctp_ver_index = 0;
static const size_t nv_mctp_dst_index = 1;
static const size_t nv_mctp_src_index = 2;
static const size_t nv_mctp_flags_index = 3;

// only support version 1 packets with no reserved bits set
static const u8 nv_mctp_expected_version_byte = 1;

// flag bits of interest
static const u8 nv_mctp_som_bit = 1 << 7;
static const u8 nv_mctp_eom_bit = 1 << 6;
static const u8 nv_mctp_to_bit = 1 << 3;

// control header bytes
static const size_t nv_mctp_ctrl_msg_type_index = 4;
static const size_t nv_mctp_ctrl_flags_index = 5;
static const size_t nv_mctp_ctrl_cc_index = 6;
static const size_t nv_mctp_ctrl_rsp_index = 7;

// control header values
static const u8 nv_mctp_ctrl_msg_type = 0;

// ctrl flags bits of interest
static const u8 nv_mctp_ctrl_req_bit = 1 << 7;
static const u8 nv_mctp_ctrl_datagram_bit = 1 << 6;

// used ctrl response codes
static const u8 nv_mctp_ctrl_rsp_ok = 0;
static const u8 nv_mctp_ctrl_rsp_error = 1;
static const u8 nv_mctp_ctrl_rsp_bad_data = 2;
static const u8 nv_mctp_ctrl_rsp_bad_cmd = 5;

// control header sizes
static const size_t nv_mctp_ctrl_cmd_data_start = 7;
static const size_t nv_mctp_ctrl_rsp_data_start = 8;

// must have entire header
static const size_t nv_min_ctrl_req_size = nv_mctp_ctrl_cmd_data_start;
// must always fit in a min MTU packet
#define NV_MAX_CTRL_MSG_SIZE 68

static bool nvidia_ta_is_control_request(struct sk_buff *skb)
{
	if (skb->len < nv_min_ctrl_req_size) {
		return false;
	}
	if (skb->len > NV_MAX_CTRL_MSG_SIZE) {
		return false;
	}

	unsigned char data[NV_MAX_CTRL_MSG_SIZE] = { 0 };
	if (0 != skb_copy_bits(skb, 0, data, nv_min_ctrl_req_size)) {
		LOG_WRN("skb_copy_bits failed when detecting packet type");
		return false;
	}
	if (data[nv_mctp_ver_index] != nv_mctp_expected_version_byte) {
		return false;
	}

	const u8 required_bits = nv_mctp_som_bit | nv_mctp_eom_bit |
				 nv_mctp_to_bit;
	if ((data[nv_mctp_flags_index] & required_bits) != required_bits) {
		return false;
	}
	if (data[nv_mctp_ctrl_msg_type_index] != nv_mctp_ctrl_msg_type) {
		// different packet type
		return false;
	}
	return true;
}

static int
nvidia_ta_handle_control_request(struct nvidia_ta_mctp_driver *driver,
				 const struct sk_buff *skb)
{
	// Ensure buffer is large enough for maximum control responses
	BUILD_BUG_ON(nv_mctp_ctrl_rsp_data_start + 16 >
		     NV_MAX_CTRL_MSG_SIZE); // Get Endpoint UUID
	BUILD_BUG_ON(nv_mctp_ctrl_rsp_data_start + 1 +
			     NVIDIA_VROT_MAX_PROTOCOLS >
		     NV_MAX_CTRL_MSG_SIZE); // Get Message Type Support

	LOG_INF("handling control message...");

	unsigned char cmd[NV_MAX_CTRL_MSG_SIZE] = { 0 };
	if (skb_copy_bits(skb, 0, cmd, skb->len) != 0) {
		return -EFAULT;
	}
	log_buffer_hex_chunks("Control CMD", cmd, skb->len);

	unsigned char rsp[NV_MAX_CTRL_MSG_SIZE] = { 0 };

	// fill MCTP header
	rsp[nv_mctp_ver_index] = cmd[nv_mctp_ver_index];
	rsp[nv_mctp_dst_index] = cmd[nv_mctp_src_index];
	rsp[nv_mctp_src_index] = cmd[nv_mctp_dst_index];
	rsp[nv_mctp_flags_index] = cmd[nv_mctp_flags_index] & ~nv_mctp_to_bit;

	// fill control header
	rsp[nv_mctp_ctrl_msg_type_index] = cmd[nv_mctp_ctrl_msg_type_index];
	rsp[nv_mctp_ctrl_flags_index] = cmd[nv_mctp_ctrl_flags_index] &
					~nv_mctp_ctrl_req_bit;
	rsp[nv_mctp_ctrl_cc_index] = cmd[nv_mctp_ctrl_cc_index];
	rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_error;

	// payload-less length
	size_t rsp_len = nv_mctp_ctrl_rsp_data_start;

	// supporting only mandatory and actually used commands
	switch (cmd[nv_mctp_ctrl_cc_index]) {
	case 1:
		LOG_INF("Set Endpoint ID");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		// accepted + no pool
		rsp[nv_mctp_ctrl_rsp_data_start + 0] = 0;
		// echo back EID, is 2nd data byte in command
		rsp[nv_mctp_ctrl_rsp_data_start + 1] =
			cmd[nv_mctp_ctrl_cmd_data_start + 1];
		// no pool
		rsp[nv_mctp_ctrl_rsp_data_start + 2] = 0;
		rsp_len = nv_mctp_ctrl_rsp_data_start + 3;
		break;

	case 2:
		LOG_INF("Get Endpoint ID");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		// echo back endpoint ID
		rsp[nv_mctp_ctrl_rsp_data_start + 0] = cmd[nv_mctp_dst_index];
		// simple endpoint + dynamic ID
		rsp[nv_mctp_ctrl_rsp_data_start + 1] = 0;
		// no transport specific info
		rsp[nv_mctp_ctrl_rsp_data_start + 2] = 0;
		rsp_len = nv_mctp_ctrl_rsp_data_start + 3;
		break;

	case 3:
		LOG_INF("Get Endpoint UUID");
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
		memcpy(&rsp[nv_mctp_ctrl_rsp_data_start],
		       &driver->config.mctp_uuid, 16);
		rsp_len = nv_mctp_ctrl_rsp_data_start + 16;
		break;

	case 4: {
		LOG_INF("Get Version Support");
		u32 version = 0;
		switch (cmd[nv_mctp_ctrl_cmd_data_start]) {
		case 0x00:
			// control version 1.3.1
			version = 0xF1F3F100;
			break;

		case 0xFF:
			// base version 1.3.1
			version = 0xF1F3F100;
			break;

		default:
			// other protocols, use version set via device tree
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

		if (version == 0) {
			// version unknown, error
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_data;
		} else {
			rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_ok;
			// versions are stored locally in native endianness, but MCTP wants big endian.
			version = cpu_to_be32(version);
			memcpy(&rsp[nv_mctp_ctrl_rsp_data_start], &version,
			       sizeof(version));
			rsp_len = nv_mctp_ctrl_rsp_data_start + sizeof(version);
		}
		break;
	}

	case 5:
		LOG_INF("Get Message Type Support");
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
		LOG_WRN("Unsupported control command: 0x%02X",
			cmd[nv_mctp_ctrl_cc_index]);
		rsp[nv_mctp_ctrl_rsp_index] = nv_mctp_ctrl_rsp_bad_cmd;
		break;
	}

	log_buffer_hex_chunks("Control RSP", rsp, rsp_len);

	struct sk_buff *rsp_skb;
	rsp_skb = netdev_alloc_skb(driver->netdev, rsp_len);
	if (!rsp_skb) {
		LOG_ERR("failed to allocate skb for read packet");
		return -ENOMEM;
	}
	rsp_skb->protocol = htons(ETH_P_MCTP);
	(void)skb_put_data(rsp_skb, rsp, rsp_len);
	skb_reset_mac_header(rsp_skb);
	skb_reset_network_header(rsp_skb);

	struct mctp_skb_cb *const cb = __mctp_cb(rsp_skb);
	cb->halen = 0;

	const int ret = netif_receive_skb(rsp_skb);
	if (ret != NET_RX_SUCCESS) {
		LOG_ERR("netif_rx failed with %d", (int)ret);
		// NOTE: skb is already freed by netif_receive_skb on error
		return -EINVAL;
	}

	return 0;
}

enum nvidia_ta_timeout_cause {
	/// @brief There is no reason to ever timeout.
	NVIDIA_TA_TIMEOUT_NONE,
	/// @brief Should timeout to process delayed work.
	NVIDIA_TA_TIMEOUT_WORK_TO_DO,
	/// @brief Should timeout to send a heartbeat.
	NVIDIA_TA_TIMEOUT_HEARTBEAT,
};

struct nvidia_ta_timeout {
	enum nvidia_ta_timeout_cause cause;
	/// @note Will be MAX_SCHEDULE_TIMEOUT if cause is none.
	u32 duration_jiffies;
};

static struct nvidia_ta_timeout
nvidia_ta_get_next_timeout(struct nvidia_ta_mctp_driver *driver)
{
	const u64 now = get_jiffies_64();

	bool should_timeout = false;
	u32 jiffies_til_work = (u32)-1;
	if (driver->tee_initialized && driver->non_atomic) {
		should_timeout = true;
		if (driver->non_atomic_work_jiffies_64 < now) {
			jiffies_til_work = 0;
		} else {
			u64 diff = driver->non_atomic_work_jiffies_64 - now;
			jiffies_til_work =
				(u32)min_t(u64, diff, MAX_SCHEDULE_TIMEOUT);
		}
	}

	u32 jiffies_til_heartbeat = (u32)-1;
	if (driver->tee_initialized) {
		should_timeout = true;
		if (driver->next_heartbeat_jiffies_64 < now) {
			jiffies_til_heartbeat = 0;
		} else {
			u64 diff = driver->next_heartbeat_jiffies_64 - now;
			jiffies_til_heartbeat =
				(u32)min_t(u64, diff, MAX_SCHEDULE_TIMEOUT);
		}
	}

	if (!should_timeout) {
		struct nvidia_ta_timeout timeout = {
			.cause = NVIDIA_TA_TIMEOUT_NONE,
			.duration_jiffies = MAX_SCHEDULE_TIMEOUT,
		};
		return timeout;
	}

	if (jiffies_til_work <= jiffies_til_heartbeat) {
		struct nvidia_ta_timeout timeout = {
			.cause = NVIDIA_TA_TIMEOUT_WORK_TO_DO,
			.duration_jiffies = jiffies_til_work,
		};
		return timeout;
	}

	struct nvidia_ta_timeout timeout = {
		.cause = NVIDIA_TA_TIMEOUT_HEARTBEAT,
		.duration_jiffies = jiffies_til_heartbeat,
	};
	return timeout;
}

static int nvidia_ta_worker(void *data)
{
	struct nvidia_ta_mctp_driver *driver = data;

	struct sk_buff *skb = NULL;

	for (;;) {
		struct nvidia_ta_timeout timeout;

		// attempt to get a packet to send until we need to handle a non-atomic operation or heartbeat
		while (!skb) {
			timeout = nvidia_ta_get_next_timeout(driver);
			if (timeout.cause != NVIDIA_TA_TIMEOUT_NONE &&
			    timeout.duration_jiffies == 0) {
				// already past a deadline, handle it without blocking
				break;
			}
			bool timeout_expired = false;
			if (timeout.cause == NVIDIA_TA_TIMEOUT_NONE) {
				// wait forever
				wait_event_interruptible(
					driver->wq,
					!skb_queue_empty(&driver->tx_queue) ||
						kthread_should_stop());
			} else {
				// wait with timeout
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
				// hit timeout, break to handle delayed work or heartbeat
				break;
			}

			// got woken up before timeout, check for a packet
			skb = skb_dequeue(&driver->tx_queue);
			netif_start_queue(driver->netdev);
			if (skb) {
				break;
			}
		}

		if (!skb) {
			// handle the cause of the timeout
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
				LOG_WRN("unexpected read timeout cause: %d",
					(int)timeout.cause);
				break;
			}
		} else if (nvidia_ta_is_control_request(skb)) {
			// handle control packet locally
			const int ret =
				nvidia_ta_handle_control_request(driver, skb);
			if (ret != 0) {
				LOG_ERR("nvidia_ta_handle_control_request failed, ret: %d",
					ret);
			}
		} else {
			// forward non-control packet
			nvidia_ta_write_to_tee(driver, skb,
					       TA_COMMAND_ID_WRITE_PACKET);
		}

		if (skb) {
			kfree_skb(skb);
			skb = NULL;
		}
	}

	return 0;
}

void nvidia_vrot_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	// NOTE: MTU reconfigured outside of setup
	dev->min_mtu = 68;
	dev->max_mtu = 68;
	dev->mtu = 68;

	dev->hard_header_len = 0;
	dev->tx_queue_len = NVIDIA_VROT_QUEUE_SIZE;
	dev->netdev_ops = &nvidia_vrot_nops;

	dev->addr_len = 0;
}

/// @brief Initializes the endpoint driver.
/// @param [in,out] driver Endpoint driver. driver->config is already init.
/// @param [in] name The name to use for the network device and worker thread.
/// @returns 0 on success, -error code on failure.
static int
nvidia_ta_mctp_init_endpoint(struct nvidia_ta_mctp_driver *const driver,
			     const char *name)
{
	int result = -1;

	init_waitqueue_head(&driver->wq);
	skb_queue_head_init(&driver->tx_queue);

	driver->non_atomic = false;
	driver->non_atomic_work_jiffies_64 = 0;
	driver->next_heartbeat_jiffies_64 = 0;
	driver->tee_initialized = false;

	// create the mctp network device
	driver->netdev = alloc_netdev(sizeof(struct nvidia_vrot_netdev_priv),
				      name, NET_NAME_PREDICTABLE,
				      nvidia_vrot_netdev_setup);
	if (!driver->netdev) {
		LOG_ERR("failed to allocate netdev");
		return -ENOMEM;
	}
	driver->netdev->min_mtu = driver->config.mtu;
	driver->netdev->mtu = driver->config.mtu;
	driver->netdev->max_mtu = driver->config.mtu;
	struct nvidia_vrot_netdev_priv *priv = netdev_priv(driver->netdev);
	priv->driver = driver;
	result = mctp_register_netdev(driver->netdev, NULL,
				      MCTP_PHYS_BINDING_VENDOR);
	if (result) {
		free_netdev(driver->netdev);
		driver->netdev = NULL;
		LOG_ERR("failed to register netdev");
		return result;
	}

	driver->worker_task =
		kthread_run(nvidia_ta_worker, driver, "%s-worker", name);
	if (IS_ERR(driver->worker_task)) {
		LOG_ERR("failed to start worker thread");
		driver->worker_task = NULL;
		return -EINVAL;
	}

	return 0;
}

static int nvidia_vrot_load_devicetree(struct platform_device *pdev,
				       struct nvidia_ta_mctp_config *config)
{
	if (!pdev) {
		LOG_ERR("platform_device pointer is NULL");
		return -EINVAL;
	}

	const struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (!np || !config) {
		LOG_ERR("Missing device tree node or config struct");
		return -EINVAL;
	}

	// Parse nvidia,ta-uuid
	const char *ta_uuid_str;
	ret = of_property_read_string(np, "nvidia,ta-uuid", &ta_uuid_str);
	if (ret) {
		LOG_ERR("Failed to read nvidia,ta-uuid from device tree");
		return ret;
	}
	ret = uuid_parse(ta_uuid_str, &config->ta_uuid);
	if (ret) {
		LOG_ERR("Failed to parse ta_uuid string '%s'", ta_uuid_str);
		return -EINVAL;
	}

	// Parse nvidia,mctp-uuid
	const char *mctp_uuid_str;
	ret = of_property_read_string(np, "nvidia,mctp-uuid", &mctp_uuid_str);
	if (ret) {
		LOG_ERR("Failed to read nvidia,mctp-uuid from device tree");
		return ret;
	}
	ret = uuid_parse(mctp_uuid_str, &config->mctp_uuid);
	if (ret) {
		LOG_ERR("Failed to parse mctp_uuid string '%s'", mctp_uuid_str);
		return -EINVAL;
	}

	// Parse nvidia,mctp-mtu
	ret = of_property_read_u32(np, "nvidia,mctp-mtu", &config->mtu);
	if (ret) {
		LOG_ERR("Failed to read nvidia,mctp-mtu from device tree");
		return ret;
	}
	if (config->mtu < 68 || config->mtu > 65536) {
		LOG_ERR("Invalid nvidia,mctp-mtu value %u (must be between 68 and 65536)",
			config->mtu);
		return -EINVAL;
	}

	// Parse nvidia,mctp-protocol-versions (array of u32)
	const __be32 *prop;
	int len;
	prop = of_get_property(np, "nvidia,mctp-protocol-versions", &len);
	if (!prop || len < 0) {
		LOG_ERR("Failed to get nvidia,mctp-protocol-versions from device tree");
		return -EINVAL;
	}
	int num_entries = len / sizeof(u32);
	if (num_entries % 2 != 0) {
		LOG_ERR("nvidia,mctp-protocol-versions must have even number of entries (protocol/version pairs), got %d",
			num_entries);
		return -EINVAL;
	}
	config->protocols_size = num_entries / 2;
	if (config->protocols_size > NVIDIA_VROT_MAX_PROTOCOLS) {
		LOG_ERR("protocols_size (%u) exceeds max supported %d",
			config->protocols_size, NVIDIA_VROT_MAX_PROTOCOLS);
		return -EINVAL;
	}

	for (u32 i = 0; i < config->protocols_size; ++i) {
		const u32 protocol = be32_to_cpu(prop[i * 2]);
		const u32 version = be32_to_cpu(prop[i * 2 + 1]);
		if (protocol > 0xFF) {
			LOG_ERR("protocol value 0x%x at index %u exceeds u8 range",
				protocol, i);
			return -EINVAL;
		}
		if (protocol == 0x00) {
			LOG_ERR("protocol value 0x%x at index %u is reserved (control protocol)",
				protocol, i);
			return -EINVAL;
		}
		if (protocol == 0xFF) {
			LOG_ERR("protocol value 0x%x at index %u is reserved (base version)",
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
	int ret = -1;
	struct device *const dev = &pdev->dev;
	dev_info(dev, "Probe starting for device: %s\n", dev_name(dev));

	struct nvidia_ta_mctp_driver *const driver =
		kzalloc(sizeof(struct nvidia_ta_mctp_driver), GFP_KERNEL);
	if (!driver) {
		dev_err(dev, "Failed to allocate memory for driver\n");
		return -ENOMEM;
	}
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

	dev_info(dev, "Probe successful for device: %s\n", dev_name(dev));
	return 0;
}

static void nvidia_vrot_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Removing device: %s\n", dev_name(&pdev->dev));
	clean_driver(pdev);
	dev_info(&pdev->dev, "Device removed: %s\n", dev_name(&pdev->dev));
}

static const struct of_device_id nvidia_vrot_match[] = {
	{ .compatible = "nvidia,optee,vrot" },
	{},
};
MODULE_DEVICE_TABLE(of, nvidia_vrot_match);

static struct platform_driver nvidia_vrot_driver = {
	.probe = nvidia_vrot_probe,
	.remove = nvidia_vrot_remove,
	.driver =
		{
			.name = "nvidia-optee-vrot",
			.of_match_table = nvidia_vrot_match,
		},
};

module_platform_driver(nvidia_vrot_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVIDIA VRoT MCTP Trusted Application Driver");
