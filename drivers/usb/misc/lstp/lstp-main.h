/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LSTP USB interface driver.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef __LSTP_MAIN_H
#define __LSTP_MAIN_H

#include <linux/build_bug.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#define LSTP_VERSION 1
#define LSTP_USB_RESPONSE_TIMEOUT_MS 1000
#define LSTP_USB_REQUEST_TIMEOUT_MS 1000
#define LSTP_USB_EP_MIN_SIZE 64
#define LSTP_USB_EP_MAX_SIZE 512
#define LSTP_MAX_CHANNELS 256 /* Includes channel 0 */
#define LSTP_ANY_RX_LEN 0 /* "no minimum" -- callers without a known min payload length */
#define LSTP_READ_LEN_ALL 0
#define LSTP_CH_NAME_LEN 16
#define LSTP_INTF_NAME_LEN (LSTP_CH_NAME_LEN + 1) /* ch0_name + '\0' */
#define LSTP_DISPLAY_NAME_LEN (LSTP_CH_NAME_LEN * 2 + 2) /* intf_name + '_' + ch_name + '\0' */

#define GET_BIT_7(u8_byte) (((u8_byte) >> 7) & 0x01)
#define GET_BIT_0_6(u8_byte) ((u8_byte) & 0x7F)
#define SET_U8_BYTE(bit_0_6, bit_7) ((((bit_7) & 0x01) << 7) | ((bit_0_6) & 0x7F))

#define LSTP_GET_PAYLOAD(pkt, type)                                  \
	({                                                           \
		typeof(pkt) _p = (pkt);                              \
		size_t _len = (size_t)le16_to_cpu(_p->hdr.length);   \
		(_len >= sizeof(type)) ? (type *)_p->payload : NULL; \
	})

/* Channel flags bit definitions */
#define LSTP_CH_FLAG_ENABLE BIT(0)

/* Buffer lock bit (resp_buffer_lock and irq_buffer_lock) */
#define LSTP_BUFFER_LOCK_BIT 0

/* Channel state bits */
#define LSTP_CHANNEL_DISABLED_BIT 0

enum lstp_channel_type {
	LSTP_CHANNEL_TYPE_MGMT = 0x00,
	LSTP_CHANNEL_TYPE_SPI = 0x01,
	LSTP_CHANNEL_TYPE_GPIO = 0x02,
	LSTP_CHANNEL_TYPE_I2C = 0x03,
	LSTP_CHANNEL_TYPE_UART = 0x04,
	LSTP_CHANNEL_TYPE_IPMI = 0x05,
	LSTP_CHANNEL_TYPE_MAX, /* sentinel, not a wire value; keep last */
};

enum lstp_status {
	LSTP_SUCCESS = 0x00,
	LSTP_ERROR = 0x01,
	LSTP_TIMEOUT = 0x02,
	LSTP_BUSY = 0x03,
	LSTP_NACK = 0x04,
	LSTP_ARB_LOST = 0x05,
	LSTP_NOT_SUPP = 0x06,
	LSTP_TOO_LARGE = 0x07,
};

struct lstp_ch0_req_read {
	u8 ch_id;
	u16 offset;
	u16 length;
} __packed;

struct lstp_ch0_req_write {
	u8 ch_id;
	u16 offset;
	u8 ch_type;
	u8 ch_flags;
	char ch_name[LSTP_CH_NAME_LEN];
	u8 ch_config[];
} __packed;

struct lstp_ch0_resp_read {
	u8 ch_type;
	u8 ch_flags;
	char ch_name[LSTP_CH_NAME_LEN];
	u8 ch_config[];
} __packed;

union lstp_ch0_req_payload {
	struct lstp_ch0_req_read read;
	struct lstp_ch0_req_write write;
};

union lstp_ch0_resp_payload {
	struct lstp_ch0_resp_read read;
};

struct lstp_header {
	u8 ch_id;
	union { /* [0:6] cmd/status, [7] 0=request/1=response */
		u8 cmd;
		u8 status;
	};
	u16 length;
} __packed;

struct lstp_packet {
	struct lstp_header hdr;
	u8 payload[];
} __packed;

/*
 * Process-context fallback for the RX completion callback when
 * usb_submit_urb() fails atomically (e.g. -ENOMEM, -EPIPE).
 */
struct lstp_rx_retry {
	struct delayed_work work;
	int last_err;
	unsigned int failures;
};

struct lstp_usb {
	struct usb_device *udev;
	struct usb_interface *intf;
	u8 bulk_in_ep;
	u8 bulk_out_ep;
	struct urb *bulk_rx_urb;
	struct lstp_rx_retry bulk_rx_retry;
	u8 *rx_buf;
	size_t bulk_tx_size;
	size_t bulk_rx_size;
	u8 lstp_version;
	char lstp_intf_name[LSTP_INTF_NAME_LEN];
	u8 max_ch_id;
	struct lstp_channel *channels[LSTP_MAX_CHANNELS];
	struct kobject lstp_kobj; /* /sys/.../lstp */
	struct kobject *channel_kobj; /* /sys/.../lstp/channel */
};

/* Callback for unsolicited packets. Called in atomic context - must not sleep. */
typedef void (*lstp_irq_callback)(struct lstp_channel *ch);

struct lstp_subsys;

struct lstp_channel {
	u8 ch_id;
	u8 ch_type;
	char display_name[LSTP_DISPLAY_NAME_LEN];
	const struct lstp_subsys *subsys;
	struct fwnode_handle *fwnode;
	struct lstp_usb *usb;
	struct mutex tx_mutex; /* One request at a time per channel */
	unsigned long resp_buffer_lock;
	unsigned long irq_buffer_lock;
	unsigned long irq_resp_buffer_lock;
	bool rx_ready; /* Response received from callback */
	bool disconnected; /* USB device disconnected */
	bool started; /* channel_start() succeeded; gates channel_stop() */
	u8 *tx_buf;
	u8 *tx_resp_buf;
	u8 *resp_buf; /* Buffer for solicited responses */
	u8 *irq_buf; /* Buffer for unsolicited requests/IRQs */
	struct urb *bulk_tx_urb;
	struct urb *bulk_tx_resp_urb;
	wait_queue_head_t rx_wq;
	lstp_irq_callback irq_callback;
	void *priv; /* Channel-specific private data (e.g., i2c_adapter) */
	struct kobject kobj; /* /sys/.../lstp/channel/%d */
	struct device *child_dev; /* Child device for sysfs link */
};

/*
 * Subsystem registration table. Each sub-component defines its own
 * descriptor via LSTP_SUBSYS() at file scope; lstp-main.c collects them
 * into a static array (lstp_subsystems[]) and drives module lifecycle
 * and per-channel dispatch from it.
 *
 * Required (positional macro args):
 *   _tag / channel_type / channel_init / channel_start
 *                         Per-channel dispatch for LSTP_CHANNEL_TYPE_*.
 * Optional (designated initializers in ...):
 *   .fwnode_compatible    Firmware node matching.
 *   .channel_stop         Undoes channel_start(); invoked only after
 *                         channel_start() returned success. Use it to
 *                         quiesce activity that depends on the LSTP device
 *                         still being reachable
 *                         (unregister userspace-facing nodes, drop refs the
 *                         device side holds, etc.). Resources owned by the
 *                         channel itself -- allocations, workqueues, IRQ
 *                         registrations, gpiochip/tty/etc. add_data calls --
 *                         should be devres-managed instead, so they:
 *                           - clean up on channel_init/start failure too,
 *                             where channel_stop never runs, and
 *                           - benefit from devres LIFO ordering vs. other
 *                             devres cleanups.
 *   .init / .exit         Module-level hooks.
 */
struct lstp_subsys {
	const char *name;

	u8 channel_type;
	const char *fwnode_compatible;
	int (*channel_init)(struct lstp_channel *ch);
	int (*channel_start)(struct lstp_channel *ch);
	void (*channel_stop)(struct lstp_channel *ch);

	int (*init)(void);
	void (*exit)(void);
};

/*
 * LSTP_SUBSYS() emits a single externally-visible descriptor named
 * lstp_<tag>_subsys. lstp-main.c declares matching externs via
 * LSTP_SUBSYS_DECLARE() and pulls them into lstp_subsystems[] via
 * LSTP_SUBSYS_REF(). Adding a new channel type therefore requires one
 * entry in that central array in addition to the per-file LSTP_SUBSYS()
 * definition -- intentional visibility for maintainers.
 */
/* clang-format off */
#define LSTP_SUBSYS(_tag, _ch_type, _init, _start, ...)                      \
	static_assert((_ch_type) > LSTP_CHANNEL_TYPE_MGMT &&                 \
		      (_ch_type) < LSTP_CHANNEL_TYPE_MAX,                    \
		      #_tag ": channel_type out of range");                  \
	const struct lstp_subsys lstp_##_tag##_subsys = {                    \
		.name = #_tag,                                               \
		.channel_type = (_ch_type),                                  \
		.channel_init = (_init),                                     \
		.channel_start = (_start),                                   \
		__VA_ARGS__                                                  \
	}
/* clang-format on */

/* Companions to LSTP_SUBSYS(): declare the extern / take its address by tag. */
#define LSTP_SUBSYS_DECLARE(_tag) extern const struct lstp_subsys lstp_##_tag##_subsys
#define LSTP_SUBSYS_REF(_tag) (&lstp_##_tag##_subsys)

/* Look up a registered subsystem that dispatches the given channel type. */
const struct lstp_subsys *lstp_subsys_by_channel_type(u8 channel_type);

/* Internal LSTP helper functions */
int lstp_status_to_errno(u8 status);
int lstp_validate_rx_pkt(struct lstp_usb *dev, struct lstp_packet *rx_pkt, size_t actual_length);
int lstp_validate_resp(struct lstp_usb *dev, struct lstp_packet *rx_pkt, size_t min_payload_len);
int lstp_ch0_read_helper(struct lstp_usb *dev, u8 ch_id, u16 offset, u16 length);

/* Module parameters */
extern bool lstp_auto_bind_spidev;

/* USB Helper Functions */
int lstp_recv_resp_helper(struct lstp_channel *ch, u8 cmd, u16 request_len, u16 min_response_len);
int lstp_alloc_irq_resp(struct lstp_channel *ch);
void lstp_send_irq_resp(struct lstp_channel *ch, u8 status);
void lstp_unlock_resp_buffer(struct lstp_channel *ch);

/**
 * lstp_ch_disconnected() - Check if the channel's USB device has been disconnected.
 * @ch: LSTP channel to check
 *
 * Pairs with smp_store_release() in lstp_signal_disconnect().
 *
 * Return: true if the USB device has been disconnected.
 */
static inline bool lstp_ch_disconnected(struct lstp_channel *ch)
{
	/* Pairs with smp_store_release() in lstp_signal_disconnect() */
	return smp_load_acquire(&ch->disconnected);
}

#endif
