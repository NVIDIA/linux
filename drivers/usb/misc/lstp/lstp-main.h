/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LSTP USB interface driver.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef __LSTP_MAIN_H
#define __LSTP_MAIN_H

#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/tty.h>
#include <linux/of.h>

#define LSTP_VERSION 1
#define LSTP_USB_RESPONSE_TIMEOUT_MS 1000
#define LSTP_USB_REQUEST_TIMEOUT_MS 1000
#define LSTP_USB_EP_MIN_SIZE 8
#define LSTP_USB_EP_MAX_SIZE 512
#define LSTP_MAX_CHANNELS 256 /* Includes channel 0 */
#define LSTP_ANY_RX_LEN 0xFFFF
#define LSTP_READ_LEN_ALL 0
#define LSTP_CH_NAME_LEN 16

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
};

enum lstp_status {
	LSTP_SUCCESS = 0x00,
	LSTP_ERROR = 0x01,
	LSTP_TIMEOUT = 0x02,
	LSTP_BUSY = 0x03,
	LSTP_NACK = 0x04,
	LSTP_ARB_LOST = 0x05,
	LSTP_NOT_SUPP = 0x06,
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

struct lstp_usb {
	struct usb_device *udev;
	struct usb_interface *intf;
	u8 bulk_in_ep;
	u8 bulk_out_ep;
	struct urb *bulk_rx_urb;
	u8 *rx_buf;
	size_t bulk_tx_size;
	size_t bulk_rx_size;
	u8 lstp_version;
	char lstp_intf_name[LSTP_CH_NAME_LEN];
	u8 max_ch_id;
	struct lstp_channel *channels[LSTP_MAX_CHANNELS];
	struct kobject *lstp_kobj; /* /sys/.../lstp */
	struct kobject *channel_kobj; /* /sys/.../lstp/channel */
};

/* Callback for unsolicited packets. Called in atomic context - must not sleep. */
typedef void (*lstp_irq_callback)(struct lstp_channel *ch);

struct lstp_channel {
	u8 ch_id;
	u8 ch_type;
	struct device_node *of_node;
	struct lstp_usb *usb;
	struct mutex tx_mutex; /* One request at a time per channel */
	unsigned long resp_buffer_lock;
	unsigned long irq_buffer_lock;
	bool rx_ready; /* Response received from callback */
	u8 *tx_buf;
	u8 *resp_buf; /* Buffer for solicited responses */
	u8 *irq_buf; /* Buffer for unsolicited requests/IRQs */
	struct urb *bulk_tx_urb;
	wait_queue_head_t rx_wq;
	lstp_irq_callback irq_callback;
	void *priv; /* Channel-specific private data (e.g., i2c_adapter) */
	struct kobject kobj; /* /sys/.../lstp/channel/%d */
	struct device *child_dev; /* Child device for sysfs link */
};

/* Channel init/start functions */
int lstp_spi_init(struct lstp_channel *ch);
int lstp_spi_start(struct lstp_channel *ch);
int lstp_i2c_init(struct lstp_channel *ch);
int lstp_i2c_start(struct lstp_channel *ch);
int lstp_ipmi_init(struct lstp_channel *ch);
int lstp_ipmi_start(struct lstp_channel *ch);

/* Internal LSTP helper functions */
int lstp_status_to_errno(u8 status);
int lstp_validate_rx_pkt(struct lstp_usb *dev, struct lstp_packet *rx_pkt, size_t actual_length);
int lstp_validate_resp(struct lstp_usb *dev, struct lstp_packet *rx_pkt,
		       size_t expected_payload_len);
int lstp_ch0_read(struct lstp_usb *dev, u8 ch_id, u16 offset, u16 length);

/* USB Helper Functions */
int lstp_recv_resp_helper(struct lstp_channel *ch, u8 cmd, u16 tx_len, u16 rx_len);
void lstp_unlock_resp_buffer(struct lstp_channel *ch);

#endif
