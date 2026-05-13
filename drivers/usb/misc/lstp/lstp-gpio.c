// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <uapi/linux/gpio.h>

/* gpio_chip set/set_multiple return int since 6.17, void before */
#define LSTP_GPIO_SET_RETURNS_INT (KERNEL_VERSION(6, 17, 0) <= LINUX_VERSION_CODE)
#include "lstp-main.h"

#define LSTP_GPIO_PIN_NAME_LEN 32

enum lstp_gpio_cmd {
	LSTP_GPIO_CMD_GET_VALUE = 0x00,
	LSTP_GPIO_CMD_SET_VALUE = 0x01,
	LSTP_GPIO_CMD_GET_IRQ_CONFIG = 0x02,
	LSTP_GPIO_CMD_SET_IRQ_CONFIG = 0x03,
	LSTP_GPIO_CMD_IRQ_EVENT = 0x04,
} __packed;

union lstp_gpio_req_payload {
	struct {
		u16 pin;
	} __packed get;
	struct {
		u16 pin;
		u8 value;
	} __packed set;
	struct {
		u16 pin;
	} __packed get_irq;
	struct {
		u16 pin;
		u8 irq_type;
	} __packed set_irq;
	struct {
		u16 pin;
		u8 value;
	} __packed irq_event;
};

union lstp_gpio_resp_payload {
	struct {
		u8 value;
	} __packed get;
	struct {
		u8 irq_type;
	} __packed get_irq;
};

enum lstp_gpio_irq_type {
	LSTP_GPIO_IRQ_NONE = 0x00,
	LSTP_GPIO_IRQ_RISING = 0x01,
	LSTP_GPIO_IRQ_FALLING = 0x02,
	LSTP_GPIO_IRQ_BOTH = 0x03,
	LSTP_GPIO_IRQ_HIGH = 0x04,
	LSTP_GPIO_IRQ_LOW = 0x05,
	LSTP_GPIO_IRQ_MAX = LSTP_GPIO_IRQ_LOW,
} __packed;

struct lstp_gpio_pin_config {
	u8 direction;
	u8 default_output;
	u8 output_drive_config;
	u8 output_persist_state; /* bool */
	u8 bias_pull_config;
	u16 bias_pull_strength; /* 16 ohm increments */
	u16 output_drive_strength; /* in mA */
	u16 slew_rate; /* in 100 ps (0.1 ns) increments */
	u16 input_debounce_time; /* in us */
	u8 reserved_0xD;
	u8 reserved_0xE;
	u8 reserved_0xF;
} __packed;

struct lstp_gpio_pin {
	char pin_name[LSTP_GPIO_PIN_NAME_LEN];
	struct lstp_gpio_pin_config config;
} __packed;

struct lstp_gpio_config {
	u8 ch_ngpio;
	struct lstp_gpio_pin pin_info[];
} __packed;

/* clang-format off */
enum lstp_gpio_direction {
	LSTP_GPIO_OUTPUT = 0x00,
	LSTP_GPIO_INPUT = 0x01
} __packed;
enum lstp_gpio_pin_state {
	LSTP_GPIO_LOW = 0x00,
	LSTP_GPIO_HIGH = 0x01
} __packed;
enum lstp_gpio_drive_config {
	LSTP_GPIO_PUSH_PULL = 0x00,
	LSTP_GPIO_OPEN_DRAIN = 0x01,
	LSTP_GPIO_OPEN_SOURCE = 0x02
} __packed;
enum lstp_gpio_bias_config {
	LSTP_GPIO_NO_PULL = 0x00,
	LSTP_GPIO_PULL_UP = 0x01,
	LSTP_GPIO_PULL_DOWN = 0x02
} __packed; /* clang-format on */

/* Private data for GPIO channel */
struct lstp_gpio_priv {
	struct lstp_channel *ch;
	u16 ngpio;
	u16 max_pins_per_read;
	struct gpio_chip gc;
	struct workqueue_struct *wq; /* Workqueue for IRQ event handling */
	struct mutex irq_lock; /* Serializes irq_chip callbacks (irq_bus_lock/sync_unlock) */
	u8 *shadow_irq_type; /* Pending IRQ type per pin, written by mask/unmask/set_type */
	u8 *hw_irq_type; /* Last successfully committed IRQ type per pin */
};

/* Work structure for handling IRQ events from device */
struct lstp_gpio_irq_event_work {
	struct work_struct work;
	struct gpio_chip *gc;
	u16 pin;
	u8 value; /* Logical GPIO value when interrupt occurred */
};

/**
 * lstp_gpio_get_value() - Read GPIO pin value. (blocking)
 * @gc: GPIO chip
 * @pin: Pin offset within the GPIO chip
 *
 * Sends GET_VALUE over LSTP and returns the logical value reported by the
 * device. Holds ch->tx_mutex.
 *
 * Return: 1 for logical high, 0 for logical low, negative errno on failure.
 */
static int lstp_gpio_get_value(struct gpio_chip *gc, unsigned int pin)
{
	int ret;
	int value;
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;
	union lstp_gpio_resp_payload *gpio_resp = (union lstp_gpio_resp_payload *)rx_pkt->payload;

	mutex_lock(&ch->tx_mutex);
	gpio_req->get.pin = cpu_to_le16((u16)pin);
	ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_GET_VALUE, sizeof(gpio_req->get),
				    sizeof(gpio_resp->get));
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not get value for pin %u (%pe)\n",
			__func__, ch->ch_id, pin, ERR_PTR(ret));
		mutex_unlock(&ch->tx_mutex);
		return ret;
	}
	value = gpio_resp->get.value;

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: pin=%u, value=%d\n", __func__, ch->ch_id, pin,
		value);

	lstp_unlock_resp_buffer(ch);
	mutex_unlock(&ch->tx_mutex);
	return value;
}

/**
 * lstp_gpio_set_value() - Set GPIO pin output value. (blocking)
 * @gc: GPIO chip
 * @pin: Pin offset within the GPIO chip
 * @value: Logical output value (0 = low, non-zero = high)
 *
 * Sends SET_VALUE over LSTP. Holds ch->tx_mutex.
 * Return type: int (6.17+), void (older).
 */
#if LSTP_GPIO_SET_RETURNS_INT
static int lstp_gpio_set_value(struct gpio_chip *gc, unsigned int pin, int value)
#else
static void lstp_gpio_set_value(struct gpio_chip *gc, unsigned int pin, int value)
#endif
{
	int ret;
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;

	mutex_lock(&ch->tx_mutex);
	gpio_req->set.pin = cpu_to_le16((u16)pin);
	gpio_req->set.value = !!value;
	ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_SET_VALUE, sizeof(gpio_req->set), 0);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not set value %d for pin %u (%pe)\n",
			__func__, ch->ch_id, value, pin, ERR_PTR(ret));
		goto out;
	}
	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: pin=%u, value=%d\n", __func__, ch->ch_id, pin,
		value);
	lstp_unlock_resp_buffer(ch);
	ret = 0;
out:
	mutex_unlock(&ch->tx_mutex);
#if LSTP_GPIO_SET_RETURNS_INT
	return ret;
#endif
}

/**
 * lstp_gpio_get_multiple() - Read multiple GPIO pin values. (blocking)
 * @gc: GPIO chip
 * @mask: Bitmask of pins to read
 * @bits: Pointer to store logical values (bitmask)
 *
 * Sends GET_VALUE payloads, chunked across multiple packets if the number of
 * pins exceeds what fits in one request. Fills @bits from responses.
 * Holds ch->tx_mutex.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_get_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
{
	int ret;
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;
	union lstp_gpio_resp_payload *gpio_resp = (union lstp_gpio_resp_payload *)rx_pkt->payload;
	int max_pins_per_tx =
		(ch->usb->bulk_tx_size - sizeof(struct lstp_header)) / sizeof(gpio_req->get);
	int max_pins_per_rx =
		(ch->usb->bulk_rx_size - sizeof(struct lstp_header)) / sizeof(gpio_resp->get);
	int max_pins_per_pkt = min(max_pins_per_tx, max_pins_per_rx);
	int total_pins = bitmap_weight(mask, gc->ngpio);
	typeof(gpio_req->get) *req;
	typeof(gpio_resp->get) *resp;
	unsigned int pin = 0;
	int pins_sent = 0;
	int pins_in_pkt;

	if (total_pins == 0)
		return 0;

	bitmap_zero(bits, gc->ngpio);
	mutex_lock(&ch->tx_mutex);

	while (pins_sent < total_pins) {
		pins_in_pkt = min(max_pins_per_pkt, total_pins - pins_sent);

		/* Build request: pack pin numbers for this packet */
		req = (typeof(req))tx_pkt->payload;
		for (int i = 0; i < pins_in_pkt; i++) {
			pin = find_next_bit(mask, gc->ngpio, pin);
			(req++)->pin = cpu_to_le16((u16)pin);
			pin++;
		}

		ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_GET_VALUE,
					    pins_in_pkt * sizeof(gpio_req->get),
					    pins_in_pkt * sizeof(gpio_resp->get));
		if (ret) {
			dev_err(&ch->usb->intf->dev,
				"%s: ch_%d: Could not get values for %d pins (%pe)\n", __func__,
				ch->ch_id, pins_in_pkt, ERR_PTR(ret));
			goto out_unlock;
		}

		/* Parse response: match values back to pin numbers */
		resp = (typeof(resp))rx_pkt->payload;
		for (int i = 0; i < pins_in_pkt; i++) {
			u16 p = le16_to_cpu(((typeof(gpio_req->get) *)tx_pkt->payload)[i].pin);

			if ((resp++)->value)
				__set_bit(p, bits);
		}

		lstp_unlock_resp_buffer(ch);
		pins_sent += pins_in_pkt;
	}

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: Get %d pins, mask=%*pb, bits=%*pb\n", __func__,
		ch->ch_id, total_pins, gc->ngpio, mask, gc->ngpio, bits);
	ret = 0;

out_unlock:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_gpio_set_multiple() - Set multiple GPIO pin output values. (blocking)
 * @gc: GPIO chip
 * @mask: Bitmask of pins to set
 * @bits: Bitmask of logical output values (0 = low, 1 = high)
 *
 * Sends SET_VALUE payloads (pin + logical value per pin), chunked across
 * multiple packets if the number of pins exceeds what fits in one request.
 * Holds ch->tx_mutex.
 * Return type: int (6.17+), void (older).
 */
#if LSTP_GPIO_SET_RETURNS_INT
static int lstp_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
#else
static void lstp_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
#endif
{
	int ret;
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;
	int max_pins_per_pkt =
		(ch->usb->bulk_tx_size - sizeof(struct lstp_header)) / sizeof(gpio_req->set);
	int total_pins = bitmap_weight(mask, gc->ngpio);
	typeof(gpio_req->set) *req;
	unsigned int pin = 0;
	int pins_sent = 0;
	int pins_in_pkt;

	mutex_lock(&ch->tx_mutex);

	while (pins_sent < total_pins) {
		pins_in_pkt = min(max_pins_per_pkt, total_pins - pins_sent);

		/* Build request: pack pin + value for this packet */
		req = (typeof(req))tx_pkt->payload;
		for (int i = 0; i < pins_in_pkt; i++) {
			pin = find_next_bit(mask, gc->ngpio, pin);
			req->pin = cpu_to_le16((u16)pin);
			req->value = test_bit(pin, bits) ? 1 : 0;
			req++;
			pin++;
		}

		ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_SET_VALUE,
					    pins_in_pkt * sizeof(gpio_req->set), 0);
		if (ret) {
			dev_err(&ch->usb->intf->dev,
				"%s: ch_%d: Could not set values for pins (mask=%*pb) (%pe)\n",
				__func__, ch->ch_id, gc->ngpio, mask, ERR_PTR(ret));
			goto out_unlock;
		}
		lstp_unlock_resp_buffer(ch);
		pins_sent += pins_in_pkt;
	}

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: Set %d pins, mask=%*pb, bits=%*pb\n", __func__,
		ch->ch_id, total_pins, gc->ngpio, mask, gc->ngpio, bits);
	ret = 0;

out_unlock:
	mutex_unlock(&ch->tx_mutex);
#if LSTP_GPIO_SET_RETURNS_INT
	return ret;
#endif
}

/**
 * lstp_gpio_get_irq_config() - Query IRQ config for a single pin from the MCU.
 * @priv: GPIO private data
 * @pin: Pin offset within the GPIO chip
 *
 * Sends GET_IRQ_CONFIG over LSTP. Holds ch->tx_mutex.
 *
 * Return: IRQ type on success, negative errno on failure.
 */
static int lstp_gpio_get_irq_config(struct lstp_gpio_priv *priv, u16 pin)
{
	int ret;
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;
	union lstp_gpio_resp_payload *gpio_resp = (union lstp_gpio_resp_payload *)rx_pkt->payload;

	mutex_lock(&ch->tx_mutex);
	gpio_req->get_irq.pin = cpu_to_le16(pin);
	ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_GET_IRQ_CONFIG, sizeof(gpio_req->get_irq),
				    sizeof(gpio_resp->get_irq));
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: GET_IRQ_CONFIG failed for pin %u (%pe)\n",
			__func__, ch->ch_id, pin, ERR_PTR(ret));
		mutex_unlock(&ch->tx_mutex);
		return ret;
	}

	ret = gpio_resp->get_irq.irq_type;
	lstp_unlock_resp_buffer(ch);
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_gpio_set_irq_config() - Send SET_IRQ_CONFIG over LSTP. (blocking)
 * @priv: GPIO private data
 * @pin: Pin number
 * @irq_type: Interrupt type (LSTP_GPIO_IRQ_* constants)
 *
 * Sends SET_IRQ_CONFIG over LSTP. Holds ch->tx_mutex.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_set_irq_config(struct lstp_gpio_priv *priv, u16 pin, u8 irq_type)
{
	int ret;
	struct lstp_channel *ch = priv->ch;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	union lstp_gpio_req_payload *gpio_req = (union lstp_gpio_req_payload *)tx_pkt->payload;

	mutex_lock(&ch->tx_mutex);
	gpio_req->set_irq.pin = cpu_to_le16(pin);
	gpio_req->set_irq.irq_type = irq_type;
	ret = lstp_recv_resp_helper(ch, LSTP_GPIO_CMD_SET_IRQ_CONFIG, sizeof(gpio_req->set_irq), 0);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: SET_IRQ_CONFIG failed for pin %u (%pe)\n",
			__func__, ch->ch_id, pin, ERR_PTR(ret));
	} else {
		lstp_unlock_resp_buffer(ch);
	}
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/*******************************************************************************
 * Pin configuration functions
 ******************************************************************************/

/**
 * lstp_gpio_read_config_range() - Read a range of pins from device into an array.
 * @priv: GPIO private data
 * @start_pin: First pin index (inclusive)
 * @n_pins: Number of pins to read (length of @pin_data); must be <= priv->max_pins_per_read
 * @pin_data: Array of @n_pins struct lstp_gpio_pin to fill (caller-allocated)
 *
 * Takes ch0->tx_mutex, does READ_CONFIG, validates response, memcpy's to @pin_data,
 * then releases resp_buf and mutex.
 *
 * lstp_ch0_read_helper() offset/length (byte offsets into blob):
 *   offset = sizeof(struct lstp_gpio_config) + start_pin * sizeof(struct lstp_gpio_pin)
 *   length = n_pins * sizeof(struct lstp_gpio_pin)
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_read_config_range(struct lstp_gpio_priv *priv, unsigned int start_pin,
				       unsigned int n_pins, struct lstp_gpio_pin *pin_data)
{
	int ret;
	struct lstp_channel *ch = priv->ch;
	struct lstp_channel *ch0 = ch->usb->channels[0];
	struct lstp_packet *rx_pkt;
	union lstp_ch0_resp_payload *ch0_resp;
	u16 offset;
	u16 length;
	size_t min_payload_len;

	if (n_pins == 0 || n_pins > priv->max_pins_per_read || start_pin + n_pins > priv->ngpio ||
	    !pin_data)
		return -EINVAL;

	offset = sizeof(struct lstp_gpio_config) + start_pin * sizeof(struct lstp_gpio_pin);
	length = n_pins * sizeof(struct lstp_gpio_pin);
	min_payload_len = sizeof(struct lstp_ch0_resp_read) + length;

	mutex_lock(&ch0->tx_mutex);
	ret = lstp_ch0_read_helper(ch->usb, ch->ch_id, offset, length);
	if (ret)
		goto out;

	rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	ret = lstp_validate_resp(ch->usb, rx_pkt, min_payload_len);
	if (ret)
		goto out;

	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	memcpy(pin_data, ch0_resp->read.ch_config, length);
	ret = 0;
out:
	lstp_unlock_resp_buffer(ch0); /* no-op if read_helper failed (already released) */
	mutex_unlock(&ch0->tx_mutex);
	return ret;
}

/**
 * lstp_gpio_get_direction() - Query GPIO pin direction.
 * @gc: GPIO chip
 * @pin: Pin offset within the GPIO chip
 *
 * Reads direction from device via ch0 read.
 *
 * Return: GPIO_LINE_DIRECTION_IN or GPIO_LINE_DIRECTION_OUT, -EINVAL on failure.
 */
static int lstp_gpio_get_direction(struct gpio_chip *gc, unsigned int pin)
{
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_gpio_pin pin_data;
	int num_pins = 1;
	int ret;

	if (pin >= priv->ngpio)
		return -EINVAL;

	ret = lstp_gpio_read_config_range(priv, pin, num_pins, &pin_data);
	if (ret)
		return ret;

	if (pin_data.config.direction == LSTP_GPIO_INPUT)
		return GPIO_LINE_DIRECTION_IN;
	else
		return GPIO_LINE_DIRECTION_OUT;
}

/**
 * lstp_gpio_direction_input() - Configure GPIO as input.
 * @gc: GPIO chip
 * @pin: Pin offset within the GPIO chip
 *
 * Direction is firmware-fixed; validates that the pin exists and is
 * actually configured as input by firmware.
 *
 * Return: 0 on success, -EINVAL if pin invalid, -EOPNOTSUPP if pin is output.
 */
static int lstp_gpio_direction_input(struct gpio_chip *gc, unsigned int pin)
{
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_gpio_pin pin_data;
	int ret;

	if (pin >= priv->ngpio)
		return -EINVAL;

	ret = lstp_gpio_read_config_range(priv, pin, 1, &pin_data);
	if (ret)
		return ret;

	if (pin_data.config.direction != LSTP_GPIO_INPUT)
		return -EOPNOTSUPP;

	return 0;
}

/**
 * lstp_gpio_direction_output() - Configure GPIO as output and set initial value.
 * @gc: GPIO chip
 * @pin: Pin offset within the GPIO chip
 * @value: Initial logical output value (0 = low, non-zero = high)
 *
 * Validates pin is output (firmware-fixed), then sets value via lstp_gpio_set_value.
 *
 * Return: 0 on success, -EINVAL if pin invalid, -EOPNOTSUPP if pin is input.
 */
static int lstp_gpio_direction_output(struct gpio_chip *gc, unsigned int pin, int value)
{
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	struct lstp_gpio_pin pin_data;
	int ret;

	if (pin >= priv->ngpio)
		return -EINVAL;

	ret = lstp_gpio_read_config_range(priv, pin, 1, &pin_data);
	if (ret)
		return ret;

	if (pin_data.config.direction != LSTP_GPIO_OUTPUT)
		return -EOPNOTSUPP;

#if LSTP_GPIO_SET_RETURNS_INT
	return lstp_gpio_set_value(gc, pin, value);
#else
	lstp_gpio_set_value(gc, pin, value);
	return 0;
#endif
}

/*******************************************************************************
 * GPIO IRQ functions
 ******************************************************************************/

/**
 * lstp_gpio_linux_irq_to_lstp() - Map Linux IRQ type to LSTP IRQ type.
 * @type: Linux IRQ type (IRQ_TYPE_EDGE_*, IRQ_TYPE_LEVEL_*)
 * @irq_type: Output LSTP IRQ type
 *
 * Return: 0 on success, -EINVAL if type unsupported.
 */
static int lstp_gpio_linux_irq_to_lstp(unsigned int type, u8 *irq_type)
{
	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		*irq_type = LSTP_GPIO_IRQ_NONE;
		break;
	case IRQ_TYPE_EDGE_RISING:
		*irq_type = LSTP_GPIO_IRQ_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		*irq_type = LSTP_GPIO_IRQ_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		*irq_type = LSTP_GPIO_IRQ_BOTH;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		*irq_type = LSTP_GPIO_IRQ_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		*irq_type = LSTP_GPIO_IRQ_LOW;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/**
 * lstp_gpio_sync_hw_irq_state() - Read MCU IRQ state for all pins at probe time.
 * @priv: GPIO private data (shadow_irq_type and hw_irq_type must be allocated)
 *
 * Queries GET_IRQ_CONFIG for each pin and populates both hw_irq_type and
 * shadow_irq_type so the driver starts in sync with the MCU, even after a
 * module reload without MCU reset.
 *
 * Return: 0 on success, negative errno on first failure.
 */
static int lstp_gpio_sync_hw_irq_state(struct lstp_gpio_priv *priv)
{
	struct lstp_channel *ch = priv->ch;
	unsigned int active = 0;
	int ret;
	u16 pin;

	for (pin = 0; pin < priv->ngpio; pin++) {
		ret = lstp_gpio_get_irq_config(priv, pin);
		if (ret < 0)
			return ret;
		if (ret > LSTP_GPIO_IRQ_MAX)
			dev_warn(&ch->usb->intf->dev, "%s: ch_%d: pin %u has unknown IRQ type %u\n",
				 __func__, ch->ch_id, pin, ret);
		priv->hw_irq_type[pin] = ret;
		priv->shadow_irq_type[pin] = ret;
		if (ret != LSTP_GPIO_IRQ_NONE)
			active++;
	}

	if (active)
		dev_info(&ch->usb->intf->dev,
			 "%s: ch_%d: Synced IRQ state, %u/%u pins with active IRQ config\n",
			 __func__, ch->ch_id, active, priv->ngpio);

	return 0;
}

/**
 * lstp_gpio_irq_mask() - Mask (disable) a GPIO interrupt.
 * @d: IRQ data
 *
 * Sets shadow to IRQ_NONE; SET_IRQ_CONFIG sent in irq_bus_sync_unlock().
 */
static void lstp_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);

	priv->shadow_irq_type[pin] = LSTP_GPIO_IRQ_NONE;
	gpiochip_disable_irq(gc, pin);
}

/**
 * lstp_gpio_irq_unmask() - Unmask (enable) a GPIO interrupt.
 * @d: IRQ data
 *
 * Restores shadow to trigger type from IRQ descriptor.
 * SET_IRQ_CONFIG sent in irq_bus_sync_unlock().
 */
static void lstp_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);
	unsigned int type = irqd_get_trigger_type(d);
	u8 irq_type;

	if (lstp_gpio_linux_irq_to_lstp(type, &irq_type)) {
		dev_err(gc->parent, "%s: Invalid trigger type %u for pin %u\n", __func__, type,
			pin);
		return;
	}

	gpiochip_enable_irq(gc, pin);
	priv->shadow_irq_type[pin] = irq_type;
}

/**
 * lstp_gpio_irq_set_type() - Set GPIO interrupt trigger type.
 * @d: IRQ data
 * @type: IRQ type flags (IRQ_TYPE_EDGE_*, IRQ_TYPE_LEVEL_*)
 *
 * IRQCHIP_SET_TYPE_MASKED ensures the IRQ is masked, so this only validates
 * the type; the shadow update is deferred to irq_unmask().
 *
 * Return: 0 on success, -EINVAL if type unsupported.
 */
static int lstp_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned int pin = irqd_to_hwirq(d);
	u8 irq_type;
	int ret;

	ret = lstp_gpio_linux_irq_to_lstp(type, &irq_type);
	if (ret)
		return ret;

	if (irqd_irq_masked(d))
		return 0;

	priv->shadow_irq_type[pin] = irq_type;
	return 0;
}

/**
 * lstp_gpio_irq_bus_lock() - Acquire IRQ configuration lock.
 * @d: IRQ data
 *
 * Serializes irq_chip callbacks in a sleepable context (bus_lock pattern).
 */
static void lstp_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);

	mutex_lock(&priv->irq_lock);
}

/**
 * lstp_gpio_irq_bus_sync_unlock() - Commit pending IRQ config and release lock.
 * @d: IRQ data for the pin that was modified
 *
 * Sends SET_IRQ_CONFIG if shadow differs from hardware for this pin.
 * On failure, reverts shadow to previous known state. Releases priv->irq_lock.
 */
static void lstp_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct lstp_gpio_priv *priv = gpiochip_get_data(gc);
	u16 pin = irqd_to_hwirq(d);

	if (priv->shadow_irq_type[pin] != priv->hw_irq_type[pin]) {
		if (lstp_gpio_set_irq_config(priv, pin, priv->shadow_irq_type[pin]))
			priv->shadow_irq_type[pin] = priv->hw_irq_type[pin];
		else
			priv->hw_irq_type[pin] = priv->shadow_irq_type[pin];
	}

	mutex_unlock(&priv->irq_lock);
}

static void lstp_gpio_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	seq_puts(p, gc->label ? gc->label : d->chip->name);
}

static const struct irq_chip lstp_gpio_irq_chip = {
	.name = "lstp-gpio",
	.irq_mask = lstp_gpio_irq_mask,
	.irq_unmask = lstp_gpio_irq_unmask,
	.irq_set_type = lstp_gpio_irq_set_type,
	.irq_bus_lock = lstp_gpio_irq_bus_lock,
	.irq_bus_sync_unlock = lstp_gpio_irq_bus_sync_unlock,
	.irq_print_chip = lstp_gpio_irq_print_chip,
	.flags = IRQCHIP_SET_TYPE_MASKED | IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/**
 * lstp_gpio_irq_event_work() - Deferred GPIO interrupt event handler.
 * @work: Work structure containing gc and pin from the IRQ event
 *
 * Workqueue handler. Finds IRQ from pin, calls handle_nested_irq(), then frees work.
 */
static void lstp_gpio_irq_event_work(struct work_struct *work)
{
	struct lstp_gpio_irq_event_work *event_work =
		container_of(work, struct lstp_gpio_irq_event_work, work);
	struct gpio_chip *gc = event_work->gc;
	int irq;

	if (!gc) {
		pr_err("%s: NULL gc (pin=%u)\n", __func__, event_work->pin);
		goto out_free;
	}
	if (event_work->pin >= gc->ngpio) {
		dev_err(gc->parent, "%s: Pin out of range (pin=%u, ngpio=%u)\n", __func__,
			event_work->pin, gc->ngpio);
		goto out_free;
	}
	if (!gc->irq.domain) {
		dev_warn(gc->parent, "%s: IRQ domain not ready for pin %u\n", __func__,
			 event_work->pin);
		goto out_free;
	}

	irq = irq_find_mapping(gc->irq.domain, event_work->pin);
	if (irq) {
		handle_nested_irq(irq);
		dev_dbg(gc->parent, "%s: Nested IRQ %d handled for pin %u\n", __func__, irq,
			event_work->pin);
	} else {
		dev_warn(gc->parent, "%s: No IRQ mapping found for pin %u\n", __func__,
			 event_work->pin);
	}

out_free:
	kfree(event_work);
}

/**
 * lstp_gpio_irq_event() - Process GPIO interrupt event from unsolicited RX packet.
 * @ch: LSTP channel (ch->irq_buf already filled by main RX completion)
 *
 * Called from USB RX completion for unsolicited requests. Parses pin and value
 * from the payload, queues work on priv->wq so handle_nested_irq runs in process
 * context. Must not block (atomic context).
 */
static void lstp_gpio_irq_event(struct lstp_channel *ch)
{
	struct lstp_gpio_priv *priv;
	struct gpio_chip *gc;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->irq_buf;
	union lstp_gpio_req_payload *gpio_req;
	u16 pin;
	struct lstp_gpio_irq_event_work *work;
	bool queued;

	/* Retrieve GPIO private data from channel context */
	priv = (struct lstp_gpio_priv *)ch->priv;
	if (!priv) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: No GPIO private data found in channel\n",
			__func__, ch->ch_id);
		return;
	}

	/* Validate IRQ event request payload */
	gpio_req = LSTP_GET_PAYLOAD(rx_pkt, union lstp_gpio_req_payload);
	if (!gpio_req) {
		dev_err(&ch->usb->intf->dev,
			"%s: ch_%d: IRQ event packet too small (got %u, need >=%zu)\n", __func__,
			ch->ch_id, le16_to_cpu(rx_pkt->hdr.length), sizeof(gpio_req->irq_event));
		return;
	}

	pin = le16_to_cpu(gpio_req->irq_event.pin);
	gc = &priv->gc;

	dev_dbg(&ch->usb->intf->dev, "%s: ch=%d (ngpio=%u), pin=%u, value=%u\n", __func__,
		ch->ch_id, gc->ngpio, pin, gpio_req->irq_event.value);

	if (pin >= gc->ngpio) {
		dev_warn(&ch->usb->intf->dev, "%s: ch_%d: Invalid IRQ pin: %u (ngpio=%u)\n",
			 __func__, ch->ch_id, pin, gc->ngpio);
		return;
	}

	/* Allocate work to defer IRQ handling to process context */
	work = kzalloc(sizeof(*work), GFP_ATOMIC);
	if (!work)
		return;

	INIT_WORK(&work->work, lstp_gpio_irq_event_work);
	work->gc = gc;
	work->pin = pin;
	work->value = gpio_req->irq_event.value;

	queued = queue_work(priv->wq, &work->work);
	if (!queued) {
		dev_warn(&ch->usb->intf->dev,
			 "%s: ch_%d: Work already pending for IRQ event on pin %u (unexpected)\n",
			 __func__, ch->ch_id, pin);
		kfree(work);
	}
	dev_dbg(&ch->usb->intf->dev, "%s: Queued work for ch=%d, pin %u\n", __func__, ch->ch_id,
		pin);
}

/*******************************************************************************
 * GPIO channel initialization
 ******************************************************************************/

static void lstp_gpio_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

/**
 * lstp_gpio_get_pin_info() - Retrieve GPIO pin names from device.
 * @priv: GPIO private data
 * @pin_names: Array of pointers to fill with pin name strings (caller-allocated)
 *
 * Called with ch0->tx_mutex held and chunk 0 in ch0->resp_buf. Uses chunk 0 in place
 * (releases resp_buf only); for further chunks temporarily releases mutex and calls
 * lstp_gpio_read_config_range(). Returns with ch0->tx_mutex held (same as on entry).
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_get_pin_info(struct lstp_gpio_priv *priv, const char **pin_names)
{
	int ret;
	struct lstp_channel *ch = priv->ch;
	struct lstp_channel *ch0 = ch->usb->channels[0];
	struct lstp_packet *rx_pkt;
	union lstp_ch0_resp_payload *ch0_resp;
	struct lstp_gpio_config *config;
	struct lstp_gpio_pin *pin_data;
	char *name;
	unsigned int max_per_read = priv->max_pins_per_read;
	unsigned int num_chunks;
	unsigned int pins_in_chunk;
	unsigned int pin_base;
	unsigned int pin;
	unsigned int chunk;
	unsigned int i;
	size_t min_payload_size;

	pin_data = kcalloc(max_per_read, sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data)
		return -ENOMEM;

	/* Calculate total number of chunks needed */
	num_chunks = (priv->ngpio + max_per_read - 1) / max_per_read;
	dev_dbg(&ch->usb->intf->dev,
		"%s: ch_%d: Reading %u GPIO pins across %u chunks (max %u per chunk)\n", __func__,
		ch->ch_id, priv->ngpio, num_chunks, max_per_read);

	for (chunk = 0; chunk < num_chunks; chunk++) {
		pin_base = chunk * max_per_read;
		pins_in_chunk = min((unsigned int)(priv->ngpio - pin_base), max_per_read);

		if (chunk == 0) {
			/* Chunk 0 already in resp_buf from lstp_init_channels */
			rx_pkt = (struct lstp_packet *)ch0->resp_buf;
			min_payload_size = sizeof(struct lstp_ch0_resp_read) +
					   sizeof(struct lstp_gpio_config) +
					   pins_in_chunk * sizeof(struct lstp_gpio_pin);
			ret = lstp_validate_resp(ch->usb, rx_pkt, min_payload_size);
			if (ret) {
				dev_err(&ch->usb->intf->dev,
					"%s: ch_%d: Chunk 0 validation failed (%pe)\n", __func__,
					ch->ch_id, ERR_PTR(ret));
				lstp_unlock_resp_buffer(ch0);
				goto out_free;
			}
			ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
			config = (struct lstp_gpio_config *)ch0_resp->read.ch_config;
			memcpy(pin_data, &config->pin_info[0],
			       pins_in_chunk * sizeof(struct lstp_gpio_pin));
			lstp_unlock_resp_buffer(ch0);
		} else {
			mutex_unlock(&ch0->tx_mutex);
			ret = lstp_gpio_read_config_range(priv, pin_base, pins_in_chunk, pin_data);
			mutex_lock(&ch0->tx_mutex);
			if (ret) {
				dev_err(&ch->usb->intf->dev,
					"%s: ch_%d: Chunk %u (pins %u-%u) failed (%pe)\n", __func__,
					ch->ch_id, chunk, pin_base, pin_base + pins_in_chunk - 1,
					ERR_PTR(ret));
				goto out_free;
			}
		}

		for (i = 0; i < pins_in_chunk; i++) {
			pin = pin_base + i;
			name = (char *)pin_names[pin];
			snprintf(name, GPIO_MAX_NAME_SIZE, "%.*s", (int)LSTP_GPIO_PIN_NAME_LEN,
				 pin_data[i].pin_name);
		}

		dev_dbg(&ch->usb->intf->dev,
			"%s: ch_%d: Processed chunk %u: pins %u-%u (%u pins)\n", __func__,
			ch->ch_id, chunk, pin_base, pin_base + pins_in_chunk - 1, pins_in_chunk);
	}

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: Configured %u GPIO pins across %u chunks\n",
		__func__, ch->ch_id, priv->ngpio, num_chunks);
	ret = 0;

out_free:
	kfree(pin_data);
	return ret;
}

/**
 * lstp_gpio_init() - Initialize GPIO channel.
 * @ch: LSTP channel configured for GPIO
 *
 * Allocates priv, buffers, shadow IRQ state; sets up gpio_chip and irq_chip (bus_lock pattern).
 * Caller must hold ch0->tx_mutex with this channel's READ_CONFIG response already in ch0->resp_buf;
 * returns with it still held. Call lstp_gpio_start() after RX URB is active.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_init(struct lstp_channel *ch)
{
	int ret;
	char *name;
	const char **pin_names = NULL;
	struct lstp_gpio_priv *priv;
	struct gpio_chip *gc;
	struct gpio_irq_chip *girq;
	struct lstp_channel *ch0 = ch->usb->channels[0];
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	union lstp_ch0_resp_payload *ch0_resp;
	struct lstp_gpio_config *config;

	/* Verify minimum READ_CONFIG response payload length */
	if (le16_to_cpu(rx_pkt->hdr.length) <
	    sizeof(struct lstp_ch0_resp_read) + sizeof(struct lstp_gpio_config)) {
		dev_err(&ch->usb->intf->dev,
			"%s: ch_0: Received wrong config size for ch_%d (got %d, expected >= %zu)\n",
			__func__, ch->ch_id, le16_to_cpu(rx_pkt->hdr.length),
			sizeof(struct lstp_ch0_resp_read) + sizeof(struct lstp_gpio_config));
		return -EINVAL;
	}
	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	config = (struct lstp_gpio_config *)ch0_resp->read.ch_config;

	/* Allocate memory */
	ch->resp_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->resp_buf)
		return -ENOMEM;

	ch->irq_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->irq_buf)
		return -ENOMEM;

	pin_names = devm_kcalloc(&ch->usb->intf->dev, config->ch_ngpio, sizeof(char *), GFP_KERNEL);
	if (!pin_names)
		return -ENOMEM;

	for (int pin = 0; pin < config->ch_ngpio; pin++) {
		name = devm_kzalloc(&ch->usb->intf->dev, GPIO_MAX_NAME_SIZE, GFP_KERNEL);
		if (!name)
			return -ENOMEM;
		pin_names[pin] = name;
	}

	priv = devm_kzalloc(&ch->usb->intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->shadow_irq_type = devm_kcalloc(&ch->usb->intf->dev, config->ch_ngpio,
					     sizeof(*priv->shadow_irq_type), GFP_KERNEL);
	if (!priv->shadow_irq_type)
		return -ENOMEM;

	priv->hw_irq_type = devm_kcalloc(&ch->usb->intf->dev, config->ch_ngpio,
					 sizeof(*priv->hw_irq_type), GFP_KERNEL);
	if (!priv->hw_irq_type)
		return -ENOMEM;

	/* Initialize private data */
	priv->ch = ch;
	priv->ngpio = config->ch_ngpio;
	if (priv->ngpio == 0) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid ch_ngpio of 0\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}
	priv->max_pins_per_read =
		(ch->usb->bulk_rx_size - sizeof(struct lstp_header) -
		 sizeof(struct lstp_ch0_resp_read) - sizeof(struct lstp_gpio_config)) /
		sizeof(struct lstp_gpio_pin);
	if (priv->max_pins_per_read == 0) {
		dev_err(&ch->usb->intf->dev,
			"%s: ch_%d: Packet size too small for even one GPIO pin config\n", __func__,
			ch->ch_id);
		return -ENOSPC;
	}
	ch->priv = priv;

	/* Truncate display_name to fit GPIO_MAX_NAME_SIZE (32 bytes including '\0') */
	if (strlen(ch->display_name) >= GPIO_MAX_NAME_SIZE) {
		dev_warn(&ch->usb->intf->dev,
			 "%s: ch_%d: display_name truncated to '%.*s' (max %d chars)\n", __func__,
			 ch->ch_id, GPIO_MAX_NAME_SIZE - 1, ch->display_name,
			 GPIO_MAX_NAME_SIZE - 1);
		ch->display_name[GPIO_MAX_NAME_SIZE - 1] = '\0';
	}

	/* Initialize GPIO chip structure */
	gc = &priv->gc;
	gc->label = ch->display_name;
	gc->owner = THIS_MODULE;
	gc->names = pin_names;
	gc->base = -1;
	gc->ngpio = config->ch_ngpio;
	gc->get = lstp_gpio_get_value;
	gc->set = lstp_gpio_set_value;
	gc->can_sleep = true;
	gc->get_multiple = lstp_gpio_get_multiple;
	gc->set_multiple = lstp_gpio_set_multiple;
	gc->get_direction = lstp_gpio_get_direction;
	gc->direction_input = lstp_gpio_direction_input;
	gc->direction_output = lstp_gpio_direction_output;
	gc->parent = &ch->usb->intf->dev;

	/* Link firmware node for property lookups (e.g. gpio-line-names, hogs) */
	if (ch->fwnode)
		gc->fwnode = ch->fwnode;

	/* Get all pin names and configurations (keeps ch0->tx_mutex held for lstp_init_channels) */
	ret = lstp_gpio_get_pin_info(priv, pin_names);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not get pin info (%pe)\n", __func__,
			ch->ch_id, ERR_PTR(ret));
		return ret;
	}

	/* Setup IRQ */
	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &lstp_gpio_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;

	mutex_init(&priv->irq_lock);

	/* Create dedicated workqueue for IRQ event handling */
	priv->wq = alloc_workqueue("lstp_gpio_wq", WQ_UNBOUND, 0);
	if (!priv->wq) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Failed to create workqueue\n", __func__,
			ch->ch_id);
		return -ENOMEM;
	}
	ret = devm_add_action_or_reset(&ch->usb->intf->dev, lstp_gpio_destroy_wq, priv->wq);
	if (ret)
		return ret;

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: Initialized as %s\n", __func__, ch->ch_id,
		gc->label);
	return 0;
}

/**
 * lstp_gpio_start() - Register GPIO chip with kernel.
 * @ch: LSTP channel with initialized GPIO chip (from lstp_gpio_init)
 *
 * Sets irq_callback and registers gpio_chip. Call only after RX URB is active
 * so IRQ events can be received.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int lstp_gpio_start(struct lstp_channel *ch)
{
	int ret;
	struct lstp_gpio_priv *priv = ch->priv;
	struct gpio_chip *gc;

	if (!priv) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: GPIO chip not initialized\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	gc = &priv->gc;

	ret = lstp_gpio_sync_hw_irq_state(priv);
	if (ret)
		return ret;

	ret = gpiochip_add_data(gc, priv);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register GPIO chip (%pe)\n",
			__func__, ch->ch_id, ERR_PTR(ret));
		return ret;
	}

	ch->irq_callback = lstp_gpio_irq_event;
	ch->child_dev = gpio_device_to_device(gc->gpiodev);

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Started as %s with %u pins\n", __func__,
		 ch->ch_id, gc->label, gc->ngpio);
	return 0;
}

/**
 * lstp_gpio_stop() - Tear down the GPIO chip's userspace/kernel surface.
 * @ch: LSTP channel previously started by lstp_gpio_start()
 */
static void lstp_gpio_stop(struct lstp_channel *ch)
{
	struct lstp_gpio_priv *priv = ch->priv;

	/* Drain RX-queued IRQ-event work while gc->irq.domain is still valid. */
	flush_workqueue(priv->wq);

	gpiochip_remove(&priv->gc);
}

/* clang-format off */
LSTP_SUBSYS(gpio, LSTP_CHANNEL_TYPE_GPIO, lstp_gpio_init, lstp_gpio_start,
	    .fwnode_compatible = "nvidia,lstp-gpio",
	    .channel_stop = lstp_gpio_stop,
);
/* clang-format on */
