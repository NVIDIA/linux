// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/spi/spi.h>

#include "lstp-main.h"

/*******************************************************************************
 * SPI command definitions
 ******************************************************************************/

enum lstp_spi_cmd {
	LSTP_SPI_CMD_CONFIG = 0x00,
	LSTP_SPI_CMD_READ = 0x01,
	LSTP_SPI_CMD_WRITE = 0x02,
	LSTP_SPI_CMD_WRITE_READ = 0x03,
	LSTP_SPI_CMD_POSTED_WRITE = 0x04
};

/* clang-format off */
enum lstp_spi_cmd_cs_toggle {
	LSTP_SPI_CMD_CS_ASSERT = 0x20,
	LSTP_SPI_CMD_CS_DEASSERT = 0x10
}; /* clang-format on */

/* clang-format off */
enum lstp_spi_cmd_cs_sel {
	LSTP_SPI_CMD_CS0 = 0x00,
	LSTP_SPI_CMD_CS1 = 0x40
}; /* clang-format on */

union lstp_spi_req_payload {
	struct {
		u32 len;
	} __packed read;
	struct {
		u32 speed_hz;
	} __packed config;
};

/* Response data for CONFIG command */
struct lstp_spi_config_resp {
	u32 speed_hz;
	u64 reserved;
} __packed;

/* Private data for SPI controller */
struct lstp_spi_priv {
	struct spi_controller *ctrl;
	u32 current_speed_hz; /* Actual speed configured on device */
	u32 requested_speed_hz; /* Last requested speed (to avoid repeated CONFIG) */
};

/* Chip select mapping: array index is chip_select number (0-1) */
/* clang-format off */
static const u8 lstp_spi_cs_map[] = {
	LSTP_SPI_CMD_CS0,
	LSTP_SPI_CMD_CS1
}; /* clang-format on */

/* SPI speed configuration */
#define LSTP_SPI_DEFAULT_SPEED_HZ 18750000 /* 18.75 MHz */

/**
 * lstp_spi_set_speed() - Configure SPI clock frequency on the device.
 * @ch:        LSTP SPI channel
 * @speed_hz:  Requested SPI clock frequency in Hz
 * @actual_hz: Output for actual configured frequency (optional, may be NULL)
 *
 * Sends a CONFIG command to set the SPI clock frequency. The device may
 * adjust to the nearest supported frequency if the exact value cannot be
 * achieved. The actual configured frequency is returned in the response
 * and stored in priv->current_speed_hz.
 *
 * If the actual frequency differs significantly (>10%) from the requested
 * frequency, a warning is logged.
 *
 * NOTE: Unlike flashrom's nv_sma_spi.c which sends CONFIG to channel 0,
 * we send CONFIG to the current SPI channel (ch->ch_id). This deviation
 * was approved as a workaround on the LSTP HW side.
 *
 * NOTE: Current implementation of the command de-asserts CS unconditionally.
 * This is not ideal, as it may cause issues with the SPI device if the CS is
 * not de-asserted at the right time.
 * New CONFIG-like command (or parameter) should be added to the protocol to
 * control speed without de-asserting CS.
 *
 * Context: Process context. Caller must NOT hold ch->tx_mutex.
 *          Performs blocking USB I/O.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_spi_set_speed(struct lstp_channel *ch, u32 speed_hz, u32 *actual_hz)
{
	int ret;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_spi_req_payload *spi_req = (union lstp_spi_req_payload *)tx_pkt->payload;
	struct lstp_spi_config_resp *rx_data;
	struct lstp_spi_priv *priv = ch->priv;
	u32 actual;

	mutex_lock(&ch->tx_mutex);

	spi_req->config.speed_hz = cpu_to_le32(speed_hz);

	ret = lstp_recv_resp_helper(ch, LSTP_SPI_CMD_CONFIG, sizeof(spi_req->config),
				    sizeof(struct lstp_spi_config_resp));
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: CONFIG command failed (%d)\n", __func__,
			ch->ch_id, ret);
		goto out_mutex;
	}

	/* Extract actual speed from response */
	rx_data = (struct lstp_spi_config_resp *)rx_pkt->payload;
	actual = le32_to_cpu(rx_data->speed_hz);
	priv->current_speed_hz = actual;
	priv->requested_speed_hz = speed_hz;

	if (actual_hz)
		*actual_hz = actual;

	/* Warn if actual frequency differs significantly from requested */
	if (speed_hz && actual != speed_hz) {
		int diff_pct = abs((int)actual - (int)speed_hz) * 100 / speed_hz;

		if (diff_pct > 10) {
			dev_warn(&ch->usb->intf->dev,
				 "%s: ch_%d: SPI speed differs: requested=%u Hz, actual=%u Hz (%d%%)\n",
				 __func__, ch->ch_id, speed_hz, actual, diff_pct);
		} else {
			dev_dbg(&ch->usb->intf->dev,
				"%s: ch_%d: SPI speed: requested=%u Hz, actual=%u Hz\n", __func__,
				ch->ch_id, speed_hz, actual);
		}
	} else {
		dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: SPI speed set to %u Hz\n", __func__,
			ch->ch_id, actual);
	}

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/*******************************************************************************
 * SPI controller callbacks
 ******************************************************************************/

/**
 * lstp_spi_transfer_one() - Execute a single SPI transfer.
 * @ctrl: SPI controller performing the transfer
 * @spi:  SPI device the transfer is for
 * @xfer: SPI transfer descriptor containing tx/rx buffers and length
 *
 * Executes a single SPI transfer over the LSTP channel. Supports three
 * transfer modes based on which buffers are provided:
 * - Write-Read (full duplex): Both tx_buf and rx_buf set
 * - Write only: Only tx_buf set
 * - Read only: Only rx_buf set
 *
 * If xfer->speed_hz is set and differs from the current configured speed,
 * a CONFIG command is sent first to update the SPI clock frequency. The
 * actual configured speed is reported back via xfer->effective_speed_hz.
 *
 * The chip select (CS0-CS1) is determined from spi->chip_select and included
 * in every command. The SPI core manages CS assertion/deassertion via the
 * separate set_cs callback.
 *
 * The transfer data is sent via lstp_recv_resp_helper() which handles
 * the LSTP protocol framing and USB transport.
 *
 * Context: Process context. Takes ch->tx_mutex. Performs blocking USB I/O.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_spi_transfer_one(struct spi_controller *ctrl, struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	int ret = 0;
	struct lstp_channel *ch = spi_controller_get_devdata(ctrl);
	struct lstp_spi_priv *priv = ch->priv;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_spi_req_payload *spi_req = (union lstp_spi_req_payload *)tx_pkt->payload;
	u8 chip_select = spi_get_chipselect(spi, 0);
	u8 cs_bits;

	/* Validate chip select index */
	if (chip_select >= ARRAY_SIZE(lstp_spi_cs_map)) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid chip select %u (max %zu)\n",
			__func__, ch->ch_id, chip_select, ARRAY_SIZE(lstp_spi_cs_map) - 1);
		return -EINVAL;
	}
	cs_bits = lstp_spi_cs_map[chip_select];

	/*
	 * Report actual speed to SPI core.
	 * Do not attempt to set the speed, as protocol does not support it well.
	 * See lstp_spi_set_speed for more details.
	 */
	xfer->effective_speed_hz = priv->current_speed_hz;

	/* Validate transfer lengths */
	if (sizeof(struct lstp_header) + xfer->len > ch->usb->bulk_tx_size ||
	    sizeof(struct lstp_header) + xfer->len > ch->usb->bulk_rx_size) {
		dev_err(&ch->usb->intf->dev,
			"%s: ch_%d: Transfer length %u exceeds buffer capacity\n", __func__,
			ch->ch_id, xfer->len);
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	if (xfer->tx_buf)
		memcpy(tx_pkt->payload, xfer->tx_buf, xfer->len);

	if (xfer->tx_buf && xfer->rx_buf) {
		ret = lstp_recv_resp_helper(ch, LSTP_SPI_CMD_WRITE_READ | cs_bits, xfer->len,
					    xfer->len);
	} else if (xfer->tx_buf) {
		ret = lstp_recv_resp_helper(ch, LSTP_SPI_CMD_WRITE | cs_bits, xfer->len,
					    LSTP_ANY_RX_LEN);
	} else if (xfer->rx_buf) {
		spi_req->read.len = cpu_to_le32(xfer->len);
		ret = lstp_recv_resp_helper(ch, LSTP_SPI_CMD_READ | cs_bits, sizeof(spi_req->read),
					    xfer->len);
	} else {
		ret = -EINVAL;
		goto out_mutex;
	}

	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Transfer failed (%d)\n", __func__,
			ch->ch_id, ret);
		goto out_mutex;
	}

	if (xfer->rx_buf)
		memcpy(xfer->rx_buf, rx_pkt->payload, xfer->len);

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_spi_max_xfer_size() - Report maximum transfer size for the SPI controller.
 * @spi: SPI device querying the maximum size
 *
 * Returns the maximum number of bytes that can be transferred in a single
 * SPI transaction. This is limited by the USB endpoint buffer size minus
 * the LSTP header overhead.
 *
 * Context: Any context.
 *
 * Return: Maximum transfer size in bytes
 */
static size_t lstp_spi_max_xfer_size(struct spi_device *spi)
{
	struct lstp_channel *ch;
	size_t ep_size = LSTP_USB_EP_MAX_SIZE;

	if (spi) {
		ch = spi_controller_get_devdata(spi->controller);
		ep_size = min(ch->usb->bulk_tx_size, ch->usb->bulk_rx_size);
	}

	return ep_size - sizeof(struct lstp_header);
}

/**
 * lstp_spi_set_cs() - Set the chip select line state.
 * @spi:   SPI device whose chip select to control
 * @state: true to deassert (high), false to assert (low)
 *
 * Controls the chip select line for the SPI device. The specific CS line
 * (CS0-CS1) is determined from spi->chip_select. Sends a zero-length read
 * command with the appropriate CS selection and toggle flags to change the
 * CS state without transferring any data.
 *
 * Note: This uses a fire-and-forget USB bulk message without waiting for
 * a response, as CS toggling is time-sensitive and the response would
 * add unnecessary latency.
 *
 * Context: Process context. Takes ch->tx_mutex. Performs blocking USB I/O.
 */
static void lstp_spi_set_cs(struct spi_device *spi, bool state)
{
	int ret = 0;
	struct lstp_channel *ch = spi_controller_get_devdata(spi->controller);
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	union lstp_spi_req_payload *spi_req = (union lstp_spi_req_payload *)tx_pkt->payload;
	u8 chip_select = spi_get_chipselect(spi, 0);
	u8 cs_bits;
	u8 cmd;

	/* Validate chip select index */
	if (chip_select >= ARRAY_SIZE(lstp_spi_cs_map)) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid chip select %u (max %zu)\n",
			__func__, ch->ch_id, chip_select, ARRAY_SIZE(lstp_spi_cs_map) - 1);
		return;
	}
	cs_bits = lstp_spi_cs_map[chip_select];

	mutex_lock(&ch->tx_mutex);

	tx_pkt->hdr.ch_id = ch->ch_id;
	tx_pkt->hdr.length = cpu_to_le16(sizeof(*spi_req));

	/* Read command with zero data should do nothing other than toggle CS, no return */
	cmd = state ? LSTP_SPI_CMD_CS_DEASSERT : LSTP_SPI_CMD_CS_ASSERT;
	cmd |= cs_bits | LSTP_SPI_CMD_READ;
	tx_pkt->hdr.cmd = SET_U8_BYTE(cmd, 0);
	spi_req->read.len = 0;

	ret = usb_bulk_msg(ch->usb->udev, usb_sndbulkpipe(ch->usb->udev, ch->usb->bulk_out_ep),
			   ch->tx_buf, sizeof(tx_pkt->hdr) + sizeof(*spi_req), NULL,
			   LSTP_USB_REQUEST_TIMEOUT_MS);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not send CS state (%d)\n", __func__,
			ch->ch_id, ret);
	}

	mutex_unlock(&ch->tx_mutex);
}

/*******************************************************************************
 * SPI channel initialization and start
 ******************************************************************************/

/**
 * lstp_spi_init() - Initialize an LSTP SPI channel.
 * @ch: LSTP channel to initialize as SPI controller
 *
 * Allocates and initializes an SPI controller structure for the channel.
 * Parses the channel configuration from the management channel response
 * (which should still be in the USB device's rx_buf from lstp_ch0_read()).
 *
 * If a device tree node is associated with the channel, it is assigned to
 * the controller so that the SPI core can automatically enumerate child
 * SPI devices from the device tree.
 *
 * Expected device tree node structure (optional)::
 *
 *   channel@M {
 *       compatible = "nv,lstp-spi";
 *       reg = <M>;                  // Channel ID
 *       #address-cells = <1>;
 *       #size-cells = <0>;
 *
 *       spi-device@N {              // SPI slave devices (optional)
 *           compatible = "...";     // SPI device driver compatible string
 *           reg = <N>;              // Chip select number
 *           spi-max-frequency = <...>;
 *           // Additional SPI device properties...
 *       };
 *   };
 *
 * This function only allocates the controller but does NOT register it.
 * Call lstp_spi_start() after the RX URB is active to register the
 * controller.
 *
 * Context: Process context. Called during probe before RX URB is active.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_spi_init(struct lstp_channel *ch)
{
	int ret;
	struct spi_controller *ctrl;
	struct lstp_spi_priv *priv;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->usb->rx_buf;
	union lstp_ch0_resp_payload *ch0_resp;

	/* Validate expected SPI config size */
	ret = lstp_validate_resp(ch->usb, rx_pkt, sizeof(struct lstp_ch0_resp_read));
	if (ret)
		return ret;

	/* Allocate memory */
	ch->resp_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->resp_buf)
		return -ENOMEM;

	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	ch->ch_type = ch0_resp->read.ch_type;
	if (ch0_resp->read.ch_name[0] == '\0') {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid SPI controller name\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	priv = devm_kzalloc(&ch->usb->intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ctrl = devm_spi_alloc_host(&ch->usb->intf->dev, 0);
	if (!ctrl)
		return -ENOMEM;

	spi_controller_set_devdata(ctrl, ch);

	ctrl->bus_num = -1;
	ctrl->num_chipselect = ARRAY_SIZE(lstp_spi_cs_map);
	ctrl->transfer_one = lstp_spi_transfer_one;
	ctrl->set_cs = lstp_spi_set_cs;
	ctrl->max_transfer_size = lstp_spi_max_xfer_size;

	/* Set DT node so SPI core auto-enumerates child devices */
	ctrl->dev.of_node = ch->of_node;

	priv->ctrl = ctrl;
	priv->current_speed_hz = 0; /* Not configured yet, use device default */
	priv->requested_speed_hz = 0; /* No speed requested yet */
	ch->priv = priv;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Initialized\n", __func__, ch->ch_id);
	return 0;
}

/**
 * lstp_spi_start() - Start an LSTP SPI channel.
 * @ch: LSTP channel to start
 *
 * Registers the SPI controller with the Linux SPI subsystem. If a device
 * tree node is associated with the controller, the SPI core will
 * automatically enumerate and create SPI devices from child nodes.
 *
 * If no device tree node is present, SPI devices can be added via device
 * tree overlays or userspace sysfs interfaces.
 *
 * Context: Process context. Called during probe after RX URB is active.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_spi_start(struct lstp_channel *ch)
{
	int ret;
	struct lstp_spi_priv *priv = ch->priv;
	struct spi_controller *ctrl;
	u32 actual_speed_hz;

	if (!priv || !priv->ctrl) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: SPI controller not initialized\n",
			__func__, ch->ch_id);
		return -EINVAL;
	}

	ctrl = priv->ctrl;

	/*
	 * Workaround: Set initial SPI speed to 18.75 MHz (only value
	 * supported by current hardware) to properly report the speed to
	 * userspace. This must be done before controller registration, as we
	 * cannot safely change speed at runtime due to protocol limitations
	 * (CONFIG command de-asserts CS unconditionally).
	 */
	ret = lstp_spi_set_speed(ch, LSTP_SPI_DEFAULT_SPEED_HZ, &actual_speed_hz);
	if (ret) {
		dev_warn(&ch->usb->intf->dev,
			 "%s: ch_%d: Could not set initial SPI speed (%d), continuing anyway\n",
			 __func__, ch->ch_id, ret);
		/* Continue with registration even if speed setting fails */
	} else {
		dev_info(&ch->usb->intf->dev, "%s: ch_%d: Initial SPI speed set to %u Hz\n",
			 __func__, ch->ch_id, actual_speed_hz);
	}

	ret = devm_spi_register_controller(&ch->usb->intf->dev, ctrl);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register SPI controller (%d)\n",
			__func__, ch->ch_id, ret);
		return ret;
	}

	ch->child_dev = &ctrl->dev;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Started\n", __func__, ch->ch_id);
	return 0;
}
