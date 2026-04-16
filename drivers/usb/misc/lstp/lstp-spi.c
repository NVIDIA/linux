// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/spi/spi.h>
#include <linux/err.h>

#include "lstp-main.h"

/*******************************************************************************
 * SPI command definitions
 ******************************************************************************/

enum lstp_spi_cmd {
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
	LSTP_SPI_CMD_CS1 = 0x40,
	LSTP_SPI_CMD_CS2 = 0x80,
	LSTP_SPI_CMD_CS3 = 0xC0
}; /* clang-format on */
/* clang-format off */
static const u8 lstp_spi_cs_map[] = {
	LSTP_SPI_CMD_CS0,
	LSTP_SPI_CMD_CS1,
	LSTP_SPI_CMD_CS2,
	LSTP_SPI_CMD_CS3
}; /* clang-format on */

union lstp_spi_req_payload {
	struct {
		u32 len;
	} __packed read;
};

struct lstp_spi_config {
	u8 num_devices;
	u32 speed_hz;
} __packed;

/* Private data for SPI controller */
struct lstp_spi_priv {
	struct spi_controller *ctrl;
	u32 current_speed_hz;
};

/* Defaults for backwards compatibility with firmware lacking SPI channel config */
#define LSTP_SPI_DEFAULT_NUM_DEVICES 2
#define LSTP_SPI_DEFAULT_SPEED_HZ 18750000 /* 18.75 MHz */

/*******************************************************************************
 * SPI controller callbacks
 ******************************************************************************/

/**
 * lstp_spi_do_transfer() - Execute a single SPI transfer with CS flags.
 * @ch:       LSTP channel to transfer on
 * @xfer:     SPI transfer descriptor containing tx/rx buffers and length
 * @cs_bits:  Chip select selection bits (LSTP_SPI_CMD_CS0..CS3)
 * @cs_flags: Chip select toggle flags (LSTP_SPI_CMD_CS_ASSERT/DEASSERT)
 *
 * Core transfer routine shared by lstp_spi_transfer_one_message(). Supports
 * three modes based on which buffers are provided:
 * - Write-Read (full duplex): Both tx_buf and rx_buf set
 * - Write only: Only tx_buf set
 * - Read only: Only rx_buf set
 *
 * The caller must hold ch->tx_mutex. On success the response buffer is
 * unlocked before returning; on failure the buffer is not locked.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_spi_do_transfer(struct lstp_channel *ch, struct spi_transfer *xfer, u8 cs_bits,
				u8 cs_flags)
{
	struct lstp_spi_priv *priv = ch->priv;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_spi_req_payload *spi_req = (union lstp_spi_req_payload *)tx_pkt->payload;
	u8 cmd;
	int ret;

	if (xfer->word_delay.value)
		dev_warn_once(&ch->usb->intf->dev,
			      "%s: ch_%d: word_delay not supported, ignoring\n", __func__,
			      ch->ch_id);

	xfer->error = SPI_TRANS_FAIL_NO_START;

	xfer->effective_speed_hz = priv->current_speed_hz;

	if (sizeof(struct lstp_header) + xfer->len > ch->usb->bulk_tx_size ||
	    sizeof(struct lstp_header) + xfer->len > ch->usb->bulk_rx_size) {
		dev_err(&ch->usb->intf->dev,
			"%s: ch_%d: Transfer length %u exceeds buffer capacity\n", __func__,
			ch->ch_id, xfer->len);
		return -EINVAL;
	}

	if (xfer->tx_buf)
		memcpy(tx_pkt->payload, xfer->tx_buf, xfer->len);

	if (xfer->tx_buf && xfer->rx_buf) {
		cmd = LSTP_SPI_CMD_WRITE_READ | cs_bits | cs_flags;
		ret = lstp_recv_resp_helper(ch, cmd, xfer->len, xfer->len);
	} else if (xfer->tx_buf) {
		cmd = LSTP_SPI_CMD_WRITE | cs_bits | cs_flags;
		ret = lstp_recv_resp_helper(ch, cmd, xfer->len, LSTP_ANY_RX_LEN);
	} else if (xfer->rx_buf) {
		spi_req->read.len = cpu_to_le32(xfer->len);
		cmd = LSTP_SPI_CMD_READ | cs_bits | cs_flags;
		ret = lstp_recv_resp_helper(ch, cmd, sizeof(spi_req->read), xfer->len);
	} else {
		return -EINVAL;
	}

	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Transfer failed (%pe)\n", __func__,
			ch->ch_id, ERR_PTR(ret));
		xfer->error = SPI_TRANS_FAIL_IO;
		return ret;
	}

	if (xfer->rx_buf)
		memcpy(xfer->rx_buf, rx_pkt->payload, xfer->len);

	lstp_unlock_resp_buffer(ch);
	xfer->error = 0;
	return 0;
}

/**
 * lstp_spi_transfer_one_message() - Execute a complete SPI message.
 * @ctrl: SPI controller performing the transfer
 * @mesg: SPI message containing one or more transfers
 *
 * Processes all transfers in a message, embedding chip select assert/deassert
 * flags directly into each transfer command. This avoids the separate USB
 * round-trips that would be needed if the SPI core managed CS via a set_cs
 * callback.
 *
 * CS management follows standard SPI semantics:
 * - CS is asserted before the first transfer
 * - CS is deasserted after the last transfer
 * - xfer->cs_change inverts the default: deasserts mid-message, or leaves
 *   CS asserted after the last transfer
 *
 * On completion (success or failure), calls spi_finalize_current_message()
 * so the subsystem can issue the next queued message.
 *
 * Context: Process context. Takes ch->tx_mutex. Performs blocking USB I/O.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_spi_transfer_one_message(struct spi_controller *ctrl, struct spi_message *mesg)
{
	struct lstp_channel *ch = spi_controller_get_devdata(ctrl);
	struct spi_transfer *xfer;
	u8 chip_select = spi_get_chipselect(mesg->spi, 0);
	u8 cs_bits;
	int ret = 0;
	bool cs_active = false;

	if (chip_select >= ctrl->num_chipselect) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid chip select %u (max %u)\n",
			__func__, ch->ch_id, chip_select, ctrl->num_chipselect - 1);
		ret = -EINVAL;
		goto finalize;
	}
	cs_bits = lstp_spi_cs_map[chip_select];

	mutex_lock(&ch->tx_mutex);

	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		u8 cs_flags = 0;
		bool is_last = list_is_last(&xfer->transfer_list, &mesg->transfers);

		if (!cs_active) {
			cs_flags |= LSTP_SPI_CMD_CS_ASSERT;
			cs_active = true;
		}

		if ((is_last && !xfer->cs_change) || (!is_last && xfer->cs_change)) {
			cs_flags |= LSTP_SPI_CMD_CS_DEASSERT;
			cs_active = false;
		}

		ret = lstp_spi_do_transfer(ch, xfer, cs_bits, cs_flags);
		if (ret)
			break;

		mesg->actual_length += xfer->len;

		if (!is_last && xfer->cs_change)
			spi_transfer_cs_change_delay_exec(mesg, xfer);

		spi_transfer_delay_exec(xfer);
	}

	mutex_unlock(&ch->tx_mutex);

finalize:
	mesg->status = ret;
	spi_finalize_current_message(ctrl);
	return ret;
}

/**
 * lstp_spi_set_cs_timing() - Validate CS timing parameters for an SPI device.
 * @spi: SPI device whose CS timing is being configured
 *
 * Called by spi_setup(). The LSTP firmware manages CS assertion internally, so
 * fine-grained cs_setup / cs_hold / cs_inactive delays cannot be honoured.
 * Accept the configuration but warn if non-zero values were requested.
 *
 * Return: 0 always (timing is silently ignored)
 */
static int lstp_spi_set_cs_timing(struct spi_device *spi)
{
	struct lstp_channel *ch = spi_controller_get_devdata(spi->controller);

	if (spi->cs_setup.value || spi->cs_hold.value || spi->cs_inactive.value)
		dev_warn(&ch->usb->intf->dev,
			 "%s: ch_%d: CS timing delays not supported, ignoring\n", __func__,
			 ch->ch_id);
	return 0;
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

/*******************************************************************************
 * SPI channel initialization and start
 ******************************************************************************/

/**
 * lstp_spi_create_spidev() - Create spidev devices for the SPI controller.
 * @ctrl: SPI controller to create spidev devices for
 * @ch: LSTP channel to create spidev devices for
 *
 * Creates spidev devices on a SPI controller based on the number of chip selects.
 * Uses the "spidev" modalias which must be present in spidev's allowlist.
 */
static void lstp_spi_create_spidev(struct spi_controller *ctrl, struct lstp_channel *ch)
{
	struct lstp_spi_priv *priv = ch->priv;
	u8 cs;
	struct spi_board_info info;

	for (cs = 0; cs < ctrl->num_chipselect; cs++) {
		info = (struct spi_board_info){
			.max_speed_hz = priv->current_speed_hz,
			.mode = SPI_MODE_0,
			.chip_select = cs,
		};
		strscpy(info.modalias, "spidev", sizeof(info.modalias));

		if (!spi_new_device(ctrl, &info)) {
			dev_err(&ch->usb->intf->dev,
				"%s: ch_%d: Could not create spidev for CS%u\n", __func__,
				ch->ch_id, cs);
		}
	}
}

/**
 * lstp_spi_init() - Initialize an LSTP SPI channel.
 * @ch: LSTP channel to initialize as SPI controller
 *
 * Allocates and initializes an SPI controller structure for the channel.
 * Parses the channel configuration from the management channel response
 * (which is in the management channel's resp_buf from lstp_ch0_read_helper()).
 *
 * This function only allocates the controller but does NOT register it.
 * Call lstp_spi_start() after the RX URB is active to register the
 * controller and create spidev devices for each chip select.
 *
 * Context: Process context. Called during probe before RX URB is active.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_spi_init(struct lstp_channel *ch)
{
	int ret;
	struct lstp_spi_priv *priv;
	struct spi_controller *ctrl;
	struct lstp_channel *ch0 = ch->usb->channels[0];
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch0->resp_buf;
	union lstp_ch0_resp_payload *ch0_resp;

	/* Validate expected SPI config size */
	ret = lstp_validate_resp(ch->usb, rx_pkt, LSTP_ANY_RX_LEN);
	if (ret)
		return ret;

	/* Allocate memory */
	ch->resp_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->resp_buf)
		return -ENOMEM;

	priv = devm_kzalloc(&ch->usb->intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ctrl = devm_spi_alloc_host(&ch->usb->intf->dev, 0);
	if (!ctrl)
		return -ENOMEM;

	/* Save channel data and name */
	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	ch->ch_type = ch0_resp->read.ch_type;
	if (ch0_resp->read.ch_name[0] == '\0') {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid SPI controller name\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	/* Parse SPI config (Uses defaults if no config is present)*/
	size_t payload_len = le16_to_cpu(rx_pkt->hdr.length);
	u8 num_devices;
	u32 speed_hz;

	if (payload_len == sizeof(struct lstp_ch0_resp_read) + sizeof(struct lstp_spi_config)) {
		struct lstp_spi_config *config = (struct lstp_spi_config *)ch0_resp->read.ch_config;

		num_devices = config->num_devices;
		speed_hz = le32_to_cpu(config->speed_hz);
	} else {
		/* TODO: Legacy support - remove this at some point */
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: No SPI channel config, using defaults\n",
			__func__, ch->ch_id);
		num_devices = LSTP_SPI_DEFAULT_NUM_DEVICES;
		speed_hz = LSTP_SPI_DEFAULT_SPEED_HZ;
	}

	if (num_devices == 0 || num_devices > ARRAY_SIZE(lstp_spi_cs_map)) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid num_devices %u (max %zu)\n",
			__func__, ch->ch_id, num_devices, ARRAY_SIZE(lstp_spi_cs_map));
		return -EINVAL;
	}

	if (speed_hz == 0) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Device returned SPI speed of %u Hz\n",
			__func__, ch->ch_id, speed_hz);
		return -EINVAL;
	}

	spi_controller_set_devdata(ctrl, ch);
	ctrl->bus_num = -1;
	ctrl->num_chipselect = num_devices;
	ctrl->transfer_one_message = lstp_spi_transfer_one_message;
	ctrl->max_transfer_size = lstp_spi_max_xfer_size;
	ctrl->set_cs_timing = lstp_spi_set_cs_timing;

	/* Set firmware node so SPI core auto-enumerates child devices (DT or ACPI) */
	device_set_node(&ctrl->dev, ch->fwnode);

	priv->ctrl = ctrl;
	priv->current_speed_hz = speed_hz;

	ch->priv = priv;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Initialized (devices=%u, speed=%u Hz)\n",
		 __func__, ch->ch_id, ctrl->num_chipselect, priv->current_speed_hz);
	return 0;
}

/**
 * lstp_spi_start() - Start an LSTP SPI channel.
 * @ch: LSTP channel to start
 *
 * Registers the SPI controller with the Linux SPI subsystem and creates
 * a spidev device for each chip select so that userspace can access the
 * bus via /dev/spidevN.CS.
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

	if (!priv || !priv->ctrl) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: SPI controller not initialized\n",
			__func__, ch->ch_id);
		return -EINVAL;
	}

	ctrl = priv->ctrl;

	ret = devm_spi_register_controller(&ch->usb->intf->dev, ctrl);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register SPI controller (%pe)\n",
			__func__, ch->ch_id, ERR_PTR(ret));
		return ret;
	}

	if (ch->fwnode) {
		unsigned int child_count = 0;
		struct fwnode_handle *child;

		fwnode_for_each_available_child_node(ch->fwnode, child)
			child_count++;

		if (child_count == 0) {
			dev_warn(&ch->usb->intf->dev,
				 "%s: ch_%d: Firmware node present but defines no SPI devices\n",
				 __func__, ch->ch_id);
		} else {
			dev_info(&ch->usb->intf->dev,
				 "%s: ch_%d: Firmware node present with %u SPI devices\n", __func__,
				 ch->ch_id, child_count);
		}
	} else if (lstp_auto_bind_spidev) {
		lstp_spi_create_spidev(ctrl, ch);
	}

	ch->child_dev = &ctrl->dev;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Started\n", __func__, ch->ch_id);
	return 0;
}
