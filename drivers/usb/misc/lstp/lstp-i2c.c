// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/i2c.h>

#include "lstp-main.h"

enum lstp_i2c_cmd {
	LSTP_I2C_CMD_BUS_RECOVERY = 0x00,
	LSTP_I2C_CMD_READ = 0x01,
	LSTP_I2C_CMD_WRITE = 0x02,
	LSTP_I2C_CMD_READ_RECVLEN = 0x03,
	LSTP_I2C_CMD_WRITE_READ = 0x04
} __packed;

enum lstp_i2c_cmd_flags { LSTP_I2C_CMD_FLAG_NO_STOP = 0x40 };

union lstp_i2c_req_payload {
	struct {
		u8 addr;
		u16 rd_len;
	} __packed read;
	struct {
		u8 addr;
		u8 wr_data[];
	} __packed write;
	struct {
		u8 addr;
	} __packed rd_recvlen;
	struct {
		u8 addr;
		u16 rd_len;
		u8 wr_data[]; /* Write length inferred from length in header */
	} __packed wr_rd;
};

enum lstp_i2c_speed {
	LSTP_I2C_SPEED_100KHZ = 0x00,
	LSTP_I2C_SPEED_400KHZ = 0x01,
	LSTP_I2C_SPEED_1000KHZ = 0x02,
	LSTP_I2C_SPEED_3400KHZ = 0x03,
};

struct lstp_i2c_config {
	u8 speed;
} __packed;

/**
 * lstp_i2c_bus_recovery() - Perform I2C bus recovery procedure.
 * @adap: I2C adapter to recover
 *
 * Return: 0 on success, negative errno on failure
 */
static int __maybe_unused lstp_i2c_bus_recovery(struct i2c_adapter *adap)
{
	int ret;
	struct lstp_channel *ch = adap->algo_data;

	mutex_lock(&ch->tx_mutex);

	ret = lstp_recv_resp_helper(ch, LSTP_I2C_CMD_BUS_RECOVERY, 0, 0);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Bus recovery failed (%d)\n", __func__, ch->ch_id,
			ret);
		goto out_mutex;
	}

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_validate_msg() - Validate an I2C message before transmission.
 * @msg: I2C message to validate
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_validate_msg(struct i2c_msg *msg)
{
	if (!msg->buf && msg->len > 0)
		return -EINVAL;

	if (msg->flags & I2C_M_TEN)
		return -EAFNOSUPPORT; /* 10-bit address not supported */

	if (msg->addr > 0x7F)
		return -EINVAL; /* Only 7-bit address supported */

	return 0;
}

/**
 * lstp_i2c_read() - Perform an I2C read transaction.
 * @adap:    I2C adapter to use
 * @msg:     I2C message describing the read
 * @no_stop: If true, omit STOP for repeated START
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_read(struct i2c_adapter *adap, struct i2c_msg *msg, bool no_stop)
{
	int ret = 0;
	u8 cmd = LSTP_I2C_CMD_READ;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;

	ret = lstp_i2c_validate_msg(msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%d)\n", __func__,
			ch->ch_id, msg->addr, ret);
		return ret;
	}

	if (msg->len > ch->usb->bulk_rx_size - sizeof(struct lstp_header)) {
		dev_err(&adap->dev,
			"%s: ch_%d: Read request message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, msg->addr, msg->len,
			ch->usb->bulk_rx_size - sizeof(struct lstp_header));
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	i2c_req->read.addr = msg->addr;
	i2c_req->read.rd_len = cpu_to_le16(msg->len);

	dev_dbg(&adap->dev, "%s: ch_%d: addr=0x%02x, len=%d\n", __func__, ch->ch_id, msg->addr,
		msg->len);

	if (no_stop)
		cmd |= LSTP_I2C_CMD_FLAG_NO_STOP;

	ret = lstp_recv_resp_helper(ch, cmd, sizeof(i2c_req->read), msg->len);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK))
			dev_err(&adap->dev, "%s: ch_%d: Read request to addr=0x%02x failed (%d)\n",
				__func__, ch->ch_id, msg->addr, ret);
		goto out_mutex;
	}

	memcpy(msg->buf, rx_pkt->payload, msg->len);

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_write() - Perform an I2C write transaction.
 * @adap:    I2C adapter to use
 * @msg:     I2C message describing the write
 * @no_stop: If true, omit STOP for repeated START
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_write(struct i2c_adapter *adap, struct i2c_msg *msg, bool no_stop)
{
	int ret = 0;
	u8 cmd = LSTP_I2C_CMD_WRITE;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;

	ret = lstp_i2c_validate_msg(msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%d)\n", __func__,
			ch->ch_id, msg->addr, ret);
		return ret;
	}

	if (sizeof(struct lstp_header) + sizeof(i2c_req->write) + msg->len >
	    ch->usb->bulk_tx_size) {
		dev_err(&adap->dev,
			"%s: ch_%d: Write message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, msg->addr, msg->len,
			ch->usb->bulk_tx_size - sizeof(struct lstp_header) -
				sizeof(i2c_req->write));
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	i2c_req->write.addr = msg->addr;
	if (msg->len > 0) {
		memcpy(i2c_req->write.wr_data, msg->buf, msg->len);
		dev_dbg(&adap->dev, "%s: ch_%d: addr=0x%02x, len=%d, data=0x%*ph\n", __func__,
			ch->ch_id, msg->addr, msg->len, msg->len, i2c_req->write.wr_data);
	} else {
		dev_dbg(&adap->dev, "%s: ch_%d: addr=0x%02x, len=%d\n", __func__, ch->ch_id,
			msg->addr, msg->len);
	}

	if (no_stop)
		cmd |= LSTP_I2C_CMD_FLAG_NO_STOP;

	ret = lstp_recv_resp_helper(ch, cmd, sizeof(i2c_req->write) + msg->len, 0);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK))
			dev_err(&adap->dev, "%s: ch_%d: Write request to addr=0x%02x failed (%d)\n",
				__func__, ch->ch_id, msg->addr, ret);
		goto out_mutex;
	}

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_read_recvlen() - SMBus block read (slave indicates length).
 * @adap:    I2C adapter to use
 * @msg:     I2C message; msg->len updated to actual bytes received
 * @no_stop: If true, omit STOP for repeated START
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_read_recvlen(struct i2c_adapter *adap, struct i2c_msg *msg, bool no_stop)
{
	int ret = 0;
	u8 cmd = LSTP_I2C_CMD_READ_RECVLEN;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;

	ret = lstp_i2c_validate_msg(msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%d)\n", __func__,
			ch->ch_id, msg->addr, ret);
		return ret;
	}

	if (msg->len > ch->usb->bulk_rx_size - sizeof(struct lstp_header)) {
		dev_err(&adap->dev,
			"%s: ch_%d: Read recvlen message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, msg->addr, msg->len,
			ch->usb->bulk_rx_size - sizeof(struct lstp_header));
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	i2c_req->rd_recvlen.addr = msg->addr;

	if (no_stop)
		cmd |= LSTP_I2C_CMD_FLAG_NO_STOP;

	ret = lstp_recv_resp_helper(ch, cmd, sizeof(i2c_req->rd_recvlen), LSTP_ANY_RX_LEN);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK))
			dev_err(&adap->dev,
				"%s: ch_%d: Read recvlen request to addr=0x%02x failed (%d)\n",
				__func__, ch->ch_id, msg->addr, ret);
		goto out_mutex;
	}

	if (le16_to_cpu(rx_pkt->hdr.length) > msg->len) {
		dev_err(&adap->dev, "%s: ch_%d: Response too large (got %u, max %u)\n", __func__,
			ch->ch_id, le16_to_cpu(rx_pkt->hdr.length), msg->len);
		ret = -EMSGSIZE;
	} else {
		msg->len = le16_to_cpu(rx_pkt->hdr.length);
		memcpy(msg->buf, rx_pkt->payload, msg->len);
	}

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_write_read() - Atomic write-then-read with repeated START.
 * @adap:   I2C adapter to use
 * @wr_msg: I2C message describing the write phase (addr, buf, len)
 * @rd_msg: I2C message describing the read phase (addr, buf, len)
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_write_read(struct i2c_adapter *adap, struct i2c_msg *wr_msg,
			       struct i2c_msg *rd_msg)
{
	int ret = 0;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;

	ret = lstp_i2c_validate_msg(wr_msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid write message to addr=0x%02x (%d)\n",
			__func__, ch->ch_id, wr_msg->addr, ret);
		return ret;
	}

	if (sizeof(struct lstp_header) + sizeof(i2c_req->wr_rd) + wr_msg->len >
	    ch->usb->bulk_tx_size) {
		dev_err(&adap->dev,
			"%s: ch_%d: Write message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, wr_msg->addr, wr_msg->len,
			ch->usb->bulk_tx_size - sizeof(struct lstp_header) -
				sizeof(i2c_req->wr_rd));
		return -EINVAL;
	}

	ret = lstp_i2c_validate_msg(rd_msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid read message to addr=0x%02x (%d)\n",
			__func__, ch->ch_id, rd_msg->addr, ret);
		return ret;
	}

	if (rd_msg->len > ch->usb->bulk_rx_size - sizeof(struct lstp_header)) {
		dev_err(&adap->dev,
			"%s: ch_%d: Read message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, rd_msg->addr, rd_msg->len,
			ch->usb->bulk_rx_size - sizeof(struct lstp_header));
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	i2c_req->wr_rd.addr = wr_msg->addr;
	i2c_req->wr_rd.rd_len = cpu_to_le16(rd_msg->len);
	memcpy(i2c_req->wr_rd.wr_data, wr_msg->buf, wr_msg->len);

	ret = lstp_recv_resp_helper(ch, LSTP_I2C_CMD_WRITE_READ,
				    sizeof(i2c_req->wr_rd) + wr_msg->len, rd_msg->len);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK))
			dev_err(&adap->dev,
				"%s: ch_%d: Write-read request to addr=0x%02x failed (%d)\n",
				__func__, ch->ch_id, wr_msg->addr, ret);
		goto out_mutex;
	}

	memcpy(rd_msg->buf, rx_pkt->payload, rd_msg->len);

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_xfer() - Main I2C transfer function for the adapter.
 * @adap: I2C adapter to use
 * @msgs: Array of I2C messages to transfer
 * @num:  Number of messages in the array
 *
 * Implements the i2c_algorithm.xfer callback. Processes an array of I2C
 * messages, handling read, write, and combined transactions.
 *
 * Return: Number of messages transferred on success, negative errno on failure
 */
static int lstp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret = 0;
	int i;
	struct lstp_channel *ch = adap->algo_data;

	if (!msgs || num <= 0)
		return -EINVAL;

	dev_dbg(&adap->dev, "%s: ch_%d: I2C transfer with %d message(s)\n", __func__, ch->ch_id,
		num);

	if (num == 2 && !(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD) &&
	    !(msgs[1].flags & I2C_M_RECV_LEN)) {
		/*
		 * TODO: This should be more generic but that requires some MCU changes
		 * 1) Support no-stop flag for atomic write-read
		 *    (skip over matching pairs in the loop instead of this special case)
		 * 2) Support recv-len for atomic write-read
		 */
		ret = lstp_i2c_write_read(adap, &msgs[0], &msgs[1]);
		return ret ? ret : num;
	}

	for (i = 0; i < num; i++) {
		bool no_stop = (i != num - 1);

		if (msgs[i].flags & I2C_M_RD) {
			if (msgs[i].flags & I2C_M_RECV_LEN)
				ret = lstp_i2c_read_recvlen(adap, &msgs[i], no_stop);
			else
				ret = lstp_i2c_read(adap, &msgs[i], no_stop);
		} else {
			ret = lstp_i2c_write(adap, &msgs[i], no_stop);
		}

		if (ret)
			return ret;
	}

	return num;
}

/**
 * lstp_i2c_functionality() - Report I2C adapter capabilities.
 * @adap: I2C adapter to query
 *
 * Return: Bitmask of supported I2C_FUNC_* flags
 */
static u32 lstp_i2c_functionality(struct i2c_adapter *adap)
{
	/* TODO: This needs to be discoverable, also need separate commands for 10-bit address */
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL_ALL;
}

static const struct i2c_algorithm lstp_i2c_algorithm = {
	.xfer = lstp_i2c_xfer,
	.functionality = lstp_i2c_functionality,
};

/**
 * lstp_i2c_init() - Initialize I2C adapter for an LSTP channel.
 * @ch: LSTP channel configured as I2C type
 *
 * Allocates adapter and parses config from usb->rx_buf. Adapter stored in
 * ch->priv but not registered; call lstp_i2c_start() after RX URB setup.
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_i2c_init(struct lstp_channel *ch)
{
	int ret;
	struct i2c_adapter *adap;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->usb->rx_buf;
	union lstp_ch0_resp_payload *ch0_resp;
	/* struct lstp_i2c_config *config = (struct lstp_i2c_config *)ch0_resp->read.ch_config; */

	/* Validate expected I2C config size */
	ret = lstp_validate_resp(ch->usb, rx_pkt,
				 sizeof(struct lstp_ch0_resp_read) +
					 sizeof(struct lstp_i2c_config));
	if (ret)
		return ret;

	/* Allocate memory */
	adap = devm_kzalloc(&ch->usb->intf->dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;
	ch->resp_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->resp_buf)
		return -ENOMEM;

	/* Save channel type and name */
	ch0_resp = (union lstp_ch0_resp_payload *)rx_pkt->payload;
	ch->ch_type = ch0_resp->read.ch_type;
	if (ch0_resp->read.ch_name[0] == '\0') {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Invalid I2C adapter name\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	/* Initialize I2C adapter */
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->algo = &lstp_i2c_algorithm;
	adap->algo_data = ch;
	adap->dev.parent = &ch->usb->intf->dev;
	snprintf(adap->name, sizeof(adap->name), "%s_%s", ch->usb->lstp_intf_name,
		 ch0_resp->read.ch_name);
	/* TODO: save i2c speed from config */
	ch->priv = adap;

	dev_dbg(&ch->usb->intf->dev, "%s: I2C channel %d initialized as %s\n", __func__, ch->ch_id,
		adap->name);
	return 0;
}

/**
 * lstp_i2c_start() - Register I2C adapter with Linux I2C core.
 * @ch: LSTP channel with initialized adapter (from lstp_i2c_init)
 *
 * Return: 0 on success, negative errno on failure
 */
int lstp_i2c_start(struct lstp_channel *ch)
{
	int ret;
	struct i2c_adapter *adap = ch->priv;

	if (!adap) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: I2C adapter not initialized\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	ret = devm_i2c_add_adapter(&ch->usb->intf->dev, adap);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register I2C adapter (%d)\n",
			__func__, ch->ch_id, ret);
		return ret;
	}

	ch->child_dev = &adap->dev;

	dev_info(&ch->usb->intf->dev, "%s: I2C channel %d registered as %s\n", __func__, ch->ch_id,
		 adap->name);
	return 0;
}
