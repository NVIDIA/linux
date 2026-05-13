// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C driver for LSTP USB interface.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/version.h>

#include "lstp-main.h"

enum lstp_i2c_cmd {
	LSTP_I2C_CMD_BUS_RECOVERY = 0x00,
	LSTP_I2C_CMD_READ = 0x01,
	LSTP_I2C_CMD_WRITE = 0x02,
	LSTP_I2C_CMD_READ_RECVLEN_DEPRECATED = 0x03, /* Do not use */
	LSTP_I2C_CMD_WRITE_READ = 0x04,
	LSTP_I2C_CMD_SMBUS_BLOCK_READ = 0x05
} __packed;

enum lstp_i2c_cmd_flags { LSTP_I2C_CMD_FLAG_NO_STOP = 0x40 };

enum lstp_i2c_smbus_block_read_flags {
	LSTP_I2C_SMBUS_BLOCK_READ_FLAG_PEC = BIT(0),
} __packed;

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
	} __packed rd_recvlen_deprecated;
	struct {
		u8 addr;
		u16 rd_len;
		u8 wr_data[]; /* Write length inferred from length in header */
	} __packed wr_rd;
	struct {
		u8 addr;
		u16 flags; /* lstp_i2c_smbus_block_read_flags */
		u8 wr_data[]; /* Write length inferred from length in header */
	} __packed smbus_block_read;
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

struct lstp_i2c_priv {
	struct i2c_adapter *adap;
	bool use_smbus_block_read; /* False once MCU has rejected LSTP_I2C_CMD_SMBUS_BLOCK_READ */
};

/**
 * lstp_i2c_bus_recovery() - Perform I2C bus recovery procedure.
 * @adap: I2C adapter to recover
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_bus_recovery(struct i2c_adapter *adap)
{
	int ret;
	struct lstp_channel *ch = adap->algo_data;

	mutex_lock(&ch->tx_mutex);

	ret = lstp_recv_resp_helper(ch, LSTP_I2C_CMD_BUS_RECOVERY, 0, 0);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Bus recovery failed (%pe)\n", __func__, ch->ch_id,
			ERR_PTR(ret));
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
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%pe)\n", __func__,
			ch->ch_id, msg->addr, ERR_PTR(ret));
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
			dev_err(&adap->dev, "%s: ch_%d: Read request to addr=0x%02x failed (%pe)\n",
				__func__, ch->ch_id, msg->addr, ERR_PTR(ret));
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
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%pe)\n", __func__,
			ch->ch_id, msg->addr, ERR_PTR(ret));
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
			dev_err(&adap->dev,
				"%s: ch_%d: Write request to addr=0x%02x failed (%pe)\n", __func__,
				ch->ch_id, msg->addr, ERR_PTR(ret));
		goto out_mutex;
	}

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_read_recvlen_deprecated() - Deprecated SMBus block read (slave indicates length). Does
 * not support PEC.
 * @adap:    I2C adapter to use
 * @msg:     I2C message; msg->len updated to actual bytes received
 * @no_stop: If true, omit STOP for repeated START
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_read_recvlen_deprecated(struct i2c_adapter *adap, struct i2c_msg *msg,
					    bool no_stop)
{
	int ret = 0;
	u8 cmd = LSTP_I2C_CMD_READ_RECVLEN_DEPRECATED;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;
	u16 pkt_len = 0;
	u16 rx_len = 0;
	u8 block_len = 0;

	ret = lstp_i2c_validate_msg(msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid message to addr=0x%02x (%pe)\n", __func__,
			ch->ch_id, msg->addr, ERR_PTR(ret));
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

	i2c_req->rd_recvlen_deprecated.addr = msg->addr;

	if (no_stop)
		cmd |= LSTP_I2C_CMD_FLAG_NO_STOP;

	ret = lstp_recv_resp_helper(ch, cmd, sizeof(i2c_req->rd_recvlen_deprecated),
				    LSTP_ANY_RX_LEN);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK))
			dev_err(&adap->dev,
				"%s: ch_%d: Read recvlen request to addr=0x%02x failed (%pe)\n",
				__func__, ch->ch_id, msg->addr, ERR_PTR(ret));
		goto out_mutex;
	}

	pkt_len = le16_to_cpu(rx_pkt->hdr.length);
	if (pkt_len == 0) {
		dev_err(&adap->dev, "%s: ch_%d: Empty response packet\n", __func__, ch->ch_id);
		ret = -EIO;
		goto out_buffer;
	}

	block_len = rx_pkt->payload[0];
	if (block_len > I2C_SMBUS_BLOCK_MAX) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid block length %u\n", __func__, ch->ch_id,
			block_len);
		ret = -EPROTO;
		goto out_buffer;
	}

	rx_len = block_len + 1;
	if (pkt_len < rx_len) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid response length (%u)\n", __func__,
			ch->ch_id, pkt_len);
		ret = -EIO;
		goto out_buffer;
	}

	msg->len = rx_len;
	memcpy(msg->buf, rx_pkt->payload, msg->len);

out_buffer:
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
		dev_err(&adap->dev, "%s: ch_%d: Invalid write message to addr=0x%02x (%pe)\n",
			__func__, ch->ch_id, wr_msg->addr, ERR_PTR(ret));
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
		dev_err(&adap->dev, "%s: ch_%d: Invalid read message to addr=0x%02x (%pe)\n",
			__func__, ch->ch_id, rd_msg->addr, ERR_PTR(ret));
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
				"%s: ch_%d: Write-read request to addr=0x%02x failed (%pe)\n",
				__func__, ch->ch_id, wr_msg->addr, ERR_PTR(ret));
		goto out_mutex;
	}

	memcpy(rd_msg->buf, rx_pkt->payload, rd_msg->len);

	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_smbus_block_read() - SMBus block read with optional write phase (slave indicates
 * length).
 * @adap:    I2C adapter to use
 * @wr_msg:  Optional write phase (addr, buf, len). If non-NULL, must target the same slave as
 * rd_msg.
 * @rd_msg:  I2C message describing the read phase; rd_msg->len updated to actual bytes received
 * @no_stop: If true, omit STOP for repeated START
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_smbus_block_read(struct i2c_adapter *adap, struct i2c_msg *wr_msg,
				     struct i2c_msg *rd_msg, bool no_stop)
{
	int ret = 0;
	u8 cmd = LSTP_I2C_CMD_SMBUS_BLOCK_READ;
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_packet *tx_pkt = (struct lstp_packet *)ch->tx_buf;
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch->resp_buf;
	union lstp_i2c_req_payload *i2c_req = (union lstp_i2c_req_payload *)tx_pkt->payload;
	u16 pkt_len = 0;
	u16 rx_len = 0;
	u16 wr_len = wr_msg ? wr_msg->len : 0;
	u8 block_len = 0;
	bool pec = rd_msg->flags & I2C_CLIENT_PEC;

	ret = lstp_i2c_validate_msg(rd_msg);
	if (ret) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid read message to addr=0x%02x (%pe)\n",
			__func__, ch->ch_id, rd_msg->addr, ERR_PTR(ret));
		return ret;
	}

	if (wr_msg) {
		ret = lstp_i2c_validate_msg(wr_msg);
		if (ret) {
			dev_err(&adap->dev,
				"%s: ch_%d: Invalid write message to addr=0x%02x (%pe)\n", __func__,
				ch->ch_id, wr_msg->addr, ERR_PTR(ret));
			return ret;
		}

		if (wr_msg->addr != rd_msg->addr) {
			dev_err(&adap->dev,
				"%s: ch_%d: Write/read addr mismatch (wr=0x%02x, rd=0x%02x)\n",
				__func__, ch->ch_id, wr_msg->addr, rd_msg->addr);
			return -EINVAL;
		}
	}

	if (sizeof(struct lstp_header) + sizeof(i2c_req->smbus_block_read) + wr_len >
	    ch->usb->bulk_tx_size) {
		dev_err(&adap->dev,
			"%s: ch_%d: Write message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, rd_msg->addr, wr_len,
			ch->usb->bulk_tx_size - sizeof(struct lstp_header) -
				sizeof(i2c_req->smbus_block_read));
		return -EINVAL;
	}

	if (rd_msg->len > ch->usb->bulk_rx_size - sizeof(struct lstp_header)) {
		dev_err(&adap->dev,
			"%s: ch_%d: Read message to addr=0x%02x too long (%u bytes, max %zu)\n",
			__func__, ch->ch_id, rd_msg->addr, rd_msg->len,
			ch->usb->bulk_rx_size - sizeof(struct lstp_header));
		return -EINVAL;
	}

	mutex_lock(&ch->tx_mutex);

	i2c_req->smbus_block_read.addr = rd_msg->addr;
	i2c_req->smbus_block_read.flags = cpu_to_le16(pec ? LSTP_I2C_SMBUS_BLOCK_READ_FLAG_PEC : 0);
	if (wr_msg && wr_len > 0)
		memcpy(i2c_req->smbus_block_read.wr_data, wr_msg->buf, wr_len);

	if (no_stop)
		cmd |= LSTP_I2C_CMD_FLAG_NO_STOP;

	ret = lstp_recv_resp_helper(ch, cmd, sizeof(i2c_req->smbus_block_read) + wr_len,
				    LSTP_ANY_RX_LEN);
	if (ret) {
		if (ret != lstp_status_to_errno(LSTP_NACK) &&
		    ret != lstp_status_to_errno(LSTP_NOT_SUPP))
			dev_err(&adap->dev,
				"%s: ch_%d: SMBus block read request to addr=0x%02x failed (%pe)\n",
				__func__, ch->ch_id, rd_msg->addr, ERR_PTR(ret));
		goto out_mutex;
	}

	pkt_len = le16_to_cpu(rx_pkt->hdr.length);
	if (pkt_len == 0) {
		dev_err(&adap->dev, "%s: ch_%d: Empty response packet\n", __func__, ch->ch_id);
		ret = -EIO;
		goto out_buffer;
	}

	block_len = rx_pkt->payload[0];
	if (block_len > I2C_SMBUS_BLOCK_MAX) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid block length %u\n", __func__, ch->ch_id,
			block_len);
		ret = -EPROTO;
		goto out_buffer;
	}

	rx_len = block_len + (pec ? 2 : 1);
	if (pkt_len < rx_len) {
		dev_err(&adap->dev, "%s: ch_%d: Invalid response length (%u)\n", __func__,
			ch->ch_id, pkt_len);
		ret = -EIO;
		goto out_buffer;
	}

	rd_msg->len = rx_len;
	memcpy(rd_msg->buf, rx_pkt->payload, rd_msg->len);

out_buffer:
	lstp_unlock_resp_buffer(ch);
out_mutex:
	mutex_unlock(&ch->tx_mutex);
	return ret;
}

/**
 * lstp_i2c_smbus_block_read_wrapper() - SMBus block read with legacy fallback.
 * @adap:    I2C adapter to use
 * @wr_msg:  Optional write phase. If non-NULL, must target the same slave as @rd_msg.
 * @rd_msg:  Read phase message; rd_msg->len is updated to actual bytes received.
 * @no_stop: If true, omit STOP for repeated START on the read phase.
 *
 * Tries the atomic SMBUS_BLOCK_READ opcode first. If the device
 * rejects it (or has done so previously this session), falls back to a
 * lstp_i2c_write() + lstp_i2c_read_recvlen_deprecated() transaction.
 * PEC clients are rejected on the fallback path because the deprecated path cannot transport the
 * trailing PEC byte.
 *
 * TODO: remove the fallback path (and lstp_i2c_read_recvlen_deprecated()) once
 * all deployed devices support SMBUS_BLOCK_READ.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_smbus_block_read_wrapper(struct i2c_adapter *adap, struct i2c_msg *wr_msg,
					     struct i2c_msg *rd_msg, bool no_stop)
{
	struct lstp_channel *ch = adap->algo_data;
	struct lstp_i2c_priv *priv = ch->priv;
	u16 wr_len = wr_msg ? wr_msg->len : 0;
	int ret;

	if (READ_ONCE(priv->use_smbus_block_read)) {
		ret = lstp_i2c_smbus_block_read(adap, wr_msg, rd_msg, no_stop);
		if (ret != lstp_status_to_errno(LSTP_NOT_SUPP))
			return ret;

		/*
		 * MCU rejected the new opcode; cache the result so subsequent
		 * calls skip straight to the legacy chain.
		 */
		dev_warn_once(&adap->dev,
			      "%s: ch_%d: Using deprecated recvlen path! -- Update LSTP Device\n",
			      __func__, ch->ch_id);
		WRITE_ONCE(priv->use_smbus_block_read, false);
	}

	/* Deprecated fallback path */
	if (rd_msg->flags & I2C_CLIENT_PEC) {
		dev_err(&adap->dev,
			"%s: ch_%d: PEC client at addr=0x%02x not supported by deprecated recv-len opcode; update MCU firmware to support LSTP_I2C_CMD_SMBUS_BLOCK_READ\n",
			__func__, ch->ch_id, rd_msg->addr);
		return -EOPNOTSUPP;
	}

	if (wr_msg && wr_len > 0) {
		ret = lstp_i2c_write(adap, wr_msg, true /* no_stop */);
		if (ret)
			return ret;
	}
	return lstp_i2c_read_recvlen_deprecated(adap, rd_msg, no_stop);
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
		 */
		ret = lstp_i2c_write_read(adap, &msgs[0], &msgs[1]);
		return ret ? ret : num;
	}

	for (i = 0; i < num; i++) {
		bool no_stop = (i != num - 1);

		if (msgs[i].flags & I2C_M_RD) {
			if (msgs[i].flags & I2C_M_RECV_LEN)
				ret = lstp_i2c_smbus_block_read_wrapper(adap, NULL, &msgs[i],
									no_stop);
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

static struct i2c_bus_recovery_info lstp_i2c_recovery_info = {
	.recover_bus = lstp_i2c_bus_recovery,
};

static const struct i2c_algorithm lstp_i2c_algorithm = {
#if KERNEL_VERSION(6, 11, 0) <= LINUX_VERSION_CODE
	.xfer = lstp_i2c_xfer,
#else
	.master_xfer = lstp_i2c_xfer,
#endif
	.functionality = lstp_i2c_functionality,
};

/**
 * lstp_i2c_init() - Initialize I2C adapter for an LSTP channel.
 * @ch: LSTP channel configured as I2C type
 *
 * Allocates adapter and parses config from ch0->resp_buf. Adapter stored in
 * ch->priv but not registered; call lstp_i2c_start() after RX URB setup.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_init(struct lstp_channel *ch)
{
	int ret;
	struct i2c_adapter *adap;
	struct lstp_i2c_priv *priv;
	struct lstp_channel *ch0 = ch->usb->channels[0];
	struct lstp_packet *rx_pkt = (struct lstp_packet *)ch0->resp_buf;

	/* Validate expected I2C config size */
	ret = lstp_validate_resp(ch->usb, rx_pkt,
				 sizeof(struct lstp_ch0_resp_read) +
					 sizeof(struct lstp_i2c_config));
	if (ret)
		return ret;

	/* Allocate memory */
	priv = devm_kzalloc(&ch->usb->intf->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	adap = devm_kzalloc(&ch->usb->intf->dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;
	ch->resp_buf = devm_kzalloc(&ch->usb->intf->dev, ch->usb->bulk_rx_size, GFP_KERNEL);
	if (!ch->resp_buf)
		return -ENOMEM;

	/* Initialize I2C adapter */
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->algo = &lstp_i2c_algorithm;
	adap->algo_data = ch;
	adap->dev.parent = &ch->usb->intf->dev;
	adap->bus_recovery_info = &lstp_i2c_recovery_info;
	device_set_node(&adap->dev, ch->fwnode);
	strscpy(adap->name, ch->display_name, sizeof(adap->name));
	/* TODO: save i2c speed from config */

	priv->adap = adap;
	priv->use_smbus_block_read = true;
	ch->priv = priv;

	dev_dbg(&ch->usb->intf->dev, "%s: ch_%d: Initialized as %s\n", __func__, ch->ch_id,
		adap->name);
	return 0;
}

/**
 * lstp_i2c_start() - Register I2C adapter with Linux I2C core.
 * @ch: LSTP channel with initialized adapter (from lstp_i2c_init)
 *
 * When the adapter's firmware node is set (from lstp_i2c_init), the
 * I2C core resolves bus numbers via aliases and auto-enumerates child
 * devices from firmware (DT or ACPI) child nodes.
 *
 * Return: 0 on success, negative errno on failure
 */
static int lstp_i2c_start(struct lstp_channel *ch)
{
	int ret;
	struct lstp_i2c_priv *priv = ch->priv;
	struct i2c_adapter *adap = priv ? priv->adap : NULL;

	if (!adap) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: I2C adapter not initialized\n", __func__,
			ch->ch_id);
		return -EINVAL;
	}

	ret = devm_i2c_add_adapter(&ch->usb->intf->dev, adap);
	if (ret) {
		dev_err(&ch->usb->intf->dev, "%s: ch_%d: Could not register I2C adapter (%pe)\n",
			__func__, ch->ch_id, ERR_PTR(ret));
		return ret;
	}

	ch->child_dev = &adap->dev;

	dev_info(&ch->usb->intf->dev, "%s: ch_%d: Started as %s\n", __func__, ch->ch_id,
		 adap->name);
	return 0;
}

/* clang-format off */
LSTP_SUBSYS(i2c, LSTP_CHANNEL_TYPE_I2C, lstp_i2c_init, lstp_i2c_start,
	    .fwnode_compatible = "nvidia,lstp-i2c",
);
/* clang-format on */
