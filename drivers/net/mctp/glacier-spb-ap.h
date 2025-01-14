/*
 * SPDX-FileCopyrightText: Copyright (c)  NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __GLACIER_SPB_AP_H__
#define __GLACIER_SPB_AP_H__

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/time64.h>
#include <linux/unistd.h>

typedef enum {
	CMD_SREG_W8 = 0x9,
	CMD_SREG_W16,
	CMD_SREG_W32,

	CMD_SREG_R8 = 0xD,
	CMD_SREG_R16,
	CMD_SREG_R32,

	CMD_MEM_W8 = 0x21,
	CMD_MEM_W16,
	CMD_MEM_W32,

	CMD_MEM_R8 = 0x25,
	CMD_MEM_R16,
	CMD_MEM_R32,

	CMD_RD_SNGL_FIFO8 = 0x28,
	CMD_RD_SNGL_FIFO16 = 0x29,
	CMD_RD_SNGL_FIFO32 = 0x2b,

	CMD_POLL_LOW = 0x2C,
	CMD_POLL_HIGH = 0x2D,
	CMD_POLL_ALL = 0x2F,

	CMD_MEM_BLK_W1 = 0x80,

	CMD_MEM_BLK_R1 = 0xA0,
	CMD_RD_BLK_FIFO1 = 0xC0,

	CMD_RD_SNGL_FIFO8_FSR = 0x68,
	CMD_RD_SNGL_FIFO16_FSR = 0x69,
	CMD_RD_SNGL_FIFO32_FSR = 0x6B,
	CMD_BLK_RD_FIFO_FSR = 0xE0,
} spb_spi_cmds_t;

typedef enum {
	SPI_CFG = 0x00,
	SPI_STS = 0x04,
	SPI_EC_STS = 0x08,
	SPI_IEN = 0x0C,
	// ...
	SPI_SPIM2EC_MBX = 0x44,
	SPI_EC2SPIM_MBX = 0x48,
} spb_spi_regs_t;

typedef enum {
	EC_ACK = 0x01000000,
	AP_REQUEST_WRITE = 0x02000000,
	AP_READY_TO_READ = 0x03000000,
	AP_FINISHED_READ = 0x04000000,
	AP_REQUEST_RESET = 0x05000000,
	EC_MSG_AVAILABLE = 0x10000000,
} spb_spi_mailbox_cmds_t;

#define mctp_prerr(fmt, ...)                                                   \
	printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#define MCTP_ASSERT_RET(cond, ret, fmt, ...)                                   \
	do {                                                                   \
		if (!(cond)) {                                                 \
			mctp_prerr("at %s:%d " fmt, __func__, __LINE__,        \
				   ##__VA_ARGS__);                             \
			return (ret);                                          \
		}                                                              \
	} while (0)

typedef enum {
	SPB_AP_OK,
	SPB_AP_MESSAGE_AVAILABLE,
	SPB_AP_ERROR_INVALID_ARGUMENT,
	SPB_AP_ERROR_TIMEOUT,
	SPB_AP_ERROR_UNKNOWN,
} SpbApStatus;

/// Callback required for libspb
typedef struct {
	int msgs_available;
	uint32_t ec2spimb;
	struct spi_device	*spi;
	wait_queue_head_t *gpio_intr_wq;
	bool *gpio_intr_cond;
	spinlock_t *gpio_intr_cond_lock;
	int (*gpio_read_interrupt_pin)(void);
	int (*on_mode_change)(bool quad, uint8_t waitCycles);

	// required for both interrupt and no interrupt
	ssize_t (*spi_xfer)(struct spi_device *spi, const char *txbuf, size_t txlen,
								const char *rxbuf, size_t rxlen, bool cs_change);
} SpbAp;

#define POLL_SREG_TIMEOUT_MSECS	  10000ULL
#define POLL_INT_TIMEOUT_MSECS	  1000ULL
#define MAX_BYTES_PER_TRANSACTION 32
#define WAIT_CYCLES		  0
#define TAR_CYCLES		  1 // 1 in single, 4 in quad
#define TAR_WAIT_CYCLES		  (WAIT_CYCLES + TAR_CYCLES)

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(x) "spb-ap: " #x
#endif

static inline uint64_t clock_msecs(void)
{
	struct timespec64 ts;
	ktime_get_ts64(&ts);
	return (ts.tv_sec * 1000ULL + (ts.tv_nsec >> 20));
}

static const char *mailbox_str(uint32_t mb)
{
	static char str[128];

	memset(str, '\0', sizeof(str));
	switch (mb & 0x0F000000) {
	case EC_ACK:
		strcat(str, "EC_ACK");
		break;
	case AP_REQUEST_WRITE:
		strcat(str, "AP_REQUEST_WRITE");
		break;
	case AP_READY_TO_READ:
		strcat(str, "AP_READY_TO_READ");
		break;
	case AP_FINISHED_READ:
		strcat(str, "AP_FINISHED_READ");
		break;
	case AP_REQUEST_RESET:
		strcat(str, "AP_REQUEST_RESET");
		break;
	default:
		break;
	}

	if (mb & 0xFF) {
		sprintf(str + strlen(str), "LEN:%02x", mb & 0xFF);
	}

	if (mb & EC_MSG_AVAILABLE) {
		sprintf(str + strlen(str), " %x", mb);

		strcat(str, (strlen(str) > 0 ? "|EC_MSG_AVAILABLE" :
					       "EC_MSG_AVAILABLE"));
	}

	if (strlen(str) == 0) {
		sprintf(str, "UNKNOWN MAILBOX: %08x", mb);
	}
	return str;
}

static inline uint32_t buf2dw(uint8_t *x)
{
	return (x[0] << 24) | (x[1] << 16) | (x[2] << 8) | (x[3] << 0);
}

static inline uint16_t buf2w(uint8_t *x)
{
	return (x[0] << 8) | (x[1] << 0);
}

static inline void dw2buf(uint32_t dw, uint8_t *buf)
{
	buf[0] = (uint8_t)((dw >> 24) & 0xff);
	buf[1] = (uint8_t)((dw >> 16) & 0xff);
	buf[2] = (uint8_t)((dw >> 8) & 0xff);
	buf[3] = (uint8_t)((dw >> 0) & 0xff);
}

static inline void w2buf(uint16_t w, uint8_t *buf)
{
	buf[0] = (uint8_t)((w >> 8) & 0xff);
	buf[1] = (uint8_t)((w >> 0) & 0xff);
}

static int wait_for_gpio_interrupt(wait_queue_head_t *gpio_intr_wq, bool *gpio_intr_cond,
											spinlock_t *gpio_intr_cond_lock, long timeout)
{
	if (wait_event_timeout(*gpio_intr_wq, *gpio_intr_cond, msecs_to_jiffies(timeout)) > 0)
	{
		unsigned long flags;
		spin_lock_irqsave(gpio_intr_cond_lock, flags);
		*gpio_intr_cond = false;
		spin_unlock_irqrestore(gpio_intr_cond_lock, flags);
		return 0;
	}
	else
	{
		return -1;
	}
}

static inline void spi_xfer(SpbAp *ap, int sendLen, uint8_t *xbuf, int recvLen,
			    uint8_t *rbuf, bool deassert)
{
	ap->spi_xfer(ap->spi, xbuf, sendLen, rbuf, recvLen, deassert);
}

uint16_t sreg_write_8(SpbAp *ap, uint16_t addr, uint8_t value)
{
	uint8_t buf[1 + 2 + 1 + TAR_WAIT_CYCLES + 2] = { 0 };

	buf[0] = CMD_SREG_W8;
	w2buf(addr, buf + 1);
	buf[3] = value;
	spi_xfer(ap, 1 + 2 + 1, buf, TAR_WAIT_CYCLES + 2, buf, true);
	// return lower 16 bits of status
	return (buf2w(buf + TAR_WAIT_CYCLES));
}

uint16_t sreg_write_32(SpbAp *ap, uint16_t addr, uint32_t value)
{
	uint8_t buf[1 + 2 + 4 + TAR_WAIT_CYCLES + 2] = { 0 };

	buf[0] = CMD_SREG_W32;
	w2buf(addr, buf + 1);
	dw2buf(value, buf + 3);
	spi_xfer(ap, 7, buf, TAR_WAIT_CYCLES + 2, buf, true);
	// return lower 16 bits of status
	return (buf2w(buf + TAR_WAIT_CYCLES));
}

uint16_t sreg_read_8(SpbAp *ap, uint16_t addr, uint8_t *val)
{
	uint8_t buf[1 + 2 + TAR_WAIT_CYCLES + 2 + 1] = { 0 };

	buf[0] = CMD_SREG_R8;
	w2buf(addr, buf + 1);
	spi_xfer(ap, 3, buf, TAR_WAIT_CYCLES + 2 + 1, buf, true);
	*val = buf[TAR_WAIT_CYCLES + 2];
	// return lower 16 bits of status
	return (buf2w(buf + TAR_WAIT_CYCLES));
}

uint16_t sreg_read_32(SpbAp *ap, uint16_t addr, uint32_t *val)
{
	uint8_t buf[1 + 2 + TAR_WAIT_CYCLES + 2 + 4] = { 0 };

	buf[0] = CMD_SREG_R32;
	w2buf(addr, buf + 1);
	spi_xfer(ap, 3, buf, TAR_WAIT_CYCLES + 2 + 4, buf, true);
	*val = buf2dw(buf + TAR_WAIT_CYCLES + 2);
	// return lower 16 bits of status
	return (buf2w(buf + TAR_WAIT_CYCLES));
}

uint32_t cmd_poll_all(SpbAp *ap)
{
	uint8_t buf[1 + TAR_CYCLES + 4] = { 0 };

	buf[0] = CMD_POLL_ALL;
	spi_xfer(ap, 1, buf, TAR_CYCLES + 4, buf, true);

	return (buf2dw(buf + TAR_CYCLES));
}

// Read RX_FIFO_EMPTY
static SpbApStatus wait_for_tx_fifo_not_empty(SpbAp *ap)
{
	uint64_t start = clock_msecs();

	while ((cmd_poll_all(ap) & 0x400) != 0) {
		if (clock_msecs() - start > POLL_SREG_TIMEOUT_MSECS) {
			return (SPB_AP_ERROR_TIMEOUT);
		}
	}
	return (SPB_AP_OK);
}

uint16_t mailbox_write(SpbAp *ap, uint32_t v)
{
	uint16_t status = sreg_write_32(ap, SPI_SPIM2EC_MBX, v);

	return (status);
}

static inline uint32_t clear_memory_write_done(SpbAp *ap)
{
	// MemoryWriteDone bit 0, write to clear
	return (sreg_write_8(ap, SPI_STS, 0x01));
}

static inline uint32_t clear_memory_read_done(SpbAp *ap)
{
	// MemoryReadDone bit 1, write to clear
	return (sreg_write_8(ap, SPI_STS, 0x02));
}

// Polling procedures with timeouts
SpbApStatus wait_for_memory_write_busy_and_rx_fifo_empty(SpbAp *ap)
{
	uint64_t start = clock_msecs();

	// RxFifoEmpty Bit 8, MemoryWriteBusy Bit 3
	while ((cmd_poll_all(ap) & 0x108) == 0) {
		if (clock_msecs() - start > POLL_SREG_TIMEOUT_MSECS)
			return (SPB_AP_ERROR_TIMEOUT);
	}

	return (SPB_AP_OK);
}

SpbApStatus spb_ap_check_ack(SpbAp *ap)
{
	uint32_t sts = 0;
	uint32_t mb = 1;
	SpbApStatus ret = SPB_AP_OK;

	// check mailbox and reset interrupt
	sts = sreg_read_32(ap, SPI_EC2SPIM_MBX, &mb);

	/* 
	 * expect ACK but only MSG_AVAILABLE is set. The workaround was made to
	 * mark ACK is set.
	 */

	if (mb & EC_MSG_AVAILABLE) {
		mb |= EC_ACK;
		ap->msgs_available++;
	}

	ap->ec2spimb = mb & ~EC_MSG_AVAILABLE;
	return (ret);
}

SpbApStatus spb_ap_wait_for_intr(SpbAp *ap, int timeout_ms, bool polling)
{
	int status = 0;

	status = wait_for_gpio_interrupt(ap->gpio_intr_wq, ap->gpio_intr_cond, ap->gpio_intr_cond_lock, timeout_ms);
	if (status < 0)
		return (SPB_AP_ERROR_TIMEOUT);

	return (spb_ap_check_ack(ap));
}

SpbApStatus wait_for_ack(SpbAp *ap)
{
	SpbApStatus status = SPB_AP_OK;

	if (spb_ap_wait_for_intr(ap, 1000, true) == SPB_AP_ERROR_TIMEOUT)
		return (SPB_AP_ERROR_TIMEOUT);

	ap->ec2spimb = 0;
	return (status);
}

SpbApStatus wait_for_length(SpbAp *ap, uint32_t *bytes)
{
	SpbApStatus status = SPB_AP_OK;

	if (spb_ap_wait_for_intr(ap, 1000, true) == SPB_AP_ERROR_TIMEOUT)
		return (SPB_AP_ERROR_TIMEOUT);

	*bytes = ap->ec2spimb & 0xFF;
	ap->ec2spimb = 0;
	return (status);
}

// Write payload with maxiumum of 32 bytes per transfer
SpbApStatus posted_write(SpbAp *ap, uint16_t offset, int len, uint8_t *payload)
{
	uint8_t buf[128] = { 0 };
	int off = 0;
	SpbApStatus status;

	while (len > 0) {
		int bytes = len > MAX_BYTES_PER_TRANSACTION ?
				    MAX_BYTES_PER_TRANSACTION :
				    len;
		if (bytes >= 4) {
			bytes &= 0xFFFFFFFC;
			buf[0] = CMD_MEM_BLK_W1 + bytes / 4 - 1;
			w2buf(offset + off, buf + 1);
			// byteswap for DWORD
			for (int ii = 0; ii < bytes; ii += 4) {
				buf[ii + 3 + 0] = payload[off + ii + 3];
				buf[ii + 3 + 1] = payload[off + ii + 2];
				buf[ii + 3 + 2] = payload[off + ii + 1];
				buf[ii + 3 + 3] = payload[off + ii + 0];
			}

			status = wait_for_memory_write_busy_and_rx_fifo_empty(
				ap);
			MCTP_ASSERT_RET(
				status == SPB_AP_OK, status,
				"wait_for_memory_write_busy_and_rx_fifo_empty failed");

			spi_xfer(ap, 3 + bytes, buf, 0, buf, true);
			clear_memory_write_done(ap);
		} else {
			for (int ii = 0; ii < bytes; ii++) {
				buf[0] = CMD_MEM_W8;
				w2buf(offset + off + ii, buf + 1);
				buf[3] = payload[off + ii];

				status =
					wait_for_memory_write_busy_and_rx_fifo_empty(
						ap);
				MCTP_ASSERT_RET(
					status == SPB_AP_OK, status,
					"wait_for_memory_write_busy_and_rx_fifo_empty failed");

				spi_xfer(ap, 3 + 1, buf, 0, buf, true);
				clear_memory_write_done(ap);
			}
		}
		off += bytes;
		len -= bytes;
	}

	return SPB_AP_OK;
}

static SpbApStatus posted_read_helper(SpbAp *ap, uint8_t cmd, uint8_t cmd2,
				      uint16_t addr, int bytes, uint8_t *buf)
{
	SpbApStatus status;
	// Send the post read command
	buf[0] = cmd;
	w2buf(addr, buf + 1);
	spi_xfer(ap, 3, buf, 0, buf, true);

	// check ok to read
	status = wait_for_tx_fifo_not_empty(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status,
			"wait_for_tx_fifo_not_empty failed: %d", status);

	// Initiate FIFO READ
	buf[0] = cmd2;
	spi_xfer(ap, 1, buf, TAR_CYCLES + 2, buf, false);

	// Check MemoryReadDone
	if ((buf[TAR_CYCLES + 1] & 0x02) == 0) {
		// Poll status
		// TODO: this section is not tested
		uint64_t start = clock_msecs();
		do {
			// read status until MemoryReadDone is set
			buf[0] = 0;
			buf[1] = 0;
			spi_xfer(ap, 0, buf, 2, buf, false);
			if (clock_msecs() - start > POLL_INT_TIMEOUT_MSECS) {
				return SPB_AP_ERROR_TIMEOUT;
			}
		} while ((buf[1] & 0x02) == 0);
	}

	// Read actual data
	spi_xfer(ap, 0, buf, bytes, buf, true);
	clear_memory_read_done(ap);

	return (SPB_AP_OK);
}

SpbApStatus posted_read(SpbAp *ap, int offset, int len, uint8_t *payload)
{
	uint8_t buf[128] = { 0 };
	int off = 0;
	SpbApStatus status = 0;
	int reg_offset = 0;

	while (len > 0) {
		int bytes = len > MAX_BYTES_PER_TRANSACTION ?
				    MAX_BYTES_PER_TRANSACTION :
				    len;
		if (bytes >= 4) {
			bytes &= 0xFFFFFFFC;
			reg_offset = (bytes / 4) - 1;

			status = posted_read_helper(
				ap, CMD_MEM_BLK_R1 + reg_offset,
				CMD_BLK_RD_FIFO_FSR + reg_offset, offset + off,
				bytes, buf);
			MCTP_ASSERT_RET(status == SPB_AP_OK, status,
					"posted_read_helper failed");

			for (int ii = 0; ii < bytes; ii += 4) {
				payload[off + ii + 3] = buf[ii + 0];
				payload[off + ii + 2] = buf[ii + 1];
				payload[off + ii + 1] = buf[ii + 2];
				payload[off + ii + 0] = buf[ii + 3];
			}
		} else {
			for (int ii = 0; ii < bytes; ii++) {
				status = posted_read_helper(
					ap, CMD_MEM_R8, CMD_RD_SNGL_FIFO8_FSR,
					offset + off + ii, 4,
					buf); // TODO: why is this 4 ?
				MCTP_ASSERT_RET(status == SPB_AP_OK, status,
						"posted_read_helper failed");
				// TODO: offset 3 because we have to read 4 bytes
				payload[off + ii] = buf[3];
			}
		}
		off += bytes;
		len -= bytes;
	}

	return (SPB_AP_OK);
}

// --- public

SpbApStatus spb_ap_reset(SpbAp *ap)
{
	// reset globals
	SpbApStatus status;

	ap->msgs_available = 0;
	ap->ec2spimb = 0;

	// initiate reset
	mailbox_write(ap, AP_REQUEST_RESET);
	// wait for ack
	status = wait_for_ack(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_ack failed.");
	return (SPB_AP_OK);
}

SpbApStatus spb_ap_initialize(SpbAp *ap)
{
	MCTP_ASSERT_RET(ap != NULL, SPB_AP_ERROR_INVALID_ARGUMENT,
			"ap is NULL.");
	MCTP_ASSERT_RET(ap->spi_xfer != NULL, SPB_AP_ERROR_INVALID_ARGUMENT,
			"spi_xfer is NULL");

	ap->msgs_available = 0;
	ap->ec2spimb = 0x00000000U;

	return (spb_ap_reset(ap));
}

SpbApStatus spb_ap_set_cfg(SpbAp *ap, bool quad, uint8_t waitCycles)
{
	SpbApStatus status = SPB_AP_OK;
	uint8_t cmd[] = { 0x03, 0x00, waitCycles,
			  (uint8_t)(quad ? 0x01 : 0x00) };

	// send set cfg
	mailbox_write(ap, AP_REQUEST_WRITE);

	status = wait_for_ack(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_ack failed.");

	status = posted_write(ap, 0, sizeof(cmd), cmd);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "posted_write failed.");

	mailbox_write(ap, sizeof(cmd));

	// mode has changed
	if (ap->on_mode_change(quad, waitCycles)) {
		status = wait_for_ack(ap);
		MCTP_ASSERT_RET(status == SPB_AP_OK, status,
				"wait_for_ack failed");
		return (SPB_AP_OK);
	}
	return (SPB_AP_ERROR_UNKNOWN);
}

SpbApStatus spb_ap_shutdown(SpbAp *ap)
{
	// leave Glacier in single spi mode
	return (spb_ap_set_cfg(ap, false, 0));
}

SpbApStatus spb_ap_on_interrupt(SpbAp *ap)
{
	uint32_t sts = 0;
	uint32_t mb = 1;
	SpbApStatus ret = SPB_AP_OK;

	// check mailbox and reset interrupt
	sts = sreg_read_32(ap, SPI_EC2SPIM_MBX, &mb);

	if (mb & EC_MSG_AVAILABLE) {
		ret = SPB_AP_MESSAGE_AVAILABLE;
		ap->msgs_available++;
	}

	ap->ec2spimb = mb & ~EC_MSG_AVAILABLE;
	return (ret);
}

SpbApStatus spb_ap_send(SpbAp *ap, int len, void *buf)
{
	SpbApStatus status;

	mailbox_write(ap, AP_REQUEST_WRITE);
	status = wait_for_ack(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_ack failed.");

	status = posted_write(ap, 0, len, (uint8_t *)(buf));
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "posted_write failed.");
	mailbox_write(ap, len);

	status = wait_for_ack(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_ack failed.");

	return (SPB_AP_OK);
}

SpbApStatus spb_ap_recv(SpbAp *ap, int len, void *buf)
{
	uint32_t bytes;
	SpbApStatus status;

	mailbox_write(ap, AP_READY_TO_READ);

	ap->msgs_available--;
	status = wait_for_length(ap, &bytes);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_length failed.");
	MCTP_ASSERT_RET(bytes <= (uint32_t)len, status,
			"Data cannot fit into the buffer.");

	status = posted_read(ap, 0x8000, bytes, (uint8_t *)(buf));
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "posted_read failed.");
	mailbox_write(ap, AP_FINISHED_READ);

	status = wait_for_ack(ap);
	MCTP_ASSERT_RET(status == SPB_AP_OK, status, "wait_for_ack failed.");

	return (SPB_AP_OK);
}

int spb_ap_msgs_available(SpbAp *ap)
{
	return (ap->msgs_available);
}

const char *spb_ap_strstatus(SpbApStatus status)
{
	switch (status) {
	case SPB_AP_OK:
		return "OK";
	case SPB_AP_MESSAGE_AVAILABLE:
		return "Message available";
	case SPB_AP_ERROR_INVALID_ARGUMENT:
		return "Invalid argument";
	case SPB_AP_ERROR_TIMEOUT:
		return "Timeout";
	case SPB_AP_ERROR_UNKNOWN:
		return "Unknown";
	}

	return ("Invalid status code.");
}

#endif // __GLACIER_SPB_AP_H__