/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The driver for BMC side of SSIF interface
 *
 * Copyright (c) 2022, Ampere Computing LLC
 *
 */
#ifndef __SSIF_BMC_H__
#define __SSIF_BMC_H__

#define DEVICE_NAME				"ipmi-ssif-host"

#define GET_8BIT_ADDR(addr_7bit)		(((addr_7bit) << 1) & 0xff)

/* A standard SMBus Transaction is limited to 32 data bytes */
#define MAX_PAYLOAD_PER_TRANSACTION		32
/* Transaction includes the address, the command, the length and the PEC byte */
#define MAX_TRANSACTION				(MAX_PAYLOAD_PER_TRANSACTION + 4)

#define MAX_IPMI_DATA_PER_START_TRANSACTION	30
#define MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION	31

#define SSIF_IPMI_SINGLEPART_WRITE		0x2
#define SSIF_IPMI_SINGLEPART_READ		0x3
#define SSIF_IPMI_MULTIPART_WRITE_START		0x6
#define SSIF_IPMI_MULTIPART_WRITE_MIDDLE	0x7
#define SSIF_IPMI_MULTIPART_WRITE_END		0x8
#define SSIF_IPMI_MULTIPART_READ_START		0x3
#define SSIF_IPMI_MULTIPART_READ_MIDDLE		0x9

/* Include netfn and cmd field */
#define MSG_PAYLOAD_LEN_MAX			254
/*
 * IPMI 2.0 Spec, section 12.7 SSIF Timing,
 * Request-to-Response Time is T6max(250ms) - T1max(20ms) - 3ms = 227ms
 * Recover ssif_bmc from busy state if it takes up to 500ms
 */
#define RESPONSE_TIMEOUT			500 /* ms */

struct ssif_msg {
	u8 len;
	u8 payload[MSG_PAYLOAD_LEN_MAX];
} __packed;

struct ssif_part_buffer {
	u8 address;
	u8 smbus_cmd;
	u8 length;
	u8 payload[MAX_PAYLOAD_PER_TRANSACTION];
	u8 pec;
	u8 index;
} __packed;

/*
 * SSIF internal states:
 *   SSIF_READY         0x00 : Ready state
 *   SSIF_START         0x01 : Start smbus transaction
 *   SSIF_SMBUS_CMD     0x02 : Received SMBus command
 *   SSIF_REQ_RECVING   0x03 : Receiving request
 *   SSIF_RES_SENDING   0x04 : Sending response
 *   SSIF_BAD_SMBUS     0x05 : Bad SMbus transaction
 */
enum ssif_state {
	SSIF_READY,
	SSIF_START,
	SSIF_SMBUS_CMD,
	SSIF_REQ_RECVING,
	SSIF_RES_SENDING,
	SSIF_ABORTING,
	SSIF_STATE_MAX
};

struct ssif_bmc_ctx {
	struct i2c_client	*client;
	struct miscdevice	miscdev;
	int			msg_idx;
	bool			pec_support;
	/* ssif bmc spinlock */
	spinlock_t		lock;
	wait_queue_head_t	wait_queue;
	u8			running;
	enum ssif_state		state;
	/* Timeout waiting for response */
	struct timer_list	response_timer;
	bool                    response_timer_inited;
	/* Flag to identify a Multi-part Read Transaction */
	bool			is_singlepart_read;
	u8			nbytes_processed;
	u8			remain_len;
	u8			recv_len;
	/* Block Number of a Multi-part Read Transaction */
	u8			block_num;
	bool			request_available;
	bool			response_in_progress;
	bool			busy;
	bool			aborting;
	/* Buffer for SSIF Transaction part*/
	struct ssif_part_buffer	part_buf;
	struct ssif_msg		response;
	struct ssif_msg		request;
};

static inline struct ssif_bmc_ctx *to_ssif_bmc(struct file *file)
{
	return container_of(file->private_data, struct ssif_bmc_ctx, miscdev);
}
#endif /* __SSIF_BMC_H__ */
