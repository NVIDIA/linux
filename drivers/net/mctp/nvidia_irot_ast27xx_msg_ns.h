/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * Message types for the NVIDIA IRoT <-> A35 IPC connection.
 *
 * These messages are restricted to 32 bytes but can contain pointers.
 * Messages are always padded to 32 bytes.  For future compatibility all
 * unused fields must be written zero and read ignored.
 */

#ifndef _NVIDIA_IROT_AST27XX_MSG_NS_H
#define _NVIDIA_IROT_AST27XX_MSG_NS_H

#ifdef __KERNEL__
#include <linux/types.h>
typedef u32 nvidia_irot_message_u32;
#else
#include <stdint.h>
typedef uint32_t nvidia_irot_message_u32;
#endif

/**
 * enum nvidia_irot_command_code - IPC command codes
 *
 * Commands are even and responses are odd.
 * See &union nvidia_irot_message_data for the data payload of each command.
 * Zero is intentionally not used.
 */
enum nvidia_irot_command_code {
	/**
	 * @nvidia_irot_cc_ping: A ping request.
	 *
	 * In the future this command may be expanded for feature detection
	 * as it contains a large number of reserved bytes.
	 *
	 * If multiple pings are in flight then the responder should respond
	 * to them in order, but if any responses are dropped then prefer to
	 * drop older ones.
	 */
	nvidia_irot_cc_ping = 2,
	/** @nvidia_irot_cc_ping_rsp: Response to a ping. */
	nvidia_irot_cc_ping_rsp,

	/** @nvidia_irot_cc_get_mctp_buffers: Request for buffer addresses. */
	nvidia_irot_cc_get_mctp_buffers,
	/**
	 * @nvidia_irot_cc_mctp_buffer: The requested buffer addresses.
	 *
	 * This response should never change unless the system reboots.
	 */
	nvidia_irot_cc_mctp_buffer,

	/** @nvidia_irot_cc_mctp: A new MCTP message. */
	nvidia_irot_cc_mctp,
	/**
	 * @nvidia_irot_cc_mctp_done: MCTP message processed, buffer freed.
	 */
	nvidia_irot_cc_mctp_done,
};

/**
 * struct nvidia_irot_message_address - Physical address in A35 address space
 * @low: low 32 bits of the address
 * @high: high 32 bits of the address
 *
 * Encoded in two u32 to avoid alignment and padding issues.
 */
struct nvidia_irot_message_address {
	nvidia_irot_message_u32 low;
	nvidia_irot_message_u32 high;
};

/**
 * struct nvidia_irot_message_span - Memory region in A35 address space
 * @address: start address
 * @size: size in bytes
 */
struct nvidia_irot_message_span {
	struct nvidia_irot_message_address address;
	nvidia_irot_message_u32 size;
};

/** struct nvidia_irot_message_data_empty - Empty message payload */
struct nvidia_irot_message_data_empty {};

/**
 * struct nvidia_irot_message_data_buffers - Shared buffer information
 * @mtu_limit: maximum transmission unit limit.
 *             read_mtu = min(mtu_limit, a35_read.size),
 *             write_mtu = min(mtu_limit, a35_write.size)
 * @a35_read: span for the A35 to read from
 * @a35_write: span for the A35 to write into
 */
struct nvidia_irot_message_data_buffers {
	nvidia_irot_message_u32 mtu_limit;
	struct nvidia_irot_message_span a35_read;
	struct nvidia_irot_message_span a35_write;
};

/**
 * struct nvidia_irot_message_data_mctp - MCTP packet message data
 * @counter: packet counter; must be echoed back in the response.
 *           If two messages are received back-to-back they must be
 *           deduplicated but the response must still be sent.
 * @packet: span of memory containing the MCTP packet.
 *          Must be fully contained within the expected region from the
 *          nvidia_irot_cc_mctp_buffer message.  If out of bounds the
 *          memory must not be accessed.
 */
struct nvidia_irot_message_data_mctp {
	nvidia_irot_message_u32 counter;
	struct nvidia_irot_message_span packet;
};

/**
 * union nvidia_irot_message_data - Data payload for IPC messages
 * @args: raw u32 arguments (for initializing and debugging)
 * @empty: empty payload (nvidia_irot_cc_get_mctp_buffers)
 * @value: single value.
 *         nvidia_irot_cc_ping / nvidia_irot_cc_ping_rsp: echoed value.
 *         nvidia_irot_cc_mctp_done: packet counter.
 * @buffers: shared buffer info (nvidia_irot_cc_mctp_buffer)
 * @mctp: MCTP packet (nvidia_irot_cc_mctp)
 */
union nvidia_irot_message_data {
	nvidia_irot_message_u32 args[7];
	struct nvidia_irot_message_data_empty empty;
	nvidia_irot_message_u32 value;
	struct nvidia_irot_message_data_buffers buffers;
	struct nvidia_irot_message_data_mctp mctp;
};

/**
 * struct nvidia_irot_message - 32-byte IPC message
 * @command: the &enum nvidia_irot_command_code.
 *           Zero is used for "empty" locally but should never be sent
 *           over IPC.
 * @data: the data payload
 */
struct nvidia_irot_message {
	nvidia_irot_message_u32 command;
	union nvidia_irot_message_data data;
};

/** NVIDIA_IROT_MESSAGE_INIT - Zero-initializer for &struct nvidia_irot_message */
#define NVIDIA_IROT_MESSAGE_INIT                       \
	{                                              \
		.command = 0, .data = {.args = { 0 } } \
	}

#if defined(__cplusplus)
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) static_assert(expr);
#elif __STDC_VERSION__ >= 202311L
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) static_assert(expr);
#elif __STDC_VERSION__ >= 201112L
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) _Static_assert(expr, #expr);
#else
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr)
#endif

/*
 * Ensure the message struct is exactly 32 bytes.  Some APIs unconditionally
 * copy the max 32-byte size so both undersized and oversized types would be
 * a problem.
 */
NVIDIA_IROT_MESSAGE_STATIC_ASSERT(sizeof(struct nvidia_irot_message) == 32)

#undef NVIDIA_IROT_MESSAGE_STATIC_ASSERT

#endif /* _NVIDIA_IROT_AST27XX_MSG_NS_H */
