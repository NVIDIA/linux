/// @file
/// @brief Message types for the nvidia IRoT <-> A35 IPC connection.
/// @details These messages are restricted to 32 bytes but can contain pointers.
/// @note The messages are always padded to 32 bytes. For future compatibility
/// all unused fields must be written zero and read ignored.

#pragma once

#ifdef __KERNEL__
#include <linux/types.h>
typedef u32 nvidia_irot_message_u32;
#else
#include <stdint.h>
typedef uint32_t nvidia_irot_message_u32;
#endif

/// @brief Command codes.
/// @details Commands are even and responses are odd.
///  See nvidia_irot_message_data for info on the data payload for each command.
/// @note Zero is intentially not used.
enum nvidia_irot_command_code {
	/// @brief A ping.
	/// @note In the future this command may be expanded to be used in feature
	/// detection as it contains a large number of reserved bytes.
	/// @note If multiple pings are in flight then the responder should respond
	/// to them in order, but if any responses are dropped then prefer to drop
	/// older ones.
	nvidia_irot_cc_ping = 2,
	/// @brief Response to the ping.
	nvidia_irot_cc_ping_rsp,

	/// @brief Request for the IRoT to send buffer addresses.
	nvidia_irot_cc_get_mctp_buffers,
	/// @brief The requested buffer addresses.
	/// @note This response should never change unless the system reboots.
	nvidia_irot_cc_mctp_buffer,

	/// @brief A new MCTP message.
	nvidia_irot_cc_mctp,
	/// @brief Notification that the MCTP message has been processed and its
	/// buffer is free to be used.
	nvidia_irot_cc_mctp_done,
};

/// @brief Memory address in the A35's physical address space.
/// @note Is encoded in two u32 to avoid alignment and padding issues.
struct nvidia_irot_message_address {
	/// @brief The low 32 bits of the address.
	nvidia_irot_message_u32 low;
	/// @brief The high 32 bits of the address.
	nvidia_irot_message_u32 high;
};

/// @brief Memory region in the A35's physical address space.
struct nvidia_irot_message_span {
	/// @brief Start address.
	struct nvidia_irot_message_address address;
	/// @brief Size in bytes.
	nvidia_irot_message_u32 size;
};

/// @brief Empty message.
struct nvidia_irot_message_data_empty {};

/// @brief Information about the shared buffers between the IRoT and A35.
struct nvidia_irot_message_data_buffers {
	/// @brief Used to compute the maximum transmission unit.
	/// @details The read_mtu is min(mtu_limit, a35_read.size) and
	/// the write_mtu is min(mtu_limit, a35_write.size).
	nvidia_irot_message_u32 mtu_limit;
	/// @brief Span for the A35 to read from.
	struct nvidia_irot_message_span a35_read;
	/// @brief Span for the A35 to write into.
	struct nvidia_irot_message_span a35_write;
};

/// @brief MCTP packet message data.
struct nvidia_irot_message_data_mctp {
	/// @brief MCTP packet counter.
	/// @details Must be echoed back in the response to the message.
	/// Additionally if two messages are received back to back they
	/// must be deduplicated but the response must still be sent.
	nvidia_irot_message_u32 counter;
	/// @brief Span of memory that contains the MCTP packet.
	/// @details The span must be fully contained within the expected
	/// region from the nvidia_irot_cc_mctp_buffer message.
	/// If the packet is out of bounds the memory must not be accessed.
	struct nvidia_irot_message_span packet;
};

/// @brief Data payload for IPC messages.
union nvidia_irot_message_data {
	/// @brief Raw arguments.
	/// @details Used only for initializing and debugging.
	nvidia_irot_message_u32 args[7];

	/// @brief Empty payload.
	/// @details Used in the message types:
	///  - nvidia_irot_cc_get_mctp_buffers
	struct nvidia_irot_message_data_empty empty;

	/// @brief A single value.
	/// @details Used in the message types:
	///  - nvidia_irot_cc_ping: for a value that is echoed back.
	///  - nvidia_irot_cc_ping_rsp: for the value that is echoed back.
	///  - nvidia_irot_cc_mctp_done: for the packet counter.
	nvidia_irot_message_u32 value;

	/// @brief Shared buffer information.
	/// @details Used in the message types:
	///  - nvidia_irot_cc_mctp_buffer
	struct nvidia_irot_message_data_buffers buffers;

	/// @brief Mctp packet.
	/// @details Used in the message types:
	///  - nvidia_irot_cc_mctp
	struct nvidia_irot_message_data_mctp mctp;
};

/// @brief 32-byte IPC message type.
struct nvidia_irot_message {
	/// @brief The nvidia_irot_command_code.
	/// @note Zero is used for empty locally, but should never be sent over IPC.
	nvidia_irot_message_u32 command;
	/// @brief The data payload.
	union nvidia_irot_message_data data;
};

/// @brief Initializer for struct nvidia_irot_message.
/// @details Zeros out all fields.
#define NVIDIA_IROT_MESSAGE_INIT                       \
	{                                              \
		.command = 0, .data = {.args = { 0 } } \
	}

#if defined(__cplusplus)
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) static_assert(expr);
#elif __STDC_VERSION__ >= 202311L
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) static_assert(expr);
#elif __STDC_VERSION__ >= 201112L
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr) _Static_assert(expr);
#else
#define NVIDIA_IROT_MESSAGE_STATIC_ASSERT(expr)
#endif

// Ensure that our message struct is exactly 32 bytes.
// Some APIs unconditionally copy the max 32 bytes size so both
// undersized type and oversized types would be a problem.
NVIDIA_IROT_MESSAGE_STATIC_ASSERT(sizeof(struct nvidia_irot_message) == 32)

#undef NVIDIA_IROT_MESSAGE_STATIC_ASSERT
