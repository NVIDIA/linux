/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Management Component Transport Protocol (MCTP)
 *
 * Copyright (c) 2021 Code Construct
 * Copyright (c) 2021 Google
 */

#ifndef __UAPI_MCTP_H
#define __UAPI_MCTP_H

#include <linux/types.h>
#include <linux/socket.h>
#include <linux/netdevice.h>

typedef __u8			mctp_eid_t;

struct mctp_addr {
	mctp_eid_t		s_addr;
};

struct sockaddr_mctp {
	__kernel_sa_family_t	smctp_family;
	__u16			__smctp_pad0;
	unsigned int		smctp_network;
	struct mctp_addr	smctp_addr;
	__u8			smctp_type;
	__u8			smctp_tag;
	__u8			__smctp_pad1;
};

struct sockaddr_mctp_ext {
	struct sockaddr_mctp	smctp_base;
	int			smctp_ifindex;
	__u8			smctp_halen;
	__u8			__smctp_pad0[3];
	__u8			smctp_haddr[MAX_ADDR_LEN];
};

/* A "fully qualified" MCTP address, which includes the system-local network ID,
 * required to uniquely resolve a routable EID.
 */
struct mctp_fq_addr {
	unsigned int	net;
	mctp_eid_t	eid;
};

#define MCTP_NET_ANY		0x0

#define MCTP_ADDR_NULL		0x00
#define MCTP_ADDR_ANY		0xff

#define MCTP_TAG_MASK		0x07
#define MCTP_TAG_OWNER		0x08
#define MCTP_TAG_PREALLOC	0x10

#define MCTP_OPT_ADDR_EXT	1
#define MCTP_OPT_ENABLE_ERRQUEUE	2

#define SIOCMCTPALLOCTAG	(SIOCPROTOPRIVATE + 0)
#define SIOCMCTPDROPTAG		(SIOCPROTOPRIVATE + 1)
#define SIOCMCTPALLOCTAG2	(SIOCPROTOPRIVATE + 2)
#define SIOCMCTPDROPTAG2	(SIOCPROTOPRIVATE + 3)

/* Deprecated: use mctp_ioc_tag_ctl2 / TAG2 ioctls instead, which defines the
 * MCTP network ID as part of the allocated tag. Using this assumes the default
 * net ID for allocated tags, which may not give correct behaviour on system
 * with multiple networks configured.
 */
struct mctp_ioc_tag_ctl {
	mctp_eid_t	peer_addr;

	/* For SIOCMCTPALLOCTAG: must be passed as zero, kernel will
	 * populate with the allocated tag value. Returned tag value will
	 * always have TO and PREALLOC set.
	 *
	 * For SIOCMCTPDROPTAG: userspace provides tag value to drop, from
	 * a prior SIOCMCTPALLOCTAG call (and so must have TO and PREALLOC set).
	 */
	__u8		tag;
	__u16		flags;
};

struct mctp_ioc_tag_ctl2 {
	/* Peer details: network ID, peer EID, local EID. All set by the
	 * caller.
	 *
	 * Local EID must be MCTP_ADDR_NULL or MCTP_ADDR_ANY in current
	 * kernels.
	 */
	unsigned int	net;
	mctp_eid_t	peer_addr;
	mctp_eid_t	local_addr;

	/* Set by caller, but no flags defined currently. Must be 0 */
	__u16		flags;

	/* For SIOCMCTPALLOCTAG2: must be passed as zero, kernel will
	 * populate with the allocated tag value. Returned tag value will
	 * always have TO and PREALLOC set.
	 *
	 * For SIOCMCTPDROPTAG2: userspace provides tag value to drop, from
	 * a prior SIOCMCTPALLOCTAG2 call (and so must have TO and PREALLOC set).
	 */
	__u8		tag;

};

/*
 * MCTP Error Queue Support
 * For receiving asynchronous errors via recvmsg(MSG_ERRQUEUE)
 */

#define MCTP_ERROR_PAYLOAD_SIZE  32  /* Capture first 32 bytes of payload */

/* Control message type for reading errors */
#define MCTP_RECVERR  1

/* Direction values */
#define MCTP_DIR_TX  0
#define MCTP_DIR_RX  1

/**
 * struct mctp_error - MCTP error information for applications
 *
 * This structure is returned to applications via recvmsg(MSG_ERRQUEUE).
 * Contains everything needed to identify and handle binding layer errors.
 *
 * @error_code: Error number (ETIMEDOUT, EPIPE, EPROTO, etc.)
 * @direction: 0=TX, 1=RX
 * @binding: DMTF binding type (1=SMBus, 2=PCIe VDM, 3=USB, 4=KCS, 5=Serial, 6=I3C)
 * @src_eid: Source EID
 * @dest_eid: Destination EID
 * @tag: MCTP tag value (0-7)
 * @msg_type: MCTP message type (0x01=PLDM, 0x05=SPDM, etc.)
 * @timestamp_ns: When error occurred (nanoseconds since boot)
 * @payload_len: Length of captured payload
 * @payload: First N bytes of message payload (includes protocol headers)
 *
 * For PLDM messages, payload contains:
 *   payload[0] = Instance ID byte (bits 4-0 = instance ID)
 *   payload[1] = PLDM Type byte (bits 5-0 = type: 2=T2, 5=T5)
 *   payload[2] = Command code
 *
 * For SPDM messages, payload contains:
 *   payload[0] = SPDM version
 *   payload[1] = Request/Response code
 *   payload[2+] = Parameters and session context
 */
struct mctp_error {
	/* Error Information */
	__u32	error_code;		/* errno value */
	__u8	direction;		/* MCTP_DIR_TX or MCTP_DIR_RX */
	__u8	binding;		/* enum mctp_phys_binding */
	__u16	reserved1;

	/* MCTP Addressing */
	__u8	src_eid;		/* Source EID */
	__u8	dest_eid;		/* Destination EID */
	__u8	tag;			/* MCTP tag (0-7) */
	__u8	msg_type;		/* MCTP message type */

	/* Timestamp */
	__u64	timestamp_ns;		/* Error timestamp */

	/* Payload Capture */
	__u16	payload_len;		/* Captured payload length */
	__u16	reserved2;
	__u8	payload[MCTP_ERROR_PAYLOAD_SIZE];

	/* Reserved for future use */
	__u32	reserved3[2];
} __attribute__((packed));

#endif /* __UAPI_MCTP_H */
