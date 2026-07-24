/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mctp-usb.h - MCTP USB transport binding: common definitions,
 * based on DMTF0283 specification:
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0283_1.0.1.pdf
 *
 * These are protocol-level definitions, that may be shared between host
 * and gadget drivers.
 *
 * Copyright (C) 2024-2025 Code Construct Pty Ltd
 */

#ifndef __LINUX_USB_MCTP_USB_H
#define __LINUX_USB_MCTP_USB_H

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/types.h>

struct mctp_usb_hdr {
	__be16	id;
	u8	rsvd;
	u8	len;
} __packed;

/* max transfer size for DSP0283 v1.0 */
#define MCTP_USB_1_0_XFER_SIZE	512
#define MCTP_USB_BTU		68
#define MCTP_USB_MTU_MIN	MCTP_USB_BTU
#define MCTP_USB_1_0_PKTLEN_MAX	U8_MAX
#define MCTP_USB_1_0_MTU_MAX	(MCTP_USB_1_0_PKTLEN_MAX - sizeof(struct mctp_usb_hdr))
#define MCTP_USB_DMTF_ID	0x1ab4

/* mctp-usblib */

/* Reasons for received data being discarded, reported via ops->rx_error */
enum mctp_usblib_rx_err {
	/* transport header has an unexpected DMTF ID */
	MCTP_USBLIB_RX_ERR_ID,
	/* transport header length is too small for a MCTP packet */
	MCTP_USBLIB_RX_ERR_HDR_LEN,
	/* transfer is too short for the header-specified length */
	MCTP_USBLIB_RX_ERR_XFER_LEN,
	/* no memory to split a packet from a multi-packet transfer */
	MCTP_USBLIB_RX_ERR_NOMEM,
};

/*
 * Optional RX hooks, for transport drivers that need per-packet visibility.
 * Both are called from the (atomic) mctp_usblib_rx_complete() context.
 */
struct mctp_usblib_rx_ops {
	/* Called for each complete packet, with @skb->data at the MCTP
	 * header, before the packet is passed to netif_rx(). A non-zero
	 * return drops the packet; usblib then frees @skb.
	 */
	int (*rx_packet)(struct net_device *netdev, struct sk_buff *skb);
	/* Called when received data is discarded. @data points to the
	 * transport header of the affected packet, with @len bytes available
	 * from there.
	 */
	void (*rx_error)(struct net_device *netdev,
			 enum mctp_usblib_rx_err err,
			 const void *data, size_t len);
};

/*
 * RX handle: drivers will typically create one on init, which persists for
 * the life of the driver. The same handle is used for progressive
 * prepare -> complete operations (for each incoming USB transfer), which
 * result in netif_rx()-ing the MCTP packets received
 */
struct mctp_usblib_rx {
	struct sk_buff *skb;
	/* optional; set by the driver after mctp_usblib_rx_init() */
	const struct mctp_usblib_rx_ops *ops;
};

void mctp_usblib_rx_init(struct mctp_usblib_rx *rx);
void mctp_usblib_rx_fini(struct mctp_usblib_rx *rx);

int mctp_usblib_rx_prepare(struct net_device *netdev,
			   struct mctp_usblib_rx *rx,
			   void **bufp, size_t *lenp, gfp_t gfp);

int mctp_usblib_rx_complete(struct net_device *netdev,
			    struct mctp_usblib_rx *rx, size_t len);

void mctp_usblib_rx_cancel(struct mctp_usblib_rx *rx);

/*
 * TX handle: created by mctp_usblib_tx_push() during the tx path, and
 * may persist across multiple packet transmits.
 *
 * Currently though, there is a 1:1 mapping between packets and transfers, so
 * the tx context will be cleared over each transmit. This will change in
 * future.
 */
struct mctp_usblib_tx_ctx;

struct mctp_usblib_tx_ops {
	/* Start a USB TX for @data. On returning success, the implementation
	 * must arrange for mctp_usblib_tx_send_complete() to be called at some
	 * later point (eg., on urb completion).
	 */
	int (*send)(struct mctp_usblib_tx_ctx *tx_ctx, void *data, size_t len);
	/* Optional: called for each packet of a transfer that has failed,
	 * either from ->send() or at completion, with the failure status in
	 * @err. @skb->data is at the transport header; @skb is freed after
	 * this returns.
	 */
	void (*tx_error)(struct mctp_usblib_tx_ctx *tx_ctx,
			 struct sk_buff *skb, int err);
};

struct mctp_usblib_tx {
	struct mctp_usblib_tx_ops ops;
	void *priv;
};

void mctp_usblib_tx_init(struct mctp_usblib_tx *tx,
			 const struct mctp_usblib_tx_ops *ops, void *priv);
void mctp_usblib_tx_fini(struct mctp_usblib_tx *tx);

void *mctp_usblib_tx_ctx_priv(struct mctp_usblib_tx_ctx *tx_ctx);
unsigned int mctp_usblib_tx_ctx_npkts(struct mctp_usblib_tx_ctx *tx_ctx);

int mctp_usblib_tx_push(struct net_device *dev,
			struct mctp_usblib_tx *tx,
			struct sk_buff *skb, bool more);

void mctp_usblib_tx_send_complete(struct mctp_usblib_tx_ctx *tx_ctx,
				  struct net_device *dev, bool ok);
void mctp_usblib_tx_send_complete_status(struct mctp_usblib_tx_ctx *tx_ctx,
					 struct net_device *dev, int status);

void mctp_usblib_tx_cancel(struct mctp_usblib_tx *tx, struct net_device *dev,
			   enum skb_drop_reason reason);

#endif /*  __LINUX_USB_MCTP_USB_H */
