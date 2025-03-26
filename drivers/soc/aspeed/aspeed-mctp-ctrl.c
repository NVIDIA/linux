// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Aspeed MCTP controller
 *
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0238_1.2.1.pdf
 *
 * Copyright (c) 2024, Ampere Computing LLC
 */

#include <linux/bitfield.h>
#include <linux/if_arp.h>
#include <linux/mfd/syscon.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/timer.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>

/* clang-format off */

/* Aspeed MCTP Controller registers */
#define MCTP_CTRL	0x000
#define  TX_TRIGGER	BIT(0)
#define  RX_READY	BIT(4)
#define  MATCHING_EID	BIT(9)

#define MCTP_INT_STS	0x00c
#define MCTP_INT_EN	0x010
#define  TX_SENT_INT	BIT(0)
#define  TX_LAST_INT	BIT(1)
#define  TX_WRONG_INT	BIT(2)
#define  RX_RECEIVE_INT	BIT(8)
#define  RX_NO_MORE_INT	BIT(9)
#define  ACTIVE_INTS	(TX_SENT_INT | RX_RECEIVE_INT | RX_NO_MORE_INT)

#define MCTP_ENGINE_CTRL		0x01c
#define  TX_MAX_PAYLOAD_SIZE_MASK	GENMASK(1, 0)
#define  RX_MAX_PAYLOAD_SIZE_MASK	GENMASK(5, 4)
#define  MAX_PAYLOAD_64			0
#define  MAX_PAYLOAD_128		1
#define  MAX_PAYLOAD_256		2
#define  MAX_PAYLOAD_512		3
#define  MAX_PAYLOAD_SIZE(N)		(1 << ((N) + 6))
#define  FIFO_LAYOUT_MASK		GENMASK(9, 8)
#define  FIFO_TX_6K_RX_2K		0
#define  FIFO_TX_4K_RX_4K		1
#define  FIFO_TX_2K_RX_6K		2

#define MCTP_RX_BUF_SIZE	0x024
#define MCTP_RX_BUF_RD_PTR	0x028
#define  UPDATE_RX_RD_PTR	BIT(31)
#define  RX_BUF_RD_PTR_MASK	GENMASK(11, 0)
#define RX_BUF_WR_PTR		0x02c
#define  RX_BUF_WR_PTR_MASK	GENMASK(11, 0)

#define MCTP_TX_BUF_SIZE	0x034
#define MCTP_TX_BUF_RD_PTR	0x038
#define  UPDATE_TX_RD_PTR	BIT(31)
#define  TX_BUF_RD_PTR_MASK	GENMASK(11, 0)
#define TX_BUF_WR_PTR		0x03c
#define  TX_BUF_WR_PTR_MASK	GENMASK(11, 0)

/* TX command */
#define MCTP_TX_BUF_ADDR	0x004
#define MCTP_TX_BUF_HI_ADDR	0x030
#define  TX_LAST_CMD		BIT_ULL(63)
#define  TX_DATA_ADDR_MASK	GENMASK_ULL(62, 32)
#define  TX_RESERVED_1_MASK	GENMASK_ULL(33, 32) /* must be 1 */
#define  TX_RESERVED_1		1
#define  TX_STOP_AFTER_CMD	BIT_ULL(16)
#define  TX_INTERRUPT_AFTER_CMD	BIT_ULL(15)
#define  TX_PACKET_SIZE_MASK	GENMASK_ULL(12, 2)
#define  TX_RESERVED_0_MASK	GENMASK_ULL(1, 0) /* MBZ */
#define  TX_RESERVED_0		0

/* RX command */
#define MCTP_RX_BUF_ADDR	0x008
#define MCTP_RX_BUF_HI_ADDR	0x020
#define  RX_INTERRUPT_AFTER_CMD	BIT(2)
#define  RX_DATA_ADDR_MASK	GENMASK(30, 0)

/* PCIe Host Controller registers */
#define PCIE_MISC_STS_1		0x0c4
#define  PCIE_PERST		BIT(19)
#define  PCIE_BUS_DEVICE	GENMASK(12, 0)

/* MCTP over PCIe-VDM */
#define MCTP_PCIEVDM_MINMTU	(sizeof(struct mctp_hdr) + 64)
/* Default to 128 (the minimum size of a TLP payload) */
#define MCTP_PCIEVDM_DEFAULTMTU	(sizeof(struct mctp_hdr) + 128)
#define MCTP_PCIEVDM_HLEN	16
#define MCTP_PCIEVDM_HLEN_DW	4
#define PCIEVDM_ALEN		2
#define PCIEVDM_HLEN		12
#define PCIEVDM_HLEN_DW		3

#define DW0_DEFAULT		0x70001000
#define  LEN_DW_MASK		GENMASK(9, 0)
#define  ROUTING_TYPE_MASK	GENMASK(26, 24)
#define DW1_DEFAULT		0x0000007F
#define  REQUESTER_MASK		GENMASK(31, 16)
#define  PAD_LEN_MASK		GENMASK(13, 12)
#define DW2_DEFAULT		0x00001AB4
#define  TARGET_MASK		GENMASK(31, 16)

/* Default config */
#define DEFAULT_RX_HW_QUEUE_DEPTH		96
#define DEFAULT_TX_HW_QUEUE_DEPTH		48
#define DEFAULT_RX_POLLING_INTERVAL_MSEC	100

/* clang-format on */

enum rx_mode {
	/* RX fast path: when we can trust hardware pointers.
	 *
	 * This path only works if hardware pointers are consistent
	 * with sofware pointers and we can rely on the pointers to
	 * detect RX packets presense.
	 */
	RX_MODE_FAST,

	/* RX warmup path: peeks at packet headers to detect packet arrival. 
	 *
	 * We have to do this in some cases where hardware pointers are unreliable
	 * after reset. This is suboptimal so we should switch to fast path
	 * when hardware pointers stablize after a few loops.
	 */
	RX_MODE_WARMUP,
};

struct tx_ring {
	u64 *cmd;
	dma_addr_t cmd_paddr;
	u8 *pkt;
	dma_addr_t pkt_paddr;
	unsigned int pkt_size;
	unsigned int pkt_count;

	u8 next_xmit;
	u8 next_reclaim;
};

struct rx_ring {
	u32 *cmd;
	dma_addr_t cmd_paddr;
	u8 *pkt;
	dma_addr_t pkt_paddr;
	unsigned int pkt_size;
	unsigned int pkt_count;

	u8 next_read;
	u8 next_refill;
	u8 mode;

	u64 scan_counter;
	u8 scan_hw_offset;
};

struct aspeed_mctp_ctrl_match_data {
	u8 starting_mode;
	u32 tx_max_payload_size_regval;
	u32 rx_max_payload_size_regval;
	u32 max_payload_size;
};

struct aspeed_mctp_ctrl {
	struct net_device *ndev;
	struct regmap *map;
	struct regmap *map_pcie;
	struct reset_control *reset;
	struct reset_control *reset_dma;
	const struct aspeed_mctp_ctrl_match_data *match_data;

	struct tx_ring tx;
	struct rx_ring rx;

	bool rc_f;
	int irq_mctp;
	int irq_perst_lo;
	int irq_perst_hi;
	u32 rx_poll_interval_jiffies;

	struct napi_struct napi;
	struct delayed_work bdf_work;
	struct work_struct rst_work;
	struct timer_list rx_poll_timer;
};

static inline u8 rx_ring_hw_ptr(struct rx_ring *rx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	u32 reg = 0;

	regmap_write_bits(priv->map, MCTP_RX_BUF_RD_PTR, UPDATE_RX_RD_PTR,
			  UPDATE_RX_RD_PTR);
	regmap_read(priv->map, MCTP_RX_BUF_RD_PTR, &reg);

	return FIELD_GET(RX_BUF_RD_PTR_MASK, reg);
}

static void rx_ring_prepare_skb(struct rx_ring *rx, struct sk_buff *skb)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	struct mctp_skb_cb *cb;
	u16 requester;
	u32 *hdr;
	u16 len;
	u8 pad;
	int i;

	/* The header from hardware uses host-endian */
	hdr = (void *)skb->data;
	len = FIELD_GET(LEN_DW_MASK, hdr[0]) * sizeof(u32);
	pad = FIELD_GET(PAD_LEN_MASK, hdr[1]);
	requester = FIELD_GET(REQUESTER_MASK, hdr[1]);

	skb_put(skb, MCTP_PCIEVDM_HLEN + len - pad);

	skb->protocol = htons(ETH_P_MCTP);
	skb_reset_mac_header(skb);

	/* The header from hardware uses host-endian,
	 * while upper networking expect network endian
	 */
	for (i = 0; i < MCTP_PCIEVDM_HLEN_DW; i++)
		hdr[i] = htonl(hdr[i]);
	skb_pull(skb, PCIEVDM_HLEN);
	skb_reset_network_header(skb);

	cb = __mctp_cb(skb);
	cb->halen = priv->ndev->addr_len;
	cb->ifindex = priv->ndev->ifindex;
	put_unaligned_be16(requester, cb->haddr);
}

/* Take the packet from the specified slot.
 *
 * Return NULL if no memory for packet allocation.
 *
 * Caller must ensure the specified slot is already written in by checking
 * the hardware pointers or peeking into the packet.
 *
 * This should be called inside NAPI context.
 */
static struct sk_buff *__rx_ring_read(struct rx_ring *rx, u8 index)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	struct sk_buff *skb = napi_alloc_skb(&priv->napi, rx->pkt_size);

	if (!skb)
		return NULL;

	memcpy(skb->data, &rx->pkt[index * rx->pkt_size], rx->pkt_size);
	rx_ring_prepare_skb(rx, skb);

	put_unaligned(0, (u32 *)&rx->pkt[index * rx->pkt_size]);

	return skb;
}

static struct sk_buff *rx_ring_read(struct rx_ring *rx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	struct sk_buff *skb;

	skb = __rx_ring_read(rx, rx->next_read);
	if (!skb)
		netdev_warn(priv->ndev, "Out of memory for RX packet");

	/* Drop the packet if we have no space left */
	smp_store_release(&rx->next_read, (rx->next_read + 1) % rx->pkt_count);

	return skb;
}

/* Return true if we ran out of TX slots. */
static inline bool tx_ring_full(struct tx_ring *tx)
{
	return (tx->next_xmit + 1) % tx->pkt_count ==
	       smp_load_acquire(&tx->next_reclaim);
}

/* Return true if we have acknowledged all the transmitted slots. */
static bool tx_ring_empty(struct tx_ring *tx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(tx, struct aspeed_mctp_ctrl, tx);
	u32 reg = 0;

	regmap_write_bits(priv->map, MCTP_TX_BUF_RD_PTR, UPDATE_TX_RD_PTR,
			  UPDATE_TX_RD_PTR);
	regmap_read(priv->map, MCTP_TX_BUF_RD_PTR, &reg);

	return tx->next_reclaim == FIELD_GET(TX_BUF_RD_PTR_MASK, reg);
}

/* Prepare the specified slot for hardware to perform TX.
 *
 * Caller must should index is safe to be written in by checking
 * the driver reclaim pointer.
 */
static void __tx_ring_xmit(struct tx_ring *tx, u8 index, struct sk_buff *skb)
{
	u8 len_dw;
	u32 *hdr;

	/* HW expects host-endian headers and big-endian body.
	 * The first 3-dword header is already taken care by the hard header,
	 * only need to address the fourth dword.
	 */
	hdr = (void *)skb->data;
	hdr[3] = ntohl(hdr[3]);

	/* Typically, network drivers should hold on to the sk_buff, do
	 * streaming DMA mapping on the sk_buff, and then free the sk_buff
	 * on reclaim.
	 *
	 * However, due to Aspeed hardware design in current AST26xx chips,
	 * the MCTP controller relies on the PCIe interface, which does not
	 * undergo a reset during system reboot.
	 *
	 * Use a coherent buffer at a reserved memory address range to
	 * avoid potential memory pollution.
	 */
	memcpy(tx->pkt + index * tx->pkt_size, skb->data, skb->len);

	/* len must be 4 align, in 4-byte unit,
	 * not counting headers len
	 */
	len_dw = ALIGN(skb->len, 4) / 4 - 4;
	tx->cmd[index] = FIELD_PREP(TX_DATA_ADDR_MASK,
				    tx->pkt_paddr + index * tx->pkt_size) |
			 FIELD_PREP(TX_RESERVED_1_MASK, TX_RESERVED_1) |
			 FIELD_PREP(TX_PACKET_SIZE_MASK, len_dw) |
			 TX_INTERRUPT_AFTER_CMD;
}

static void tx_ring_xmit(struct tx_ring *tx, struct sk_buff *skb)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(tx, struct aspeed_mctp_ctrl, tx);

	__tx_ring_xmit(tx, tx->next_xmit, skb);
	smp_store_release(&tx->next_xmit, (tx->next_xmit + 1) % tx->pkt_count);
	regmap_write(priv->map, TX_BUF_WR_PTR, tx->next_xmit);
}

/* Acknowledge the next transmitted slot.
 *
 * Caller must ensure the next slot is already transmitted in by checking
 * the hardware pointers when TX done interrupt happens.
 */
static void tx_ring_reclaim(struct tx_ring *tx)
{
	/* We do not actually have to do anything in particular to ACK.
	 * Zero the command to make sure TX command wrong interrupt is fired
	 * when hardware tries to TX this again.
	 */
	tx->cmd[tx->next_reclaim] = 0;
	smp_store_release(&tx->next_reclaim,
			  (tx->next_reclaim + 1) % tx->pkt_count);
}

static void tx_ring_reclaim_many(struct tx_ring *tx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(tx, struct aspeed_mctp_ctrl, tx);

	while (!tx_ring_empty(tx))
		tx_ring_reclaim(tx);

	/* Queue should never be stopped when TX ring is empty, so we can
	 * always wake up the queue even if we reclaimed zero slots.
	 */
	if (unlikely(netif_queue_stopped(priv->ndev)))
		netif_wake_queue(priv->ndev);
}

static bool __rx_ring_peek_warmup(struct rx_ring *rx, u8 index)
{
	return __get_unaligned_t(u32, &rx->pkt[index * rx->pkt_size]) != 0;
}

static inline bool rx_ring_peek_warmup(struct rx_ring *rx)
{
	return __rx_ring_peek_warmup(rx, rx->next_read);
}

static void rx_ring_update_hw_offset_warmup(struct rx_ring *rx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	u8 ptr;

	for (ptr = 0; ptr < rx->pkt_count; ++ptr) {
		if (!__rx_ring_peek_warmup(rx, ptr))
			continue;

		if (ptr == 0)
			break;

		netdev_warn(priv->ndev, "RX runaway 0 -> %d", ptr);

		/* Make sure the controller still wraps at rx->pkt_count
		 *
		 * The hardware ptr will now run
		 *   from 0         to rx->pkt_count - hw_offset,
		 * while software ptr will run
		 *   from hw_offset to rx->pkt_count
		 */
		rx->scan_hw_offset = ptr;
		rx->next_read = ptr;
		rx->next_refill = ptr;
		regmap_write(priv->map, MCTP_RX_BUF_SIZE,
			     rx->pkt_count - rx->scan_hw_offset);

		return;
	}
}

/* Prepare the next write slot for hardware to write into.
 *
 * Caller must ensure the next write slot is safe to write by checking
 * the driver read pointer.
 */
static void rx_ring_refill_warmup(struct rx_ring *rx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	u8 ptr;

	/* Translate our indices to potentially offset-ed
	 * hardware pointer due to RX runaway.
	 */
	if (rx->scan_hw_offset == 0) {
		ptr = rx->next_refill;
		rx->mode = RX_MODE_FAST;
	} else if (rx->next_refill >= rx->scan_hw_offset) {
		ptr = rx->next_refill - rx->scan_hw_offset;
	} else {
		/* we wrapped around, reset the buffer to normal again */
		ptr = rx->next_refill;
		rx->scan_hw_offset = 0;
		rx->mode = RX_MODE_FAST;
		regmap_write(priv->map, MCTP_RX_BUF_SIZE, rx->pkt_count);
	}

	/* Ideally we have to allocate a packet on the fly and
	 * map it to hardware here, but since we use a coherent DMA buffer,
	 * we only need to increate the refill pointer for now.
	 */

	smp_store_release(&rx->next_refill,
			  (rx->next_refill + 1) % rx->pkt_count);
	regmap_write_bits(priv->map, RX_BUF_WR_PTR, RX_BUF_WR_PTR_MASK, ptr);
}

static int rx_ring_read_refill_many_warmup(struct rx_ring *rx, int budget,
					    struct list_head *skb_list)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	struct sk_buff *skb = NULL;
	int done = 0;

	/* For the first packet, we have to scan to find the location */
	if (unlikely(rx->scan_counter == 0))
		rx_ring_update_hw_offset_warmup(rx);

	for (; done != budget && rx_ring_peek_warmup(rx); done += 1) {
		skb = rx_ring_read(rx);
		if (!skb) {
			dev_core_stats_rx_dropped_inc(priv->ndev);
			break;
		}

		rx_ring_refill_warmup(rx);
		list_add_tail(&skb->list, skb_list);
		rx->scan_counter += 1;
		dev_sw_netstats_rx_add(priv->ndev, skb->len);
	}

	return done;
}

static int rx_ring_process_warmup(struct rx_ring *rx, int budget)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	struct list_head skb_list;
	int done = 0;

	INIT_LIST_HEAD(&skb_list);

	done += rx_ring_read_refill_many_warmup(rx, budget, &skb_list);
	netif_receive_skb_list(&skb_list);

	/* Update the register for viewing in regmap debugfs */
	regmap_write(priv->map, MCTP_RX_BUF_RD_PTR, UPDATE_RX_RD_PTR);

	return done;
}

static inline void rx_ring_refill_many_fast(struct rx_ring *rx)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	smp_store_release(&rx->next_refill, smp_load_acquire(&rx->next_read));
	regmap_write_bits(priv->map, RX_BUF_WR_PTR, RX_BUF_WR_PTR_MASK,
			  rx->next_refill);
}

static int rx_ring_read_many_fast(struct rx_ring *rx, int budget,
				  struct list_head *skb_list)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);
	u8 hw_ptr = rx_ring_hw_ptr(rx);
	struct sk_buff *skb = NULL;
	int done = 0;

	for (; done != budget && rx->next_read != hw_ptr; done += 1) {
		skb = rx_ring_read(rx);
		if (!skb) {
			dev_core_stats_rx_dropped_inc(priv->ndev);
			break;
		}

		list_add_tail(&skb->list, skb_list);
		dev_sw_netstats_rx_add(priv->ndev, skb->len);
	}

	return done;
}

static int rx_ring_process_fast(struct rx_ring *rx, int budget)
{
	struct list_head skb_list;
	int done = 0;

	INIT_LIST_HEAD(&skb_list);

	done += rx_ring_read_many_fast(rx, budget, &skb_list);
	rx_ring_refill_many_fast(rx);
	netif_receive_skb_list(&skb_list);

	return done;
}

/* Pick the appropriate RX path.
 *
 * See the comment on enum rx_mode for a description of what each path is.
 */
static inline int rx_ring_process(struct rx_ring *rx, int budget)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(rx, struct aspeed_mctp_ctrl, rx);

	if (likely(rx->mode == RX_MODE_FAST))
		return rx_ring_process_fast(rx, budget);

	if (rx->mode == RX_MODE_WARMUP)
		return rx_ring_process_warmup(rx, budget);

	/* unreachable */
	netdev_err(priv->ndev, "unreachable: unsupported RX mode: %d",
		   rx->mode);

	return -EPROTO;
}

/* Enable "active" irqs, i.e TX send, RX receive, and RX full interrupts.
 *
 * Those interrupts are typically turned on and off per NAPI and
 * net device opening, closing.
 *
 * On Root Complex, we use a periodic timer for the lack of interrupts.
 */
static inline void
aspeed_mctp_ctrl_enable_active_irq(struct aspeed_mctp_ctrl *priv)
{
	if (priv->rc_f)
		mod_timer(&priv->rx_poll_timer,
			  jiffies + priv->rx_poll_interval_jiffies);
	else
		regmap_write_bits(priv->map, MCTP_INT_EN, ACTIVE_INTS,
				  ACTIVE_INTS);
}

static inline void
aspeed_mctp_ctrl_disable_active_irq(struct aspeed_mctp_ctrl *priv)
{
	if (priv->rc_f)
		del_timer_sync(&priv->rx_poll_timer);
	else
		regmap_write_bits(priv->map, MCTP_INT_EN, ACTIVE_INTS, 0);
}

/* Enable "passive" irq, i.e TX wrong.
 *
 * That interrupt is typically turned on and off per PCIe reset.
 */
static inline void
aspeed_mctp_ctrl_enable_passive_irq(struct aspeed_mctp_ctrl *priv)
{
	if (priv->rc_f)
		return;

	regmap_write_bits(priv->map, MCTP_INT_EN, TX_WRONG_INT, TX_WRONG_INT);
}

static inline void
aspeed_mctp_ctrl_disable_passive_irq(struct aspeed_mctp_ctrl *priv)
{
	if (priv->rc_f)
		return;

	regmap_write_bits(priv->map, MCTP_INT_EN, TX_WRONG_INT, 0);
}

static int aspeed_mctp_ctrl_napi_poll(struct napi_struct *napi, int budget)
{
	struct aspeed_mctp_ctrl *priv = container_of(napi, typeof(*priv), napi);
	struct tx_ring *tx = &priv->tx;
	struct rx_ring *rx = &priv->rx;
	int done = 0;

	done = rx_ring_process(rx, budget);
	tx_ring_reclaim_many(tx);

	if (done < budget && napi_complete_done(napi, done)) {
		aspeed_mctp_ctrl_enable_active_irq(priv);

		/* Setting RX_READY just in case the controller was stopped
		 * due to RX full,
		 *
		 * If any code need to clear RX_READY, it must wait for any
		 * outstanding NAPI work to avoid racing.
		 */
		regmap_write_bits(priv->map, MCTP_CTRL, RX_READY, RX_READY);
	}

	return done;
}

static netdev_tx_t aspeed_mctp_ctrl_start_xmit(struct sk_buff *skb,
					       struct net_device *ndev)
{
	struct aspeed_mctp_ctrl *priv = netdev_priv(ndev);
	struct tx_ring *tx = &priv->tx;

	skb_tx_timestamp(skb);

	if (unlikely(tx_ring_full(tx))) {
		netdev_err(priv->ndev, "BUG! TX ring full when queue awake!\n");
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	tx_ring_xmit(tx, skb);
	dev_sw_netstats_tx_add(priv->ndev, 1, skb->len);
	dev_kfree_skb(skb);

	if (tx_ring_full(tx))
		netif_stop_queue(priv->ndev);

	regmap_write_bits(priv->map, MCTP_CTRL, TX_TRIGGER, 0);
	regmap_write_bits(priv->map, MCTP_CTRL, TX_TRIGGER, TX_TRIGGER);

	return NETDEV_TX_OK;
}

static irqreturn_t aspeed_mctp_ctrl_irq_handler(int irq, void *arg)
{
	struct aspeed_mctp_ctrl *priv = arg;
	irqreturn_t ret = IRQ_NONE;
	u32 status = 0;

	regmap_read(priv->map, MCTP_INT_STS, &status);

	/* Reserved interrupt so branch should always be the case */
	if (likely(status & ACTIVE_INTS)) {
		if (unlikely(status & RX_NO_MORE_INT)) {
			netdev_warn(priv->ndev,
				    "RX full, packets might be dropped");
		}
		aspeed_mctp_ctrl_disable_active_irq(priv);
		napi_schedule(&priv->napi);
		ret = IRQ_HANDLED;
	}

	/* This should never happen. If it ever happens, we are screwed anyway
	 * so just print the command buffer inside the interrupt handler.
	 */
	if (unlikely(status & TX_WRONG_INT)) {
		print_hex_dump(KERN_ERR, "BUG: wrong TX cmd",
			       DUMP_PREFIX_OFFSET, sizeof(*priv->tx.cmd), 1,
			       priv->tx.cmd,
			       sizeof(*priv->tx.cmd) * priv->tx.pkt_count,
			       false);
		netdev_warn(priv->ndev, "xmit %d reclaim %d",
			    priv->tx.next_xmit, priv->tx.next_reclaim);
		ret = IRQ_HANDLED;
	}

	regmap_write(priv->map, MCTP_INT_STS, status);

	return ret;
}

static void aspeed_mctp_ctrl_reset(struct aspeed_mctp_ctrl *priv)
{
	struct tx_ring *tx = &priv->tx;
	struct rx_ring *rx = &priv->rx;
	int i;

	regmap_write_bits(priv->map, MCTP_ENGINE_CTRL, FIFO_LAYOUT_MASK,
			  FIELD_PREP(FIFO_LAYOUT_MASK, FIFO_TX_2K_RX_6K));

	regmap_write_bits(
		priv->map, MCTP_ENGINE_CTRL, TX_MAX_PAYLOAD_SIZE_MASK,
		FIELD_PREP(TX_MAX_PAYLOAD_SIZE_MASK,
			   priv->match_data->tx_max_payload_size_regval));
	regmap_write_bits(
		priv->map, MCTP_ENGINE_CTRL, RX_MAX_PAYLOAD_SIZE_MASK,
		FIELD_PREP(RX_MAX_PAYLOAD_SIZE_MASK,
			   priv->match_data->rx_max_payload_size_regval));

	/* Reset TX registers */
	tx->next_xmit = 0;
	tx->next_reclaim = 0;

	memset(tx->pkt, 0, tx->pkt_size * tx->pkt_count);
	memset(tx->cmd, 0, sizeof(*tx->cmd) * tx->pkt_count);
	regmap_write(priv->map, MCTP_TX_BUF_ADDR, tx->cmd_paddr);
	regmap_write(priv->map, TX_BUF_WR_PTR, 0);
	regmap_write(priv->map, MCTP_TX_BUF_SIZE, tx->pkt_count);

	/* Reset RX registers */
	rx->next_read = 0;
	rx->next_refill = 0;
	memset(rx->pkt, 0, rx->pkt_size * rx->pkt_count);
	for (i = 0; i < rx->pkt_count; i++) {
		rx->cmd[i] = FIELD_PREP(RX_DATA_ADDR_MASK,
					rx->pkt_paddr + i * rx->pkt_size) |
			     RX_INTERRUPT_AFTER_CMD;
	}
	regmap_write(priv->map, MCTP_RX_BUF_ADDR, rx->cmd_paddr);
	regmap_write(priv->map, RX_BUF_WR_PTR, 0);
	regmap_write(priv->map, MCTP_RX_BUF_SIZE, rx->pkt_count);

	/* Workarounds data */
	rx->mode = priv->match_data->starting_mode;
	rx->scan_counter = 0;
	rx->scan_hw_offset = 0;
}

static void aspeed_mctp_ctrl_handle_rx_poll_timer(struct timer_list *timer)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(timer, typeof(*priv), rx_poll_timer);
	napi_schedule(&priv->napi);
	mod_timer(&priv->rx_poll_timer,
		  jiffies + priv->rx_poll_interval_jiffies);
}

static void aspeed_mctp_ctrl_bdf_work(struct work_struct *work)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(work, typeof(*priv), bdf_work.work);
	u32 reg = 0;
	u8 bdf[2];

	regmap_read(priv->map_pcie, PCIE_MISC_STS_1, &reg);
	/* Make space for 3-bit function number (0 by default) */
	reg = FIELD_GET(PCIE_BUS_DEVICE, reg) << 3;
	put_unaligned_be16(reg, bdf);
	dev_addr_set(priv->ndev, bdf);
	if (!reg) {
		/* We don't have a definitive delay between PERST and
		 * enumeration, so we just retry until we are assigned
		 * a BDF.
		 */
		schedule_delayed_work(&priv->bdf_work, msecs_to_jiffies(1000));
	}
}

/* Enable interrupts, NAPI and wake TX queue.
 *
 * Callers ensure no other instances of this function and aspeed_mctp_ctrl_pause
 * is running to avoid tearing the interrupts and ready state.
 *
 * No-op if already resumed.
 */
static void aspeed_mctp_ctrl_resume(struct aspeed_mctp_ctrl *priv)
{
	schedule_delayed_work(&priv->bdf_work, 0);
	aspeed_mctp_ctrl_enable_passive_irq(priv);
	aspeed_mctp_ctrl_enable_active_irq(priv);
	regmap_write_bits(priv->map, MCTP_CTRL, RX_READY, RX_READY);
	netif_wake_queue(priv->ndev);
}

/* Disable interrupts, NAPI and stop TX queue.
 *
 * Callers ensure no other instances of this function and aspeed_mctp_ctrl_resume
 * is running to avoid tearing the interrupts and ready state.
 *
 * No-op if already paused.
 */
static void aspeed_mctp_ctrl_pause(struct aspeed_mctp_ctrl *priv)
{
	netif_stop_queue(priv->ndev);
	/* Make sure no pending NAPI can modify the registers below */
	napi_disable(&priv->napi);
	regmap_write_bits(priv->map, MCTP_CTRL, RX_READY, 0);
	aspeed_mctp_ctrl_disable_passive_irq(priv);
	aspeed_mctp_ctrl_disable_active_irq(priv);
	/* Pair the napi_disable above, but hardware is essentially stopped */
	napi_enable(&priv->napi);
	cancel_delayed_work_sync(&priv->bdf_work);
}

static void aspeed_mctp_ctrl_perst_work(struct work_struct *work)
{
	struct aspeed_mctp_ctrl *priv =
		container_of(work, typeof(*priv), rst_work);
	u32 reg = 0;

	regmap_read(priv->map_pcie, PCIE_MISC_STS_1, &reg);
	reg = FIELD_GET(PCIE_PERST, reg);

	if (!netif_carrier_ok(priv->ndev) && reg) {
		netif_carrier_on(priv->ndev);
		aspeed_mctp_ctrl_reset(priv);
		aspeed_mctp_ctrl_resume(priv);
		return;
	}

	if (netif_carrier_ok(priv->ndev) && !reg) {
		netif_carrier_off(priv->ndev);
		aspeed_mctp_ctrl_pause(priv);
		return;
	}
}

static irqreturn_t aspeed_mctp_ctrl_handle_perst_irq(int irq, void *arg)
{
	struct aspeed_mctp_ctrl *priv = arg;

	schedule_work(&priv->rst_work);

	return IRQ_HANDLED;
}

static int aspeed_mctp_ctrl_request_mctp_irq(struct aspeed_mctp_ctrl *priv)
{
	int ret = 0;

	if (priv->rc_f)
		return 0;

	ret = request_irq(priv->irq_mctp, aspeed_mctp_ctrl_irq_handler,
			  IRQF_SHARED, netdev_name(priv->ndev), priv);
	if (ret) {
		netdev_err(priv->ndev, "Cannot request IRQ mctp, error %d",
			   ret);
		return ret;
	}

	return 0;
}

static int aspeed_mctp_ctrl_request_perst_irq(struct aspeed_mctp_ctrl *priv)
{
	int ret = 0;

	ret = request_irq(priv->irq_perst_hi, aspeed_mctp_ctrl_handle_perst_irq,
			  IRQF_SHARED, netdev_name(priv->ndev), priv);
	if (ret) {
		netdev_err(priv->ndev, "Cannot request IRQ perst_hi, error %d",
			   ret);
		goto err_hi;
	}

	ret = request_irq(priv->irq_perst_lo, aspeed_mctp_ctrl_handle_perst_irq,
			  IRQF_SHARED, netdev_name(priv->ndev), priv);
	if (ret) {
		netdev_err(priv->ndev, "Cannot request IRQ perst_lo, error %d",
			   ret);
		goto err_lo;
	}

	return 0;

err_lo:
	free_irq(priv->irq_perst_hi, priv);
err_hi:
	return ret;
}

static void aspeed_mctp_ctrl_free_mctp_irq(struct aspeed_mctp_ctrl *priv)
{
	if (priv->rc_f)
		return;

	free_irq(priv->irq_mctp, priv);
}

static void aspeed_mctp_ctrl_free_perst_irq(struct aspeed_mctp_ctrl *priv)
{
	free_irq(priv->irq_perst_lo, priv);
	free_irq(priv->irq_perst_hi, priv);
}

static int aspeed_mctp_ctrl_open(struct net_device *ndev)
{
	struct aspeed_mctp_ctrl *priv = netdev_priv(ndev);
	int ret = 0;

	napi_enable(&priv->napi);

	ret = aspeed_mctp_ctrl_request_mctp_irq(priv);
	if (ret)
		goto err_mctp_irq;

	/* PERST irq is enabled later below, so there should be no
	 * others who can change carrier state or resume, causing races.
	 */
	if (netif_carrier_ok(ndev))
		aspeed_mctp_ctrl_resume(priv);

	ret = aspeed_mctp_ctrl_request_perst_irq(priv);
	if (ret)
		goto err_perst_irq;

	/* Level-trigger the reset one time to initialize the state.
	 * The reset work is no-op when already initialized.
	 */
	schedule_work(&priv->rst_work);

	return 0;

err_perst_irq:
	aspeed_mctp_ctrl_pause(priv);
	aspeed_mctp_ctrl_free_mctp_irq(priv);
err_mctp_irq:
	napi_disable(&priv->napi);
	return ret;
}

static int aspeed_mctp_ctrl_stop(struct net_device *ndev)
{
	struct aspeed_mctp_ctrl *priv = netdev_priv(ndev);

	/* Make sure rst_work is stopped, so that it is safe to pause */
	aspeed_mctp_ctrl_free_perst_irq(priv);
	cancel_work_sync(&priv->rst_work);
	aspeed_mctp_ctrl_pause(priv);
	aspeed_mctp_ctrl_free_mctp_irq(priv);
	napi_disable(&priv->napi);

	return 0;
}

/* Create an MCTP over PCIe-VDM header
 *
 * The header created has host endian.
 *
 * Currently we only implement DSP0238 1.2.1, which uses
 * non-flit mode TLP packet.
 */
static int aspeed_mctp_ctrl_header_create(struct sk_buff *skb,
					  struct net_device *ndev,
					  unsigned short type,
					  const void *daddr, const void *saddr,
					  unsigned int len)
{
	u8 routing_type;
	u16 requester;
	u16 target;
	u32 *hdr;
	u16 dw;
	u8 pad;

	/* MCTP network layer should have already taken care of
	 * adding the mctp_hdr and disassembling the messages
	 * into approriate packet len.
	 */
	if (len < sizeof(struct mctp_hdr))
		return -EPROTO;
	if (len >= ndev->max_mtu)
		return -EMSGSIZE;

	len -= sizeof(struct mctp_hdr);
	dw = ALIGN(len, 4) / 4;
	pad = ALIGN(len, 4) - len;

	if (!saddr || !daddr)
		return -EINVAL;

	requester = get_unaligned_be16(saddr);
	target = get_unaligned_be16(daddr);

	if (requester == 0x0000 || requester == 0xFFFF)
		return -EINVAL;

	if (target == 0x0000)
		routing_type = 0;
	else if (target == 0xFFFF)
		routing_type = 3;
	else
		routing_type = 2;

	skb_push(skb, PCIEVDM_HLEN);
	skb_reset_mac_header(skb);

	hdr = (void *)skb_mac_header(skb);

	hdr[0] = DW0_DEFAULT;
	hdr[0] |= FIELD_PREP(ROUTING_TYPE_MASK, routing_type);
	hdr[0] |= FIELD_PREP(LEN_DW_MASK, dw);

	hdr[1] = DW1_DEFAULT;
	hdr[1] |= FIELD_PREP(REQUESTER_MASK, requester);
	hdr[1] |= FIELD_PREP(PAD_LEN_MASK, pad);

	hdr[2] = DW2_DEFAULT;
	hdr[2] |= FIELD_PREP(TARGET_MASK, target);

	return PCIEVDM_HLEN;
}

static const struct header_ops aspeed_mctp_ctrl_headops = {
	.create = aspeed_mctp_ctrl_header_create,
};

static const struct net_device_ops aspeed_mctp_ctrl_netops = {
	.ndo_open = aspeed_mctp_ctrl_open,
	.ndo_stop = aspeed_mctp_ctrl_stop,
	.ndo_start_xmit = aspeed_mctp_ctrl_start_xmit,
	.ndo_get_stats64 = dev_get_tstats64,
};

static const struct mctp_netdev_ops aspeed_mctp_ctrl_mctpdev_ops = {};

static void aspeed_mctp_ctrl_setup(struct net_device *ndev)
{
	/* the same as other mctp_* devices */
	ndev->type = ARPHRD_MCTP;
	ndev->min_mtu = MCTP_PCIEVDM_MINMTU;
	ndev->mtu = MCTP_PCIEVDM_DEFAULTMTU;
	ndev->hard_header_len = PCIEVDM_HLEN;
	ndev->addr_len = PCIEVDM_ALEN;
	ndev->netdev_ops = &aspeed_mctp_ctrl_netops;
	ndev->header_ops = &aspeed_mctp_ctrl_headops;
	memset(ndev->broadcast, 0xFF, ndev->addr_len);
	ndev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;
}

static int aspeed_mctp_ctrl_init_tx(struct aspeed_mctp_ctrl *priv,
				    unsigned int pkt_size,
				    unsigned int pkt_count)
{
	struct device *pdev = priv->ndev->dev.parent;
	struct tx_ring *tx = &priv->tx;
	unsigned int alloc_cmd_size;
	unsigned int alloc_pkt_size;

	memset(tx, 0, sizeof(*tx));

	tx->pkt_size = pkt_size;
	tx->pkt_count = pkt_count;

	alloc_cmd_size = sizeof(*tx->cmd) * tx->pkt_count;
	tx->cmd = dmam_alloc_coherent(pdev, alloc_cmd_size, &tx->cmd_paddr,
				      GFP_KERNEL);
	if (!tx->cmd || !dma_coherent_ok(pdev, tx->cmd_paddr, alloc_cmd_size)) {
		netdev_err(priv->ndev,
			   "Failed to alloc coherent DMA for TX cmds");
		return -ENOMEM;
	}

	alloc_pkt_size = tx->pkt_size * tx->pkt_count;
	tx->pkt = dmam_alloc_coherent(pdev, alloc_pkt_size, &tx->pkt_paddr,
				      GFP_KERNEL);
	if (!tx->pkt || !dma_coherent_ok(pdev, tx->pkt_paddr, alloc_pkt_size)) {
		netdev_err(priv->ndev,
			   "Failed to alloc coherent DMA for TX packets");
		return -ENOMEM;
	}

	return 0;
}

static int aspeed_mctp_ctrl_init_rx(struct aspeed_mctp_ctrl *priv,
				    unsigned int pkt_size,
				    unsigned int pkt_count)
{
	struct device *pdev = priv->ndev->dev.parent;
	struct rx_ring *rx = &priv->rx;
	unsigned int alloc_cmd_size;
	unsigned int alloc_pkt_size;

	memset(rx, 0, sizeof(*rx));

	rx->pkt_size = pkt_size;
	rx->pkt_count = pkt_count;

	alloc_cmd_size = sizeof(*rx->cmd) * rx->pkt_count;
	rx->cmd = dmam_alloc_coherent(pdev, alloc_cmd_size, &rx->cmd_paddr,
				      GFP_KERNEL);
	if (!rx->cmd || !dma_coherent_ok(pdev, rx->cmd_paddr, alloc_cmd_size)) {
		netdev_err(priv->ndev,
			   "Failed to alloc coherent DMA for RX cmds");
		return -ENOMEM;
	}

	alloc_pkt_size = rx->pkt_size * rx->pkt_count;
	rx->pkt = dmam_alloc_coherent(pdev, alloc_pkt_size, &rx->pkt_paddr,
				      GFP_KERNEL);
	if (!rx->pkt || !dma_coherent_ok(pdev, rx->pkt_paddr, alloc_pkt_size)) {
		netdev_err(priv->ndev,
			   "Failed to alloc coherent DMA for RX packets");
		return -ENOMEM;
	}

	return 0;
}

static void aspeed_mctp_ctrl_release_reserved_mem(void *d)
{
	of_reserved_mem_device_release(d);
}

static void aspeed_mctp_ctrl_free_netdev(void *d)
{
	free_netdev(d);
}

static int aspeed_mctp_ctrl_probe(struct platform_device *pdev)
{
	struct aspeed_mctp_ctrl *priv = NULL;
	struct net_device *ndev = NULL;
	u32 rx_poll_interval_msec = 0;
	void __iomem *regs = NULL;
	u32 rx_packet_count = 0;
	u32 tx_packet_count = 0;
	int ret = 0;

	dev_info(&pdev->dev, "Aspeed MCTP controller driver");

	ndev = alloc_netdev(sizeof(struct aspeed_mctp_ctrl *), "mctppcie%d",
			    NET_NAME_ENUM, aspeed_mctp_ctrl_setup);
	if (!ndev)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "Cannot allocate netdev");

	ret = devm_add_action_or_reset(&pdev->dev, aspeed_mctp_ctrl_free_netdev,
				       ndev);
	if (ret)
		return ret;

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->ndev->dev.parent = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	priv->rc_f = of_find_property(pdev->dev.of_node, "pcie_rc", NULL);
	if (priv->rc_f)
		dev_warn(&pdev->dev,
			 "Root Complex detected. Interrupts are disabled.");

	priv->match_data = of_device_get_match_data(&pdev->dev);
	if (!priv->match_data) {
		return dev_err_probe(&pdev->dev, -ENOENT,
				     "Cannot get device match data");
	}
	priv->ndev->max_mtu =
		sizeof(struct mctp_hdr) + priv->match_data->max_payload_size;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(regs),
				     "Cannot ioremap resources");
	}

	static const struct regmap_config aspeed_mctp_ctrl_regmap_cfg = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.max_register = TX_BUF_WR_PTR,
	};
	priv->map = devm_regmap_init_mmio(&pdev->dev, regs,
					  &aspeed_mctp_ctrl_regmap_cfg);
	if (IS_ERR(priv->map)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->map),
				     "Cannot initialize regmap");
	}

	priv->reset = devm_reset_control_get_shared_by_index(&pdev->dev, 0);
	if (IS_ERR(priv->reset)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->reset),
				     "Cannot get reset control");
	}
	reset_control_deassert(priv->reset);

	if (priv->rc_f) {
		priv->reset_dma =
			devm_reset_control_get_shared_by_index(&pdev->dev, 1);
		if (IS_ERR(priv->reset_dma)) {
			return dev_err_probe(&pdev->dev,
					     PTR_ERR(priv->reset_dma),
					     "Cannot get DMA reset control");
		}
		reset_control_deassert(priv->reset_dma);
	}

	priv->map_pcie = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							 "aspeed,pcieh");
	if (IS_ERR(priv->map_pcie)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->map_pcie),
				     "Cannot find PCIe regmap");
	}

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		return dev_err_probe(&pdev->dev, ret,
				     "Cannot initialize reserved memory");
	}

	ret = devm_add_action_or_reset(
		&pdev->dev, aspeed_mctp_ctrl_release_reserved_mem, &pdev->dev);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		return dev_err_probe(&pdev->dev, ret,
				     "Cannot set coherent mask");
	}

	priv->irq_mctp = platform_get_irq_byname_optional(pdev, "mctp");
	if (priv->irq_mctp == -EPROBE_DEFER) {
		return priv->irq_mctp;
	}

	priv->irq_perst_hi = platform_get_irq_byname(pdev, "perst-hi");
	if (priv->irq_perst_hi < 0)
		return priv->irq_perst_hi;

	priv->irq_perst_lo = platform_get_irq_byname(pdev, "perst-lo");
	if (priv->irq_perst_lo < 0)
		return priv->irq_perst_lo;

	INIT_WORK(&priv->rst_work, aspeed_mctp_ctrl_perst_work);
	INIT_DELAYED_WORK(&priv->bdf_work, aspeed_mctp_ctrl_bdf_work);
	if (priv->rc_f) {
		ret = device_property_read_u32(
			&pdev->dev, "aspeed,rx-polling-interval-msec",
			&rx_poll_interval_msec);
		if (ret) {
			rx_poll_interval_msec =
				DEFAULT_RX_POLLING_INTERVAL_MSEC;
		}

		priv->rx_poll_interval_jiffies =
			msecs_to_jiffies(rx_poll_interval_msec);

		timer_setup(&priv->rx_poll_timer,
			    aspeed_mctp_ctrl_handle_rx_poll_timer, 0);
	}
	netif_napi_add(ndev, &priv->napi, aspeed_mctp_ctrl_napi_poll);

	ret = device_property_read_u32(&pdev->dev, "aspeed,rx-hw-queue-depth",
				       &rx_packet_count);
	if (ret)
		rx_packet_count = DEFAULT_RX_HW_QUEUE_DEPTH;

	ret = aspeed_mctp_ctrl_init_rx(
		priv, ndev->hard_header_len + ndev->max_mtu, rx_packet_count);
	if (ret)
		return ret;

	ret = device_property_read_u32(&pdev->dev, "aspeed,tx-hw-queue-depth",
				       &tx_packet_count);
	if (ret)
		tx_packet_count = DEFAULT_TX_HW_QUEUE_DEPTH;

	ret = aspeed_mctp_ctrl_init_tx(
		priv, ndev->hard_header_len + ndev->max_mtu, tx_packet_count);
	if (ret)
		return ret;

	netif_carrier_off(priv->ndev);
	ret = mctp_register_netdev(priv->ndev, &aspeed_mctp_ctrl_mctpdev_ops);
	if (ret) {
		return dev_err_probe(&pdev->dev, ret,
				     "Cannot register MCTP net device");
	}

	return 0;
}

static void aspeed_mctp_ctrl_remove(struct platform_device *pdev)
{
	struct aspeed_mctp_ctrl *priv = platform_get_drvdata(pdev);

	mctp_unregister_netdev(priv->ndev);
	netif_napi_del(&priv->napi);
	reset_control_assert(priv->reset);
	if (priv->rc_f)
		reset_control_assert(priv->reset_dma);
}

static const struct aspeed_mctp_ctrl_match_data ast2600_mctp_match_data = {
	/* Due to hardware quirks in AST2600, MCTP controller hardware RX pointer
	 * may appear to be "runaway" after reset, writing into a random position.
	 *
	 * The only reliable way to get the correct write position here is to
	 * scan the buffer until we sync hardware and software pointers.
	 */
	.starting_mode = RX_MODE_WARMUP,

	/* For some reasons, AST2600 only allows TX max payload up to 256 bytes,
	 * but you need to set it to 512 bytes to be able to use 256 bytes.
	 */
	.tx_max_payload_size_regval = MAX_PAYLOAD_512,

	/* Due to symmetry in TX/RX MTU in Linux networking, RX is capped
	 * to 256 bytes, the same as TX.
	 */
	.rx_max_payload_size_regval = MAX_PAYLOAD_256,

	/* Derived from RX max payload and TX max payload above.
	 */
	.max_payload_size = 256,
};

static const struct of_device_id aspeed_mctp_ctrl_match_table[] = {
	{ .compatible = "aspeed,ast2600-mctp",
	  .data = &ast2600_mctp_match_data },
	{}
};

static struct platform_driver aspeed_mctp_ctrl_driver = {
	.driver	= {
		.name		= "aspeed-mctp-ctrl",
		.of_match_table	= of_match_ptr(aspeed_mctp_ctrl_match_table),
	},
	.probe		= aspeed_mctp_ctrl_probe,
	.remove_new	= aspeed_mctp_ctrl_remove,
};

static int __init aspeed_mctp_ctrl_init(void)
{
	return platform_driver_register(&aspeed_mctp_ctrl_driver);
}

static void __exit aspeed_mctp_ctrl_exit(void)
{
	platform_driver_unregister(&aspeed_mctp_ctrl_driver);
}

module_init(aspeed_mctp_ctrl_init);
module_exit(aspeed_mctp_ctrl_exit);

MODULE_DEVICE_TABLE(of, aspeed_mctp_ctrl_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Khang D Nguyen <khangng@os.amperecomputing.com>");
MODULE_DESCRIPTION("Aspeed MCTP Controller driver");
