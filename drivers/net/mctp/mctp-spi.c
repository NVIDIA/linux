#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/ethtool.h>

#include <linux/if_arp.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>
#include <net/mctpdevice.h>

#include <trace/events/mctp.h>

#include "glacier-spb-ap.h"
#include "mctp-spi-internal.h"
#include <net/mctp-stats.h>
#include "mctp-spi-error-inject.h"

static DEFINE_IDA(mctp_spi_ida);

#define MCTP_SPI_MAXMTU (64 + 4)
#define MCTP_SPI_MINMTU (64 + 4)
#define MCTP_SPI_TX_QUEUE_LEN 1100
#define BUFSIZE			256
#define MCTP_SPI_TX_WORK_LEN 100
#define MCTP_COMMAND_CODE 0x02

#define ERR_SPI_RX_NO_DATA -2

/**
 * spb_ap_status_to_errno - Convert SPB AP status codes to Linux error codes
 * @status: SPB AP status code from spb_ap_send() or other SPB AP functions
 *
 * Translates positive SPB AP status codes to negative Linux errno values
 * for consistent error handling throughout the driver.
 *
 * Note: SPB_AP_MESSAGE_AVAILABLE is a success status (not an error) that
 * indicates a message is available to read after a successful send.
 */
static inline int spb_ap_status_to_errno(SpbApStatus status)
{
	switch (status) {
	case SPB_AP_OK:
	case SPB_AP_MESSAGE_AVAILABLE:
		return 0;
	case SPB_AP_ERROR_TIMEOUT:
		return -ETIMEDOUT;
	case SPB_AP_ERROR_INVALID_ARGUMENT:
		return -EINVAL;
	case SPB_AP_ERROR_UNKNOWN:
	default:
		return -EIO;
	}
}

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static const struct spi_device_id mctp_spi_ids[] = {
	{ .name = "mctp-spi" },
	{},
};
MODULE_DEVICE_TABLE(spi, mctp_spi_ids);

static const struct of_device_id mctp_spi_dt_ids[] = {
	{ .compatible = "mctp-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, mctp_spi_dt_ids);

/*-------------------------------------------------------------------------*/

static netdev_tx_t mctp_spi_net_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct mctp_spi *midev = netdev_priv(dev);

	spin_lock(&midev->tx_queue.lock);
	if (skb_queue_len(&midev->tx_queue) >= MCTP_SPI_TX_WORK_LEN) {
		netif_stop_queue(dev);
		spin_unlock(&midev->tx_queue.lock);
		netdev_err(dev, "BUG! Tx Ring full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	__skb_queue_tail(&midev->tx_queue, skb);
	if (skb_queue_len(&midev->tx_queue) == MCTP_SPI_TX_WORK_LEN)
		netif_stop_queue(dev);
	spin_unlock(&midev->tx_queue.lock);

	wake_up(&midev->main_thread_wq);
	return NETDEV_TX_OK;
}

static int mctp_spi_net_recv(struct mctp_spi *midev, uint8_t *rx_buffer)
{
	struct net_device *ndev = midev->ndev;
	struct sk_buff *skb;
	struct mctp_skb_cb *cb;
	unsigned long flags;
	size_t recvlen;
	int status;

	recvlen = midev->spidev->rx_len;

	skb = netdev_alloc_skb(ndev, recvlen);
	if (!skb) {
		ndev->stats.rx_dropped++;
		/* Can't extract EID - no packet data available yet */
		MCTP_STAT_INC(midev, MCTP_EID_UNKNOWN, rx_drop_no_memory);
		trace_mctp_transport_error("spi", ndev, "rx_drop_no_memory", recvlen);
		return -ENOMEM;
	}
	skb->protocol = htons(ETH_P_MCTP);
	if (!rx_buffer) 
		rx_buffer = midev->spidev->rx_buffer;
	skb_put_data(skb, rx_buffer, recvlen);
	skb_reset_network_header(skb);
	cb = __mctp_cb(skb);
	cb->halen = 0;

	/* We need to ensure that the netif is not used once netdev
	 * unregister occurs
	 */
	spin_lock_irqsave(&midev->lock, flags);
	if (midev->allow_rx) {
		reinit_completion(&midev->rx_done);
		spin_unlock_irqrestore(&midev->lock, flags);
		status = netif_rx(skb);
		complete(&midev->rx_done);
	} else {
		status = NET_RX_DROP;
		spin_unlock_irqrestore(&midev->lock, flags);
	}

	if (status == NET_RX_SUCCESS) {
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += recvlen;
		netdev_dbg(ndev, "MCTP SPI: RX success, %zu bytes\n", recvlen);
		trace_mctp_transport_rx("spi", ndev, 0, recvlen);
	} else {
		ndev->stats.rx_dropped++;
		/* Extract EID for per-EID tracking (SKB has packet data) */
		if (skb->len >= sizeof(struct mctp_spi_hdr) + sizeof(struct mctp_hdr)) {
			struct mctp_hdr *mh = (struct mctp_hdr *)(skb->data + sizeof(struct mctp_spi_hdr));
			MCTP_STAT_INC(midev, mh->src, rx_drop_not_ready);
		} else {
			MCTP_STAT_INC(midev, MCTP_EID_UNKNOWN, rx_drop_not_ready);
		}
		netdev_dbg(ndev, "MCTP SPI: RX dropped, status=%d\n", status);
		trace_mctp_transport_error("spi", ndev, "rx_dropped", status);
	}

	return 0;
}

static int mctp_spi_rx(struct mctp_spi *midev)
{
	ssize_t len;
	ssize_t payload_len;
	const size_t hdr_size = sizeof(struct mctp_spi_hdr);
	SpbApStatus status = 0;
	struct mctp_spi_hdr *spi_hdr_rx;
	u8 tmp_rx_buffer[RX_BUFFER_SIZE];

	spi_hdr_rx = (struct mctp_spi_hdr *)tmp_rx_buffer;

	status = spb_ap_recv(midev->ap, RX_BUFFER_SIZE, tmp_rx_buffer);
	if(status != SPB_AP_OK) {
		int err = spb_ap_status_to_errno(status);
		
		midev->ndev->stats.rx_dropped++;
		/* Can't extract EID - SPI receive failed, no data */
		MCTP_STAT_INC(midev, MCTP_EID_UNKNOWN, rx_drop_spi_error);
		netdev_dbg(midev->ndev, "MCTP SPI: RX error, SPB status=%d, errno=%d\n", status, err);
		trace_mctp_transport_error("spi", midev->ndev, "rx_spi_error", err);
		return ERR_SPI_RX_NO_DATA;
	}

	payload_len = spi_hdr_rx->byte_count;
	len = payload_len + hdr_size;

	midev->spidev->rx_len = payload_len;
	// memcpy(midev->spidev->rx_buffer, tmp_rx_buffer + hdr_size, payload_len);
	mctp_spi_net_recv(midev, tmp_rx_buffer + hdr_size);

	return 0;
}

static void mctp_spi_ndo_uninit(struct net_device *dev)
{

}

static int mctp_spi_ndo_open(struct net_device *dev)
{
	struct mctp_spi *midev = netdev_priv(dev);
	unsigned long flags;

	/* spi rx thread can only pass packets once the netdev is registered */
	spin_lock_irqsave(&midev->lock, flags);
	midev->allow_rx = true;
	spin_unlock_irqrestore(&midev->lock, flags);
	return 0;
}

static int mctp_spi_header_create(struct sk_buff *skb, struct net_device *dev,
				  unsigned short type, const void *daddr,
	   const void *saddr, unsigned int len)
{
	struct mctp_spi_hdr *hdr;

	if (len > MCTP_SPI_MAXMTU)
		return -EMSGSIZE;

	skb_push(skb, sizeof(struct mctp_spi_hdr));
	skb_reset_mac_header(skb);
	hdr = (void *)skb_mac_header(skb);
	hdr->command_code = MCTP_COMMAND_CODE;
	hdr->byte_count = len;
	hdr->resrv[0] = 0;
	hdr->resrv[1] = 0;

	return sizeof(struct mctp_spi_hdr);
}

/* Ethtool statistics support */
/* Per-EID stat descriptors with abbreviated names */
struct mctp_spi_eid_stat_desc {
	const char *name;
	size_t offset;
};

#define MCTP_SPI_EID_STAT(abbrev, field) { \
	.name = abbrev, \
	.offset = offsetof(struct mctp_spi_eid_stats, field) \
}

static const struct mctp_spi_eid_stat_desc mctp_spi_eid_stat_descs[] = {
	MCTP_SPI_EID_STAT("rx_drop_no_memory",          rx_drop_no_memory),
	MCTP_SPI_EID_STAT("rx_drop_not_ready",          rx_drop_not_ready),
	MCTP_SPI_EID_STAT("rx_drop_spi_error",          rx_drop_spi_error),
	MCTP_SPI_EID_STAT("tx_drop_etimedout",          tx_drop_etimedout),
	MCTP_SPI_EID_STAT("tx_drop_einval",             tx_drop_einval),
	MCTP_SPI_EID_STAT("tx_drop_eio",                tx_drop_eio),
	MCTP_SPI_EID_STAT("tx_drop_spi_error",          tx_drop_spi_error),
	MCTP_SPI_EID_STAT("gpio_interrupts",            gpio_interrupts),
};

#define MCTP_SPI_EID_NUM_STATS ARRAY_SIZE(mctp_spi_eid_stat_descs)

/* Generate ethtool helper functions using shared macros */
MCTP_EID_STATS_HELPERS(mctp_spi, struct mctp_spi,
		       struct mctp_spi_eid_stats, mctp_spi_eid_stat_descs)

static void mctp_spi_get_strings(struct net_device *ndev, u32 stringset,
				 u8 *data)
{
	struct mctp_spi *midev = netdev_priv(ndev);
	unsigned int i;
	int eid;

	if (stringset != ETH_SS_STATS)
		return;

	/* Output aggregate stats (computed from per-EID) */
	for (i = 0; i < MCTP_SPI_EID_NUM_STATS; i++) {
		snprintf(data, ETH_GSTRING_LEN, "%-30s", mctp_spi_eid_stat_descs[i].name);
		data += ETH_GSTRING_LEN;
	}

	/* Separator line */
	snprintf(data, ETH_GSTRING_LEN, "                              ");
	data += ETH_GSTRING_LEN;

	/* Output per-EID stats (only for active EIDs with non-zero stats) */
	for_each_set_bit(eid, midev->eid_stats.active, 257) {
		struct mctp_spi_eid_stats *es = &midev->eid_stats.eid[eid];
		u8 *base = (u8 *)es;
		int nz = mctp_spi_count_eid_nonzero(midev, eid);

		if (nz == 0)
			continue;

		/* EID header line with explanation
		 * - UNKNOWN: Errors where EID could not be determined (GPIO interrupt
		 *   events, low-level SPI transfer failures, or allocation failures
		 *   before packet parsing)
		 * - EID_0: Valid packets with source/dest EID=0 (null/unallocated
		 *   endpoint per MCTP spec DSP0236)
		 * - EID_X: Normal endpoints (X = 1..253)
		 */
		if (eid == MCTP_EID_UNKNOWN) {
			snprintf(data, ETH_GSTRING_LEN, "UNKNOWN: SPI/GPIO errors      ");
		} else if (eid == 0) {
			snprintf(data, ETH_GSTRING_LEN, "EID_0: null endpoint          ");
		} else {
			snprintf(data, ETH_GSTRING_LEN, "EID_%-3u                       ", eid);
		}
		data += ETH_GSTRING_LEN;

		/* Individual non-zero stats for this EID */
		for (i = 0; i < MCTP_SPI_EID_NUM_STATS; i++) {
			u64 val = *(u64 *)(base + mctp_spi_eid_stat_descs[i].offset);
			if (val != 0) {
				snprintf(data, ETH_GSTRING_LEN, "%-30s",
					 mctp_spi_eid_stat_descs[i].name);
				data += ETH_GSTRING_LEN;
			}
		}
	}
}

static int mctp_spi_get_sset_count(struct net_device *ndev, int sset)
{
	struct mctp_spi *midev = netdev_priv(ndev);

	if (sset == ETH_SS_STATS)
		return MCTP_SPI_EID_NUM_STATS + 1 + mctp_spi_count_eid_stats(midev);
		       /* aggregates */  /* separator */  /* per-EID */

	return -EOPNOTSUPP;
}

static void mctp_spi_get_ethtool_stats(struct net_device *ndev,
				       struct ethtool_stats *stats,
				       u64 *data)
{
	struct mctp_spi *midev = netdev_priv(ndev);
	unsigned int i, idx = 0;
	int eid;

	/* Output aggregate stats (computed by summing across all EIDs) */
	for (i = 0; i < MCTP_SPI_EID_NUM_STATS; i++) {
		u64 total = 0;
		
		for_each_set_bit(eid, midev->eid_stats.active, 257) {
			u8 *base = (u8 *)&midev->eid_stats.eid[eid];
			total += *(u64 *)(base + mctp_spi_eid_stat_descs[i].offset);
		}
		data[idx++] = total;
	}

	/* Separator (blank line in output) */
	data[idx++] = 0;

	/* Output per-EID stats (only for active EIDs with non-zero stats) */
	for_each_set_bit(eid, midev->eid_stats.active, 257) {
		struct mctp_spi_eid_stats *es = &midev->eid_stats.eid[eid];
		u8 *base = (u8 *)es;
		int nz = mctp_spi_count_eid_nonzero(midev, eid);

		if (nz == 0)
			continue;

		/* EID header: value is sum of all stats for this EID */
		data[idx++] = mctp_spi_eid_stats_total(midev, eid);

		/* Individual non-zero stats for this EID */
		for (i = 0; i < MCTP_SPI_EID_NUM_STATS; i++) {
			u64 val = *(u64 *)(base + mctp_spi_eid_stat_descs[i].offset);
			if (val != 0)
				data[idx++] = val;
		}
	}
}

static const struct ethtool_ops mctp_spi_ethtool_ops = {
	.get_strings = mctp_spi_get_strings,
	.get_sset_count = mctp_spi_get_sset_count,
	.get_ethtool_stats = mctp_spi_get_ethtool_stats,
};

static const struct net_device_ops mctp_spi_ops = {
	.ndo_start_xmit = mctp_spi_net_xmit,
	.ndo_uninit = mctp_spi_ndo_uninit,
	.ndo_open = mctp_spi_ndo_open,
};

static const struct header_ops mctp_spi_headops = {
	.create = mctp_spi_header_create,
};

static void mctp_spi_net_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_SPI_MAXMTU;
	dev->min_mtu = MCTP_SPI_MINMTU;
	dev->max_mtu = MCTP_SPI_MAXMTU;
	dev->tx_queue_len = MCTP_SPI_TX_QUEUE_LEN;

	dev->hard_header_len = sizeof(struct mctp_spi_hdr);
	dev->addr_len = 0;

	dev->netdev_ops		= &mctp_spi_ops;
	dev->header_ops		= &mctp_spi_headops;
	dev->ethtool_ops	= &mctp_spi_ethtool_ops;
}

static irqreturn_t mctp_pkg_rec_wake(int irq, void *data)
{
	struct mctp_spi *midev = data;
	spin_lock(&midev->gpio_intr_cond_lock);
	midev->gpio_intr_cond = true;
	/* GPIO interrupts have no EID context */
	MCTP_STAT_INC(midev, MCTP_EID_UNKNOWN, gpio_interrupts);
	spin_unlock(&midev->gpio_intr_cond_lock);

	wake_up(&midev->main_thread_wq); // Wake up the consumer waiting on the condition
	return IRQ_HANDLED;
}

ssize_t spi_raw_transfer(struct spi_device *spi, const char *txbuf, size_t txlen,
								const char *rxbuf, size_t rxlen, bool cs_change)
{
	ssize_t	status;
	// unsigned long missing; unused variable
	struct spi_message	msg;

	struct spi_transfer	xfer = {
			.tx_buf		= txbuf,
			.rx_buf		= rxbuf,
			.len		= txlen + rxlen,
			.delay.value = rxlen,
			.bits_per_word = 8,
			.cs_change = cs_change,
			.speed_hz = 1000000,
		};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	status = spi_sync(spi, &msg);
	return status;
}

static bool is_gpio_interrupt(struct mctp_spi *midev)
{
    bool tmp;
	unsigned long flags;
    spin_lock_irqsave(&midev->gpio_intr_cond_lock, flags);
    tmp = midev->gpio_intr_cond;
    spin_unlock_irqrestore(&midev->gpio_intr_cond_lock, flags);
    return tmp;
}

static bool is_skb_queue_empty(struct mctp_spi *midev)
{
    bool tmp;
	spin_lock(&midev->tx_queue.lock);
    tmp = skb_queue_empty(&midev->tx_queue);
	spin_unlock(&midev->tx_queue.lock);
    return tmp;
}

static int mctp_spi_tx_thread(void *data)
{
	struct mctp_spi *midev = data;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned char txbuf[BUFSIZE];
	bool gpio_wake = false;
	SpbApStatus status = 0;

	spb_ap_initialize(midev->ap);

	for (;;) {
		if (kthread_should_stop())
			break;
		
		if(gpio_wake) {
			gpio_wake = false;
			spb_ap_on_interrupt(midev->ap);
			while (midev->ap->msgs_available > 0)
				mctp_spi_rx(midev);
		}

		if (skb) {
			int injected_status = 0;
			/* Extract destination EID for per-EID tracking */
			struct mctp_hdr *mh = (void *)(skb->data + sizeof(struct mctp_spi_hdr));
			u8 dest_eid = mh->dest;

			skb_copy_bits(skb, 0, txbuf, skb->len);

			/* ERROR INJECTION POINT: Check if we should inject error */
			if (mctp_spi_error_inject_tx(midev, skb, &injected_status)) {
				/* Inject the configured SPB_AP error */
				status = injected_status;
			} else {
				//Send SPI package
				status = spb_ap_send(midev->ap, skb->len, txbuf);
			}
			
	if(status == SPB_AP_OK) {
		midev->ndev->stats.tx_packets++;
		midev->ndev->stats.tx_bytes += skb->len;
		netdev_dbg(midev->ndev, "MCTP SPI: TX success, %u bytes\n", skb->len);
		trace_mctp_transport_tx("spi", midev->ndev, 0, skb->len);
	} else {
		int err = spb_ap_status_to_errno(status);
		
		midev->ndev->stats.tx_dropped++;
		/* SPB AP only returns: TIMEOUT, INVALID_ARGUMENT, or UNKNOWN */
		if (err == -ETIMEDOUT) {
			MCTP_STAT_INC(midev, dest_eid, tx_drop_etimedout);
		} else if (err == -EINVAL) {
			MCTP_STAT_INC(midev, dest_eid, tx_drop_einval);
		} else if (err == -EIO) {
			MCTP_STAT_INC(midev, dest_eid, tx_drop_eio);
		} else {
			/* Catch-all for any unmapped errors */
			MCTP_STAT_INC(midev, dest_eid, tx_drop_spi_error);
		}
		netdev_dbg(midev->ndev, "MCTP SPI: TX failed, SPB status=%d, errno=%d\n", status, err);
		trace_mctp_transport_error("spi", midev->ndev, "spb_ap_send_failed", err);

			/* TX ERROR: Report to application via error queue */
			struct sock *sk;
			struct mctp_sk_key *key = NULL;
			int error_code;

			/* Map SPB_AP status codes to Linux errno */
			switch (status) {
			case SPB_AP_ERROR_TIMEOUT:
				error_code = ETIMEDOUT;
				break;
			case SPB_AP_ERROR_INVALID_ARGUMENT:
				error_code = EINVAL;
				break;
			case SPB_AP_ERROR_UNKNOWN:
			default:
				error_code = EIO;
				break;
			}
			
			/* Remove SPI header to expose MCTP header for socket lookup.
				* Since we're going to free the SKB anyway, we can modify it directly.
				* This preserves skb->sk naturally without needing to clone.
				*/
			if (skb->len > sizeof(struct mctp_spi_hdr)) {
				skb_pull(skb, sizeof(struct mctp_spi_hdr));
				skb_reset_network_header(skb);
				
				/* Lookup socket and report error */
				sk = mctp_lookup_sock_for_error(skb, midev->ndev, NULL, &key);
				if (sk) {
					mctp_queue_error(sk, skb, error_code, midev->ndev,
							MCTP_DIR_TX, MCTP_PHYS_BINDING_SERIAL, key);
					sock_put(sk);
				}
			}		
		}
			kfree_skb(skb);
			while (midev->ap->msgs_available > 0) {
				status = mctp_spi_rx(midev);
				if (status == ERR_SPI_RX_NO_DATA)
					break;
			}
			midev->ap->msgs_available = 0;
		}

		wait_event_idle(midev->main_thread_wq,
				!is_skb_queue_empty(midev) ||
				kthread_should_stop() || is_gpio_interrupt(midev));
		// Pop skb if any
		spin_lock(&midev->tx_queue.lock);
		skb = __skb_dequeue(&midev->tx_queue);
		if (netif_queue_stopped(midev->ndev))
			netif_wake_queue(midev->ndev);
		spin_unlock(&midev->tx_queue.lock);
		// Check if this is GPIO interrupt wake
		spin_lock_irqsave(&midev->gpio_intr_cond_lock, flags);
		gpio_wake = midev->gpio_intr_cond;
		midev->gpio_intr_cond = false;
		spin_unlock_irqrestore(&midev->gpio_intr_cond_lock, flags);
	}

	return 0;
}

static int mctp_spi_probe(struct spi_device *spi)
{
	int (*match)(struct device *dev);
	struct spidev_data	*spidev;
	SpbAp *ap;
	int	status, idx, rc;
    struct net_device *ndev;
    struct mctp_spi *mctp_spi_dev;
    char name[32];
	unsigned long flags;

	match = device_get_match_data(&spi->dev);
	if (match) {
		status = match(&spi->dev);
		if (status)
			return status;
	}

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	spidev->speed_hz = spi->max_speed_hz;

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

    idx = ida_alloc(&mctp_spi_ida, GFP_KERNEL);
	if (idx < 0)
		return idx;

	snprintf(name, sizeof(name), "mctpspi%u_%u", spi->controller->bus_num,
		 spi_get_chipselect(spi, 0));
	ndev = alloc_netdev(sizeof(*mctp_spi_dev), name, NET_NAME_ENUM,
			    mctp_spi_net_setup);
	if (!ndev) {
		rc = -ENOMEM;
		goto free_ida;
	}
	mctp_spi_dev = netdev_priv(ndev);
	mctp_spi_dev->idx = idx;
	spi_set_drvdata(spi, ndev);

	mctp_spi_dev->rx_alert = devm_gpiod_get(&spi->dev, "alert", GPIOD_IN);

        SET_NETDEV_DEV(ndev, &spi->dev);
	if(IS_ERR(mctp_spi_dev->rx_alert)) {
		goto err_netdev;
	}

	mctp_spi_dev->rx_alert_irq = gpiod_to_irq(mctp_spi_dev->rx_alert);
	if (mctp_spi_dev->rx_alert_irq < 0) {
        rc = mctp_spi_dev->rx_alert_irq;
		goto err_netdev;
    }

	/*Init GPIO Interrupt*/
	spin_lock_init(&mctp_spi_dev->gpio_intr_cond_lock);
	init_waitqueue_head(&mctp_spi_dev->gpio_intr_wq);
	mctp_spi_dev->gpio_intr_cond = 0;

	rc = request_irq(mctp_spi_dev->rx_alert_irq, mctp_pkg_rec_wake,
			       IRQF_TRIGGER_FALLING, "mctp_spi_pkg_rec_wake", mctp_spi_dev);

	if (rc < 0)
		goto err_netdev;

	spin_lock_init(&mctp_spi_dev->lock);
	init_waitqueue_head(&mctp_spi_dev->main_thread_wq);
	skb_queue_head_init(&mctp_spi_dev->tx_queue);
	rc = mctp_register_netdev(ndev, NULL, MCTP_PHYS_BINDING_SERIAL);
	if (rc)
		goto err_netdev;
	init_completion(&mctp_spi_dev->rx_done);
	complete(&mctp_spi_dev->rx_done);
	spin_lock_irqsave(&mctp_spi_dev->lock, flags);
	mctp_spi_dev->allow_rx = false;
	spin_unlock_irqrestore(&mctp_spi_dev->lock, flags);
	mctp_spi_dev->ndev = ndev;
	mctp_spi_dev->spidev = spidev;
	mctp_spi_dev->tx_thread = kthread_create(mctp_spi_tx_thread, mctp_spi_dev,
					  "%s/tx", ndev->name);
	if (IS_ERR(mctp_spi_dev->tx_thread))
		return PTR_ERR(mctp_spi_dev->tx_thread);

	/* Init SPB */
	ap = kzalloc(sizeof(*ap), GFP_KERNEL);
	ap->spi_xfer = spi_raw_transfer;
	ap->spi = spi;
	ap->gpio_intr_wq = &mctp_spi_dev->main_thread_wq;
	ap->gpio_intr_cond = &mctp_spi_dev->gpio_intr_cond;
	ap->gpio_intr_cond_lock = &mctp_spi_dev->gpio_intr_cond_lock;
	mctp_spi_dev->ap = ap;

	/* Setup error injection after netdev registration (debugfs needs the netdev name) */
	mctp_spi_error_inject_init(mctp_spi_dev);

	/* Start the mctp tx worker thread */
	wake_up_process(mctp_spi_dev->tx_thread);
	return status;
err_netdev:
    free_netdev(ndev);
free_ida:
    ida_free(&mctp_spi_ida, idx);
    return rc;
}

static void mctp_spi_remove(struct spi_device *spi)
{
	struct net_device *ndev = spi_get_drvdata(spi);
	struct mctp_spi *midev = netdev_priv(ndev);
	struct spidev_data *spidev = midev->spidev;
	unsigned long flags;

	/* Stop feeding the rx path, then stop the tx/rx worker thread.
	 * The thread uses netif_*() and the SPB AP context, so it must be
	 * gone before we unregister the netdev or free midev->ap.
	 */
	spin_lock_irqsave(&midev->lock, flags);
	midev->allow_rx = false;
	spin_unlock_irqrestore(&midev->lock, flags);

	if (midev->tx_thread) {
		kthread_stop(midev->tx_thread);
		midev->tx_thread = NULL;
	}

	/* The alert IRQ handler touches midev; release it before midev goes
	 * away with free_netdev().
	 */
	free_irq(midev->rx_alert_irq, midev);
	mctp_spi_error_inject_cleanup(midev);

	/* Drop the MCTP core's reference and unregister */
	mctp_unregister_netdev(ndev);

	skb_queue_purge(&midev->tx_queue);
	kfree(midev->ap);
	ida_free(&mctp_spi_ida, midev->idx);
	free_netdev(ndev);
	kfree(spidev);
}

static struct spi_driver mctp_spi_driver = {
	.driver = {
		.name =		"mctpspi",
		.of_match_table = mctp_spi_dt_ids,
	},
	.probe =	mctp_spi_probe,
	.remove =	mctp_spi_remove,
	.id_table =	mctp_spi_ids,
};

/*-------------------------------------------------------------------------*/

static int __init mctp_spi_mod_init(void)
{
	int rc;
	
	pr_info("MCTP SPI interface driver\n");
	
	/* Initialize error injection infrastructure */
	rc = mctp_spi_error_inject_module_init();
	if (rc)
		pr_warn("MCTP SPI: Error injection initialization failed, continuing without it\n");
	
	rc = spi_register_driver(&mctp_spi_driver);
	if (rc) {
		mctp_spi_error_inject_module_exit();
		return rc;
	}
	
	return 0;
}
module_init(mctp_spi_mod_init);

static void __exit mctp_spi_mod_exit(void)
{
	spi_unregister_driver(&mctp_spi_driver);
	mctp_spi_error_inject_module_exit();
}
module_exit(mctp_spi_mod_exit);

MODULE_LICENSE("GPL");
