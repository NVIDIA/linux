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

#include <linux/if_arp.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>
#include <net/mctpdevice.h>

#include "glacier-spb-ap.h"

static DEFINE_IDA(mctp_spi_ida);

#define MCTP_SPI_MAXMTU (64 + 4)
#define MCTP_SPI_MINMTU (64 + 4)
#define MCTP_SPI_TX_QUEUE_LEN 1100
#define BUFSIZE			256
#define MCTP_SPI_TX_WORK_LEN 100
#define MCTP_COMMAND_CODE 0x02

#define RX_BUFFER_SIZE 1024

#define ERR_SPI_RX_NO_DATA -2

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	size_t	tx_len;
	u8			rx_buffer[RX_BUFFER_SIZE];
	size_t rx_len;
	u32			speed_hz;
};

struct mctp_spi {
	struct net_device	*ndev;
	struct spidev_data *spidev;

	struct task_struct *tx_thread;
	wait_queue_head_t main_thread_wq;
	struct sk_buff_head tx_queue;
	spinlock_t lock;
	bool allow_rx;
	struct completion rx_done;

	struct gpio_desc *rx_alert; //Input gpio to alert about the incoming package from SPI
	int	rx_alert_irq;

	SpbAp *ap;
	wait_queue_head_t gpio_intr_wq;
	bool gpio_intr_cond;
	spinlock_t gpio_intr_cond_lock;
};

struct mctp_spi_hdr {
	u8 command_code;
	u8 byte_count;
	u8 resrv[2];
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

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

static int mctp_spi_net_recv(struct mctp_spi *midev)
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
		return -ENOMEM;
	}
	skb->protocol = htons(ETH_P_MCTP);
	skb_put_data(skb, midev->spidev->rx_buffer, recvlen);
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
	} else {
		ndev->stats.rx_dropped++;
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
	if(status != SPB_AP_OK)
		return ERR_SPI_RX_NO_DATA;

	payload_len = spi_hdr_rx->byte_count;
	len = payload_len + hdr_size;

	midev->spidev->rx_len = payload_len;
	memcpy(midev->spidev->rx_buffer, tmp_rx_buffer + hdr_size, payload_len);
	mctp_spi_net_recv(midev);

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
	dev->addr_len = 1;

	dev->netdev_ops		= &mctp_spi_ops;
	dev->header_ops		= &mctp_spi_headops;
}

static irqreturn_t mctp_pkg_rec_wake(int irq, void *data)
{
	struct mctp_spi *midev = data;
	spin_lock(&midev->gpio_intr_cond_lock);
	midev->gpio_intr_cond = true;
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
			skb_copy_bits(skb, 0, txbuf, skb->len);
			//Send SPI package
			status = spb_ap_send(midev->ap, skb->len, txbuf);
			if(status == SPB_AP_OK) {
				midev->ndev->stats.tx_packets++;
				midev->ndev->stats.tx_bytes += skb->len;
			}
			else
				midev->ndev->stats.rx_dropped++;
			kfree_skb(skb);
			if (spb_ap_msgs_available(midev->ap) > 0) {
				/* We can't rely on msg_avaiable counts to issues read transactions
				* becasue we only have one mailbox status which can't tell how many
				* response pending in glaicer sides. The solution is to read from
				* glacier and read all pending responses once.
				*/
				while (1) {
					status = mctp_spi_rx(midev);
					if (status == ERR_SPI_RX_NO_DATA)
						break;
				}
				midev->ap->msgs_available = 0;
			}
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

	snprintf(name, sizeof(name), "mctpspi%d", idx);
	ndev = alloc_netdev(sizeof(*mctp_spi_dev), name, NET_NAME_ENUM,
			    mctp_spi_net_setup);
	if (!ndev) {
		rc = -ENOMEM;
		goto free_ida;
	}
	mctp_spi_dev = netdev_priv(ndev);

	mctp_spi_dev->rx_alert = devm_gpiod_get(&spi->dev, "alert", GPIOD_IN);

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
	rc = mctp_register_netdev(ndev, NULL);
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
		return ERR_CAST(mctp_spi_dev->tx_thread); //warning: returning 'void *' from a function with return type 'int'

	/* Init SPB */
	ap = kzalloc(sizeof(*ap), GFP_KERNEL);
	ap->spi_xfer = spi_raw_transfer;
	ap->spi = spi;
	ap->gpio_intr_wq = &mctp_spi_dev->main_thread_wq;
	ap->gpio_intr_cond = &mctp_spi_dev->gpio_intr_cond;
	ap->gpio_intr_cond_lock = &mctp_spi_dev->gpio_intr_cond_lock;
	mctp_spi_dev->ap = ap;

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
	struct spidev_data	*spidev = spi_get_drvdata(spi);
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	list_del(&spidev->device_entry);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);
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
	return spi_register_driver(&mctp_spi_driver);
}
module_init(mctp_spi_mod_init);

static void __exit mctp_spi_mod_exit(void)
{
	spi_unregister_driver(&mctp_spi_driver);
}
module_exit(mctp_spi_mod_exit);

MODULE_LICENSE("GPL");
