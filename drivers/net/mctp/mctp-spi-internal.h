/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP SPI Internal Definitions
 * Shared between mctp-spi.c and mctp-spi-error-inject.c
 */

#ifndef __MCTP_SPI_INTERNAL_H
#define __MCTP_SPI_INTERNAL_H

#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/gpio/consumer.h>

#include "mctp-spi-error-inject.h"
#include "glacier-spb-ap.h"

#define RX_BUFFER_SIZE 1024

/* SPI device data structure */
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

/* Main device structure - one per SPI device */
struct mctp_spi {
	struct net_device	*ndev;
	struct spidev_data *spidev;

	struct task_struct *tx_thread;
	wait_queue_head_t main_thread_wq;
	struct sk_buff_head tx_queue;
	spinlock_t lock;
	bool allow_rx;
	struct completion rx_done;

	struct gpio_desc *rx_alert; /* Input gpio to alert about the incoming package from SPI */
	int	rx_alert_irq;
	int	idx;		/* mctp_spi_ida index, freed on remove */

	SpbAp *ap;
	wait_queue_head_t gpio_intr_wq;
	bool gpio_intr_cond;
	spinlock_t gpio_intr_cond_lock;

	/* Per-EID statistics tracking - SINGLE source of truth
	 *
	 * All statistics are tracked per-endpoint-ID (EID). Two special EIDs:
	 * - EID 0: "null endpoint" - valid packets with EID=0 (unallocated endpoint)
	 * - EID 256 (MCTP_EID_UNKNOWN): errors where EID could not be determined
	 *   (GPIO interrupts, SPI transfer errors, allocation failures)
	 */
	struct {
		DECLARE_BITMAP(active, 257);  /* Which EIDs have activity */
		struct mctp_spi_eid_stats {
			/* RX stats */
			u64 rx_drop_no_memory;
			u64 rx_drop_not_ready;       /* Tracked as UNKNOWN (before EID known) */
			u64 rx_drop_spi_error;       /* Tracked as UNKNOWN */
			
			/* TX stats - SPB AP specific errors */
			u64 tx_drop_etimedout;       /* SPB_AP_ERROR_TIMEOUT */
			u64 tx_drop_einval;          /* SPB_AP_ERROR_INVALID_ARGUMENT */
			u64 tx_drop_eio;             /* SPB_AP_ERROR_UNKNOWN */
			u64 tx_drop_spi_error;       /* Catch-all for unmapped errors */
			
			/* GPIO interrupt tracking (UNKNOWN - no EID context) */
			u64 gpio_interrupts;
		} eid[257];
	} eid_stats;
	
	/* Error injection support */
	struct mctp_spi_error_inject error_inject;
	struct dentry *debugfs_dir;
};

/* SPI transport header */
struct mctp_spi_hdr {
	u8 command_code;
	u8 byte_count;
	u8 resrv[2];
};

#endif /* __MCTP_SPI_INTERNAL_H */

