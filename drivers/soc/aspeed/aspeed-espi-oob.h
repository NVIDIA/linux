/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 Aspeed Technology Inc.
 */
#ifndef _ASPEED_ESPI_OOB_H_
#define _ASPEED_ESPI_OOB_H_

struct oob_tx_dma_desc {
	u32 data_addr;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 msg_type : 3;
	u8 raz0 : 1;
	u8 pec : 1;
	u8 int_en : 1;
	u8 pause : 1;
	u8 raz1 : 1;
	u32 raz2;
	u32 raz3;
} __packed;

struct oob_rx_dma_desc {
	u32 data_addr;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 raz : 7;
	u8 dirty : 1;
} __packed;

struct aspeed_espi_oob_dma {
	u32 tx_desc_num;
	u32 rx_desc_num;

	struct oob_tx_dma_desc *tx_desc;
	dma_addr_t tx_desc_addr;

	struct oob_rx_dma_desc *rx_desc;
	dma_addr_t rx_desc_addr;

	void *tx_virt;
	dma_addr_t tx_addr;

	void *rx_virt;
	dma_addr_t rx_addr;
};

struct aspeed_espi_oob {
	u32 dma_mode;
	struct aspeed_espi_oob_dma dma;

	u32 rx_ready;
	wait_queue_head_t wq;
	/* Locks rx resources and allow one receive at a time */
	struct mutex get_rx_mtx;
	/* Locks tx resources and allow one transmit at a time */
	struct mutex put_tx_mtx;
	/* Lock to synchronize receive in irq context */
	spinlock_t lock;

	struct miscdevice mdev;
	struct aspeed_espi_ctrl *ctrl;
};

void aspeed_espi_oob_event(u32 sts, struct aspeed_espi_oob *espi_oob);
void aspeed_espi_oob_enable(struct aspeed_espi_oob *espi_oob);
void *aspeed_espi_oob_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl);
void aspeed_espi_oob_free(struct device *dev, struct aspeed_espi_oob *espi_oob);

#endif
