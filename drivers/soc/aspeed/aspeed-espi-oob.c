// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Aspeed Technology Inc.
 */
#include <linux/aspeed-espi-ioc.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/regmap.h>
#include <linux/vmalloc.h>

#include "aspeed-espi-ctrl.h"
#include "aspeed-espi-oob.h"

#define OOB_MDEV_NAME	"aspeed-espi-oob"

#define OOB_DMA_DESC_MAX_NUM		1024
#define OOB_DMA_TX_DESC_CUST		0x04

#define OOB_DEFAULT_RX_PACKET_LEN	0x1000

/*
 * Descriptor-based RX DMA handling
 */
static long aspeed_espi_oob_dma_desc_get_rx(struct file *fp,
					    struct aspeed_espi_ioc *ioc,
					    struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;
	int rc = 0;
	struct espi_comm_hdr *hdr;
	struct oob_rx_dma_desc *d;
	unsigned long flags;
	u32 wptr, sptr;
	u32 pkt_len;
	u32 reg;
	u8 *pkt;

	regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA_WS_PTR, &reg);
	wptr = (reg & ASPEED_ESPI_OOB_RX_DMA_WS_PTR_WP_MASK) >>
		ASPEED_ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT;
	sptr = (reg & ASPEED_ESPI_OOB_RX_DMA_WS_PTR_SP_MASK) >>
		ASPEED_ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT;
	d = &espi_oob->dma.rx_desc[sptr];
	if (!d->dirty)
		return -EFAULT;

	pkt_len = ((d->len) ? : OOB_DEFAULT_RX_PACKET_LEN) + sizeof(struct espi_comm_hdr);
	if (ioc->pkt_len < pkt_len)
		return -EINVAL;

	pkt = vmalloc(pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = d->cyc;
	hdr->tag = d->tag;
	hdr->len_h = ESPI_LEN_HIGH(d->len);
	hdr->len_l = ESPI_LEN_LOW(d->len);
	memcpy(hdr + 1, espi_oob->dma.rx_virt + (PAGE_SIZE * sptr), pkt_len - sizeof(*hdr));
	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/* make current descriptor available again */
	d->dirty = 0;
	sptr = (sptr + 1) % espi_oob->dma.rx_desc_num;
	wptr = (wptr + 1) % espi_oob->dma.rx_desc_num;
	reg = ASPEED_ESPI_OOB_RX_DMA_WS_PTR_RECV_EN;
	reg |= (wptr << ASPEED_ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT) &
	       ASPEED_ESPI_OOB_RX_DMA_WS_PTR_WP_MASK;
	reg |= (sptr << ASPEED_ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT) &
	       ASPEED_ESPI_OOB_RX_DMA_WS_PTR_SP_MASK;

	regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA_WS_PTR, reg);
	spin_lock_irqsave(&espi_oob->lock, flags);
	espi_oob->rx_ready = espi_oob->dma.rx_desc[sptr].dirty;
	spin_unlock_irqrestore(&espi_oob->lock, flags);

free_n_out:
	vfree(pkt);
	return rc;
}

static long aspeed_espi_oob_get_rx(struct file *fp,
				   struct aspeed_espi_ioc *ioc,
				   struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;
	int i, rc = 0;
	struct espi_comm_hdr *hdr;
	unsigned long flags;
	u32 cyc, tag, len;
	u32 pkt_len;
	u32 reg;
	u8 *pkt;

	if (fp->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&espi_oob->get_rx_mtx))
			return -EBUSY;

		if (!espi_oob->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	} else {
		mutex_lock(&espi_oob->get_rx_mtx);
		if (!espi_oob->rx_ready) {
			rc = wait_event_interruptible(espi_oob->wq, espi_oob->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}
	if (espi_oob->dma_mode && espi_ctrl->model->version != ASPEED_ESPI_AST2500) {
		rc = aspeed_espi_oob_dma_desc_get_rx(fp, ioc, espi_oob);
		goto unlock_mtx_n_out;
	}

	regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_RX_CTRL, &reg);
	cyc = (reg & ASPEED_ESPI_OOB_RX_CTRL_CYC_MASK) >> ASPEED_ESPI_OOB_RX_CTRL_CYC_SHIFT;
	tag = (reg & ASPEED_ESPI_OOB_RX_CTRL_TAG_MASK) >> ASPEED_ESPI_OOB_RX_CTRL_TAG_SHIFT;
	len = (reg & ASPEED_ESPI_OOB_RX_CTRL_LEN_MASK) >> ASPEED_ESPI_OOB_RX_CTRL_LEN_SHIFT;

	/*
	 * Calculate the length of the rest part of the eSPI packet to be read from HW
	 * and copied to user space.
	 */
	pkt_len = (len ?: ASPEED_ESPI_PLD_LEN_MAX) + sizeof(*hdr);
	if (ioc->pkt_len < pkt_len) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = ESPI_LEN_HIGH(len);
	hdr->len_l = ESPI_LEN_LOW(len);
	if (espi_oob->dma_mode) {
		memcpy(hdr + 1, espi_oob->dma.rx_virt, pkt_len - sizeof(*hdr));
	} else {
		for (i = sizeof(*hdr); i < pkt_len; ++i) {
			regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_RX_PORT, &reg);
			pkt[i] = reg & 0xff;
		}
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	regmap_write_bits(espi_ctrl->map, ASPEED_ESPI_OOB_RX_CTRL,
			  ASPEED_ESPI_OOB_RX_CTRL_PEND_SERV,
			  ASPEED_ESPI_OOB_RX_CTRL_PEND_SERV);

	spin_lock_irqsave(&espi_oob->lock, flags);
	espi_oob->rx_ready = 0;
	spin_unlock_irqrestore(&espi_oob->lock, flags);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_oob->get_rx_mtx);
	return rc;
}

static long aspeed_espi_oob_dma_desc_put_tx(struct file *fp,
					    struct aspeed_espi_ioc *ioc,
					    struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;
	int rc = 0;
	struct espi_comm_hdr *hdr;
	struct oob_tx_dma_desc *d;
	u32 rptr, wptr;
	u8 *pkt;

	pkt = vzalloc(ioc->pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;
	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA_RD_PTR,
		     ASPEED_ESPI_OOB_TX_DMA_RD_PTR_UPDATE);
	regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA_RD_PTR, &rptr);
	regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA_WR_PTR, &wptr);
	if (((wptr + 1) % espi_oob->dma.tx_desc_num) == rptr) {
		rc = -EBUSY;
		goto free_n_out;
	}

	d = &espi_oob->dma.tx_desc[wptr];
	d->cyc = hdr->cyc;
	d->tag = hdr->tag;
	d->len = ESPI_LEN(hdr->len_h, hdr->len_l);
	d->msg_type = OOB_DMA_TX_DESC_CUST;
	memcpy(espi_oob->dma.tx_virt + (PAGE_SIZE * wptr), hdr + 1, ioc->pkt_len - sizeof(*hdr));

	wptr = (wptr + 1) % espi_oob->dma.tx_desc_num;
	wptr |= ASPEED_ESPI_OOB_TX_DMA_WR_PTR_SEND_EN;
	regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA_WR_PTR, wptr);

free_n_out:
	vfree(pkt);
	return rc;
}

static long aspeed_espi_oob_put_tx(struct file *fp,
				   struct aspeed_espi_ioc *ioc,
				   struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;
	int i, rc = 0;
	struct espi_comm_hdr *hdr;
	u32 reg, len;
	u8 *pkt;

	if (!mutex_trylock(&espi_oob->put_tx_mtx))
		return -EBUSY;

	if (espi_oob->dma_mode && espi_ctrl->model->version != ASPEED_ESPI_AST2500) {
		rc = aspeed_espi_oob_dma_desc_put_tx(fp, ioc, espi_oob);
		goto unlock_mtx_n_out;
	}

	regmap_read(espi_ctrl->map, ASPEED_ESPI_OOB_TX_CTRL, &reg);
	if (reg & ASPEED_ESPI_OOB_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len > ASPEED_ESPI_PKT_LEN_MAX) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	if (espi_oob->dma_mode) {
		memcpy(espi_oob->dma.tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
	} else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_PORT, pkt[i]);
	}

	len = ESPI_LEN(hdr->len_h, hdr->len_l);
	reg = (hdr->cyc << ASPEED_ESPI_OOB_TX_CTRL_CYC_SHIFT) & ASPEED_ESPI_OOB_TX_CTRL_CYC_MASK;
	reg |= (hdr->tag << ASPEED_ESPI_OOB_TX_CTRL_TAG_SHIFT) & ASPEED_ESPI_OOB_TX_CTRL_TAG_MASK;
	reg |= (len << ASPEED_ESPI_OOB_TX_CTRL_LEN_SHIFT) & ASPEED_ESPI_OOB_TX_CTRL_LEN_MASK;
	reg |= ASPEED_ESPI_OOB_TX_CTRL_TRIGGER;

	regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_CTRL, reg);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_oob->put_tx_mtx);
	return rc;
}

static long aspeed_espi_oob_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_oob *espi_oob = container_of(fp->private_data,
							struct aspeed_espi_oob,
							mdev);
	struct aspeed_espi_ioc ioc;

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
		return -EFAULT;

	if (ioc.pkt_len > ASPEED_ESPI_PKT_LEN_MAX)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_OOB_GET_RX:
		return aspeed_espi_oob_get_rx(fp, &ioc, espi_oob);
	case ASPEED_ESPI_OOB_PUT_TX:
		return aspeed_espi_oob_put_tx(fp, &ioc, espi_oob);
	};

	return -ENOTTY;
}

void aspeed_espi_oob_event(u32 sts, struct aspeed_espi_oob *espi_oob)
{
	unsigned long flags;

	if (sts & ASPEED_ESPI_INT_STS_OOB_RX_CMPLT) {
		spin_lock_irqsave(&espi_oob->lock, flags);
		espi_oob->rx_ready = 1;
		spin_unlock_irqrestore(&espi_oob->lock, flags);
		wake_up_interruptible(&espi_oob->wq);
	}
}

static void aspeed_espi_oob_dma_init(struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;
	struct aspeed_espi_oob_dma *dma = &espi_oob->dma;
	int i;

	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL,
			   ASPEED_ESPI_CTRL_OOB_TX_DMA_EN | ASPEED_ESPI_CTRL_OOB_RX_DMA_EN,
			   ASPEED_ESPI_CTRL_OOB_TX_DMA_EN | ASPEED_ESPI_CTRL_OOB_RX_DMA_EN);

	if (espi_ctrl->model->version == ASPEED_ESPI_AST2500) {
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA, dma->tx_addr);
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA, dma->rx_addr);
	} else {
		for (i = 0; i < dma->tx_desc_num; ++i)
			dma->tx_desc[i].data_addr = dma->tx_addr + (i * PAGE_SIZE);

		for (i = 0; i < dma->rx_desc_num; ++i) {
			dma->rx_desc[i].data_addr = dma->rx_addr + (i * PAGE_SIZE);
			dma->rx_desc[i].dirty = 0;
		}
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA, dma->tx_desc_addr);
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_TX_DMA_RB_SIZE,
			     dma->tx_desc_num);
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA, dma->rx_desc_addr);
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA_RB_SIZE,
			     dma->rx_desc_num);
		regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_OOB_RX_DMA_WS_PTR,
				   ASPEED_ESPI_OOB_RX_DMA_WS_PTR_RECV_EN,
				   ASPEED_ESPI_OOB_RX_DMA_WS_PTR_RECV_EN);
	}
}

void aspeed_espi_oob_enable(struct aspeed_espi_oob *espi_oob)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_oob->ctrl;

	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL,
			   ASPEED_ESPI_CTRL_OOB_SW_RDY | ASPEED_ESPI_CTRL_OOB_RX_SW_RST, 0);

	if (espi_oob->dma_mode)
		regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL,
				   ASPEED_ESPI_CTRL_OOB_TX_DMA_EN | ASPEED_ESPI_CTRL_OOB_RX_DMA_EN,
				   0);
	else
		regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_CTRL,
			     ASPEED_ESPI_OOB_RX_CTRL_PEND_SERV);

	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL, ASPEED_ESPI_CTRL_OOB_RX_SW_RST,
			   ASPEED_ESPI_CTRL_OOB_RX_SW_RST);
	regmap_write(espi_ctrl->map, ASPEED_ESPI_OOB_RX_CTRL, ASPEED_ESPI_OOB_RX_CTRL_PEND_SERV);

	if (espi_oob->dma_mode)
		aspeed_espi_oob_dma_init(espi_oob);

	regmap_write(espi_ctrl->map, ASPEED_ESPI_INT_STS, ASPEED_ESPI_INT_STS_OOB_BITS);
	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_INT_EN,
			   ASPEED_ESPI_INT_EN_OOB_BITS,
			   ASPEED_ESPI_INT_EN_OOB_BITS);
	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL,
			   ASPEED_ESPI_CTRL_OOB_SW_RDY,
			   ASPEED_ESPI_CTRL_OOB_SW_RDY);
}

static const struct file_operations aspeed_espi_oob_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_oob_ioctl,
};

static int aspeed_espi_oob_dma_alloc(struct device *dev, struct aspeed_espi_oob *espi_oob,
				     u32 version)
{
	struct aspeed_espi_oob_dma *dma = &espi_oob->dma;

	if (version != ASPEED_ESPI_AST2500) {
		of_property_read_u32(dev->of_node, "oob,dma-tx-desc-num", &dma->tx_desc_num);
		of_property_read_u32(dev->of_node, "oob,dma-rx-desc-num", &dma->rx_desc_num);

		if (!dma->tx_desc_num || !dma->rx_desc_num) {
			dev_err(dev, "invalid zero number of DMA channels\n");
			return -EINVAL;
		}

		if (dma->tx_desc_num >= OOB_DMA_DESC_MAX_NUM ||
		    dma->rx_desc_num >= OOB_DMA_DESC_MAX_NUM) {
			dev_err(dev, "too many number of DMA channels\n");
			return -EINVAL;
		}

		dma->tx_desc = dmam_alloc_coherent(dev, sizeof(*dma->tx_desc) * dma->tx_desc_num,
						   &dma->tx_desc_addr, GFP_KERNEL);
		if (!dma->tx_desc) {
			dev_err(dev, "cannot allocate DMA TX descriptor\n");
			return -ENOMEM;
		}

		dma->rx_desc = dmam_alloc_coherent(dev, sizeof(*dma->rx_desc) * dma->rx_desc_num,
						   &dma->rx_desc_addr, GFP_KERNEL);
		if (!dma->rx_desc) {
			dev_err(dev, "cannot allocate DMA RX descriptor\n");
			return -ENOMEM;
		}
	}
	/*
	 * DMA descriptors are consumed in the circular queue paradigm.
	 * Therefore, one dummy slot is reserved to detect the full
	 * condition. For AST2500 without DMA descriptors supported,
	 * the number of the queue slot should be 1 here.
	 */
	dma->tx_desc_num += 1;
	dma->rx_desc_num += 1;
	dma->tx_virt = dmam_alloc_coherent(dev, PAGE_SIZE * dma->tx_desc_num, &dma->tx_addr,
					   GFP_KERNEL);
	if (!dma->tx_virt)
		return -ENOMEM;

	dma->rx_virt = dmam_alloc_coherent(dev, PAGE_SIZE * dma->rx_desc_num, &dma->rx_addr,
					   GFP_KERNEL);
	if (!dma->rx_virt)
		return -ENOMEM;
	return 0;
}

void *aspeed_espi_oob_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl)
{
	struct aspeed_espi_oob *espi_oob =
		devm_kzalloc(dev, sizeof(struct aspeed_espi_oob), GFP_KERNEL);
	int rc = 0;

	if (!espi_oob)
		return ERR_PTR(-ENOMEM);

	espi_oob->ctrl = espi_ctrl;
	init_waitqueue_head(&espi_oob->wq);
	spin_lock_init(&espi_oob->lock);
	mutex_init(&espi_oob->put_tx_mtx);
	mutex_init(&espi_oob->get_rx_mtx);
	if (of_property_read_bool(dev->of_node, "oob,dma-mode")) {
		rc = aspeed_espi_oob_dma_alloc(dev, espi_oob, espi_ctrl->model->version);
		if (rc)
			return ERR_PTR(rc);
		espi_oob->dma_mode = 1;
	}

	espi_oob->mdev.parent = dev;
	espi_oob->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_oob->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", OOB_MDEV_NAME);
	espi_oob->mdev.fops = &aspeed_espi_oob_fops;
	rc = misc_register(&espi_oob->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return ERR_PTR(rc);
	}

	aspeed_espi_oob_enable(espi_oob);
	return espi_oob;
}

void aspeed_espi_oob_free(struct device *dev, struct aspeed_espi_oob *espi_oob)
{
	mutex_destroy(&espi_oob->put_tx_mtx);
	mutex_destroy(&espi_oob->get_rx_mtx);
	misc_deregister(&espi_oob->mdev);
}
