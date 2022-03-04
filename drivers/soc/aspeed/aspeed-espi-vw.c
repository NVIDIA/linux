// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 ASPEED Technology Inc.
 */
#include <linux/aspeed-espi-ioc.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>

#include "aspeed-espi-ctrl.h"
#include "aspeed-espi-vw.h"

#define VW_MDEV_NAME	"aspeed-espi-vw"

static long aspeed_espi_vw_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_vw *espi_vw = container_of(fp->private_data,
						      struct aspeed_espi_vw,
						      mdev);
	struct aspeed_espi_ctrl *espi_ctrl = espi_vw->ctrl;
	u32 val = 0;

	switch (cmd) {
	case ASPEED_ESPI_VW_GET_GPIO_VAL:
		regmap_read(espi_ctrl->map, ASPEED_ESPI_VW_GPIO_VAL, &val);
		if (put_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		break;

	case ASPEED_ESPI_VW_PUT_GPIO_VAL:
		if (get_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		regmap_write(espi_ctrl->map, ASPEED_ESPI_VW_GPIO_VAL, val);
		break;

	default:
		return -ENOTTY;
	};

	return 0;
}

void aspeed_espi_vw_enable(struct aspeed_espi_vw *espi_vw)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_vw->ctrl;

	regmap_write(espi_ctrl->map, ASPEED_ESPI_INT_STS,
		     ASPEED_ESPI_INT_STS_VW_BITS);

	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_INT_EN,
			   ASPEED_ESPI_INT_EN_VW_BITS,
			   ASPEED_ESPI_INT_EN_VW_BITS);

	regmap_update_bits(espi_ctrl->map, ASPEED_ESPI_CTRL,
			   ASPEED_ESPI_CTRL_VW_SW_RDY,
			   ASPEED_ESPI_CTRL_VW_SW_RDY);
}

static const struct file_operations aspeed_espi_vw_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_vw_ioctl,
};

void *aspeed_espi_vw_init(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl)
{
	int rc;
	struct aspeed_espi_vw *espi_vw;

	espi_vw = devm_kzalloc(dev, sizeof(*espi_vw), GFP_KERNEL);
	if (!espi_vw)
		return ERR_PTR(-ENOMEM);

	espi_vw->ctrl = espi_ctrl;

	espi_vw->mdev.parent = dev;
	espi_vw->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_vw->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", VW_MDEV_NAME);
	espi_vw->mdev.fops = &aspeed_espi_vw_fops;
	rc = misc_register(&espi_vw->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return ERR_PTR(rc);
	}

	aspeed_espi_vw_enable(espi_vw);

	return espi_vw;
}

void aspeed_espi_vw_fini(struct device *dev, struct aspeed_espi_vw *espi_vw)
{
	misc_deregister(&espi_vw->mdev);
}
