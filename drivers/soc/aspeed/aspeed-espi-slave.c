// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2015-2019, Intel Corporation.

#include <linux/aspeed-espi-ioc.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched/signal.h>
#include <linux/soc/aspeed/aspeed-espi.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include "aspeed-espi-ctrl.h"
#include "aspeed-espi-oob.h"

struct aspeed_espi {
	struct regmap		*map;
	struct clk		*clk;
	struct device		*dev;
	int			irq;

	/* for PLTRST_N signal monitoring interface */
	struct miscdevice	pltrstn_miscdev;
	/* for SMI Interrupt monitoring interface */
	struct miscdevice	smi_miscdev;
	spinlock_t		pltrstn_lock; /* for PLTRST_N signal sampling */
	wait_queue_head_t	pltrstn_waitq;
	char			pltrstn;
	bool			pltrstn_in_avail;
	spinlock_t		smi_lock; /* for SMI signal sampling */
	wait_queue_head_t	smi_waitq;
	char			smi;
	bool			smi_is_avail;
	struct aspeed_espi_ctrl *espi_ctrl;
	aspeed_espi_irq_handler gpio_irq;
	void			*gpio_data;
};

void aspeed_espi_register_gpio(struct device *device, aspeed_espi_irq_handler handler, void *data)
{
	struct aspeed_espi *espi_ctrl = dev_get_drvdata(device);

	espi_ctrl->gpio_irq = handler;
	espi_ctrl->gpio_data = data;
}

static void aspeed_espi_sys_event(struct aspeed_espi *priv)
{
	u32 sts, evt;

	regmap_read(priv->map, ASPEED_ESPI_SYSEVT_INT_STS, &sts);
	regmap_read(priv->map, ASPEED_ESPI_SYSEVT, &evt);

	dev_dbg(priv->dev, "sys: sts = %08x, evt = %08x\n", sts, evt);

	if (!(evt & ASPEED_ESPI_SYSEVT_SLAVE_BOOT_STATUS)) {
		regmap_write(priv->map, ASPEED_ESPI_SYSEVT,
			     evt | ASPEED_ESPI_SYSEVT_SLAVE_BOOT_STATUS |
			     ASPEED_ESPI_SYSEVT_SLAVE_BOOT_DONE);
		dev_dbg(priv->dev, "Setting espi slave boot done\n");
	}
	if (sts & ASPEED_ESPI_SYSEVT_HOST_RST_WARN) {
		if (evt & ASPEED_ESPI_SYSEVT_HOST_RST_WARN)
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT,
					  ASPEED_ESPI_SYSEVT_HOST_RST_ACK,
					  ASPEED_ESPI_SYSEVT_HOST_RST_ACK);
		else
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT,
					  ASPEED_ESPI_SYSEVT_HOST_RST_ACK, 0);
		dev_dbg(priv->dev, "SYSEVT_HOST_RST_WARN: acked\n");
	}
	if (sts & ASPEED_ESPI_SYSEVT_OOB_RST_WARN) {
		if (evt & ASPEED_ESPI_SYSEVT_OOB_RST_WARN)
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT,
					  ASPEED_ESPI_SYSEVT_OOB_RST_ACK,
					  ASPEED_ESPI_SYSEVT_OOB_RST_ACK);
		else
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT,
					  ASPEED_ESPI_SYSEVT_OOB_RST_ACK, 0);
		dev_dbg(priv->dev, "SYSEVT_OOB_RST_WARN: acked\n");
	}
	if (sts & ASPEED_ESPI_SYSEVT_PLTRSTN || priv->pltrstn == 'U') {
		spin_lock(&priv->pltrstn_lock);
		priv->pltrstn = (evt & ASPEED_ESPI_SYSEVT_PLTRSTN) ? '1' : '0';
		priv->pltrstn_in_avail = true;
		spin_unlock(&priv->pltrstn_lock);
		wake_up_interruptible(&priv->pltrstn_waitq);
		dev_dbg(priv->dev, "SYSEVT_PLTRSTN: %c\n", priv->pltrstn);
	}
	if (sts & ASPEED_ESPI_SYSEVT_SMI_OUT || priv->smi == 'U') {
		spin_lock(&priv->smi_lock);
		priv->smi = (evt & ASPEED_ESPI_SYSEVT_SMI_OUT) ? '0' : '1';
		priv->smi_is_avail = true;
		spin_unlock(&priv->smi_lock);
		wake_up_interruptible(&priv->smi_waitq);
		dev_dbg(priv->dev, "SYSEVT_SMI: %c\n", priv->smi);
	}

	regmap_write(priv->map, ASPEED_ESPI_SYSEVT_INT_STS, sts);
}

static void aspeed_espi_sys_event1(struct aspeed_espi *priv)
{
	u32 sts, evt;

	regmap_read(priv->map, ASPEED_ESPI_SYSEVT1_INT_STS, &sts);
	regmap_read(priv->map, ASPEED_ESPI_SYSEVT1, &evt);

	dev_dbg(priv->dev, "sys event1: sts = %08x, evt = %08x\n", sts, evt);

	if (sts & ASPEED_ESPI_SYSEVT1_SUSPEND_WARN) {
		if  (evt & ASPEED_ESPI_SYSEVT1_SUSPEND_WARN)
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT1,
					  ASPEED_ESPI_SYSEVT1_SUSPEND_ACK,
					  ASPEED_ESPI_SYSEVT1_SUSPEND_ACK);
		else
			regmap_write_bits(priv->map, ASPEED_ESPI_SYSEVT1,
					  ASPEED_ESPI_SYSEVT1_SUSPEND_ACK, 0);
		dev_dbg(priv->dev, "SYSEVT1_SUS_WARN: acked\n");
	}

	regmap_write(priv->map, ASPEED_ESPI_SYSEVT1_INT_STS, sts);
}

static void aspeed_espi_boot_ack(struct aspeed_espi *priv)
{
	u32 evt;

	regmap_read(priv->map, ASPEED_ESPI_SYSEVT, &evt);
	if (!(evt & ASPEED_ESPI_SYSEVT_SLAVE_BOOT_STATUS)) {
		regmap_write(priv->map, ASPEED_ESPI_SYSEVT,
			     evt | ASPEED_ESPI_SYSEVT_SLAVE_BOOT_STATUS |
			     ASPEED_ESPI_SYSEVT_SLAVE_BOOT_DONE);
		dev_dbg(priv->dev, "Setting espi slave boot done\n");
	}

	regmap_read(priv->map, ASPEED_ESPI_SYSEVT1, &evt);
	if (evt & ASPEED_ESPI_SYSEVT1_SUSPEND_WARN &&
	    !(evt & ASPEED_ESPI_SYSEVT1_SUSPEND_ACK)) {
		regmap_write(priv->map, ASPEED_ESPI_SYSEVT1,
			     evt | ASPEED_ESPI_SYSEVT1_SUSPEND_ACK);
		dev_dbg(priv->dev, "Boot SYSEVT1_SUS_WARN: acked\n");
	}
}

static void aspeed_espi_config_irq(struct aspeed_espi *priv)
{
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT_INT_T0, ASPEED_ESPI_SYSEVT_INT_T0_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT_INT_T1, ASPEED_ESPI_SYSEVT_INT_T1_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT_INT_T2, ASPEED_ESPI_SYSEVT_INT_T2_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT_INT_EN, 0xFFFFFFFF);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT1_INT_T0, ASPEED_ESPI_SYSEVT1_INT_T0_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT1_INT_T1, ASPEED_ESPI_SYSEVT1_INT_T1_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT1_INT_T2, ASPEED_ESPI_SYSEVT1_INT_T2_MASK);
	regmap_write(priv->map, ASPEED_ESPI_SYSEVT1_INT_EN, ASPEED_ESPI_SYSEVT1_INT_MASK);
	regmap_write_bits(priv->map, ASPEED_ESPI_INT_EN, ASPEED_ESPI_INT_MASK,
			  ASPEED_ESPI_INT_MASK);
}

static irqreturn_t aspeed_espi_irq(int irq, void *arg)
{
	struct aspeed_espi *priv = arg;
	u32 sts, sts_handled = 0;

	regmap_read(priv->map, ASPEED_ESPI_INT_STS, &sts);

	dev_dbg(priv->dev, "INT_STS: 0x%08x\n", sts);

	if (priv->gpio_irq)
		priv->gpio_irq(irq, priv->gpio_data);

	if (sts & ASPEED_ESPI_VW_SYSEVT) {
		aspeed_espi_sys_event(priv);
		sts_handled |= ASPEED_ESPI_VW_SYSEVT;
	}
	if (sts & ASPEED_ESPI_VW_SYSEVT1) {
		aspeed_espi_sys_event1(priv);
		sts_handled |= ASPEED_ESPI_VW_SYSEVT1;
	}
	if (sts & ASPEED_ESPI_INT_STS_OOB_BITS) {
		aspeed_espi_oob_event(sts, priv->espi_ctrl->oob);
		sts_handled |= ASPEED_ESPI_INT_STS_OOB_BITS;
	}
	if (sts & ASPEED_ESPI_HW_RESET) {
		spin_lock(&priv->pltrstn_lock);
		priv->pltrstn = 'U';
		priv->pltrstn_in_avail = true;
		spin_unlock(&priv->pltrstn_lock);
		wake_up_interruptible(&priv->pltrstn_waitq);
		dev_dbg(priv->dev, "SYSEVT_PLTRSTN: %c\n", priv->pltrstn);

		regmap_write_bits(priv->map, ASPEED_ESPI_CTRL,
				  ASPEED_ESPI_CTRL_SW_RESET, 0);
		regmap_write_bits(priv->map, ASPEED_ESPI_CTRL,
				  ASPEED_ESPI_CTRL_SW_RESET,
				  ASPEED_ESPI_CTRL_SW_RESET);

		aspeed_espi_config_irq(priv);

		aspeed_espi_boot_ack(priv);
		aspeed_espi_oob_enable(priv->espi_ctrl->oob);

		sts_handled |= ASPEED_ESPI_HW_RESET;
	}

	regmap_write(priv->map, ASPEED_ESPI_INT_STS, sts);

	return sts != sts_handled ? IRQ_NONE : IRQ_HANDLED;
}

static inline struct aspeed_espi *to_aspeed_espi(struct file *filp)
{
	return container_of(filp->private_data, struct aspeed_espi,
			    pltrstn_miscdev);
}

static inline struct aspeed_espi *to_aspeed_espi_smi(struct file *filp)
{
	return container_of(filp->private_data, struct aspeed_espi, smi_miscdev);
}

static int aspeed_espi_pltrstn_open(struct inode *inode, struct file *filp)
{
	struct aspeed_espi *priv = to_aspeed_espi(filp);

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
		return -EACCES;
	priv->pltrstn_in_avail = true ; /*Setting true returns first data after file open*/

	return 0;
}

static int aspeed_espi_smi_open(struct inode *inode, struct file *filp)
{
	struct aspeed_espi *priv = to_aspeed_espi_smi(filp);

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
		return -EACCES;
	priv->smi_is_avail = true;

	return 0;
}

static long aspeed_espi_smi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi *espi_smi = to_aspeed_espi_smi(fp);
	u32 val = 0;

	switch (cmd) {
	case ASPEED_ESPI_SMI_GET:
		regmap_read(espi_smi->map, ASPEED_ESPI_SYSEVT, &val);
		if (put_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		break;

	default:
		return -ENOTTY;
	};

	return 0;
}

static ssize_t aspeed_espi_pltrstn_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offset)
{
	struct aspeed_espi *priv = to_aspeed_espi(filp);
	DECLARE_WAITQUEUE(wait, current);
	char data, old_sample;
	int ret = 0;

	spin_lock_irq(&priv->pltrstn_lock);

	if (filp->f_flags & O_NONBLOCK) {
		if (!priv->pltrstn_in_avail) {
			ret = -EAGAIN;
			goto out_unlock;
		}
		data = priv->pltrstn;
		priv->pltrstn_in_avail = false;
	} else {
		add_wait_queue(&priv->pltrstn_waitq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		old_sample = priv->pltrstn;

		do {
			if (old_sample != priv->pltrstn) {
				data = priv->pltrstn;
				priv->pltrstn_in_avail = false;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
			} else {
				spin_unlock_irq(&priv->pltrstn_lock);
				schedule();
				spin_lock_irq(&priv->pltrstn_lock);
			}
		} while (!ret);

		remove_wait_queue(&priv->pltrstn_waitq, &wait);
		set_current_state(TASK_RUNNING);
	}
out_unlock:
	spin_unlock_irq(&priv->pltrstn_lock);

	if (ret)
		return ret;

	ret = put_user(data, buf);
	if (!ret)
		ret = sizeof(data);

	return ret;
}

static ssize_t aspeed_espi_smi_read(struct file *filp, char __user *buf,
				    size_t count, loff_t *offset)
{
	struct aspeed_espi *priv = to_aspeed_espi_smi(filp);
	DECLARE_WAITQUEUE(wait, current);
	char data, old_sample;
	int ret = 0;

	spin_lock_irq(&priv->smi_lock);

	if (filp->f_flags & O_NONBLOCK) {
		if (!priv->smi_is_avail) {
			ret = -EAGAIN;
			goto out_unlock;
		}
		data = priv->smi;
		priv->smi_is_avail = false;
	} else {
		add_wait_queue(&priv->smi_waitq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		old_sample = priv->smi;

		do {
			if (old_sample != priv->smi) {
				data = priv->smi;
				priv->smi_is_avail = false;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
			} else {
				spin_unlock_irq(&priv->smi_lock);
				schedule();
				spin_lock_irq(&priv->smi_lock);
			}
		} while (!ret);

		remove_wait_queue(&priv->smi_waitq, &wait);
		set_current_state(TASK_RUNNING);
	}
out_unlock:
	spin_unlock_irq(&priv->smi_lock);

	if (ret)
		return ret;

	ret = put_user(data, buf);
	if (ret)
		return ret;

	return sizeof(data);
}

static unsigned int aspeed_espi_pltrstn_poll(struct file *file,
						 poll_table *wait)
{
	struct aspeed_espi *priv = to_aspeed_espi(file);
	unsigned int mask = 0;

	poll_wait(file, &priv->pltrstn_waitq, wait);
	if (priv->pltrstn_in_avail)
		mask |= POLLIN;
	return mask;
}

static unsigned int aspeed_espi_smi_poll(struct file *file, poll_table *wait)
{
	struct aspeed_espi *priv = to_aspeed_espi_smi(file);
	unsigned int mask = 0;

	poll_wait(file, &priv->smi_waitq, wait);

	if (priv->smi_is_avail)
		mask |= POLLIN;
	return mask;
}

static const struct file_operations aspeed_espi_pltrstn_fops = {
	.owner	= THIS_MODULE,
	.open	= aspeed_espi_pltrstn_open,
	.read	= aspeed_espi_pltrstn_read,
	.poll	= aspeed_espi_pltrstn_poll,
};

static const struct file_operations aspeed_espi_smi_fops = {
	.owner	= THIS_MODULE,
	.open	= aspeed_espi_smi_open,
	.read	= aspeed_espi_smi_read,
	.poll	= aspeed_espi_smi_poll,
	.unlocked_ioctl = aspeed_espi_smi_ioctl,
};

static int aspeed_espi_probe(struct platform_device *pdev)
{
	struct aspeed_espi_ctrl *espi_ctrl;
	struct aspeed_espi *priv;
	struct device_node *node;
	u32 ctrl;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	espi_ctrl = devm_kzalloc(&pdev->dev, sizeof(*espi_ctrl), GFP_KERNEL);
	if (!espi_ctrl)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, priv);
	priv->dev = &pdev->dev;
	priv->espi_ctrl = espi_ctrl;
	espi_ctrl->model = of_device_get_match_data(&pdev->dev);

	priv->map = device_node_to_regmap(pdev->dev.of_node);

	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);
	espi_ctrl->map = priv->map;
	aspeed_espi_config_irq(priv);
	espi_ctrl->oob = aspeed_espi_oob_alloc(&pdev->dev, espi_ctrl);
	if (IS_ERR(espi_ctrl->oob)) {
		dev_err(&pdev->dev, "Failed to allocate espi out-of-band channel\n");
		return PTR_ERR(espi_ctrl->oob);
	}

	spin_lock_init(&priv->pltrstn_lock);
	spin_lock_init(&priv->smi_lock);
	init_waitqueue_head(&priv->pltrstn_waitq);
	init_waitqueue_head(&priv->smi_waitq);
	priv->pltrstn = 'U'; /* means it's not reported yet from master */
	priv->smi = 'U';

	for_each_child_of_node(priv->dev->of_node, node) {
		if (!of_platform_device_create(node, NULL, priv->dev))
			dev_warn(&pdev->dev, "Unable to create espi child instance\n");
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	ret = devm_request_irq(&pdev->dev, priv->irq, aspeed_espi_irq, 0,
			       "aspeed-espi-irq", priv);
	if (ret)
		return ret;

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->clk),
				     "couldn't get clock\n");
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "couldn't enable clock\n");
		return ret;
	}

	/*
	 * We check that the regmap works on this very first access, but as this
	 * is an MMIO-backed regmap, subsequent regmap access is not going to
	 * fail and we skip error checks from this point.
	 */
	ret = regmap_read(priv->map, ASPEED_ESPI_CTRL, &ctrl);
	if (ret) {
		dev_err(&pdev->dev, "failed to read ctrl register\n");
		goto err_clk_disable_out;
	}

	priv->pltrstn_miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->pltrstn_miscdev.name = "espi-pltrstn";
	priv->pltrstn_miscdev.fops = &aspeed_espi_pltrstn_fops;
	priv->pltrstn_miscdev.parent = &pdev->dev;

	priv->smi_miscdev.minor = MISC_DYNAMIC_MINOR - 1;
	priv->smi_miscdev.name = "espi-smi";
	priv->smi_miscdev.fops = &aspeed_espi_smi_fops;
	priv->smi_miscdev.parent = &pdev->dev;

	ret = misc_register(&priv->pltrstn_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register device\n");
		goto err_clk_disable_out;
	}

	ret = misc_register(&priv->smi_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register SMI device\n");
		goto err_clk_disable_out;
	}

	aspeed_espi_boot_ack(priv);

	dev_info(&pdev->dev, "eSPI registered, irq %d\n", priv->irq);
	return 0;

err_clk_disable_out:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int aspeed_espi_remove(struct platform_device *pdev)
{
	struct aspeed_espi *priv = dev_get_drvdata(&pdev->dev);

	aspeed_espi_oob_free(priv->dev, priv->espi_ctrl->oob);
	misc_deregister(&priv->pltrstn_miscdev);
	misc_deregister(&priv->smi_miscdev);
	clk_disable_unprepare(priv->clk);
	return 0;
}

static const struct aspeed_espi_model ast2600_model = {
	.version = ASPEED_ESPI_AST2600,
};

static const struct of_device_id of_espi_match_table[] = {
	{ .compatible = "aspeed,ast2500-espi-slave" },
	{ .compatible = "aspeed,ast2600-espi-slave",
	  .data	      = &ast2600_model},
	{ }
};
MODULE_DEVICE_TABLE(of, of_espi_match_table);

static struct platform_driver aspeed_espi_driver = {
	.driver	= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= of_match_ptr(of_espi_match_table),
	},
	.probe	= aspeed_espi_probe,
	.remove	= aspeed_espi_remove,
};
module_platform_driver(aspeed_espi_driver);

MODULE_AUTHOR("Haiyue Wang <haiyue.wang@linux.intel.com>");
MODULE_AUTHOR("Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>");
MODULE_DESCRIPTION("Aspeed eSPI driver");
MODULE_LICENSE("GPL v2");
