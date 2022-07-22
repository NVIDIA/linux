// SPDX-License-Identifier: GPL-2.0
/*
 * H2X driver for the Aspeed SoC
 *
 */
#include <linux/init.h>

#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/reset.h>
#include <asm/io.h>

#include <linux/aspeed_pcie_io.h>

#include "h2x-ast2600.h"
#include "../pci.h"

/* reg 0x24 */
#define PCIE_TX_IDLE			BIT(31)

#define PCIE_STATUS_OF_TX		GENMASK(25, 24)
#define	PCIE_RC_TX_COMPLETE		0
#define	PCIE_RC_L_TX_COMPLETE	BIT(24)
#define	PCIE_RC_H_TX_COMPLETE	BIT(25)

#define PCIE_TRIGGER_TX			BIT(0)

/* reg 0x80, 0xC0 */
#define PCIE_RX_TAG_MASK		GENMASK(23, 16)
#define PCIE_RX_DMA_EN			BIT(9)
#define PCIE_RX_LINEAR			BIT(8)
#define PCIE_RX_MSI_SEL			BIT(7)
#define PCIE_RX_MSI_EN			BIT(6)
#define PCIE_1M_ADDRESS_EN		BIT(5)
#define PCIE_UNLOCK_RX_BUFF		BIT(4)
#define PCIE_RX_TLP_TAG_MATCH	BIT(3)
#define PCIE_Wait_RX_TLP_CLR	BIT(2)
#define PCIE_RC_RX_ENABLE		BIT(1)
#define PCIE_RC_ENABLE			BIT(0)

/* reg 0x88, 0xC8 : RC ISR */
#define PCIE_RC_CPLCA_ISR		BIT(6)
#define PCIE_RC_CPLUR_ISR		BIT(5)
#define PCIE_RC_RX_DONE_ISR		BIT(4)

#define PCIE_RC_INTD_ISR		BIT(3)
#define PCIE_RC_INTC_ISR		BIT(2)
#define PCIE_RC_INTB_ISR		BIT(1)
#define PCIE_RC_INTA_ISR		BIT(0)

struct aspeed_h2x_info {
	void __iomem *reg_base;
	int irq;
	u8 txTag;
	struct reset_control *reset;
};

struct aspeed_h2x_info *aspeed_h2x;

extern u8 aspeed_pcie_inb(u32 addr)
{
	int timeout = 0;
	void __iomem *pcie_rc_base = aspeed_h2x->reg_base + 0xc0;

	printk("aspeed_pcie_inb addr %x \n", addr);
	writel(BIT(4) | readl(pcie_rc_base), pcie_rc_base);

	writel(0x02000001, aspeed_h2x->reg_base + 0x10);
	writel(0x00002000 | (0x1 << (addr & 0x3)), aspeed_h2x->reg_base + 0x14);
	writel(addr & (~0x3), aspeed_h2x->reg_base + 0x18);
	writel(0x00000000, aspeed_h2x->reg_base + 0x1c);

	//trigger
	writel((readl(aspeed_h2x->reg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, aspeed_h2x->reg_base + 0x24);

	//wait tx idle
	while(!(readl(aspeed_h2x->reg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
			printk("aspeed_pcie_inb timeout\n");
			return 0xff;
		}
	};

	//write clr tx idle
	writel(1, aspeed_h2x->reg_base + 0x08);

	timeout = 0;

	while(!(readl(pcie_rc_base + 0x08) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}
	writel(readl(pcie_rc_base + 0x08), pcie_rc_base + 0x08);
//	writel(BIT(4) | readl(pcie_rc_base), pcie_rc_base);
	return ((readl(pcie_rc_base + 0x0C) >> ((addr & 0x3) * 8)) & 0xff);

}

EXPORT_SYMBOL_GPL(aspeed_pcie_inb);

extern void aspeed_pcie_outb(u8 value, u32 addr)
{
	int timeout = 0;
	u32 wvalue = value;
	void __iomem *pcie_rc_base = aspeed_h2x->reg_base + 0xc0;
	printk("aspeed_pcie_outb addr %x \n", addr);

	writel(BIT(4) | readl(pcie_rc_base), pcie_rc_base);

	writel(0x42000001, aspeed_h2x->reg_base + 0x10);
	writel(0x00002000 | (0x1 << (addr & 0x3)), aspeed_h2x->reg_base + 0x14);
	writel(addr & (~0x3), aspeed_h2x->reg_base + 0x18);
	writel(0x00000000, aspeed_h2x->reg_base + 0x1c);

	writel((wvalue << (8 * (addr & 0x3))), aspeed_h2x->reg_base + 0x20);

	//trigger
	writel((readl(aspeed_h2x->reg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, aspeed_h2x->reg_base + 0x24);

	//wait tx idle
	while(!(readl(aspeed_h2x->reg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
			printk("aspeed_pcie_outb timeout\n");
			return;
		}
	};

	//write clr tx idle
	writel(1, aspeed_h2x->reg_base + 0x08);

	while(!(readl(pcie_rc_base + 0x08) & PCIE_RC_RX_DONE_ISR));
	writel(readl(pcie_rc_base + 0x08), pcie_rc_base + 0x08);
	writel(BIT(4) | readl(pcie_rc_base), pcie_rc_base);

}

EXPORT_SYMBOL_GPL(aspeed_pcie_outb);

extern int aspeed_h2x_rd_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 timeout = 0;
	u32 bdf_offset;
	int rx_done_fail = 0;
	u32 type = 0;

	//H2X80[4] (unlock) is write-only.
	//Driver may set H2X80[4]=1 before triggering next TX config.
	writel(BIT(4) | readl(pcie->h2x_rc_base), pcie->h2x_rc_base);

	if(bus->number)
		type = 1;
	else
		type = 0;

	bdf_offset = (bus->number << 24) |
					(PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) |
					(where & ~3);

	aspeed_h2x->txTag %= 0x7;

	writel(0x04000001 | (type << 24), aspeed_h2x->reg_base + 0x10);
	writel(0x0000200f | (aspeed_h2x->txTag << 8), aspeed_h2x->reg_base + 0x14);
	writel(bdf_offset, aspeed_h2x->reg_base + 0x18);
	writel(0x00000000, aspeed_h2x->reg_base + 0x1c);

	//trigger tx
	writel((readl(aspeed_h2x->reg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, aspeed_h2x->reg_base + 0x24);

	//wait tx idle
	while(!(readl(aspeed_h2x->reg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
//			printk("time out b : %d, d : %d, f: %d \n", bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn));
			*val = 0xffffffff;
			goto out;
		}
	};

	//write clr tx idle
	writel(1, aspeed_h2x->reg_base + 0x08);

	timeout = 0;
	//check tx status
	switch(readl(aspeed_h2x->reg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(aspeed_h2x->reg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					rx_done_fail = 1;
					*val = 0xffffffff;
					break;
				}
				mdelay(1);
			}
			if(!rx_done_fail) {
				if(readl(aspeed_h2x->reg_base + 0x94) & BIT(13)) {
					*val = 0xffffffff;
				} else
					*val = readl(aspeed_h2x->reg_base + 0x8C);
			}
			writel(BIT(4) | readl(aspeed_h2x->reg_base + 0x80), aspeed_h2x->reg_base + 0x80);
			writel(readl(aspeed_h2x->reg_base + 0x88), aspeed_h2x->reg_base + 0x88);
			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(aspeed_h2x->reg_base + 0xC8) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					rx_done_fail = 1;
					*val = 0xffffffff;
					break;
				}
				mdelay(1);
			}
			if(!rx_done_fail) {
				if(readl(aspeed_h2x->reg_base + 0xD4) & BIT(13)) {
					*val = 0xffffffff;
				} else
					*val = readl(aspeed_h2x->reg_base + 0xCC);
			}
			writel(BIT(4) | readl(aspeed_h2x->reg_base + 0xC0), aspeed_h2x->reg_base + 0xC0);
			writel(readl(aspeed_h2x->reg_base + 0xC8), aspeed_h2x->reg_base + 0xC8);
			break;
		default:	//read rc data
			*val = readl(aspeed_h2x->reg_base + 0x0C);
			break;
	}

	switch (size) {
		case 1:
			*val = (*val >> ((where & 3) * 8)) & 0xff;
			break;
		case 2:
			*val = (*val >> ((where & 2) * 8)) & 0xffff;
			break;
	}
out:
	aspeed_h2x->txTag++;

	return PCIBIOS_SUCCESSFUL;

}
EXPORT_SYMBOL_GPL(aspeed_h2x_rd_conf);

extern int
aspeed_h2x_wr_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{
	u32 timeout = 0;
	u32 type = 0;
	u32 shift = 8 * (where & 3);
	u32 bdf_offset;
	u8 byte_en = 0;
	struct aspeed_pcie *pcie = bus->sysdata;

	writel(BIT(4) | readl(pcie->h2x_rc_base), pcie->h2x_rc_base);

	switch (size) {
	case 1:
		switch(where % 4) {
			case 0:
				byte_en = 0x1;
				break;
			case 1:
				byte_en = 0x2;
				break;
			case 2:
				byte_en = 0x4;
				break;
			case 3:
				byte_en = 0x8;
				break;
		}
		val = (val & 0xff) << shift;
		break;
	case 2:
		switch((where >> 1) % 2 ) {
			case 0:
				byte_en = 0x3;
				break;
			case 1:
				byte_en = 0xc;
				break;
		}
		val = (val & 0xffff) << shift;
		break;
	default:
		byte_en = 0xf;
		break;
	}

	if(bus->number)
		type = 1;
	else
		type = 0;

	bdf_offset = (bus->number << 24) | (PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) | (where & ~3);

	aspeed_h2x->txTag %= 0x7;

	writel(0x44000001 | (type << 24), aspeed_h2x->reg_base + 0x10);
	writel(0x00002000 | (aspeed_h2x->txTag << 8) | byte_en, aspeed_h2x->reg_base + 0x14);
	writel(bdf_offset, aspeed_h2x->reg_base + 0x18);
	writel(0x00000000, aspeed_h2x->reg_base + 0x1C);

	writel(val, aspeed_h2x->reg_base + 0x20);

	//trigger tx
	writel((readl(aspeed_h2x->reg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, aspeed_h2x->reg_base + 0x24);

//wait tx idle
	while(!(readl(aspeed_h2x->reg_base + 0x24) & BIT(31))) {
		timeout++;
		if(timeout > 10000) {
			printk("time out \n");
			goto out;
		}
	};

	//write clr tx idle
	writel(1, aspeed_h2x->reg_base + 0x08);

	timeout = 0;
	//check tx status and clr rx done int
	switch(readl(aspeed_h2x->reg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(aspeed_h2x->reg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					break;
				}
				mdelay(1);
			}
			writel(PCIE_RC_RX_DONE_ISR, aspeed_h2x->reg_base + 0x88);

			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(aspeed_h2x->reg_base + 0xC8) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					break;
				}
				mdelay(1);
			}
			writel(PCIE_RC_RX_DONE_ISR, aspeed_h2x->reg_base + 0xC8);
			break;
	}

out:
	aspeed_h2x->txTag++;

	return PCIBIOS_SUCCESSFUL;

}
EXPORT_SYMBOL_GPL(aspeed_h2x_wr_conf);

/* INTx Functions */
extern void aspeed_h2x_intx_ack_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

	writel(readl(pcie->h2x_rc_base + 0x04) | BIT(d->hwirq), pcie->h2x_rc_base + 0x04);
}

extern void aspeed_h2x_intx_mask_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

	writel(readl(pcie->h2x_rc_base + 0x04) & ~BIT(d->hwirq), pcie->h2x_rc_base + 0x04);
}

extern void aspeed_h2x_intx_unmask_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

	//Enable IRQ ..
	writel(readl(pcie->h2x_rc_base + 0x04) | BIT(d->hwirq), pcie->h2x_rc_base + 0x04);
}

extern void aspeed_h2x_msi_enable(struct aspeed_pcie *pcie)
{
	writel(0xffffffff, pcie->h2x_rc_base + 0x20);
	writel(0xffffffff, pcie->h2x_rc_base + 0x24);
}

extern void aspeed_h2x_rc_intr_handler(struct aspeed_pcie *pcie)
{
	u32 bit;
	u32 virq;
	unsigned long status;
	int i;
	unsigned long intx = readl(pcie->h2x_rc_base + 0x08) & 0xf;

	//intx isr
	if(intx) {
		for_each_set_bit(bit, &intx, 32) {
			virq = irq_find_mapping(pcie->leg_domain, bit);
			if (virq)
				generic_handle_irq(virq);
			else
				dev_err(pcie->dev, "unexpected Int - X\n");
		}
	}
	//msi isr
	for (i = 0; i < 2; i++) {
		status = readl(pcie->h2x_rc_base + 0x28 + (i * 4));
//		printk("aspeed_pcie_intr_handler  status %lx \n", status);
		writel(status, pcie->h2x_rc_base + 0x28 + (i * 4));
//		printk("read  status %x \n", readl(pcie->h2xreg_base + 0xe8 + (i * 4)));
		if (!status)
				continue;

		for_each_set_bit(bit, &status, 32) {
			if(i) {
				bit += 32;
			}
			virq = irq_find_mapping(pcie->msi_domain, bit);
//			printk("[%d] : find bit %d mapping irq #%d \n", i, bit, virq);
			if (virq)
				generic_handle_irq(virq);
			else
				dev_err(pcie->dev, "unexpected MSI\n");
		}
	}
}

extern void aspeed_h2x_workaround(struct aspeed_pcie *pcie)
{
	u32 timeout = 0;

	writel(BIT(4) | readl(pcie->h2x_rc_base), pcie->h2x_rc_base);

	writel(0x74000001, aspeed_h2x->reg_base + 0x10);
	writel(0x00400050, aspeed_h2x->reg_base + 0x14);
	writel(0x00000000, aspeed_h2x->reg_base + 0x18);
	writel(0x00000000, aspeed_h2x->reg_base + 0x1c);

	writel(0x1a, aspeed_h2x->reg_base + 0x20);

	//trigger tx
	writel(PCIE_TRIGGER_TX, aspeed_h2x->reg_base + 0x24);

	//wait tx idle
	while(!(readl(aspeed_h2x->reg_base + 0x24) & BIT(31))) {
		timeout++;
		if(timeout > 1000) {
			return;
		}
	};

	//write clr tx idle
	writel(1, aspeed_h2x->reg_base + 0x08);
	timeout = 0;

	//check tx status and clr rx done int
	while(!(readl(pcie->h2x_rc_base + 0x08) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}
	writel(PCIE_RC_RX_DONE_ISR, pcie->h2x_rc_base + 0x08);

}
EXPORT_SYMBOL(aspeed_h2x_workaround);

extern void aspeed_h2x_rc_init(struct aspeed_pcie *pcie)
{
	pcie->h2x_rc_base = aspeed_h2x->reg_base + pcie->rc_offset;

	//todo clr intx isr
	writel(0x0, pcie->h2x_rc_base + 0x04);

	//clr msi isr
	writel(0xFFFFFFFF, pcie->h2x_rc_base + 0x28);
	writel(0xFFFFFFFF, pcie->h2x_rc_base + 0x2c);

	//rc_l
	writel( PCIE_RX_DMA_EN | PCIE_RX_LINEAR | PCIE_RX_MSI_SEL | PCIE_RX_MSI_EN |
			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
	pcie->h2x_rc_base);
	//assign debug tx tag
	writel(0x28, pcie->h2x_rc_base + 0x3C);
}

static int aspeed_h2x_probe(struct platform_device *pdev)
{
	struct resource *res;

	if (!(aspeed_h2x = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_h2x_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	aspeed_h2x->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_h2x->reg_base) {
		return -EIO;
	}
#if 0
	aspeed_h2x->irq = platform_get_irq(pdev, 0);
	if (aspeed_h2x->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return -ENOENT;
	}
#endif

	aspeed_h2x->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_h2x->reset)) {
		dev_err(&pdev->dev, "can't get h2x reset\n");
		return PTR_ERR(aspeed_h2x->reset);
	}
	aspeed_h2x->txTag = 0;

	//scu init
	reset_control_assert(aspeed_h2x->reset);
	reset_control_deassert(aspeed_h2x->reset);

	//init
	writel(0x1, aspeed_h2x->reg_base + 0x00);

	//ahb to pcie rc
	writel(0xe0006000, aspeed_h2x->reg_base + 0x60);
	writel(0x00000000, aspeed_h2x->reg_base + 0x64);
	writel(0xFFFFFFFF, aspeed_h2x->reg_base + 0x68);
	return 0;
}


static const struct of_device_id aspeed_h2x_of_match[] = {
	{ .compatible = "aspeed,ast2600-h2x", },
	{},
};

static struct platform_driver aspeed_h2x_driver = {
	.driver = {
		.name	= "aspeed-h2x",
		.of_match_table = aspeed_h2x_of_match,
	},
	.probe = aspeed_h2x_probe,
};

static int __init aspeed_h2x_init(void)
{
	return platform_driver_register(&aspeed_h2x_driver);
}

arch_initcall(aspeed_h2x_init);
