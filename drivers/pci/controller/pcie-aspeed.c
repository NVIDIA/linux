// SPDX-License-Identifier: GPL-2.0
/*
 * PCIE driver for the Aspeed SoC
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/delay.h>

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/msi.h>

#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>

#include "h2x-ast2600.h"
#include "../pci.h"

#define MAX_LEGACY_IRQS			4
#define MAX_MSI_HOST_IRQS		64

/*	PCI Host Controller registers */
#define ASPEED_PCIE_CLASS_CODE		0x04
#define ASPEED_PCIE_GLOBAL			0x30
#define ASPEED_PCIE_CFG_DIN			0x50
#define ASPEED_PCIE_CFG3			0x58
#define ASPEED_PCIE_LOCK			0x7C
#define ASPEED_PCIE_LINK			0xC0
#define ASPEED_PCIE_INT				0xC4
#define ASPEED_PCIE_LINK_STS		0xD0
/*	AST_PCIE_CFG2			0x04 */
#define PCIE_CFG_CLASS_CODE(x)	(x << 8)
#define PCIE_CFG_REV_ID(x)		(x)
/*	AST_PCIE_GLOBAL			0x30 */
#define ROOT_COMPLEX_ID(x)		(x << 4)
/*	AST_PCIE_LOCK			0x7C */
#define PCIE_UNLOCK				0xa8
/*	AST_PCIE_LINK			0xC0 */
#define PCIE_LINK_STS			BIT(5)
/*  ASPEED_PCIE_LINK_STS	0xD0 */
#define PCIE_LINK_5G			BIT(17)
#define PCIE_LINK_2_5G			BIT(16)

static DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_HOST_IRQS);

/* MSI functions */

/**
 * aspeed_pcie_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void aspeed_pcie_destroy_msi(unsigned int irq)
{
	struct msi_desc *msi;
	struct aspeed_pcie *pcie;
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	if (!test_bit(hwirq, msi_irq_in_use)) {
		msi = irq_get_msi_desc(irq);
		pcie = msi_desc_to_pci_sysdata(msi);
		dev_err(pcie->dev, "Trying to free unused MSI#%d\n", irq);
	} else {
		clear_bit(hwirq, msi_irq_in_use);
	}
}

/**
 * aspeed_pcie_assign_msi - Allocate MSI number
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int aspeed_pcie_assign_msi(void)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, MAX_MSI_HOST_IRQS);
	if (pos < MAX_MSI_HOST_IRQS)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

/**
 * aspeed_msi_teardown_irq - Destroy the MSI
 * @chip: MSI Chip descriptor
 * @irq: MSI IRQ to destroy
 */
static void aspeed_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	aspeed_pcie_destroy_msi(irq);
	irq_dispose_mapping(irq);
}

/**
 * aspeed_pcie_msi_setup_irq - Setup MSI request
 * @chip: MSI chip pointer
 * @pdev: PCIe device pointer
 * @desc: MSI descriptor pointer
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct aspeed_pcie *pcie = pdev->bus->sysdata;
	unsigned int irq;
	int hwirq;
	struct msi_msg msg;

	hwirq = aspeed_pcie_assign_msi();
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(pcie->msi_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_hi = 0;
	msg.address_lo = pcie->msi_address;
//	msg.data = irq;
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

/**
 * aspeed_pcie_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int aspeed_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	struct aspeed_pcie *pcie = (struct aspeed_pcie *)domain->host_data;

	irq_set_chip_and_handler(irq,
				&pcie->aspeed_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/**
 * aspeed_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int aspeed_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
					irq_hw_number_t hwirq)
{
	struct aspeed_pcie *pcie = (struct aspeed_pcie *)domain->host_data;

	irq_set_chip_and_handler(irq,
				&pcie->aspeed_h2x_intx_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/**
 * aspeed_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
static irqreturn_t aspeed_pcie_intr_handler(int irq, void *data)
{
	struct aspeed_pcie *pcie = (struct aspeed_pcie *)data;

	aspeed_h2x_rc_intr_handler(pcie);
	return IRQ_HANDLED;
}

/**
 * aspeed_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_init_irq_domain(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pcie_intc_node;

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return -ENODEV;
	}

	pcie->aspeed_h2x_intx_chip.name = "IntX";
	pcie->aspeed_h2x_intx_chip.irq_ack	= aspeed_h2x_intx_ack_irq;
	pcie->aspeed_h2x_intx_chip.irq_mask	= aspeed_h2x_intx_mask_irq;
	pcie->aspeed_h2x_intx_chip.irq_unmask = aspeed_h2x_intx_unmask_irq;
	pcie->intx_domain_ops.map = aspeed_pcie_intx_map;
	pcie->leg_domain = irq_domain_add_linear(pcie_intc_node,
				MAX_LEGACY_IRQS, &pcie->intx_domain_ops, pcie);
	of_node_put(pcie_intc_node);
	if (!pcie->leg_domain) {
		dev_err(dev, "unable get a INTx IRQ domain\n");
		return -ENODEV;
	}

#ifdef CONFIG_PCI_MSI
	/* MSI Domain operations */
	pcie->msi_domain_ops.map = aspeed_pcie_msi_map;
	pcie->aspeed_pcie_msi_chip.setup_irq = aspeed_pcie_msi_setup_irq;
	pcie->aspeed_pcie_msi_chip.teardown_irq = aspeed_msi_teardown_irq;
	pcie->aspeed_msi_irq_chip.name = "MSI";
	pcie->aspeed_msi_irq_chip.irq_enable = pci_msi_unmask_irq;
	pcie->aspeed_msi_irq_chip.irq_disable = pci_msi_mask_irq;
	pcie->aspeed_msi_irq_chip.irq_mask = pci_msi_mask_irq;
	pcie->aspeed_msi_irq_chip.irq_unmask = pci_msi_unmask_irq;
#endif

	/* Setup MSI */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pcie->msi_domain =
				irq_domain_add_linear(node,
							MAX_MSI_HOST_IRQS,
							&pcie->msi_domain_ops,
							pcie);
		if (!pcie->msi_domain) {
			dev_err(dev, "unable to get a MSI IRQ domain\n");
			return -ENODEV;
		}
		//enable all msi interrupt
		aspeed_h2x_msi_enable(pcie);
	}

	return 0;
}

/**
 * aspeed_pcie_init_port - Initialize hardware
 * @port: PCIe port information
 */

#define AHBC_UNLOCK	0xAEED1A03
static void aspeed_pcie_init_port(struct aspeed_pcie *pcie)
{
	struct regmap *ahbc =
		syscon_regmap_lookup_by_compatible("aspeed,aspeed-ahbc");
	if (!IS_ERR(ahbc)) {
		//ahbc remap enable
		regmap_write(ahbc, 0x00, AHBC_UNLOCK);
		regmap_update_bits(ahbc, 0x8C, BIT(5), BIT(5));
		regmap_write(ahbc, 0x00, 0x1);
	} else {
		dev_warn(pcie->dev,
			"Unable to remap AHBC. %ld\n", PTR_ERR(ahbc));
	}

	aspeed_h2x_rc_init(pcie);

	//plda init
	writel(PCIE_UNLOCK, pcie->pciereg_base + ASPEED_PCIE_LOCK);
//	writel(PCIE_CFG_CLASS_CODE(0x60000) | PCIE_CFG_REV_ID(4),
//				pcie->pciereg_base + ASPEED_PCIE_CLASS_CODE);
	writel(ROOT_COMPLEX_ID(0x3), pcie->pciereg_base + ASPEED_PCIE_GLOBAL);

	/* Don't register host if link is down */
	if (readl(pcie->pciereg_base + ASPEED_PCIE_LINK) & PCIE_LINK_STS) {
		aspeed_h2x_workaround(pcie);

		if (readl(pcie->pciereg_base
				+ ASPEED_PCIE_LINK_STS) & PCIE_LINK_5G)
			dev_info(pcie->dev, "PCIE- Link up : 5G\n");
		if (readl(pcie->pciereg_base
				+ ASPEED_PCIE_LINK_STS) & PCIE_LINK_2_5G)
			dev_info(pcie->dev, "PCIE- Link up : 2.5G\n");
	} else {
		dev_info(pcie->dev, "PCIE- Link down\n");
	}

}

/**
 * aspeed_pcie_parse_dt - Parse Device tree
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_parse_dt(struct aspeed_pcie *pcie,
					struct platform_device *pdev)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		pcie->pciereg_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->pciereg_base))
			return PTR_ERR(pcie->pciereg_base);
	}

	pcie->irq = irq_of_parse_and_map(node, 0);

	of_property_read_u32(node, "rc_offset", &pcie->rc_offset);
	of_property_read_u32(node, "msi_address", &pcie->msi_address);
	return 0;
}

static int aspeed_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_pcie *pcie;
	struct pci_bus *bus;
	struct pci_bus *child;
	struct pci_host_bridge *bridge;
//	struct regmap *scu;
	int err;
//	resource_size_t iobase = 0;
	LIST_HEAD(res);

	if (!dev->of_node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;

	err = aspeed_pcie_parse_dt(pcie, pdev);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	pcie->reset = devm_reset_control_get(&pdev->dev, 0);
	if (IS_ERR(pcie->reset)) {
		dev_err(&pdev->dev, "can't get pcie reset\n");
		return PTR_ERR(pcie->reset);
	}
#if 0
	scu = syscon_regmap_lookup_by_compatible("aspeed,aspeed-scu");
	if (!IS_ERR(scu)) {
		regmap_update_bits(scu, 0x500, BIT(24), BIT(24));
		regmap_update_bits(scu, 0x40, BIT(19) | BIT(18),
							BIT(19) | BIT(18));
	} else
		dev_warn(pcie->dev, "Unable to remap SCU. %ld\n", PTR_ERR(scu));
#else
	reset_control_assert(pcie->reset);
	mdelay(50);
	reset_control_deassert(pcie->reset);
	mdelay(50);
#endif
	aspeed_pcie_init_port(pcie);

	err = aspeed_pcie_init_irq_domain(pcie);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		goto error;
	}

	err = devm_request_pci_bus_resources(dev, &res);
	if (err)
		goto error;

	list_splice_init(&res, &bridge->windows);

	/* PCIe operations */
	pcie->aspeed_pcie_ops.read = aspeed_h2x_rd_conf;
	pcie->aspeed_pcie_ops.write = aspeed_h2x_wr_conf;
	bridge->dev.parent = dev;
	bridge->sysdata = pcie;
	bridge->busnr = 0;
	bridge->ops = &pcie->aspeed_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
//	bridge->swizzle_irq = pci_common_swizzle;

#ifdef CONFIG_PCI_MSI
	pcie->aspeed_pcie_msi_chip.dev = dev;
// TBD	bridge->msi = &pcie->aspeed_pcie_msi_chip;
#endif
	err = pci_scan_root_bus_bridge(bridge);
	if (err)
		goto error;

	bus = bridge->bus;
	pci_assign_unassigned_bus_resources(bus);
	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);
	pci_bus_add_devices(bus);

	err = devm_request_irq(dev, pcie->irq, aspeed_pcie_intr_handler,
						   0,
						   "aspeed-pcie", pcie);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", pcie->irq);
		return err;
	}

	return 0;

error:
	pci_free_resource_list(&res);
	return err;
}

static const struct of_device_id aspeed_pcie_of_match[] = {
	{ .compatible = "aspeed,ast2600-pcie", },
	{},
};

static struct platform_driver aspeed_pcie_driver = {
	.driver = {
		.name	= "aspeed-pcie",
		.of_match_table = aspeed_pcie_of_match,
	},
	.probe = aspeed_pcie_probe,
};

builtin_platform_driver(aspeed_pcie_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PCIe Host driver");
MODULE_LICENSE("GPL");
