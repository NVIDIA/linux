// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

#define INT_NUM			480
#define INTM_NUM		50
#define SWINT_NUM		16

#define INTM_BASE		(INT_NUM)
#define SWINT_BASE		(INT_NUM + INTM_NUM)
#define INT0_NUM		(INT_NUM + INTM_NUM + SWINT_NUM)

#define GIC_P2P_SPI_END		128
#define GIC_SWINT_SPI_BASE	144
#define GIC_SWINT_SPI_NUM	16
#define GIC_INTM0_SPI_BASE	192
#define GIC_INTM30_SPI_BASE	208
#define GIC_INTM40_SPI_BASE	224

#define INTC0_SWINT_IER		0x10
#define INTC0_SWINT_ISR		0x14
#define INTC0_INTBANKX_IER		0x1000
#define INTC0_INTBANK_GROUPS	11
#define INTC0_INTBANKS_PER_GRP	3
#define INTC0_INTMX_IER		0x1b00
#define INTC0_INTMX_ISR		0x1b04
#define INTC0_INTM_BANK_NUM	3
#define INTM_IRQS_PER_BANK	10

struct intc0_pin_range {
	u32 int_base;
	u32 gic_base;
	u32 cnt;
};

struct aspeed_intc_ic {
	struct device		*dev;
	void __iomem		*base;
	raw_spinlock_t		intc_lock;
	struct irq_domain	*irq_domain;
	struct intc0_pin_range	*pin_ranges;
	unsigned int		pin_range_cnt;
};

static void aspeed_swint_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;
	unsigned int mask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	mask = readl(intc_ic->base + INTC0_SWINT_IER) & ~BIT(bit);
	writel(mask, intc_ic->base + INTC0_SWINT_IER);
	irq_chip_mask_parent(data);
}

static void aspeed_swint_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;
	unsigned int unmask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	unmask = readl(intc_ic->base + INTC0_SWINT_IER) | BIT(bit);
	writel(unmask, intc_ic->base + INTC0_SWINT_IER);
	irq_chip_unmask_parent(data);
}

static void aspeed_swint_irq_eoi(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;

	writel(BIT(bit), intc_ic->base + INTC0_SWINT_ISR);
	irq_chip_eoi_parent(data);
}

static struct irq_chip aspeed_swint_chip = {
	.name			= "ast2700-swint",
	.irq_eoi		= aspeed_swint_irq_eoi,
	.irq_mask		= aspeed_swint_irq_mask,
	.irq_unmask		= aspeed_swint_irq_unmask,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static void aspeed_intc0_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;
	unsigned int mask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	mask = readl(intc_ic->base + INTC0_INTMX_IER + bank * 0x10) & ~BIT(bit);
	writel(mask, intc_ic->base + INTC0_INTMX_IER + bank * 0x10);
	irq_chip_mask_parent(data);
}

static void aspeed_intc0_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;
	unsigned int unmask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	unmask = readl(intc_ic->base + INTC0_INTMX_IER + bank * 0x10) | BIT(bit);
	writel(unmask, intc_ic->base + INTC0_INTMX_IER + bank * 0x10);
	irq_chip_unmask_parent(data);
}

static void aspeed_intc0_irq_eoi(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;

	writel(BIT(bit), intc_ic->base + INTC0_INTMX_ISR + bank * 0x10);
	irq_chip_eoi_parent(data);
}

static struct irq_chip aspeed_intm_chip = {
	.name			= "ast2700-intmerge",
	.irq_eoi		= aspeed_intc0_irq_eoi,
	.irq_mask		= aspeed_intc0_irq_mask,
	.irq_unmask		= aspeed_intc0_irq_unmask,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,

};

static struct irq_chip linear_intr_irq_chip = {
	.name			= "ast2700-int",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static int aspeed_intc_ic0_map_irq_domain(struct irq_domain *domain, unsigned int irq,
					  irq_hw_number_t hwirq)
{
	if (hwirq < GIC_P2P_SPI_END)
		irq_set_chip_and_handler(irq, &linear_intr_irq_chip, handle_level_irq);
	else if (hwirq < INTM_BASE)
		irq_set_chip_and_handler(irq, &aspeed_swint_chip, handle_level_irq);
	else if (hwirq < SWINT_BASE)
		return -EINVAL;
	else if (hwirq < INT0_NUM)
		irq_set_chip_and_handler(irq, &aspeed_intm_chip, handle_level_irq);
	else
		return -EINVAL;

	irq_set_chip_data(irq, domain->host_data);
	return 0;
}

static int aspeed_intc0_irq_domain_translate(struct irq_domain *domain,
					     struct irq_fwspec *fwspec,
					     unsigned long *hwirq,
					     unsigned int *type)
{
	if (fwspec->param_count != 1)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = IRQ_TYPE_NONE;
	return 0;
}

static int get_parent_hwirq(struct aspeed_intc_ic *intc_ic, u32 local_hwirq,
			    irq_hw_number_t *parent_hwirq)
{
	u32 out_hwirq;

	if (local_hwirq < GIC_P2P_SPI_END)
		out_hwirq = local_hwirq;
	else if (local_hwirq < INTM_BASE)
		return -EINVAL;
	else if (local_hwirq < INTM_BASE + 10)
		out_hwirq = local_hwirq - INTM_BASE + GIC_INTM0_SPI_BASE;
	else if (local_hwirq < INTM_BASE + 30)
		return -EINVAL;
	else if (local_hwirq < INTM_BASE + 40)
		out_hwirq = local_hwirq - (INTM_BASE + 30) + GIC_INTM30_SPI_BASE;
	else if (local_hwirq < INTM_BASE + 50)
		out_hwirq = local_hwirq - (INTM_BASE + 40) + GIC_INTM40_SPI_BASE;
	else if (local_hwirq < SWINT_BASE + SWINT_NUM)
		out_hwirq = local_hwirq - SWINT_BASE + GIC_SWINT_SPI_BASE;
	else
		return -EINVAL;

	for (int i = 0; i < intc_ic->pin_range_cnt; i++) {
		u32 int_base = intc_ic->pin_ranges[i].int_base;
		u32 cnt = intc_ic->pin_ranges[i].cnt;

		if (out_hwirq >= int_base && out_hwirq < int_base + cnt) {
			*parent_hwirq = out_hwirq;
			return 0;
		}
	}

	return -EINVAL;
}

static int aspeed_intc0_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
					 unsigned int nr_irqs, void *data)
{
	struct aspeed_intc_ic *intc_ic = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t parent_hwirq;
	struct irq_chip *chip;
	unsigned long hwirq;
	unsigned int type;
	int ret;

	ret = aspeed_intc0_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq >= GIC_P2P_SPI_END && hwirq < INT_NUM)
		return -EINVAL;

	if (hwirq < INTM_BASE)
		chip = &linear_intr_irq_chip;
	else if (hwirq < SWINT_BASE)
		chip = &aspeed_intm_chip;
	else
		chip = &aspeed_swint_chip;

	ret = get_parent_hwirq(intc_ic, (u32)hwirq, &parent_hwirq);
	if (ret)
		return ret;

	parent_fwspec.fwnode = domain->parent->fwnode;
	/* The driver assumes parent is GICv3 */
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0] = GIC_SPI;
	parent_fwspec.param[1] = parent_hwirq;
	parent_fwspec.param[2] = IRQ_TYPE_LEVEL_HIGH;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &parent_fwspec);
	if (ret)
		return ret;

	for (int i = 0; i < nr_irqs; ++i, ++hwirq, ++virq) {
		ret = irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
						    chip,
						    domain->host_data);
		if (ret)
			return ret;
	}

	return 0;
}

static int aspeed_intc0_irq_domain_activate(struct irq_domain *domain,
					    struct irq_data *data, bool reserve)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);

	if (data->hwirq < GIC_P2P_SPI_END) {
		int bank = data->hwirq / 32;
		int bit = data->hwirq % 32;
		u32 mask = BIT(bit);

		guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
		for (int i = 0; i < 3; i++) {
			void __iomem *sel = intc_ic->base + 0x200 + bank * 4 + 0x100 * i;

			if (readl(sel) & mask) {
				writel(readl(sel) & ~mask, sel);
				if (readl(sel) & mask)
					return -EACCES;
			}
		}
	} else if (data->hwirq < INT_NUM) {
		return -EINVAL;
	} else if (data->hwirq < INT0_NUM) {
		return 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

static const struct irq_domain_ops aspeed_intc0_ic_irq_domain_ops = {
	.translate = aspeed_intc0_irq_domain_translate,
	.alloc = aspeed_intc0_irq_domain_alloc,
	.free = irq_domain_free_irqs_common,
	.map = aspeed_intc_ic0_map_irq_domain,
	.activate = aspeed_intc0_irq_domain_activate,
};

static int aspeed_intc0_parse_ranges(struct device_node *node,
				     struct device_node *parent_node,
				     struct intc0_pin_range *pin_ranges,
				     unsigned int *range_cnt)
{
	const __be32 *ranges;
	const __be32 *ranges_end;
	int len;
	unsigned int count = 0;

	ranges = of_get_property(node, "aspeed,interrupt-ranges", &len);
	if (!ranges)
		return -EINVAL;

	if (len % sizeof(__be32))
		return -EINVAL;

	if (!pin_ranges) {
		*range_cnt = len / (3 * sizeof(__be32));
		return 0;
	}

	ranges_end = ranges + (len / sizeof(__be32));
	for (; ranges + 3 <= ranges_end; ) {
		struct device_node *target;
		phandle parent_handle;
		u32 target_cells;
		u32 range_size;
		u32 pin_out;
		u32 gic_type;
		u32 gic_base;
		u32 gic_flags;

		pin_out = be32_to_cpu(ranges[0]);
		range_size = be32_to_cpu(ranges[1]);
		parent_handle = be32_to_cpu(ranges[2]);

		target = of_find_node_by_phandle(parent_handle);
		if (!target)
			return -EINVAL;

		if (of_property_read_u32(target, "#interrupt-cells", &target_cells)) {
			of_node_put(target);
			return -EINVAL;
		}

		if (ranges + 3 + target_cells > ranges_end) {
			of_node_put(target);
			return -EINVAL;
		}

		if (target != parent_node) {
			of_node_put(target);
			ranges += 3 + target_cells;
			continue;
		}

		if (target_cells != 3) {
			of_node_put(target);
			return -EINVAL;
		}

		gic_type = be32_to_cpu(ranges[3]);
		gic_base = be32_to_cpu(ranges[4]);
		gic_flags = be32_to_cpu(ranges[5]);

		if (gic_type == GIC_SPI && gic_flags == IRQ_TYPE_LEVEL_HIGH) {
			pin_ranges[count].int_base = pin_out;
			pin_ranges[count].cnt = range_size;
			pin_ranges[count].gic_base = gic_base;
			count++;
		}

		of_node_put(target);
		ranges += 3 + target_cells;
	}

	*range_cnt = count;
	return 0;
}

static int aspeed_intc0_init_gic_ranges(struct aspeed_intc_ic *intc_ic,
					struct device_node *node,
					struct device_node *parent_node)
{
	struct intc0_pin_range *pin_ranges;
	unsigned int range_cnt = 0;
	int ret;

	if (!of_device_is_compatible(parent_node, "arm,gic-v3"))
		return -ENOTSUPP;

	ret = aspeed_intc0_parse_ranges(node, parent_node, NULL, &range_cnt);
	if (ret)
		return ret;

	if (!range_cnt)
		return -EINVAL;

	pin_ranges = devm_kmalloc_array(intc_ic->dev, range_cnt,
					sizeof(*pin_ranges), GFP_KERNEL);
	if (!pin_ranges)
		return -ENOMEM;

	ret = aspeed_intc0_parse_ranges(node, parent_node, pin_ranges, &range_cnt);
	if (ret)
		return ret;

	if (!range_cnt)
		return -EINVAL;

	pin_ranges = devm_krealloc_array(intc_ic->dev, pin_ranges, range_cnt,
					 sizeof(*pin_ranges), GFP_KERNEL);
	if (!pin_ranges)
		return -ENOMEM;

	intc_ic->pin_range_cnt = range_cnt;
	intc_ic->pin_ranges = pin_ranges;

	return 0;
}

static void aspeed_intc0_disable_swint(struct aspeed_intc_ic *intc_ic)
{
	writel(0, intc_ic->base + INTC0_SWINT_IER);
}

static void aspeed_intc0_disable_intbank(struct aspeed_intc_ic *intc_ic)
{
	int i, j;

	for (i = 0; i < INTC0_INTBANK_GROUPS; i++) {
		for (j = 0; j < INTC0_INTBANKS_PER_GRP; j++) {
			u32 base = INTC0_INTBANKX_IER + (0x100 * i) + (0x10 * j);

			writel(0, intc_ic->base + base);
		}
	}
}

static void aspeed_intc0_disable_intm(struct aspeed_intc_ic *intc_ic)
{
	int i;

	for (i = 0; i < INTC0_INTM_BANK_NUM; i++)
		writel(0, intc_ic->base + INTC0_INTMX_IER + (0x10 * i));
}

static int aspeed_intc0_ic_probe(struct platform_device *pdev, struct device_node *parent)
{
	struct device_node *node = pdev->dev.of_node;
	struct irq_domain *parent_domain;
	struct aspeed_intc_ic *intc_ic;
	int ret;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	intc_ic = devm_kzalloc(&pdev->dev, sizeof(*intc_ic), GFP_KERNEL);
	if (!intc_ic)
		return -ENOMEM;

	intc_ic->dev = &pdev->dev;
	intc_ic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc_ic->base))
		return PTR_ERR(intc_ic->base);

	aspeed_intc0_disable_swint(intc_ic);
	aspeed_intc0_disable_intbank(intc_ic);
	aspeed_intc0_disable_intm(intc_ic);

	raw_spin_lock_init(&intc_ic->intc_lock);

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("unable to obtain parent domain\n");
		return -ENODEV;
	}

	intc_ic->irq_domain = irq_domain_create_hierarchy(parent_domain, 0, INT0_NUM,
							  of_fwnode_handle(node),
							  &aspeed_intc0_ic_irq_domain_ops,
							  intc_ic);
	if (!intc_ic->irq_domain)
		return -ENOMEM;

	ret = aspeed_intc0_init_gic_ranges(intc_ic, node, parent);
	if (ret < 0) {
		irq_domain_remove(intc_ic->irq_domain);
		return ret;
	}

	return 0;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(ast2700_intc0)
IRQCHIP_MATCH("aspeed,ast2700-intc0-ic", aspeed_intc0_ic_probe)
IRQCHIP_PLATFORM_DRIVER_END(ast2700_intc0)
