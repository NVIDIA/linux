// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */
#include "irq-ast2700.h"

#include <linux/dev_printk.h>
#include <linux/device/devres.h>

int aspeed_intc_populate_ranges(struct device *dev,
				struct aspeed_intc_interrupt_ranges *ranges)
{
	struct aspeed_intc_interrupt_range *arr;
	const __be32 *pvs, *pve;
	struct device_node *dn;
	int len;

	if (!dev || !ranges)
		return -EINVAL;

	dn = dev->of_node;

	pvs = of_get_property(dn, "aspeed,interrupt-ranges", &len);
	if (!pvs)
		return -EINVAL;

	if (len % sizeof(__be32))
		return -EINVAL;

	/* Over-estimate the range entry count for now */
	ranges->ranges = devm_kmalloc_array(dev, (len / (3 * sizeof(__be32))),
					    sizeof(*ranges->ranges),
					    GFP_KERNEL);
	if (!ranges->ranges)
		return -ENOMEM;

	pve = pvs + (len / sizeof(__be32));
	for (unsigned int i = 0; pve - pvs >= 3; i++) {
		struct aspeed_intc_interrupt_range *r;
		struct device_node *target;
		u32 target_cells;

		target = of_find_node_by_phandle(be32_to_cpu(pvs[2]));
		if (!target)
			return -EINVAL;

		if (of_property_read_u32(target, "#interrupt-cells",
					 &target_cells)) {
			of_node_put(target);
			return -EINVAL;
		}

		if (!target_cells || target_cells > IRQ_DOMAIN_IRQ_SPEC_PARAMS) {
			of_node_put(target);
			return -EINVAL;
		}

		if (pve - pvs < 3 + target_cells) {
			of_node_put(target);
			return -EINVAL;
		}

		r = &ranges->ranges[i];
		r->start = be32_to_cpu(pvs[0]);
		r->count = be32_to_cpu(pvs[1]);

		{
			struct of_phandle_args args = {
				.np = target,
				.args_count = target_cells,
			};

			for (u32 j = 0; j < target_cells; j++)
				args.args[j] = be32_to_cpu(pvs[3 + j]);

			of_phandle_args_to_fwspec(target, args.args,
						  args.args_count,
						  &r->upstream);
		}

		if (target_cells >= 1)
			dev_dbg(dev,
				"Mapped %u outputs from %u to %u on parent %s",
				r->count, r->start, r->upstream.param[0], target->full_name);
		else
			dev_dbg(dev,
				"Registered interrupt range from %u for count %u\n",
				r->start, r->count);

		of_node_put(target);
		pvs += 3 + target_cells;
		ranges->nranges++;
	}

	/* Re-fit the range array now we know the entry count */
	arr = devm_krealloc_array(dev, ranges->ranges, ranges->nranges,
				  sizeof(*ranges->ranges), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;
	ranges->ranges = arr;

	return 0;
}
