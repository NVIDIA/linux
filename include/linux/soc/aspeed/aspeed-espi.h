/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Intel Corporation. */

#ifndef _LINUX_SOC_ASPEED_ESPI_H
#define _LINUX_SOC_ASPEED_ESPI_H

typedef void (*aspeed_espi_irq_handler)(int irq, void *data);
void aspeed_espi_register_gpio(struct device *dev,
			       aspeed_espi_irq_handler handler, void *data);

#endif /* _LINUX_SOC_ASPEED_ESPI_H */
