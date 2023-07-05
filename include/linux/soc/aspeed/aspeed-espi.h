/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Intel Corporation. */

typedef void (*aspeed_espi_irq_handler)(int, void *);
void aspeed_espi_register_gpio(struct device *espi_ctrl,
			       aspeed_espi_irq_handler handler, void *data);
