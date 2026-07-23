/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP SPI Error Injection Infrastructure
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 * Provides debugfs-based error injection for testing MCTP error queue
 * functionality over SPI binding.
 *
 * Generic plumbing lives in mctp-error-inject-common.c.  SPI is a TX-only
 * binding (no RX/fragment injection), and its TX path uses SPB_AP status
 * codes, so only the SPI-specific error code/rate and hook are kept here.
 */

#ifndef __MCTP_SPI_ERROR_INJECT_H
#define __MCTP_SPI_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>

#include <net/mctp-error-inject-common.h>

/* Forward declarations */
struct dentry;

/* Error injection control structure - common state plus SPI-specific fields. */
struct mctp_spi_error_inject {
	struct mctp_ei_common common;

	/* TX injection - SPI uses SPB_AP return codes */
	int spi_tx_error_code;   /* SPB_AP error code (SPB_AP_ERROR_TIMEOUT, etc.) */
	u32 spi_tx_error_rate;   /* Percentage (0-100) or count */
	u32 spi_tx_inject_count; /* Counter for count mode */
	u32 spi_tx_errors_injected;
};

/* Forward declaration - full definition in mctp-spi.c */
struct mctp_spi;

/* Module init/exit functions */
int mctp_spi_error_inject_module_init(void);
void mctp_spi_error_inject_module_exit(void);

/* Public API for main driver */
void mctp_spi_error_inject_init(struct mctp_spi *midev);
void mctp_spi_error_inject_cleanup(struct mctp_spi *midev);

int mctp_spi_error_inject_tx(struct mctp_spi *midev, struct sk_buff *skb, int *injected_status);

#endif /* __MCTP_SPI_ERROR_INJECT_H */
