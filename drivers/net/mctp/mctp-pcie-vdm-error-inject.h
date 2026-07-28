/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP-over-PCIe-VDM diagnostic error injection
 *
 * Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef __MCTP_PCIE_VDM_ERROR_INJECT_H
#define __MCTP_PCIE_VDM_ERROR_INJECT_H

#include <linux/kconfig.h>
#include <linux/types.h>

struct sk_buff;
struct mctp_pcie_vdm_error_inject;

#if IS_ENABLED(CONFIG_MCTP_TRANSPORT_PCIE_VDM_ERROR_INJECT)

struct mctp_pcie_vdm_error_inject *
mctp_pcie_vdm_error_inject_init(const char *ifname);
void mctp_pcie_vdm_error_inject_cleanup(struct mctp_pcie_vdm_error_inject *inject);
bool mctp_pcie_vdm_error_inject_tx(struct mctp_pcie_vdm_error_inject *inject,
				   struct sk_buff *skb);
bool mctp_pcie_vdm_error_inject_rx(struct mctp_pcie_vdm_error_inject *inject,
				   struct sk_buff *skb);

#else

static inline struct mctp_pcie_vdm_error_inject *
mctp_pcie_vdm_error_inject_init(const char *ifname)
{
	return NULL;
}

static inline void
mctp_pcie_vdm_error_inject_cleanup(struct mctp_pcie_vdm_error_inject *inject)
{
}

static inline bool
mctp_pcie_vdm_error_inject_tx(struct mctp_pcie_vdm_error_inject *inject,
			      struct sk_buff *skb)
{
	return false;
}

static inline bool
mctp_pcie_vdm_error_inject_rx(struct mctp_pcie_vdm_error_inject *inject,
			      struct sk_buff *skb)
{
	return false;
}

#endif

#endif /* __MCTP_PCIE_VDM_ERROR_INJECT_H */
