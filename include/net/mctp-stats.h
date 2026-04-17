/*
 * SPDX-FileCopyrightText: Copyright (c)  NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * MCTP Per-EID Statistics - Shared Infrastructure
 *
 * This header provides macros and helpers for tracking statistics
 * per-endpoint-ID (EID) across all MCTP bindings.
 */

#ifndef __MCTP_STATS_H
#define __MCTP_STATS_H

#include <linux/bitmap.h>
#include <linux/types.h>
#include <linux/ethtool.h>

/**
 * MCTP_EID_UNKNOWN - Special EID value for errors where EID is not yet known
 *
 * Value: 256 (out-of-band value to avoid conflict with valid MCTP EIDs 0-254,
 *        and distinct from broadcast address 255)
 *
 * This EID is used for tracking errors that occur when the actual endpoint ID
 * cannot be determined. This happens in several scenarios:
 *
 * 1. PRE-PARSE ERRORS: Packets are corrupted/invalid before we can read the
 *    MCTP header (e.g., invalid I2C command code, PEC checksum failure,
 *    packet too short)
 *
 * 2. ALLOCATION FAILURES: Memory allocation failed before we could parse the
 *    packet to extract the source/destination EID
 *
 * 3. ASYNCHRONOUS EVENTS: Hardware/transport events with no packet context
 *    (e.g., USB URB completion callbacks, I3C IBI events, GPIO interrupts,
 *    device hotplug events)
 *
 * 4. TRANSPORT-LEVEL ERRORS: Low-level bus failures before packet transmission
 *    (e.g., SPI transfer errors, I2C arbitration loss on unrelated traffic)
 *
 * IMPORTANT DISTINCTION from EID 0:
 * - UNKNOWN (256): We don't know the EID (error occurred too early)
 * - EID 0: We successfully read the EID from the packet, and it was 0
 *          (EID 0 is the "null endpoint" per MCTP spec DSP0236, used for
 *          unallocated/unassigned endpoints)
 */
#define MCTP_EID_UNKNOWN 256

/**
 * MCTP_STAT_INC - Increment per-EID statistics and mark EID as active
 * @dev_ptr: Pointer to device structure (e.g., midev, mbus, musb)
 * @eid_val: EID value (0-256) - use MCTP_EID_UNKNOWN if EID not yet known
 * @stat_field: Name of the statistics field
 *
 * This macro is the SINGLE way to track statistics. It updates the per-EID
 * stats counter for the specified field and marks the EID as active.
 *
 * Usage:
 *   // When EID is known:
 *   MCTP_STAT_INC(midev, dest_eid, tx_drop_timeout);
 *
 *   // When EID is not yet known (before parsing):
 *   MCTP_STAT_INC(midev, MCTP_EID_UNKNOWN, rx_drop_invalid_cmd);
 */
#define MCTP_STAT_INC(dev_ptr, eid_val, stat_field) do { \
	typeof(dev_ptr) __dev = (dev_ptr); \
	unsigned int __eid = (eid_val); \
	set_bit(__eid, __dev->eid_stats.active); \
	__dev->eid_stats.eid[__eid].stat_field++; \
} while (0)

/**
 * MCTP_EID_STATS_HELPERS - Generate ethtool helper functions for per-EID stats
 * @prefix: Prefix for generated function names (e.g., mctp_i2c, mctp_usb)
 * @dev_type: Type of device structure (e.g., struct mctp_i2c_dev)
 * @eid_stats_type: Type of per-EID stats structure
 * @stat_descs: Array name of stat descriptors
 *
 * Generates three helper functions:
 * - prefix_count_eid_nonzero(): Count non-zero stats for one EID
 * - prefix_count_eid_stats(): Count total stats for all active EIDs
 * - prefix_eid_stats_total(): Sum all stats for one EID
 *
 * These functions are used by ethtool callbacks to determine output size
 * and generate the per-EID statistics display.
 */
#define MCTP_EID_STATS_HELPERS(prefix, dev_type, eid_stats_type, stat_descs) \
\
static int prefix##_count_eid_nonzero(dev_type *dev, unsigned int eid) \
{ \
	eid_stats_type *es = &dev->eid_stats.eid[eid]; \
	u8 *base = (u8 *)es; \
	int count = 0; \
	unsigned int i; \
	\
	for (i = 0; i < ARRAY_SIZE(stat_descs); i++) { \
		if (*(u64 *)(base + stat_descs[i].offset) != 0) \
			count++; \
	} \
	return count; \
} \
\
static int prefix##_count_eid_stats(dev_type *dev) \
{ \
	int count = 0; \
	int eid; \
	\
	for_each_set_bit(eid, dev->eid_stats.active, 257) { \
		int nz = prefix##_count_eid_nonzero(dev, eid); \
		if (nz > 0) \
			count += 1 + nz; \
	} \
	return count; \
} \
\
static u64 prefix##_eid_stats_total(dev_type *dev, unsigned int eid) \
{ \
	eid_stats_type *es = &dev->eid_stats.eid[eid]; \
	u8 *base = (u8 *)es; \
	u64 total = 0; \
	unsigned int i; \
	\
	for (i = 0; i < ARRAY_SIZE(stat_descs); i++) \
		total += *(u64 *)(base + stat_descs[i].offset); \
	return total; \
}

/**
 * MCTP_EID_GET_STRINGS - Generate ethtool string names for per-EID stats
 * @dev: Device structure pointer
 * @data_ptr: Pointer to data buffer pointer (will be advanced)
 * @stat_descs: Array of stat descriptors
 *
 * Outputs per-EID stat names in ethtool format. Only outputs stats for
 * active EIDs with non-zero values. EID 256 (MCTP_EID_UNKNOWN) is displayed
 * as "UNKNOWN" instead of "EID_256".
 */
#define MCTP_EID_GET_STRINGS(dev, data_ptr, stat_descs, count_nonzero_fn) do { \
	int eid; \
	unsigned int i; \
	\
	for_each_set_bit(eid, (dev)->eid_stats.active, 257) { \
		typeof(&(dev)->eid_stats.eid[0]) es = &(dev)->eid_stats.eid[eid]; \
		u8 *base = (u8 *)es; \
		int nz = count_nonzero_fn((dev), eid); \
		\
		if (nz == 0) \
			continue; \
		\
		if (eid == MCTP_EID_UNKNOWN) \
			snprintf(*(data_ptr), ETH_GSTRING_LEN, "UNKNOWN                       "); \
		else \
			snprintf(*(data_ptr), ETH_GSTRING_LEN, "EID_%-3u                       ", eid); \
		*(data_ptr) += ETH_GSTRING_LEN; \
		\
		for (i = 0; i < ARRAY_SIZE(stat_descs); i++) { \
			u64 val = *(u64 *)(base + stat_descs[i].offset); \
			if (val != 0) { \
				snprintf(*(data_ptr), ETH_GSTRING_LEN, "%-30s", \
					 stat_descs[i].name); \
				*(data_ptr) += ETH_GSTRING_LEN; \
			} \
		} \
	} \
} while (0)

/**
 * MCTP_EID_GET_STATS - Generate ethtool stat values for per-EID stats
 * @dev: Device structure pointer
 * @data_ptr: Pointer to data array pointer (will be advanced)
 * @stat_descs: Array of stat descriptors
 * @count_nonzero_fn: Function to count non-zero stats
 * @total_fn: Function to get total for an EID
 *
 * Outputs per-EID stat values in ethtool format. Only outputs stats for
 * active EIDs with non-zero values.
 */
#define MCTP_EID_GET_STATS(dev, data_ptr, stat_descs, count_nonzero_fn, total_fn) do { \
	int eid; \
	unsigned int i; \
	\
	for_each_set_bit(eid, (dev)->eid_stats.active, 257) { \
		typeof(&(dev)->eid_stats.eid[0]) es = &(dev)->eid_stats.eid[eid]; \
		u8 *base = (u8 *)es; \
		int nz = count_nonzero_fn((dev), eid); \
		\
		if (nz == 0) \
			continue; \
		\
		(*(data_ptr))[0] = total_fn((dev), eid); \
		(*data_ptr)++; \
		\
		for (i = 0; i < ARRAY_SIZE(stat_descs); i++) { \
			u64 val = *(u64 *)(base + stat_descs[i].offset); \
			if (val != 0) { \
				(*(data_ptr))[0] = val; \
				(*data_ptr)++; \
			} \
		} \
	} \
} while (0)

#endif /* __MCTP_STATS_H */
