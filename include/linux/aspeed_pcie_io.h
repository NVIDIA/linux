/*
 * arch/arm/mach-ixp4xx/include/mach/io.h
 *
 * Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * Copyright (C) 2002-2005  MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

extern u8 aspeed_pcie_inb(u32 addr);
extern void aspeed_pcie_outb(u8 value, u32 addr);

#endif /* __ASM_ARM_ARCH_IO_H */
