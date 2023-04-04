/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2006 PathScale, Inc.  All Rights Reserved.
 */

#include <linux/linkage.h>

/*
 * override generic version in lib/iomap_copy.c
 */
SYM_FUNC_START(__iowrite32_copy)
	movl %edx,%ecx
<<<<<<< HEAD
	rep movsd
=======
	rep movsl
>>>>>>> origin/linux_6.1.15_upstream
	RET
SYM_FUNC_END(__iowrite32_copy)
