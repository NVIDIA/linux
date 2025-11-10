/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MCTP Socket Error Injection - Header
 * 
 * Binding-agnostic error injection at AF_MCTP socket layer.
 * Tests application handling of sendto() failures.
 */

#ifndef __MCTP_SOCKET_ERROR_INJECT_H
#define __MCTP_SOCKET_ERROR_INJECT_H

#include <linux/types.h>
#include <linux/debugfs.h>

/* Socket-level error injection state */
struct mctp_socket_error_inject {
	/* Error injection control */
	bool enabled;                /* Enable/disable error injection */
	int error_code;              /* Error to return (e.g. -EBUSY, -EHOSTUNREACH) */
	
	/* Statistics */
	u64 sendmsg_calls;           /* Total sendmsg() calls */
	u64 errors_injected;         /* Errors injected */
	
	/* Debugfs */
	struct dentry *debugfs_dir;
	spinlock_t lock;
};

/* Global instance */
extern struct mctp_socket_error_inject mctp_socket_ei;

/* Initialize/cleanup */
int mctp_socket_error_inject_init(void);
void mctp_socket_error_inject_cleanup(void);

/* Check if error should be injected in sendmsg() */
int mctp_socket_error_inject_sendmsg(void);

#endif /* __MCTP_SOCKET_ERROR_INJECT_H */

