/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
 */
#ifndef _BUILTIN_H
#define _BUILTIN_H

#include <subcmd/parse-options.h>

extern const struct option check_options[];
<<<<<<< HEAD
extern bool no_fp, no_unreachable, retpoline, module, backtrace, uaccess, stats,
            validate_dup, vmlinux, mcount, noinstr, backup, sls;
=======

struct opts {
	/* actions: */
	bool dump_orc;
	bool hack_jump_label;
	bool hack_noinstr;
	bool ibt;
	bool mcount;
	bool noinstr;
	bool orc;
	bool retpoline;
	bool rethunk;
	bool unret;
	bool sls;
	bool stackval;
	bool static_call;
	bool uaccess;

	/* options: */
	bool backtrace;
	bool backup;
	bool dryrun;
	bool link;
	bool module;
	bool no_unreachable;
	bool sec_address;
	bool stats;
};

extern struct opts opts;
>>>>>>> origin/linux_6.1.15_upstream

extern int cmd_parse_options(int argc, const char **argv, const char * const usage[]);

extern int objtool_run(int argc, const char **argv);

#endif /* _BUILTIN_H */
