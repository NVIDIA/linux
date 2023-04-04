/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _BLK_CGROUP_H
#define _BLK_CGROUP_H
/*
 * Common Block IO controller cgroup interface
 *
 * Based on ideas and code from CFQ, CFS and BFQ:
 * Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 *
 * Copyright (C) 2008 Fabio Checconi <fabio@gandalf.sssup.it>
 *		      Paolo Valente <paolo.valente@unimore.it>
 *
 * Copyright (C) 2009 Vivek Goyal <vgoyal@redhat.com>
 * 	              Nauman Rafique <nauman@google.com>
 */

<<<<<<< HEAD
#include <linux/cgroup.h>
#include <linux/percpu.h>
#include <linux/percpu_counter.h>
#include <linux/u64_stats_sync.h>
#include <linux/seq_file.h>
#include <linux/radix-tree.h>
#include <linux/blkdev.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/blk-mq.h>
=======
#include <linux/types.h>
>>>>>>> origin/linux_6.1.15_upstream

struct bio;
struct cgroup_subsys_state;
struct gendisk;

#define FC_APPID_LEN              129

#ifdef CONFIG_BLK_CGROUP
extern struct cgroup_subsys_state * const blkcg_root_css;

<<<<<<< HEAD
/**
 * bio_blkcg - grab the blkcg associated with a bio
 * @bio: target bio
 *
 * This returns the blkcg associated with a bio, %NULL if not associated.
 * Callers are expected to either handle %NULL or know association has been
 * done prior to calling this.
 */
static inline struct blkcg *bio_blkcg(struct bio *bio)
{
	if (bio && bio->bi_blkg)
		return bio->bi_blkg->blkcg;
	return NULL;
}

static inline bool blk_cgroup_congested(void)
{
	struct cgroup_subsys_state *css;
	bool ret = false;

	rcu_read_lock();
	css = kthread_blkcg();
	if (!css)
		css = task_css(current, io_cgrp_id);
	while (css) {
		if (atomic_read(&css->cgroup->congestion_count)) {
			ret = true;
			break;
		}
		css = css->parent;
	}
	rcu_read_unlock();
	return ret;
}

/**
 * bio_issue_as_root_blkg - see if this bio needs to be issued as root blkg
 * @return: true if this bio needs to be submitted with the root blkg context.
 *
 * In order to avoid priority inversions we sometimes need to issue a bio as if
 * it were attached to the root blkg, and then backcharge to the actual owning
 * blkg.  The idea is we do bio_blkcg() to look up the actual context for the
 * bio and attach the appropriate blkg to the bio.  Then we call this helper and
 * if it is true run with the root blkg for that queue and then do any
 * backcharging to the originating cgroup once the io is complete.
 */
static inline bool bio_issue_as_root_blkg(struct bio *bio)
{
	return (bio->bi_opf & (REQ_META | REQ_SWAP)) != 0;
}

/**
 * blkcg_parent - get the parent of a blkcg
 * @blkcg: blkcg of interest
 *
 * Return the parent blkcg of @blkcg.  Can be called anytime.
 */
static inline struct blkcg *blkcg_parent(struct blkcg *blkcg)
{
	return css_to_blkcg(blkcg->css.parent);
}

/**
 * __blkg_lookup - internal version of blkg_lookup()
 * @blkcg: blkcg of interest
 * @q: request_queue of interest
 * @update_hint: whether to update lookup hint with the result or not
 *
 * This is internal version and shouldn't be used by policy
 * implementations.  Looks up blkgs for the @blkcg - @q pair regardless of
 * @q's bypass state.  If @update_hint is %true, the caller should be
 * holding @q->queue_lock and lookup hint is updated on success.
 */
static inline struct blkcg_gq *__blkg_lookup(struct blkcg *blkcg,
					     struct request_queue *q,
					     bool update_hint)
{
	struct blkcg_gq *blkg;

	if (blkcg == &blkcg_root)
		return q->root_blkg;

	blkg = rcu_dereference(blkcg->blkg_hint);
	if (blkg && blkg->q == q)
		return blkg;

	return blkg_lookup_slowpath(blkcg, q, update_hint);
}

/**
 * blkg_lookup - lookup blkg for the specified blkcg - q pair
 * @blkcg: blkcg of interest
 * @q: request_queue of interest
 *
 * Lookup blkg for the @blkcg - @q pair.  This function should be called
 * under RCU read lock.
 */
static inline struct blkcg_gq *blkg_lookup(struct blkcg *blkcg,
					   struct request_queue *q)
{
	WARN_ON_ONCE(!rcu_read_lock_held());
	return __blkg_lookup(blkcg, q, false);
}

/**
 * blk_queue_root_blkg - return blkg for the (blkcg_root, @q) pair
 * @q: request_queue of interest
 *
 * Lookup blkg for @q at the root level. See also blkg_lookup().
 */
static inline struct blkcg_gq *blk_queue_root_blkg(struct request_queue *q)
{
	return q->root_blkg;
}

/**
 * blkg_to_pdata - get policy private data
 * @blkg: blkg of interest
 * @pol: policy of interest
 *
 * Return pointer to private data associated with the @blkg-@pol pair.
 */
static inline struct blkg_policy_data *blkg_to_pd(struct blkcg_gq *blkg,
						  struct blkcg_policy *pol)
{
	return blkg ? blkg->pd[pol->plid] : NULL;
}

static inline struct blkcg_policy_data *blkcg_to_cpd(struct blkcg *blkcg,
						     struct blkcg_policy *pol)
{
	return blkcg ? blkcg->cpd[pol->plid] : NULL;
}

/**
 * pdata_to_blkg - get blkg associated with policy private data
 * @pd: policy private data of interest
 *
 * @pd is policy private data.  Determine the blkg it's associated with.
 */
static inline struct blkcg_gq *pd_to_blkg(struct blkg_policy_data *pd)
{
	return pd ? pd->blkg : NULL;
}

static inline struct blkcg *cpd_to_blkcg(struct blkcg_policy_data *cpd)
{
	return cpd ? cpd->blkcg : NULL;
}

extern void blkcg_destroy_blkgs(struct blkcg *blkcg);

/**
 * blkcg_pin_online - pin online state
 * @blkcg: blkcg of interest
 *
 * While pinned, a blkcg is kept online.  This is primarily used to
 * impedance-match blkg and cgwb lifetimes so that blkg doesn't go offline
 * while an associated cgwb is still active.
 */
static inline void blkcg_pin_online(struct blkcg *blkcg)
{
	refcount_inc(&blkcg->online_pin);
}

/**
 * blkcg_unpin_online - unpin online state
 * @blkcg: blkcg of interest
 *
 * This is primarily used to impedance-match blkg and cgwb lifetimes so
 * that blkg doesn't go offline while an associated cgwb is still active.
 * When this count goes to zero, all active cgwbs have finished so the
 * blkcg can continue destruction by calling blkcg_destroy_blkgs().
 */
static inline void blkcg_unpin_online(struct blkcg *blkcg)
{
	do {
		if (!refcount_dec_and_test(&blkcg->online_pin))
			break;
		blkcg_destroy_blkgs(blkcg);
		blkcg = blkcg_parent(blkcg);
	} while (blkcg);
}

/**
 * blkg_path - format cgroup path of blkg
 * @blkg: blkg of interest
 * @buf: target buffer
 * @buflen: target buffer length
 *
 * Format the path of the cgroup of @blkg into @buf.
 */
static inline int blkg_path(struct blkcg_gq *blkg, char *buf, int buflen)
{
	return cgroup_path(blkg->blkcg->css.cgroup, buf, buflen);
}

/**
 * blkg_get - get a blkg reference
 * @blkg: blkg to get
 *
 * The caller should be holding an existing reference.
 */
static inline void blkg_get(struct blkcg_gq *blkg)
{
	percpu_ref_get(&blkg->refcnt);
}

/**
 * blkg_tryget - try and get a blkg reference
 * @blkg: blkg to get
 *
 * This is for use when doing an RCU lookup of the blkg.  We may be in the midst
 * of freeing this blkg, so we can only use it if the refcnt is not zero.
 */
static inline bool blkg_tryget(struct blkcg_gq *blkg)
{
	return blkg && percpu_ref_tryget(&blkg->refcnt);
}

/**
 * blkg_put - put a blkg reference
 * @blkg: blkg to put
 */
static inline void blkg_put(struct blkcg_gq *blkg)
{
	percpu_ref_put(&blkg->refcnt);
}

/**
 * blkg_for_each_descendant_pre - pre-order walk of a blkg's descendants
 * @d_blkg: loop cursor pointing to the current descendant
 * @pos_css: used for iteration
 * @p_blkg: target blkg to walk descendants of
 *
 * Walk @c_blkg through the descendants of @p_blkg.  Must be used with RCU
 * read locked.  If called under either blkcg or queue lock, the iteration
 * is guaranteed to include all and only online blkgs.  The caller may
 * update @pos_css by calling css_rightmost_descendant() to skip subtree.
 * @p_blkg is included in the iteration and the first node to be visited.
 */
#define blkg_for_each_descendant_pre(d_blkg, pos_css, p_blkg)		\
	css_for_each_descendant_pre((pos_css), &(p_blkg)->blkcg->css)	\
		if (((d_blkg) = __blkg_lookup(css_to_blkcg(pos_css),	\
					      (p_blkg)->q, false)))

/**
 * blkg_for_each_descendant_post - post-order walk of a blkg's descendants
 * @d_blkg: loop cursor pointing to the current descendant
 * @pos_css: used for iteration
 * @p_blkg: target blkg to walk descendants of
 *
 * Similar to blkg_for_each_descendant_pre() but performs post-order
 * traversal instead.  Synchronization rules are the same.  @p_blkg is
 * included in the iteration and the last node to be visited.
 */
#define blkg_for_each_descendant_post(d_blkg, pos_css, p_blkg)		\
	css_for_each_descendant_post((pos_css), &(p_blkg)->blkcg->css)	\
		if (((d_blkg) = __blkg_lookup(css_to_blkcg(pos_css),	\
					      (p_blkg)->q, false)))

bool __blkcg_punt_bio_submit(struct bio *bio);

static inline bool blkcg_punt_bio_submit(struct bio *bio)
{
	if (bio->bi_opf & REQ_CGROUP_PUNT)
		return __blkcg_punt_bio_submit(bio);
	else
		return false;
}

static inline void blkcg_bio_issue_init(struct bio *bio)
{
	bio_issue_init(&bio->bi_issue, bio_sectors(bio));
}

static inline void blkcg_use_delay(struct blkcg_gq *blkg)
{
	if (WARN_ON_ONCE(atomic_read(&blkg->use_delay) < 0))
		return;
	if (atomic_add_return(1, &blkg->use_delay) == 1)
		atomic_inc(&blkg->blkcg->css.cgroup->congestion_count);
}

static inline int blkcg_unuse_delay(struct blkcg_gq *blkg)
{
	int old = atomic_read(&blkg->use_delay);

	if (WARN_ON_ONCE(old < 0))
		return 0;
	if (old == 0)
		return 0;

	/*
	 * We do this song and dance because we can race with somebody else
	 * adding or removing delay.  If we just did an atomic_dec we'd end up
	 * negative and we'd already be in trouble.  We need to subtract 1 and
	 * then check to see if we were the last delay so we can drop the
	 * congestion count on the cgroup.
	 */
	while (old) {
		int cur = atomic_cmpxchg(&blkg->use_delay, old, old - 1);
		if (cur == old)
			break;
		old = cur;
	}

	if (old == 0)
		return 0;
	if (old == 1)
		atomic_dec(&blkg->blkcg->css.cgroup->congestion_count);
	return 1;
}

/**
 * blkcg_set_delay - Enable allocator delay mechanism with the specified delay amount
 * @blkg: target blkg
 * @delay: delay duration in nsecs
 *
 * When enabled with this function, the delay is not decayed and must be
 * explicitly cleared with blkcg_clear_delay(). Must not be mixed with
 * blkcg_[un]use_delay() and blkcg_add_delay() usages.
 */
static inline void blkcg_set_delay(struct blkcg_gq *blkg, u64 delay)
{
	int old = atomic_read(&blkg->use_delay);

	/* We only want 1 person setting the congestion count for this blkg. */
	if (!old && atomic_cmpxchg(&blkg->use_delay, old, -1) == old)
		atomic_inc(&blkg->blkcg->css.cgroup->congestion_count);

	atomic64_set(&blkg->delay_nsec, delay);
}

/**
 * blkcg_clear_delay - Disable allocator delay mechanism
 * @blkg: target blkg
 *
 * Disable use_delay mechanism. See blkcg_set_delay().
 */
static inline void blkcg_clear_delay(struct blkcg_gq *blkg)
{
	int old = atomic_read(&blkg->use_delay);

	/* We only want 1 person clearing the congestion count for this blkg. */
	if (old && atomic_cmpxchg(&blkg->use_delay, old, 0) == old)
		atomic_dec(&blkg->blkcg->css.cgroup->congestion_count);
}

/**
 * blk_cgroup_mergeable - Determine whether to allow or disallow merges
 * @rq: request to merge into
 * @bio: bio to merge
 *
 * @bio and @rq should belong to the same cgroup and their issue_as_root should
 * match. The latter is necessary as we don't want to throttle e.g. a metadata
 * update because it happens to be next to a regular IO.
 */
static inline bool blk_cgroup_mergeable(struct request *rq, struct bio *bio)
{
	return rq->bio->bi_blkg == bio->bi_blkg &&
		bio_issue_as_root_blkg(rq->bio) == bio_issue_as_root_blkg(bio);
}

void blk_cgroup_bio_start(struct bio *bio);
void blkcg_add_delay(struct blkcg_gq *blkg, u64 now, u64 delta);
void blkcg_schedule_throttle(struct request_queue *q, bool use_memdelay);
=======
void blkcg_schedule_throttle(struct gendisk *disk, bool use_memdelay);
>>>>>>> origin/linux_6.1.15_upstream
void blkcg_maybe_throttle_current(void);
bool blk_cgroup_congested(void);
void blkcg_pin_online(struct cgroup_subsys_state *blkcg_css);
void blkcg_unpin_online(struct cgroup_subsys_state *blkcg_css);
struct list_head *blkcg_get_cgwb_list(struct cgroup_subsys_state *css);
struct cgroup_subsys_state *bio_blkcg_css(struct bio *bio);

#else	/* CONFIG_BLK_CGROUP */

#define blkcg_root_css	((struct cgroup_subsys_state *)ERR_PTR(-EINVAL))

static inline void blkcg_maybe_throttle_current(void) { }
static inline bool blk_cgroup_congested(void) { return false; }
<<<<<<< HEAD

#ifdef CONFIG_BLOCK

static inline void blkcg_schedule_throttle(struct request_queue *q, bool use_memdelay) { }

static inline struct blkcg_gq *blkg_lookup(struct blkcg *blkcg, void *key) { return NULL; }
static inline struct blkcg_gq *blk_queue_root_blkg(struct request_queue *q)
{ return NULL; }
static inline int blkcg_init_queue(struct request_queue *q) { return 0; }
static inline void blkcg_exit_queue(struct request_queue *q) { }
static inline int blkcg_policy_register(struct blkcg_policy *pol) { return 0; }
static inline void blkcg_policy_unregister(struct blkcg_policy *pol) { }
static inline int blkcg_activate_policy(struct request_queue *q,
					const struct blkcg_policy *pol) { return 0; }
static inline void blkcg_deactivate_policy(struct request_queue *q,
					   const struct blkcg_policy *pol) { }

static inline struct blkcg *__bio_blkcg(struct bio *bio) { return NULL; }
static inline struct blkcg *bio_blkcg(struct bio *bio) { return NULL; }

static inline struct blkg_policy_data *blkg_to_pd(struct blkcg_gq *blkg,
						  struct blkcg_policy *pol) { return NULL; }
static inline struct blkcg_gq *pd_to_blkg(struct blkg_policy_data *pd) { return NULL; }
static inline char *blkg_path(struct blkcg_gq *blkg) { return NULL; }
static inline void blkg_get(struct blkcg_gq *blkg) { }
static inline void blkg_put(struct blkcg_gq *blkg) { }

static inline bool blkcg_punt_bio_submit(struct bio *bio) { return false; }
static inline void blkcg_bio_issue_init(struct bio *bio) { }
static inline void blk_cgroup_bio_start(struct bio *bio) { }
static inline bool blk_cgroup_mergeable(struct request *rq, struct bio *bio) { return true; }

#define blk_queue_for_each_rl(rl, q)	\
	for ((rl) = &(q)->root_rl; (rl); (rl) = NULL)

#endif	/* CONFIG_BLOCK */
#endif	/* CONFIG_BLK_CGROUP */

#ifdef CONFIG_BLK_CGROUP_FC_APPID
/*
 * Sets the fc_app_id field associted to blkcg
 * @app_id: application identifier
 * @cgrp_id: cgroup id
 * @app_id_len: size of application identifier
 */
static inline int blkcg_set_fc_appid(char *app_id, u64 cgrp_id, size_t app_id_len)
{
	struct cgroup *cgrp;
	struct cgroup_subsys_state *css;
	struct blkcg *blkcg;
	int ret  = 0;

	if (app_id_len > FC_APPID_LEN)
		return -EINVAL;

	cgrp = cgroup_get_from_id(cgrp_id);
	if (!cgrp)
		return -ENOENT;
	css = cgroup_get_e_css(cgrp, &io_cgrp_subsys);
	if (!css) {
		ret = -ENOENT;
		goto out_cgrp_put;
	}
	blkcg = css_to_blkcg(css);
	/*
	 * There is a slight race condition on setting the appid.
	 * Worst case an I/O may not find the right id.
	 * This is no different from the I/O we let pass while obtaining
	 * the vmid from the fabric.
	 * Adding the overhead of a lock is not necessary.
	 */
	strlcpy(blkcg->fc_app_id, app_id, app_id_len);
	css_put(css);
out_cgrp_put:
	cgroup_put(cgrp);
	return ret;
}

/**
 * blkcg_get_fc_appid - get the fc app identifier associated with a bio
 * @bio: target bio
 *
 * On success return the fc_app_id, on failure return NULL
 */
static inline char *blkcg_get_fc_appid(struct bio *bio)
=======
static inline struct cgroup_subsys_state *bio_blkcg_css(struct bio *bio)
>>>>>>> origin/linux_6.1.15_upstream
{
	return NULL;
}
#endif	/* CONFIG_BLK_CGROUP */

int blkcg_set_fc_appid(char *app_id, u64 cgrp_id, size_t app_id_len);
char *blkcg_get_fc_appid(struct bio *bio);

#endif	/* _BLK_CGROUP_H */
