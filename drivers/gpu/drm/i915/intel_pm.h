/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_PM_H__
#define __INTEL_PM_H__

#include <linux/types.h>

struct drm_i915_private;
struct intel_crtc_state;
struct intel_plane_state;

void intel_init_clock_gating(struct drm_i915_private *dev_priv);
void intel_suspend_hw(struct drm_i915_private *dev_priv);
int ilk_wm_max_level(const struct drm_i915_private *dev_priv);
void intel_init_pm(struct drm_i915_private *dev_priv);
void intel_init_clock_gating_hooks(struct drm_i915_private *dev_priv);
void intel_pm_setup(struct drm_i915_private *dev_priv);
void g4x_wm_get_hw_state(struct drm_i915_private *dev_priv);
void vlv_wm_get_hw_state(struct drm_i915_private *dev_priv);
void ilk_wm_get_hw_state(struct drm_i915_private *dev_priv);
void g4x_wm_sanitize(struct drm_i915_private *dev_priv);
void vlv_wm_sanitize(struct drm_i915_private *dev_priv);
<<<<<<< HEAD
void skl_wm_sanitize(struct drm_i915_private *dev_priv);
bool intel_can_enable_sagv(struct drm_i915_private *dev_priv,
			   const struct intel_bw_state *bw_state);
void intel_sagv_pre_plane_update(struct intel_atomic_state *state);
void intel_sagv_post_plane_update(struct intel_atomic_state *state);
const struct skl_wm_level *skl_plane_wm_level(const struct skl_pipe_wm *pipe_wm,
					      enum plane_id plane_id,
					      int level);
const struct skl_wm_level *skl_plane_trans_wm(const struct skl_pipe_wm *pipe_wm,
					      enum plane_id plane_id);
bool skl_wm_level_equals(const struct skl_wm_level *l1,
			 const struct skl_wm_level *l2);
bool skl_ddb_allocation_overlaps(const struct skl_ddb_entry *ddb,
				 const struct skl_ddb_entry *entries,
				 int num_entries, int ignore_idx);
void skl_write_plane_wm(struct intel_plane *plane,
			const struct intel_crtc_state *crtc_state);
void skl_write_cursor_wm(struct intel_plane *plane,
			 const struct intel_crtc_state *crtc_state);
=======
>>>>>>> origin/linux_6.1.15_upstream
bool ilk_disable_lp_wm(struct drm_i915_private *dev_priv);
bool intel_wm_plane_visible(const struct intel_crtc_state *crtc_state,
			    const struct intel_plane_state *plane_state);
void intel_print_wm_latency(struct drm_i915_private *dev_priv,
			    const char *name, const u16 wm[]);

bool intel_set_memory_cxsr(struct drm_i915_private *dev_priv, bool enable);

#endif /* __INTEL_PM_H__ */
