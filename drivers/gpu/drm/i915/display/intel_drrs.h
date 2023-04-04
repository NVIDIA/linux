/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __INTEL_DRRS_H__
#define __INTEL_DRRS_H__

#include <linux/types.h>

<<<<<<< HEAD
struct drm_i915_private;
struct intel_crtc_state;
struct intel_connector;
struct intel_dp;

void intel_edp_drrs_enable(struct intel_dp *intel_dp,
			   const struct intel_crtc_state *crtc_state);
void intel_edp_drrs_disable(struct intel_dp *intel_dp,
			    const struct intel_crtc_state *crtc_state);
void intel_edp_drrs_update(struct intel_dp *intel_dp,
			   const struct intel_crtc_state *crtc_state);
void intel_edp_drrs_invalidate(struct drm_i915_private *dev_priv,
			       unsigned int frontbuffer_bits);
void intel_edp_drrs_flush(struct drm_i915_private *dev_priv,
			  unsigned int frontbuffer_bits);
void intel_dp_drrs_compute_config(struct intel_dp *intel_dp,
				  struct intel_crtc_state *pipe_config,
				  int output_bpp, bool constant_n);
struct drm_display_mode *intel_dp_drrs_init(struct intel_connector *connector,
					    struct drm_display_mode *fixed_mode);
=======
enum drrs_type;
struct drm_i915_private;
struct intel_atomic_state;
struct intel_crtc;
struct intel_crtc_state;
struct intel_connector;

const char *intel_drrs_type_str(enum drrs_type drrs_type);
bool intel_drrs_is_active(struct intel_crtc *crtc);
void intel_drrs_activate(const struct intel_crtc_state *crtc_state);
void intel_drrs_deactivate(const struct intel_crtc_state *crtc_state);
void intel_drrs_invalidate(struct drm_i915_private *dev_priv,
			   unsigned int frontbuffer_bits);
void intel_drrs_flush(struct drm_i915_private *dev_priv,
		      unsigned int frontbuffer_bits);
void intel_crtc_drrs_init(struct intel_crtc *crtc);
>>>>>>> origin/linux_6.1.15_upstream

#endif /* __INTEL_DRRS_H__ */
