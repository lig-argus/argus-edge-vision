#pragma once

#include <cmath>

#include "argus/control/target/include/target_memory.hpp"

namespace follow {
// Select a CV velocity at observation time; no later gap check depends on validity flags.
inline void update_cv_motion_cache(TargetMemory& memory, const ShortMotionEstimate& motion,
                                   bool allow_unconfirmed_motion = false) {
    if (motion.valid && motion.phase == ObservedMotionPhase::Stopped && !motion.departure_pending) {
        memory.cv_velocity_north_m_s = memory.cv_velocity_east_m_s = 0;
        memory.cv_velocity_trusted = true;
        memory.cv_last_observed_moving = false;
        memory.cv_velocity_unconfirmed = false;
        memory.cv_velocity_fit_sample_ns = motion.fit_sample_ns;
    } else if (!std::isfinite(motion.velocity_north_m_s) ||
               !std::isfinite(motion.velocity_east_m_s)) {
        return;
    } else if (motion.phase == ObservedMotionPhase::Moving &&
               ((motion.valid && motion.feedforward_valid) || allow_unconfirmed_motion)) {
        memory.cv_velocity_north_m_s = motion.velocity_north_m_s;
        memory.cv_velocity_east_m_s = motion.velocity_east_m_s;
        // "trusted" latches this cache against Unknown samples; raw provenance remains explicit.
        memory.cv_velocity_trusted = true;
        memory.cv_last_observed_moving = true;
        memory.cv_velocity_unconfirmed = !(motion.valid && motion.feedforward_valid);
        memory.cv_velocity_fit_sample_ns = motion.fit_sample_ns;
    } else if (!memory.cv_velocity_trusted) {
        memory.cv_velocity_north_m_s = motion.velocity_north_m_s;
        memory.cv_velocity_east_m_s = motion.velocity_east_m_s;
        memory.cv_velocity_unconfirmed = true;
        memory.cv_velocity_fit_sample_ns = motion.fit_sample_ns;
    }
}
}  // namespace follow
