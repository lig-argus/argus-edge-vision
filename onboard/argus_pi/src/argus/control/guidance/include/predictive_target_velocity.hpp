#pragma once

#include <array>
#include <cmath>

#include "argus/control/target/include/target_selector.hpp"

namespace follow {
inline std::array<double, 2> predictive_target_velocity(const TargetSelectionSnapshot& selection,
                                                        double max_speed_m_s) {
    const auto& recent = selection.short_motion;
    std::array<double, 2> velocity{};
    if (selection.box_motion_suspect) {
        if (std::isfinite(selection.cv_velocity_north_m_s) &&
            std::isfinite(selection.cv_velocity_east_m_s)) {
            velocity = {selection.cv_velocity_north_m_s, selection.cv_velocity_east_m_s};
        }
    } else if (recent.valid && (recent.feedforward_valid || recent.departure_pending)) {
        velocity = {recent.velocity_north_m_s, recent.velocity_east_m_s};
    }
    const double speed = std::hypot(velocity[0], velocity[1]);
    if (speed > max_speed_m_s) {
        velocity[0] *= max_speed_m_s / speed;
        velocity[1] *= max_speed_m_s / speed;
    }
    return velocity;
}
}  // namespace follow
