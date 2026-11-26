#include "argus/control/guidance/include/yaw_only_follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace follow {
YawOnlyFollower::YawOnlyFollower(double kp, double maximum_rate, double bearing_deadband,
                                 YawSmoothingConfig smoothing)
    : kp_(kp), maximum_rate_(maximum_rate), bearing_deadband_(bearing_deadband), smoother_([&] {
          smoothing.speed_scaling = false;
          return smoothing;
      }()) {
    if (!std::isfinite(kp + maximum_rate + bearing_deadband) || kp <= 0 || maximum_rate <= 0 ||
        bearing_deadband < 0) {
        throw std::invalid_argument("invalid yaw-only follower configuration");
    }
}
double YawOnlyFollower::update(double bearing_deg, double dt_s) {
    if (!std::isfinite(bearing_deg + dt_s) || dt_s <= 0) {
        throw std::invalid_argument("invalid yaw-only follower input");
    }
    if (std::abs(bearing_deg) <= bearing_deadband_) {
        reset();
        return 0;
    }
    const auto raw = std::clamp(kp_ * bearing_deg, -maximum_rate_, maximum_rate_);
    return std::clamp(smoother_.update(raw, dt_s, 0), -maximum_rate_, maximum_rate_);
}
void YawOnlyFollower::reset() { smoother_.reset(); }
}  // namespace follow
