#include "argus/control/guidance/include/yaw_rate_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace follow {
YawRateSmoother::YawRateSmoother(YawSmoothingConfig c) : config_(c) {
    if (!std::isfinite(c.deadzone_deg_s + c.maximum_change_deg_s2 + c.alpha + c.minimum_speed_m_s +
                       c.maximum_speed_m_s + c.low_speed_factor) ||
        c.deadzone_deg_s < 0 || c.maximum_change_deg_s2 <= 0 || c.alpha < 0 || c.alpha > 1 ||
        c.minimum_speed_m_s < 0 || c.maximum_speed_m_s <= c.minimum_speed_m_s ||
        c.low_speed_factor < 0 || c.low_speed_factor > 1) {
        throw std::invalid_argument("invalid yaw smoothing config");
    }
}
void YawRateSmoother::reset() { previous_limited_ = filtered_ = 0; }
double YawRateSmoother::update(double raw, double dt, double speed) {
    if (!std::isfinite(raw + dt + speed) || dt <= 0) {
        throw std::invalid_argument("invalid yaw smoothing input");
    }
    if (!config_.enabled) {
        return raw;
    }
    double rate = std::copysign(std::max(0., std::abs(raw) - config_.deadzone_deg_s), raw);
    if (config_.speed_scaling) {
        const double fraction =
            std::clamp((std::abs(speed) - config_.minimum_speed_m_s) /
                           (config_.maximum_speed_m_s - config_.minimum_speed_m_s),
                       0., 1.);
        rate *= config_.low_speed_factor + fraction * (1 - config_.low_speed_factor);
    }
    const double change = config_.maximum_change_deg_s2 * dt;
    previous_limited_ = std::clamp(rate, previous_limited_ - change, previous_limited_ + change);
    filtered_ = config_.alpha * previous_limited_ + (1 - config_.alpha) * filtered_;
    return filtered_;
}
}  // namespace follow
