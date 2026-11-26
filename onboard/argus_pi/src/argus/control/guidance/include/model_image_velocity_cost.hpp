#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace follow {
// V3 영상 속도 비용 가중치. 원 SITL 비교에서 쓴 값만 허용한다 (0이면 끔).
inline double checked_image_velocity_weight(double value) {
    if (!std::isfinite(value) || (value != 0. && value != .25 && value != 1. && value != 4.)) {
        throw std::invalid_argument("image velocity weight must be 0, .25, 1 or 4");
    }
    return value;
}

// Units: centre [-1,1], rate s^-1, tau seconds; no history from actual bbox frames.
class ModelImageVelocityCost {
public:
    ModelImageVelocityCost(std::array<double, 2> initial, double weight)
        : previous_(initial), weight_(weight) {}
    std::optional<double> add(std::array<double, 2> centre, double dt) {
        if (!std::isfinite(previous_[0] + previous_[1] + centre[0] + centre[1] + dt + weight_) ||
            dt <= 0 || weight_ < 0) {
            return {};
        }
        rate_ = {(centre[0] - previous_[0]) / dt, (centre[1] - previous_[1]) / dt};
        constexpr double tau = .4;
        const double dx = tau * rate_[0], dy = tau * rate_[1];
        const double cost = weight_ * (2 * dx * dx + .5 * dy * dy);
        if (!std::isfinite(cost)) {
            return {};
        }
        previous_ = centre;
        return cost;
    }
    const std::array<double, 2>& rate() const { return rate_; }

private:
    std::array<double, 2> previous_, rate_{};
    double weight_;
};
}  // namespace follow
