#include "argus/tracking/motion/include/short_motion_estimator.hpp"

#include <cmath>
#include <stdexcept>

namespace follow {
ShortMotionEstimator::ShortMotionEstimator(ShortMotionConfig c) : config_(c) {
    if (!std::isfinite(c.window_s + c.minimum_span_s + c.confirmation_s + c.stop_speed_m_s +
                       c.resume_speed_m_s + c.maximum_rmse_m + c.maximum_sample_gap_s) ||
        c.window_s <= 0 || c.window_s > 1 || c.minimum_span_s <= 0 ||
        c.minimum_span_s >= c.window_s || c.minimum_samples < 3 || c.minimum_samples > 64 ||
        c.confirmation_s < 0 || c.confirmation_s > 1 || c.stop_speed_m_s <= 0 ||
        c.resume_speed_m_s <= c.stop_speed_m_s || c.resume_speed_m_s > 2 || c.maximum_rmse_m <= 0 ||
        c.maximum_rmse_m > 1 || c.maximum_sample_gap_s <= 0 || c.maximum_sample_gap_s > 1) {
        throw std::invalid_argument("invalid short motion configuration");
    }
}
void ShortMotionEstimator::reset() {
    points_.clear();
    latest_ = {};
    pending_.reset();
    stopped_at_.reset();
}
ShortMotionEstimate ShortMotionEstimator::update(std::int64_t sample_ns,
                                                 std::array<double, 2> point) {
    if (!config_.enabled) {
        return {};
    }
    if (sample_ns <= 0 || !std::isfinite(point[0] + point[1])) {
        reset();
        return latest_;
    }
    const double stamp = sample_ns / 1e9;
    if (!points_.empty() && stamp <= points_.back()[0]) {
        return latest_;
    }
    if (!points_.empty() && stamp - points_.back()[0] > config_.maximum_sample_gap_s) {
        reset();
    }
    points_.push_back({stamp, point[0], point[1]});
    latest_.departure_pending = false;
    latest_.position_changed_since_stop =
        stopped_at_ && std::hypot(point[0] - (*stopped_at_)[0], point[1] - (*stopped_at_)[1]) >
                           config_.stop_speed_m_s * config_.window_s;
    while (points_.size() > 64 || stamp - points_.front()[0] > config_.window_s) {
        points_.pop_front();
    }
    if (points_.size() < config_.minimum_samples ||
        stamp - points_.front()[0] < config_.minimum_span_s) {
        latest_.valid = false;
        latest_.direction_valid = false;
        latest_.feedforward_valid = false;
        return latest_;
    }
    double st = 0, stt = 0, sn = 0, se = 0, stn = 0, ste = 0;
    for (const auto& p : points_) {
        const double t = p[0] - stamp;
        st += t;
        stt += t * t;
        sn += p[1];
        se += p[2];
        stn += t * p[1];
        ste += t * p[2];
    }
    const double count = points_.size(), denominator = count * stt - st * st;
    if (denominator <= 1e-9) {
        reset();
        return latest_;
    }
    latest_.velocity_north_m_s = (count * stn - st * sn) / denominator;
    latest_.velocity_east_m_s = (count * ste - st * se) / denominator;
    latest_.fit_sample_ns = sample_ns;
    latest_.north_m = (sn - latest_.velocity_north_m_s * st) / count;
    latest_.east_m = (se - latest_.velocity_east_m_s * st) / count;
    double residual = 0;
    for (const auto& p : points_) {
        const double t = p[0] - stamp;
        residual += std::pow(p[1] - latest_.north_m - latest_.velocity_north_m_s * t, 2) +
                    std::pow(p[2] - latest_.east_m - latest_.velocity_east_m_s * t, 2);
    }
    latest_.rmse_m = std::sqrt(residual / count);
    latest_.valid = std::isfinite(latest_.velocity_north_m_s + latest_.velocity_east_m_s) &&
                    latest_.rmse_m <= config_.maximum_rmse_m;
    if (!latest_.valid) {
        latest_.phase = ObservedMotionPhase::Unknown;
        latest_.direction_valid = false;
        latest_.feedforward_valid = false;
        pending_.reset();
        return latest_;
    }
    const double speed = std::hypot(latest_.velocity_north_m_s, latest_.velocity_east_m_s);
    const bool displaced = stopped_at_ && std::hypot(latest_.north_m - (*stopped_at_)[0],
                                                     latest_.east_m - (*stopped_at_)[1]) >=
                                              config_.resume_speed_m_s * config_.window_s;
    const auto candidate = speed <= config_.stop_speed_m_s ? ObservedMotionPhase::Stopped
                           : speed >= config_.resume_speed_m_s || displaced
                               ? ObservedMotionPhase::Moving
                               : ObservedMotionPhase::Unknown;
    if (candidate == ObservedMotionPhase::Unknown || candidate == latest_.phase) {
        pending_.reset();
    } else {
        if (!pending_ || *pending_ != candidate) {
            pending_ = candidate;
            pending_at_s_ = stamp;
        }
        if (stamp - pending_at_s_ >= config_.confirmation_s) {
            latest_.phase = candidate;
            pending_.reset();
            if (candidate == ObservedMotionPhase::Stopped) {
                stopped_at_ = std::array<double, 2>{latest_.north_m, latest_.east_m};
            } else {
                stopped_at_.reset();
            }
        }
    }
    latest_.direction_valid =
        latest_.phase == ObservedMotionPhase::Moving && speed > config_.stop_speed_m_s;
    latest_.feedforward_valid = latest_.direction_valid && speed >= config_.resume_speed_m_s;
    latest_.departure_pending =
        latest_.phase == ObservedMotionPhase::Stopped && latest_.position_changed_since_stop &&
        pending_ == ObservedMotionPhase::Moving && speed >= config_.resume_speed_m_s;
    return latest_;
}
}  // namespace follow
