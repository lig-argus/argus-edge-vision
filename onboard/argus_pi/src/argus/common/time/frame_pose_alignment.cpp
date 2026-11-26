#include "argus/common/time/include/frame_pose_alignment.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
namespace follow {
namespace {
std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
double wrap(double a) {
    constexpr double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2 * pi);
    if (a < 0) {
        a += 2 * pi;
    }
    return a - pi;
}
}  // namespace
double FramePoseAlignment::unwrap(std::uint32_t value, bool att) {
    auto& previous = att ? last_attitude_ms_ : last_position_ms_;
    auto& wraps = att ? attitude_wraps_ : position_wraps_;
    if (previous && value < *previous) {
        if (*previous > 0xf0000000u && value < 0x0fffffffu) {
            ++wraps;
        } else {
            positions_.clear();
            attitudes_.clear();
            latest_.reset();
            epoch_invalid_ = true;
            ++epoch_;
        }
    }
    previous = value;
    return (static_cast<double>(value) + static_cast<double>(wraps) * 4294967296.) / 1000.;
}
void FramePoseAlignment::position(std::uint32_t boot, const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& v, std::int64_t received) {
    if (!p.allFinite() || !v.allFinite()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    positions_.push_back({unwrap(boot, false), p});
    if (positions_.size() > 3000) {
        positions_.pop_front();
    }
    latest_ = FCPositionSample{boot, received, p, v};
}
void FramePoseAlignment::attitude(std::uint32_t boot, const Eigen::Vector3d& rpy) {
    if (!rpy.allFinite()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    attitudes_.push_back({unwrap(boot, true), rpy});
    if (attitudes_.size() > 3000) {
        attitudes_.pop_front();
    }
}
AlignedFCPose FramePoseAlignment::aligned(double stamp, double max_delta,
                                          std::int64_t deadline) const {
    if (!std::isfinite(stamp) || !std::isfinite(max_delta) || max_delta <= 0) {
        throw std::invalid_argument("invalid FC alignment query");
    }
    while (now_ns() < deadline) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (epoch_invalid_) {
                throw std::runtime_error("FC clock epoch discontinuity");
            }
            if (positions_.size() >= 2 && attitudes_.size() >= 2 &&
                positions_.back().stamp >= stamp && attitudes_.back().stamp >= stamp) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch_invalid_) {
        throw std::runtime_error("FC clock epoch discontinuity");
    }
    AlignedFCPose output;
    output.sensor_boot_s = stamp;
    output.epoch = epoch_;
    for (int kind = 0; kind < 2; ++kind) {
        const auto& samples = kind ? attitudes_ : positions_;
        if (samples.size() < 2) {
            throw std::runtime_error("FC sample history unavailable");
        }
        auto upper = std::lower_bound(samples.begin(), samples.end(), stamp,
                                      [](const Sample& s, double t) { return s.stamp < t; });
        if (upper == samples.begin() || upper == samples.end()) {
            throw std::runtime_error("FC sensor exposure bracket unavailable");
        }
        const auto& before = *std::prev(upper);
        const auto& after = *upper;
        const double error = std::max(stamp - before.stamp, after.stamp - stamp);
        if (error > max_delta || after.stamp <= before.stamp) {
            throw std::runtime_error("FC sensor exposure bracket too wide");
        }
        Eigen::Vector3d delta = after.value - before.value;
        if (kind) {
            for (int i = 0; i < 3; ++i) {
                delta[i] = wrap(delta[i]);
            }
        }
        Eigen::Vector3d value =
            before.value + (stamp - before.stamp) / (after.stamp - before.stamp) * delta;
        if (kind) {
            output.rpy = value;
        } else {
            output.position = value;
        }
        output.alignment_error_ms = std::max(output.alignment_error_ms, error * 1000.);
    }
    return output;
}
std::optional<double> FramePoseAlignment::current_sim_s() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch_invalid_ || positions_.empty()) {
        return {};
    }
    return positions_.back().stamp;
}
std::optional<FCPositionSample> FramePoseAlignment::latest_position() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}
std::optional<Eigen::Vector3d> FramePoseAlignment::latest_rpy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (attitudes_.empty()) {
        return {};
    }
    return attitudes_.back().value;
}
}  // namespace follow
