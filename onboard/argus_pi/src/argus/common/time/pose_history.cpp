// 위치와 자세를 각자의 수신 표본으로 보간하고, 시간 범위 밖의 외삽은 거부한다.
#include "argus/common/time/include/pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace follow {
namespace {
double interpolate_angle(double a, double b, double t) {
    const double delta = std::remainder(b - a, 360.0);
    return std::remainder(a + t * delta, 360.0);
}
struct Pair {
    const Pose* before;
    const Pose* after;
};
template <typename Predicate, typename Timestamp>
std::optional<Pair> bracket(const std::deque<Pose>& samples, const std::string& domain,
                            std::int64_t time_ns, std::chrono::nanoseconds max_distance,
                            Predicate valid, Timestamp timestamp) {
    const Pose* before = nullptr;
    const Pose* after = nullptr;
    for (const auto& sample : samples) {
        if (sample.clock_domain != domain || !valid(sample)) {
            continue;
        }
        const auto t = timestamp(sample);
        if (t <= time_ns && (!before || t > timestamp(*before))) {
            before = &sample;
        }
        if (t >= time_ns && (!after || t < timestamp(*after))) {
            after = &sample;
        }
    }
    if (!before || !after) {
        return std::nullopt;
    }
    if (time_ns - timestamp(*before) > max_distance.count() ||
        timestamp(*after) - time_ns > max_distance.count()) {
        return std::nullopt;
    }
    return Pair{before, after};
}
double ratio(std::int64_t time_ns, std::int64_t before_ns, std::int64_t after_ns) {
    if (before_ns == after_ns) {
        return 0.0;
    }
    return static_cast<double>(time_ns - before_ns) / static_cast<double>(after_ns - before_ns);
}
}  // namespace

PoseHistory::PoseHistory(std::size_t capacity) : capacity_(capacity) {
    if (capacity_ < 2 || capacity_ > 4096) {
        throw std::invalid_argument("invalid pose history capacity");
    }
}

void PoseHistory::offer(const Pose& pose) {
    const bool missing_clock = pose.clock_domain.empty();
    const bool invalid_position =
        (pose.have_position && (pose.position_monotonic_ns <= 0 || !std::isfinite(pose.north_m) ||
                                !std::isfinite(pose.east_m) || !std::isfinite(pose.down_m)));
    const bool invalid_attitude =
        (pose.have_attitude && (pose.attitude_monotonic_ns <= 0 || !std::isfinite(pose.roll_deg) ||
                                !std::isfinite(pose.pitch_deg) || !std::isfinite(pose.yaw_deg)));
    if (missing_clock || invalid_position || invalid_attitude) {
        throw std::invalid_argument("invalid timed pose sample");
    }
    if (!pose.have_position && !pose.have_attitude) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples_.empty() && samples_.back().clock_domain != pose.clock_domain) {
        samples_.clear();
    }
    samples_.push_back(pose);
    while (samples_.size() > capacity_) {
        samples_.pop_front();
    }
}

std::optional<Pose> PoseHistory::interpolate(std::int64_t time_ns, const std::string& domain,
                                             std::chrono::nanoseconds max_distance) const {
    if (time_ns <= 0 || domain.empty() || max_distance.count() <= 0) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto position_pair = bracket(
        samples_, domain, time_ns, max_distance, [](const Pose& p) { return p.have_position; },
        [](const Pose& p) { return p.position_monotonic_ns; });
    const auto attitude_pair = bracket(
        samples_, domain, time_ns, max_distance, [](const Pose& p) { return p.have_attitude; },
        [](const Pose& p) { return p.attitude_monotonic_ns; });
    if (!position_pair || !attitude_pair) {
        return std::nullopt;
    }

    Pose result;
    result.clock_domain = domain;
    result.have_position = result.have_attitude = true;
    const auto& pb = *position_pair->before;
    const auto& pa = *position_pair->after;
    const double pt = ratio(time_ns, pb.position_monotonic_ns, pa.position_monotonic_ns);
    result.north_m = pb.north_m + pt * (pa.north_m - pb.north_m);
    result.east_m = pb.east_m + pt * (pa.east_m - pb.east_m);
    result.down_m = pb.down_m + pt * (pa.down_m - pb.down_m);
    result.have_velocity = pb.have_velocity && pa.have_velocity;
    if (result.have_velocity) {
        result.velocity_north_m_s =
            pb.velocity_north_m_s + pt * (pa.velocity_north_m_s - pb.velocity_north_m_s);
        result.velocity_east_m_s =
            pb.velocity_east_m_s + pt * (pa.velocity_east_m_s - pb.velocity_east_m_s);
        result.velocity_down_m_s =
            pb.velocity_down_m_s + pt * (pa.velocity_down_m_s - pb.velocity_down_m_s);
    }
    result.position_monotonic_ns = time_ns;
    result.position_at = std::max(pb.position_at, pa.position_at);

    const auto& ab = *attitude_pair->before;
    const auto& aa = *attitude_pair->after;
    const double at = ratio(time_ns, ab.attitude_monotonic_ns, aa.attitude_monotonic_ns);
    result.roll_deg = interpolate_angle(ab.roll_deg, aa.roll_deg, at);
    result.pitch_deg = interpolate_angle(ab.pitch_deg, aa.pitch_deg, at);
    result.yaw_deg = interpolate_angle(ab.yaw_deg, aa.yaw_deg, at);
    result.attitude_monotonic_ns = time_ns;
    result.attitude_at = std::max(ab.attitude_at, aa.attitude_at);
    return result;
}
}  // namespace follow
