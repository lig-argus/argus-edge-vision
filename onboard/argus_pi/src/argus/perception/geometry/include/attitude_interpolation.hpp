#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace follow {
struct TimedAttitudeSample {
    std::int64_t sample_ns;
    std::int64_t received_ns;
    Eigen::Quaterniond rotation;
    std::size_t history_index;
    // 2026-11-16: FC ATTITUDE의 몸체 각속도 rollspeed·pitchspeed·yawspeed (rad/s). 없으면 외삽 안
    // 함.
    std::optional<Eigen::Vector3d> body_rate{};
};
struct InterpolatedAttitude {
    Eigen::Vector3d rpy;
    std::size_t before_index;
    std::size_t after_index;
    double fraction;
    std::int64_t endpoint_delta_ns;
    std::int64_t extrapolated_ns{0};  // 2026-11-16: 외삽한 시간 (보간이면 0)
};
inline Eigen::Quaterniond attitude_rotation(double roll, double pitch, double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
// Keep the first receipt of a source sample: repeated publication cannot renew its TTL.
// No extrapolation; both endpoints must obey the original age/alignment limits.
inline std::optional<InterpolatedAttitude> interpolate_attitude(
    std::vector<TimedAttitudeSample> samples, std::int64_t target_ns, std::int64_t now_ns) {
    std::sort(samples.begin(), samples.end(), [](const auto& left, const auto& right) {
        return left.sample_ns < right.sample_ns ||
               (left.sample_ns == right.sample_ns && left.received_ns < right.received_ns);
    });
    samples.erase(std::unique(samples.begin(), samples.end(),
                              [](const auto& left, const auto& right) {
                                  return left.sample_ns == right.sample_ns;
                              }),
                  samples.end());
    std::optional<TimedAttitudeSample> before;
    std::optional<TimedAttitudeSample> after;
    for (const auto& sample : samples) {
        if (sample.sample_ns <= 0 || sample.received_ns < sample.sample_ns ||
            sample.received_ns > now_ns || now_ns - sample.sample_ns > 200000000 ||
            now_ns - sample.received_ns > 200000000 || !sample.rotation.coeffs().allFinite() ||
            sample.rotation.norm() < 1e-12) {
            continue;
        }
        if (sample.sample_ns <= target_ns) {
            before = sample;
        }
        if (sample.sample_ns >= target_ns) {
            after = sample;
            break;
        }
    }
    if (!before || !after || target_ns > now_ns ||
        after->sample_ns - before->sample_ns > 50000000 ||
        target_ns - before->sample_ns > 50000000 || after->sample_ns - target_ns > 50000000) {
        return std::nullopt;
    }
    const auto span = after->sample_ns - before->sample_ns;
    const double fraction =
        span == 0 ? 0.
                  : static_cast<double>(target_ns - before->sample_ns) / static_cast<double>(span);
    const auto rotation =
        before->rotation.normalized().slerp(fraction, after->rotation.normalized());
    const Eigen::Matrix3d matrix = rotation.normalized().toRotationMatrix();
    const Eigen::Vector3d rpy(std::atan2(matrix(2, 1), matrix(2, 2)),
                              std::asin(std::clamp(-matrix(2, 0), -1., 1.)),
                              std::atan2(matrix(1, 0), matrix(0, 0)));
    return InterpolatedAttitude{
        rpy, before->history_index, after->history_index, fraction,
        std::max(target_ns - before->sample_ns, after->sample_ns - target_ns)};
}

// 2026-11-16 사용자 지시(외삽 2단계): 노출 시각 뒤의 자세 표본이 아직 없을 때만 쓴다.
// 직전 표본에 몸체 각속도 × Δt를 더한다 (R(t+Δt) = R(t)·exp([ω]×Δt), 몸체 각속도라 오른쪽 곱).
// 1단계 검증(FC ULog 837s, 자이로 적분 참값): Δt 10~25ms 회전 오차 p95 0.10~0.14°,
// 각속도 40°/s 초과에서 오차가 커진다. 상한을 넘거나 각속도가 없으면 외삽하지 않는다(기존 대기).
// 뒤 표본이 이미 있으면 외삽하지 않는다 (그때는 보간이 맞고, 보간 실패는 다른 이유다).
inline std::optional<InterpolatedAttitude> extrapolate_attitude(
    std::vector<TimedAttitudeSample> samples, std::int64_t target_ns, std::int64_t now_ns,
    std::int64_t max_extrapolation_ns, double max_rate_rad_s) {
    if (max_extrapolation_ns <= 0 || !(max_rate_rad_s > 0.)) {
        return std::nullopt;
    }
    std::sort(samples.begin(), samples.end(), [](const auto& left, const auto& right) {
        return left.sample_ns < right.sample_ns ||
               (left.sample_ns == right.sample_ns && left.received_ns < right.received_ns);
    });
    std::optional<TimedAttitudeSample> before;
    for (const auto& sample : samples) {
        if (sample.sample_ns <= 0 || sample.received_ns < sample.sample_ns ||
            sample.received_ns > now_ns || now_ns - sample.sample_ns > 200000000 ||
            now_ns - sample.received_ns > 200000000 || !sample.rotation.coeffs().allFinite() ||
            sample.rotation.norm() < 1e-12) {
            continue;
        }
        if (sample.sample_ns > target_ns) {
            return std::nullopt;
        }
        before = sample;
    }
    if (!before || target_ns > now_ns || !before->body_rate) {
        return std::nullopt;
    }
    const std::int64_t delta_ns = target_ns - before->sample_ns;
    const Eigen::Vector3d rate = *before->body_rate;
    if (delta_ns < 0 || delta_ns > max_extrapolation_ns || !rate.allFinite() ||
        rate.norm() > max_rate_rad_s) {
        return std::nullopt;
    }
    const double angle = rate.norm() * static_cast<double>(delta_ns) * 1e-9;
    const Eigen::Quaterniond step =
        angle > 1e-12 ? Eigen::Quaterniond(Eigen::AngleAxisd(angle, rate.normalized()))
                      : Eigen::Quaterniond::Identity();
    const Eigen::Matrix3d matrix =
        (before->rotation.normalized() * step).normalized().toRotationMatrix();
    const Eigen::Vector3d rpy(std::atan2(matrix(2, 1), matrix(2, 2)),
                              std::asin(std::clamp(-matrix(2, 0), -1., 1.)),
                              std::atan2(matrix(1, 0), matrix(0, 0)));
    return InterpolatedAttitude{rpy,     before->history_index, before->history_index, 0., delta_ns,
                                delta_ns};
}
}  // namespace follow
