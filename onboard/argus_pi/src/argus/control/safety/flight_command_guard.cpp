// 이미 계산된 후보를 송신 직전 다시 검사해 오래된 명령의 통과를 막는다.
#include "argus/control/safety/include/flight_command_guard.hpp"

#include <algorithm>
#include <cmath>

#include "argus/control/contracts/include/v2_1_features.hpp"

namespace follow {

FlightCommandGuard::FlightCommandGuard(TargetSelector& selector,
                                       const IVehicleFlightStatusSource& flight_state_source,
                                       IVehiclePort& vehicle_port, bool yaw_only,
                                       std::recursive_mutex* context_mutex)
    : selector_(selector),
      flight_state_source_(flight_state_source),
      vehicle_port_(vehicle_port),
      yaw_only_(yaw_only),
      context_mutex_(context_mutex) {}

// 2026-11-18 데모 모드: 고도 유지가 켜져 있고, 상한 이내이며, 범위 밖으로 더 밀지 않을 때만.
// NED down 기준이라 vz < 0이 상승이다. 높이 = ground_down_m - pose.down_m.
bool FlightCommandGuard::yaw_only_vertical_allowed(double vz_m_s, Clock::time_point now) const {
    if (!altitude_band_.enabled || pose_source_ == nullptr ||
        !(altitude_band_.max_vertical_m_s > 0) ||
        std::abs(vz_m_s) > altitude_band_.max_vertical_m_s + 1e-9) {
        return false;
    }
    const auto pose = pose_source_->read();
    if (!pose.have_position || !std::isfinite(pose.down_m) || now < pose.position_at ||
        now - pose.position_at >= std::chrono::seconds(1)) {
        return false;
    }
    const double height_m = altitude_band_.ground_down_m - pose.down_m;
    if (!std::isfinite(height_m)) {
        return false;
    }
    const bool climb_above = height_m >= altitude_band_.max_height_m && vz_m_s < 0;
    const bool descend_below = height_m <= altitude_band_.min_height_m && vz_m_s > 0;
    return !climb_above && !descend_below;
}

FlightCommandGuard::Result FlightCommandGuard::send(const FlightGuidanceResult& candidate,
                                                    const TargetSelectionSnapshot& expected,
                                                    Clock::time_point now) {
    // Serialize selected-ID updates with the final check and context-bound enqueue.
    std::unique_lock<std::recursive_mutex> context_lock;
    if (context_mutex_) {
        context_lock = std::unique_lock<std::recursive_mutex>(*context_mutex_);
        now = std::max(now, Clock::now());
    }
    const bool invalid_lifetime = !candidate.send_setpoint ||
                                  candidate.selection_command_epoch != expected.command_epoch ||
                                  candidate.valid_until <= now;
    const bool nonfinite_command = !std::isfinite(candidate.command.vx_m_s) ||
                                   !std::isfinite(candidate.command.vy_m_s) ||
                                   !std::isfinite(candidate.command.vz_m_s) ||
                                   !std::isfinite(candidate.command.yaw_rate_deg_s);
    if (invalid_lifetime || nonfinite_command) {
        return Result::Rejected;
    }
    if (std::hypot(candidate.command.vx_m_s, candidate.command.vy_m_s) > limits_[0] + 1e-9 ||
        std::abs(candidate.command.vz_m_s) > limits_[1] + 1e-9 ||
        std::abs(candidate.command.yaw_rate_deg_s) > limits_[2] + 1e-9) {
        return Result::Rejected;
    }
    if (yaw_only_ && (candidate.command.vx_m_s != 0 || candidate.command.vy_m_s != 0)) {
        return Result::Rejected;
    }
    if (yaw_only_ && candidate.command.vz_m_s != 0 &&
        !yaw_only_vertical_allowed(candidate.command.vz_m_s, now)) {
        return Result::Rejected;
    }
    // target과 FC 모드를 송신 직전에 각각 다시 읽어 경쟁 구간의 후보를 걸러낸다.
    const auto latest = selector_.snapshot(now);
    if (latest.latest_frame && camera_only_tracks(*latest.latest_frame)) {
        return Result::Rejected;
    }
    if (!same_selection(expected, latest)) {
        return Result::SelectionChanged;
    }
    if (latest.phase != TargetSelectionSnapshot::Phase::Following) {
        return Result::Rejected;
    }
    if (latest.target_phase == TargetPhase::Lost) {
        return Result::Rejected;
    }
    if ((candidate.mission == "GAP_CV" || candidate.command.mode == "GAP_CV") &&
        (!latest.recovery_pause || !latest.cv_gap_active)) {
        return Result::SelectionChanged;
    }
    if (latest.recovery_pause && (candidate.command.vx_m_s != 0 || candidate.command.vy_m_s != 0 ||
                                  candidate.command.yaw_rate_deg_s != 0)) {
        const auto origin = Clock::time_point(std::chrono::nanoseconds(latest.cv_origin_sample_ns));
        const bool cv_allowed =
            latest.cv_gap_active && expected.cv_gap_active && latest.cv_gap_deadline &&
            expected.cv_gap_deadline && *latest.cv_gap_deadline == *expected.cv_gap_deadline &&
            latest.cv_origin_sample_ns == expected.cv_origin_sample_ns &&
            latest.cv_origin_sample_ns > 0 && origin <= now &&
            *latest.cv_gap_deadline == origin + std::chrono::seconds(3) &&
            now < *latest.cv_gap_deadline && candidate.valid_until <= *latest.cv_gap_deadline &&
            candidate.mission == "GAP_CV" && candidate.command.mode == "GAP_CV";
        if (!cv_allowed) {
            return Result::Rejected;
        }
    }
    const auto flight = flight_state_source_.read();
    if (!flight.connected || !flight.armed || !flight.in_air || !flight.offboard ||
        flight.rc_override) {
        return Result::Rejected;
    }
    return vehicle_port_.send_until(candidate.command, candidate.valid_until) ? Result::Sent
                                                                              : Result::Rejected;
}
}  // namespace follow
