// 지상 yaw 수신 시험은 비행 추종과 별도의 제한 계약을 사용한다.
#pragma once
#include <cmath>

#include "argus/vehicle/contracts/include/i_vehicle_state_source.hpp"
#include "argus/vehicle/contracts/include/vehicle_flight_status.hpp"

namespace follow {
inline bool ground_yaw_bench_request_valid(bool props_removed, double rate, double duration) {
    return props_removed && std::isfinite(rate) && std::abs(rate) <= 10 &&
           std::isfinite(duration) && duration > 0 && duration <= 5;
}

inline bool ground_yaw_bench_state_valid(const VehicleFlightStatus& state, const Pose& pose,
                                         Clock::time_point now,
                                         std::chrono::milliseconds max_pose_age) {
    return state.connected && state.ground_known && state.landed && !state.in_air &&
           !state.offboard && pose.fresh(now) && now >= pose.position_at &&
           now >= pose.attitude_at && now - pose.position_at <= max_pose_age &&
           now - pose.attitude_at <= max_pose_age;
}
}  // namespace follow
