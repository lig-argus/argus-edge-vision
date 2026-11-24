// 속도 setpoint 의 값 검사
// 사용: SetpointInbox (받을 때), FcControl (제자리 유지의 목표를 정할 때)
#ifndef ARGUS_ONBOARD_SETPOINT_VALIDATION_HPP_
#define ARGUS_ONBOARD_SETPOINT_VALIDATION_HPP_

#include <cmath>
#include <limits>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// MAVLink 메시지의 float 필드로 변환할 수 있는 값인지 (NaN, 무한대, float 범위 초과가 아님)
inline bool fits_float(double value) {
    return std::isfinite(value) &&
           std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

// setpoint 의 속도와 yaw 회전 속도가 모두 FC 로 송신할 수 있는 값인지
inline bool has_sendable_values(const vehicle_api::VelocitySetpoint& setpoint) {
    return fits_float(setpoint.forward_m_s) && fits_float(setpoint.right_m_s) &&
           fits_float(setpoint.down_m_s) && fits_float(setpoint.yaw_rate_rad_s);
}

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SETPOINT_VALIDATION_HPP_
