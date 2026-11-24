// FC 로 송신하는 SET_POSITION_TARGET_LOCAL_NED 메시지의 구성
// 규약: docs/spec/icd.md 2장
// mavlink_gateway 의 구현 파일들이 함께 사용하며 모듈 밖에서는 include 하지 않는다
#ifndef ARGUS_ONBOARD_SRC_SETPOINT_ENCODING_HPP_
#define ARGUS_ONBOARD_SRC_SETPOINT_ENCODING_HPP_

#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway::detail {

// 기체 기준(전진, 우측, 하강) 속도와 yaw 회전 속도의 setpoint. 좌표계 MAV_FRAME_BODY_NED.
// 위치, 가속도, yaw 는 사용하지 않는 값으로 표시한다
mavlink_message_t encode_velocity_setpoint(const vehicle_api::VelocitySetpoint& setpoint);

// local NED 위치와 yaw 를 목표로 하는 setpoint. 좌표계 MAV_FRAME_LOCAL_NED (PX4 는 위치 값을 이
// 좌표계에서만 사용한다). 속도, 가속도, yaw 회전 속도는 사용하지 않는 값으로 표시한다
mavlink_message_t encode_position_setpoint(double north_m, double east_m, double down_m,
                                           double yaw_rad);

}  // namespace argus::mavlink_gateway::detail

#endif  // ARGUS_ONBOARD_SRC_SETPOINT_ENCODING_HPP_
