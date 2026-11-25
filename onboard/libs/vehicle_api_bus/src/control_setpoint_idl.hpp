// control/setpoint 의 IDL 타입과 vehicle_api::VelocitySetpoint 사이의 변환. 생성된 IDL 타입
// (argus_msg/control_setpoint.idl 의 argus::idl::ControlSetpoint)을 사용하는 코드는 이 파일의 두
// 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_IDL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_IDL_HPP_

#include "argus/vehicle_api/types.hpp"
#include "argus_msg/control_setpoint.hpp"

namespace argus::vehicle_api_bus {

void to_idl_control_setpoint(const vehicle_api::VelocitySetpoint& setpoint,
                             idl::ControlSetpoint& sample);
// 표본의 모든 값을 받아들이므로 항상 true 를 반환한다
bool from_idl_control_setpoint(const idl::ControlSetpoint& sample,
                               vehicle_api::VelocitySetpoint& setpoint);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_IDL_HPP_
