// fc/attitude, fc/state, fc/mode_event 의 IDL 타입과 vehicle_api 값 타입 사이의 변환. 생성된 IDL
// 타입 (argus_msg/fc_attitude.idl, fc_state.idl, fc_mode_event.idl 의 argus::idl::FcAttitude,
// FcState, FcModeEvent)을 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_IDL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_IDL_HPP_

#include "argus/vehicle_api/types.hpp"
#include "argus_msg/fc_attitude.hpp"
#include "argus_msg/fc_mode_event.hpp"
#include "argus_msg/fc_state.hpp"

namespace argus::vehicle_api_bus {

// from_idl_* 는 표본의 모든 값을 받아들이므로 항상 true 를 반환한다
void to_idl_fc_attitude(const vehicle_api::Attitude& attitude, idl::FcAttitude& sample);
bool from_idl_fc_attitude(const idl::FcAttitude& sample, vehicle_api::Attitude& attitude);

// fc/state 는 local, downward_distance, global, status 만 싣는다. state.attitude 는 싣지 않으며
// 받은 값의 attitude 는 항상 nullopt 다 (자세는 fc/attitude 로 보낸다)
void to_idl_fc_state(const vehicle_api::VehicleState& state, idl::FcState& sample);
bool from_idl_fc_state(const idl::FcState& sample, vehicle_api::VehicleState& state);

void to_idl_fc_mode_event(const vehicle_api::FlightModeEvent& event, idl::FcModeEvent& sample);
bool from_idl_fc_mode_event(const idl::FcModeEvent& sample, vehicle_api::FlightModeEvent& event);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_IDL_HPP_
