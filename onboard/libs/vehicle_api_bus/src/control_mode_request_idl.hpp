// control/mode_request 의 요청, 응답 IDL 타입과 vehicle_api 값 타입 사이의 변환. 생성된 IDL 타입
// (argus_msg/control_mode_request.idl 의 argus::idl::ControlModeRequest, ControlModeReply)을
// 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_IDL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_IDL_HPP_

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus_msg/control_mode_request.hpp"

namespace argus::vehicle_api_bus {

void to_idl_flight_mode_request(const FlightModeRequest& request, idl::ControlModeRequest& sample);
// action 이 정해진 값(OFFBOARD, HOLD)이 아니면 false
bool from_idl_flight_mode_request(const idl::ControlModeRequest& sample,
                                  FlightModeRequest& request);

void to_idl_flight_mode_reply(const vehicle_api::FlightModeReply& reply,
                              idl::ControlModeReply& sample);
// id 가 없으면 false
bool from_idl_flight_mode_reply(const idl::ControlModeReply& sample,
                                vehicle_api::FlightModeReply& reply);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_IDL_HPP_
