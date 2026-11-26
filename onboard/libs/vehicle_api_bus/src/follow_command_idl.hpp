// follow/command 의 요청 IDL 타입, 응답 follow/result IDL 타입과 vehicle_api 값 타입 사이의 변환.
// 생성된 IDL 타입(argus_msg/follow_command.idl 의 argus::idl::FollowCommand, FollowResult)을
// 사용하는 코드는 이 파일의 함수뿐이다. follow_command.idl 에 정의된 enum(Sensor, FollowOutcome)의
// 변환은 이 enum 을 사용하는 다른 IDL 타입(follow_status.idl)의 변환도 사용한다.
// 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_IDL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_IDL_HPP_

#include <optional>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus_msg/follow_command.hpp"

namespace argus::vehicle_api_bus {

idl::Sensor sensor_to_idl(vehicle_api::Sensor sensor);
// 이 코드에 정의되지 않은 값(표본을 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::Sensor sensor_from_idl(idl::Sensor sensor);

idl::FollowOutcome outcome_to_idl(vehicle_api::FollowOutcome outcome);
// UNSPECIFIED 와 이 코드에 정의되지 않은 값은 nullopt
std::optional<vehicle_api::FollowOutcome> outcome_from_idl(idl::FollowOutcome outcome);

// ScanRequest 의 비어 있는 항목은 표본에서 NaN 으로 보낸다 (argus/vehicle_api/types.hpp)
void to_idl_follow_command(const FollowCommandRequest& request, idl::FollowCommand& sample);
// action 의 판별 값이 정해진 값이 아니거나 stop 의 reason 이 정해진 값이 아니면 false
bool from_idl_follow_command(const idl::FollowCommand& sample, FollowCommandRequest& request);

void to_idl_follow_result(const vehicle_api::FollowResult& result, idl::FollowResult& sample);
// outcome 이 정해진 값이 아니면 false
bool from_idl_follow_result(const idl::FollowResult& sample, vehicle_api::FollowResult& result);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_IDL_HPP_
