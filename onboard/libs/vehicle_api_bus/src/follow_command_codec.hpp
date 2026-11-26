// follow/command 의 요청 메시지, 응답 follow/result 메시지와 vehicle_api 값 타입 사이의 변환.
// 생성된 메시지 타입(argus_msg/follow_command.proto 의 argus::msg::FollowCommand, FollowResult)을
// 사용하는 코드는 이 파일의 함수뿐이다. follow_command.proto 에 정의된 enum(Sensor,
// FollowOutcome)의 변환은 이 enum 을 사용하는 다른 메시지(follow_status.proto)의 변환도 사용한다.
// 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_

#include <optional>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus_msg/follow_command.pb.h"

namespace argus::vehicle_api_bus {

msg::Sensor sensor_to_message(vehicle_api::Sensor sensor);
// 이 코드에 정의되지 않은 값(메시지를 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::Sensor sensor_from_message(msg::Sensor sensor);

msg::FollowOutcome outcome_to_message(vehicle_api::FollowOutcome outcome);
// UNSPECIFIED 와 이 코드에 정의되지 않은 값은 nullopt
std::optional<vehicle_api::FollowOutcome> outcome_from_message(msg::FollowOutcome outcome);

// ScanRequest 의 비어 있는 항목은 메시지에서 NaN 으로 보낸다 (argus/vehicle_api/types.hpp)
bus_zmq::Bytes encode_follow_command(const FollowCommandRequest& request);
// 본문을 파싱할 수 없거나 header 또는 action 이 없거나 stop 의 reason 이 정해진 값이 아니면
// nullopt
std::optional<FollowCommandRequest> decode_follow_command(const bus_zmq::Bytes& body);

bus_zmq::Bytes encode_follow_result(const vehicle_api::FollowResult& result);
// 본문을 파싱할 수 없거나 id 가 없거나 outcome 이 정해진 값이 아니면 nullopt
std::optional<vehicle_api::FollowResult> decode_follow_result(const bus_zmq::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_
