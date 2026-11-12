// follow/command 의 요청 메시지, 응답 follow/result 메시지와 vehicle_api 값 타입 사이의 변환.
// 생성된 메시지 타입(argus_msg/follow_command.proto 의 argus::msg::FollowCommand, FollowResult)을
// 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_

#include <optional>
#include <variant>

#include "argus/bus/message.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_bus {

// 요청 하나. action 의 타입이 IFollowControl 의 함수 3개(track_start, stop, scan_start)에 대응한다
struct FollowCommandRequest {
    vehicle_api::RequestHeader header;
    std::variant<vehicle_api::TrackTarget, vehicle_api::StopReason, vehicle_api::ScanRequest>
        action;
};

// ScanRequest 의 비어 있는 항목은 메시지에서 NaN 으로 보낸다 (argus/vehicle_api/types.hpp)
bus::Bytes encode_follow_command(const FollowCommandRequest& request);
// 본문을 파싱할 수 없거나 header 또는 action 이 없거나 stop 의 reason 이 정해진 값이 아니면 nullopt
std::optional<FollowCommandRequest> decode_follow_command(const bus::Bytes& body);

bus::Bytes encode_follow_result(const vehicle_api::FollowResult& result);
// 본문을 파싱할 수 없거나 id 가 없거나 outcome 이 정해진 값이 아니면 nullopt
std::optional<vehicle_api::FollowResult> decode_follow_result(const bus::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_COMMAND_CODEC_HPP_
