// control/mode_request 의 요청, 응답 메시지와 vehicle_api 값 타입 사이의 변환. 생성된 메시지 타입
// (argus_msg/control_mode_request.proto 의 argus::msg::ControlModeRequest, ControlModeReply)을
// 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_

#include <cstdint>
#include <optional>

#include "argus/bus/message.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_bus {

// 요청 하나. IFlightModeControl 의 함수 3개에 대응한다
struct FlightModeRequest {
    enum class Action : std::uint8_t { Offboard, Hold };

    vehicle_api::RequestHeader header;
    Action action = Action::Hold;
};

bus::Bytes encode_flight_mode_request(const FlightModeRequest& request);
// 본문을 파싱할 수 없거나 header 또는 action 이 없으면 nullopt
std::optional<FlightModeRequest> decode_flight_mode_request(const bus::Bytes& body);

bus::Bytes encode_flight_mode_reply(const vehicle_api::FlightModeReply& reply);
// 본문을 파싱할 수 없으면 nullopt
std::optional<vehicle_api::FlightModeReply> decode_flight_mode_reply(const bus::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_
