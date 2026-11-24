// control/mode_request 의 요청, 응답 메시지와 vehicle_api 값 타입 사이의 변환. 생성된 메시지 타입
// (argus_msg/control_mode_request.proto 의 argus::msg::ControlModeRequest, ControlModeReply)을
// 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_

#include <optional>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"

namespace argus::vehicle_api_bus {

bus_zmq::Bytes encode_flight_mode_request(const FlightModeRequest& request);
// 본문을 파싱할 수 없거나 header 또는 action 이 없으면 nullopt
std::optional<FlightModeRequest> decode_flight_mode_request(const bus_zmq::Bytes& body);

bus_zmq::Bytes encode_flight_mode_reply(const vehicle_api::FlightModeReply& reply);
// 본문을 파싱할 수 없으면 nullopt
std::optional<vehicle_api::FlightModeReply> decode_flight_mode_reply(const bus_zmq::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CONTROL_MODE_REQUEST_CODEC_HPP_
