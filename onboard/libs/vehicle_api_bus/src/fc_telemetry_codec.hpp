// fc/attitude, fc/state, fc/mode_event 메시지와 vehicle_api 값 타입 사이의 변환. 생성된 메시지 타입
// (argus_msg/fc_attitude.proto, fc_state.proto, fc_mode_event.proto 의 argus::msg::FcAttitude,
// FcState, FcModeEvent)을 사용하는 코드는 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_CODEC_HPP_

#include <cstdint>
#include <optional>

#include "argus/bus/message.hpp"
#include "argus/vehicle_api/types.hpp"
#include "codec_common.hpp"

namespace argus::vehicle_api_bus {

// seq 는 토픽마다 따로 세며 발행마다 1 증가한다 (process_interfaces.md 3장 공통 필드).
// decode_* 는 본문을 파싱할 수 없으면 nullopt

bus::Bytes encode_fc_attitude(const vehicle_api::Attitude& attitude, std::uint64_t seq);
std::optional<Decoded<vehicle_api::Attitude>> decode_fc_attitude(const bus::Bytes& body);

// fc/state 는 local, downward_distance, global, status 만 싣는다. state.attitude 는 싣지 않으며
// 받은 값의 attitude 는 항상 nullopt 다 (자세는 fc/attitude 로 보낸다)
bus::Bytes encode_fc_state(const vehicle_api::VehicleState& state, std::uint64_t seq);
std::optional<Decoded<vehicle_api::VehicleState>> decode_fc_state(const bus::Bytes& body);

bus::Bytes encode_fc_mode_event(const vehicle_api::FlightModeEvent& event, std::uint64_t seq);
std::optional<Decoded<vehicle_api::FlightModeEvent>> decode_fc_mode_event(const bus::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FC_TELEMETRY_CODEC_HPP_
