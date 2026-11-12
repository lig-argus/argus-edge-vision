// control/setpoint 메시지와 vehicle_api::VelocitySetpoint 사이의 변환. 생성된 메시지 타입
// (argus_msg/control_setpoint.proto 의 argus::msg::ControlSetpoint)을 사용하는 코드는 이 파일의
// 두 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_CODEC_HPP_

#include <cstdint>
#include <optional>

#include "argus/bus/message.hpp"
#include "argus/vehicle_api/types.hpp"
#include "codec_common.hpp"

namespace argus::vehicle_api_bus {

// seq 는 발행마다 1 증가하는 번호 (process_interfaces.md 3장 공통 필드)
bus::Bytes encode_control_setpoint(const vehicle_api::VelocitySetpoint& setpoint,
                                   std::uint64_t seq);

// 본문을 파싱할 수 없으면 nullopt
std::optional<Decoded<vehicle_api::VelocitySetpoint>> decode_control_setpoint(
    const bus::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CONTROL_SETPOINT_CODEC_HPP_
