// follow/status 메시지와 vehicle_api 값 타입 사이의 변환.
// 생성된 메시지 타입(argus_msg/follow_status.proto 의 argus::msg::FollowStatus)을 사용하는 코드는
// 이 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_CODEC_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_CODEC_HPP_

#include <cstdint>
#include <optional>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/vehicle_api/types.hpp"
#include "codec_common.hpp"

namespace argus::vehicle_api_bus {

bus_zmq::Bytes encode_follow_status(const vehicle_api::FollowStatus& status, std::uint64_t seq);
// 본문을 파싱할 수 없으면 nullopt.
// enum 의 UNSPECIFIED 와 이 코드에 정의되지 않은 값은 Unknown 으로 바꾼다. outcome 이 정해진 값이
// 아닌 명령 결과는 어느 결과인지 판단할 수 없으므로 command_results 에 넣지 않는다
std::optional<Decoded<vehicle_api::FollowStatus>> decode_follow_status(const bus_zmq::Bytes& body);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_CODEC_HPP_
