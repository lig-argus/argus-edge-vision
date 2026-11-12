// 여러 메시지의 변환 함수가 함께 쓰는 것: 바이트 변환, 공통 타입(argus_msg/common.proto)의 변환
// 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_CODEC_COMMON_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_CODEC_COMMON_HPP_

#include <cstdint>

#include "argus/bus/message.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus_msg/common.pb.h"

namespace argus::vehicle_api_bus {

// 받은 메시지에서 꺼낸 값과 그 메시지의 seq
template <typename T>
struct Decoded {
    T value;
    std::uint64_t seq = 0;
};

// 직렬화에 실패하면 빈 바이트를 반환한다. 실패하는 경우는 메시지 크기가 2 GB 를 넘을 때뿐이며 이
// 라이브러리의 메시지에서는 생기지 않는다
template <typename Message>
bus::Bytes to_bytes(const Message& message) {
    bus::Bytes body(message.ByteSizeLong());
    if (!message.SerializeToArray(body.data(), static_cast<int>(body.size()))) {
        body.clear();
    }
    return body;
}

template <typename Message>
bool parse(const bus::Bytes& body, Message& message) {
    return message.ParseFromArray(body.data(), static_cast<int>(body.size()));
}

void to_message(const vehicle_api::RequestId& id, msg::RequestId& message);
vehicle_api::RequestId from_message(const msg::RequestId& message);

void to_message(const vehicle_api::RequestHeader& header, msg::RequestHeader& message);
vehicle_api::RequestHeader from_message(const msg::RequestHeader& message);

msg::FlightMode to_message(vehicle_api::FlightMode mode);
// 이 코드에 정의되지 않은 값(메시지를 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::FlightMode from_message(msg::FlightMode mode);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_CODEC_COMMON_HPP_
