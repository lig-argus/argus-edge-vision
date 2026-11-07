// bus 가 나르는 메시지 하나. 직렬화된 바이트를 그대로 전달하고, 구조체로 바꾸는 일은 어댑터가 한다.
// 규약: docs/spec/process_interfaces.md 3장 (직렬화한 본문 하나와, 필요하면 본문과 따로 보내는
// 바이트 열 하나)
#ifndef ARGUS_BUS_MESSAGE_HPP_
#define ARGUS_BUS_MESSAGE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace argus::bus {

using Bytes = std::vector<std::uint8_t>;

struct BusMessage {
    std::string topic;  // 토픽 이름 또는 요청-응답 이름 (argus/protocol/topics.hpp 의 상수)
    Bytes body;         // 직렬화된 본문
    Bytes binary;       // 선택. 큰 바이너리(예: gray8 픽셀). 비어 있으면 없음

    friend bool operator==(const BusMessage& lhs, const BusMessage& rhs) {
        return lhs.topic == rhs.topic && lhs.body == rhs.body && lhs.binary == rhs.binary;
    }
    friend bool operator!=(const BusMessage& lhs, const BusMessage& rhs) { return !(lhs == rhs); }
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_MESSAGE_HPP_
