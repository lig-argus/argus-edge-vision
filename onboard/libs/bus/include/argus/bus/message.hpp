// bus 가 나르는 ZeroMQ multipart 메시지 하나.
// bus 는 MessagePack 을 풀지 않고 바이트 그대로 전달한다. 구조체로 바꾸는 일은 어댑터가 한다.
// 규약: docs/spec/process_interfaces.md 3장
//       (frame[0] 토픽, frame[1] MessagePack 본문, frame[2] 바이너리)
#ifndef ARGUS_BUS_MESSAGE_HPP_
#define ARGUS_BUS_MESSAGE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace argus::bus {

using Bytes = std::vector<std::uint8_t>;

struct BusMessage {
    // frame[0]: 토픽 이름 또는 RPC 이름 (argus/protocol/topics.hpp 의 상수)
    std::string topic;
    // frame[1] 부터의 frame. parts[0] = MessagePack 본문, parts[1] = 바이너리(선택, 예: gray8 픽셀)
    std::vector<Bytes> parts;

    friend bool operator==(const BusMessage& lhs, const BusMessage& rhs) {
        return lhs.topic == rhs.topic && lhs.parts == rhs.parts;
    }
    friend bool operator!=(const BusMessage& lhs, const BusMessage& rhs) { return !(lhs == rhs); }
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_MESSAGE_HPP_
