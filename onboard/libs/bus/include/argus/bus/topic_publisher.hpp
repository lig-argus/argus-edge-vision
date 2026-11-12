// 토픽 발행 인터페이스. 토픽 하나에 묶이며 IBus::create_publisher() 가 만든다.
// 규약: docs/spec/process_interfaces.md 3장, 5.1
// 구현: (#59) ZmqTopicPublisher, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_TOPIC_PUBLISHER_HPP_
#define ARGUS_BUS_TOPIC_PUBLISHER_HPP_

#include <string_view>

#include "argus/bus/message.hpp"

namespace argus::bus {

class ITopicPublisher {
public:
    ITopicPublisher() = default;
    virtual ~ITopicPublisher() = default;
    ITopicPublisher(const ITopicPublisher&) = delete;
    ITopicPublisher& operator=(const ITopicPublisher&) = delete;
    ITopicPublisher(ITopicPublisher&&) = delete;
    ITopicPublisher& operator=(ITopicPublisher&&) = delete;

    virtual std::string_view topic() const = 0;

    // 기다리지 않고 바로 반환한다 (받는 쪽이 느려도 보내는 쪽이 멈추지 않게). 보내지 못했으면 false
    // 이며 그 메시지는 어느 구독자에게도 가지 않는다. true 는 구현이 메시지를 받았다는 뜻이고, 모든
    // 구독자가 받는다는 보장이 아니다. 큐를 구독자마다 따로 두는 구현은 느린 구독자에게 갈 메시지만
    // 버리며 이때도 true 를 반환한다. 유실 여부는 받는 쪽이 메시지의 seq 로 판정한다. 구현은 body
    // 와 binary 를 하나로 합쳐 보낼 수 있다. 받는 쪽은 BusMessage 로 둘을 다시 받는다.
    virtual bool publish(const Bytes& body, const Bytes& binary) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_TOPIC_PUBLISHER_HPP_
