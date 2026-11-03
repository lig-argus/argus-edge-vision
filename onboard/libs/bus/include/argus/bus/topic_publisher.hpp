// 토픽 발행 인터페이스
// 규약: docs/spec/process_interfaces.md 3장, 5.1
// 구현: (다음 PR) ZmqTopicPublisher, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_TOPIC_PUBLISHER_HPP_
#define ARGUS_BUS_TOPIC_PUBLISHER_HPP_

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

    // message.topic 토픽으로 내보낸다. 기다리지 않고 바로 반환한다.
    // 보낼 큐가 차 있으면 이 메시지를 버린다 (받는 쪽이 느려도 보내는 쪽이 막히지 않게).
    virtual void publish(const BusMessage& message) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_TOPIC_PUBLISHER_HPP_
