// 토픽 구독 인터페이스. 토픽 하나에 묶이며 IBus::create_subscriber() 가 만든다.
// 규약: docs/spec/process_interfaces.md 3장, 5.1
// 구현: (#59) ZmqTopicSubscriber, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_TOPIC_SUBSCRIBER_HPP_
#define ARGUS_BUS_TOPIC_SUBSCRIBER_HPP_

#include <functional>
#include <string_view>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

class ITopicSubscriber {
public:
    using Handler = std::function<void(const BusMessage&)>;

    ITopicSubscriber() = default;
    virtual ~ITopicSubscriber() = default;
    ITopicSubscriber(const ITopicSubscriber&) = delete;
    ITopicSubscriber& operator=(const ITopicSubscriber&) = delete;
    ITopicSubscriber(ITopicSubscriber&&) = delete;
    ITopicSubscriber& operator=(ITopicSubscriber&&) = delete;

    virtual std::string_view topic() const = 0;

    // 이 토픽의 메시지가 오면 handler 를 부른다. 여러 번 등록하면 등록한 순서대로 모두 부른다.
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 반환된 Subscription 이 사라지면 등록이 해제된다.
    [[nodiscard]] virtual core::Subscription subscribe(Handler handler) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_TOPIC_SUBSCRIBER_HPP_
