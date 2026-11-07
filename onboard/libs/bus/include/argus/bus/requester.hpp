// 요청-응답의 요청하는 쪽 인터페이스. 이름 하나에 묶이며 IBus::create_requester() 가 만든다.
// 규약: docs/spec/process_interfaces.md 3장, 5.2
// 구현: (#59) ZmqRequester, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_REQUESTER_HPP_
#define ARGUS_BUS_REQUESTER_HPP_

#include <chrono>
#include <functional>
#include <optional>
#include <string_view>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

class IRequester {
public:
    // 응답, 또는 timeout 안에 응답이 없으면 nullopt
    using ReplyHandler = std::function<void(std::optional<BusMessage>)>;

    IRequester() = default;
    virtual ~IRequester() = default;
    IRequester(const IRequester&) = delete;
    IRequester& operator=(const IRequester&) = delete;
    IRequester(IRequester&&) = delete;
    IRequester& operator=(IRequester&&) = delete;

    virtual std::string_view name() const = 0;

    // 요청을 보내고 바로 반환한다 (응답을 기다리지 않는다).
    // on_reply 는 core::IPoller::process_events() 안에서 정확히 한 번 불린다.
    // 반환된 Subscription 이 먼저 사라지면 on_reply 는 불리지 않는다.
    [[nodiscard]] virtual core::Subscription request(const Bytes& request,
                                                     std::chrono::milliseconds timeout,
                                                     ReplyHandler on_reply) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_REQUESTER_HPP_
