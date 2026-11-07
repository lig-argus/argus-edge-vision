// 요청-응답의 응답하는 쪽 인터페이스. 이름 하나에 묶이며 IBus::create_replier() 가 만든다.
// 규약: docs/spec/process_interfaces.md 3장, 5.2
// 구현: (#59) ZmqReplier, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_REPLIER_HPP_
#define ARGUS_BUS_REPLIER_HPP_

#include <functional>
#include <string_view>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

class IReplier {
public:
    // 요청을 받아 응답 본문을 반환한다. 반환값이 그대로 요청한 쪽에 간다
    using RequestHandler = std::function<Bytes(const BusMessage&)>;

    IReplier() = default;
    virtual ~IReplier() = default;
    IReplier(const IReplier&) = delete;
    IReplier& operator=(const IReplier&) = delete;
    IReplier(IReplier&&) = delete;
    IReplier& operator=(IReplier&&) = delete;

    virtual std::string_view name() const = 0;

    // 이 이름의 요청이 오면 handler 를 불러 그 반환값으로 응답한다. handler 는 하나이고, 다시
    // 등록하면 앞의 등록을 대신한다. handler 는 core::IPoller::process_events() 안에서, 그 함수를
    // 부른 스레드에서 불린다. 반환된 Subscription 이 사라지면 등록이 해제된다.
    [[nodiscard]] virtual core::Subscription serve(RequestHandler handler) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_REPLIER_HPP_
