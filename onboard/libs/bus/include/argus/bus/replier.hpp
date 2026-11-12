// 요청-응답의 응답하는 쪽 인터페이스. 이름 하나에 묶이며 IBus::create_replier() 가 만든다.
// 규약: docs/spec/process_interfaces.md 3장, 5.2
// 구현: ZmqBus (bus_zmq) 가 만드는 ZmqReplier, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_REPLIER_HPP_
#define ARGUS_BUS_REPLIER_HPP_

#include <functional>
#include <string_view>

#include "argus/core/subscription.hpp"

namespace argus::bus {

class IReplier {
public:
    // request 는 요청 타입의 값, reply 는 구현이 기본값으로 만든 응답 타입의 값을 가리킨다.
    // handler 가 reply 를 채우면 그 값이 요청한 쪽에 간다
    using RequestHandler = std::function<void(const void* request, void* reply)>;

    IReplier() = default;
    virtual ~IReplier() = default;
    IReplier(const IReplier&) = delete;
    IReplier& operator=(const IReplier&) = delete;
    IReplier(IReplier&&) = delete;
    IReplier& operator=(IReplier&&) = delete;

    virtual std::string_view name() const = 0;

    // 이 이름의 요청이 오면 handler 를 불러 handler 가 채운 reply 로 응답한다. handler 는 하나이고,
    // 다시 등록하면 앞의 등록을 대신한다. handler 는 core::IPoller::process_events() 안에서, 그
    // 함수를 부른 스레드에서 불린다. 반환된 Subscription 이 사라지면 등록이 해제된다.
    [[nodiscard]] virtual core::Subscription serve(RequestHandler handler) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_REPLIER_HPP_
