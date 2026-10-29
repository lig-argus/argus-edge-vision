// RPC 응답 인터페이스 (요청을 받아 응답을 돌려준다)
// 규약: docs/spec/process_interfaces.md 3장, 5.2
// 구현: (다음 PR) ZmqRpcServer, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_RPC_SERVER_HPP_
#define ARGUS_BUS_RPC_SERVER_HPP_

#include <functional>
#include <string_view>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

class IRpcServer {
public:
    // 요청을 받아 응답을 반환한다. 반환값이 그대로 요청한 쪽에 간다
    using RequestHandler = std::function<BusMessage(const BusMessage&)>;

    IRpcServer() = default;
    virtual ~IRpcServer() = default;
    IRpcServer(const IRpcServer&) = delete;
    IRpcServer& operator=(const IRpcServer&) = delete;
    IRpcServer(IRpcServer&&) = delete;
    IRpcServer& operator=(IRpcServer&&) = delete;

    // name 이름의 요청이 오면 handler 를 불러 그 반환값으로 응답한다. 한 이름에 handler 는
    // 하나이고, 같은 이름을 다시 등록하면 앞의 등록을 대신한다. 등록된 이름이 아닌 요청에는 parts
    // 가 빈 응답(topic = 요청 이름)을 보낸다. handler 는 core::IPoller::process_events() 안에서, 그
    // 함수를 부른 스레드에서 불린다. 반환된 Subscription 이 사라지면 등록이 해제된다.
    [[nodiscard]] virtual core::Subscription serve(std::string_view name,
                                                   RequestHandler handler) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_RPC_SERVER_HPP_
