// RPC 요청 인터페이스 (요청을 보내고 응답을 비동기로 받는다)
// 규약: docs/spec/process_interfaces.md 3장, 5.2
// 구현: (다음 PR) ZmqRpcClient, 테스트용 FakeBus (bus_testing)
#ifndef ARGUS_BUS_RPC_CLIENT_HPP_
#define ARGUS_BUS_RPC_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <optional>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

class IRpcClient {
public:
    // 응답, 또는 timeout 안에 응답이 없으면 nullopt
    using ReplyHandler = std::function<void(std::optional<BusMessage>)>;

    IRpcClient() = default;
    virtual ~IRpcClient() = default;
    IRpcClient(const IRpcClient&) = delete;
    IRpcClient& operator=(const IRpcClient&) = delete;
    IRpcClient(IRpcClient&&) = delete;
    IRpcClient& operator=(IRpcClient&&) = delete;

    // request.topic 이름의 RPC 를 요청하고 바로 반환한다 (응답을 기다리지 않는다).
    // on_reply 는 core::IPoller::process_events() 안에서 정확히 한 번 불린다.
    // 반환된 Subscription 이 먼저 사라지면 on_reply 는 불리지 않는다.
    // 같은 클라이언트의 요청은 보낸 순서대로 하나씩 처리된다.
    [[nodiscard]] virtual core::Subscription call(const BusMessage& request,
                                                  std::chrono::milliseconds timeout,
                                                  ReplyHandler on_reply) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_RPC_CLIENT_HPP_
