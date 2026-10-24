// 등록된 소켓의 이벤트를 기다리고 처리하는 인터페이스
// 규약: docs/design/uml/class_onboard.md 1절
// 구현: (다음 PR) ZmqPoller (bus) — zmq_poll 로 ZeroMQ 소켓과 MAVLink UDP 소켓을 함께 기다림,
//      테스트용 FakePoller (core_testing)
// 사용: EventLoop
#ifndef ARGUS_CORE_POLLER_HPP_
#define ARGUS_CORE_POLLER_HPP_

#include <chrono>

namespace argus::core {

// 기다리는 일과 handler 를 부르는 일만 한다 (Reactor 패턴).
// 받은 데이터의 해석은 소켓에서 데이터를 읽어 해석하는 클래스(ZmqTopicSubscriber,
// UdpMavlinkConnection 등)가 하고, 처리는 등록된 handler 가 한다.
class IPoller {
public:
    IPoller() = default;
    virtual ~IPoller() = default;
    IPoller(const IPoller&) = delete;
    IPoller& operator=(const IPoller&) = delete;
    IPoller(IPoller&&) = delete;
    IPoller& operator=(IPoller&&) = delete;

    // 등록된 모든 소켓(ZeroMQ, UDP)을 최대 wait 동안 함께 기다리고,
    // 데이터가 도착한 소켓의 handler 를 최대 budget 시간 동안 부른 뒤 반환한다.
    // 처리하지 못한 이벤트는 다음 호출에서 처리한다.
    // 모든 handler 는 이 함수 안에서, 이 함수를 부른 스레드에서만 불린다.
    // handler 는 짧게 끝내야 한다. 오래 걸리면 다른 소켓 이벤트와 주기 작업이 밀린다.
    virtual void process_events(std::chrono::milliseconds wait,
                                std::chrono::milliseconds budget) = 0;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_POLLER_HPP_
