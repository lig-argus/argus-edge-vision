// file descriptor 에 읽을 데이터가 있는 경우 handler 를 호출하도록 등록하는 인터페이스
// 구현: IPoller 를 구현하는 클래스 중 file descriptor 를 함께 기다릴 수 있는 클래스
// 사용: UDP 소켓을 여는 클래스(onboard 의 UdpMavlinkConnection)
#ifndef ARGUS_CORE_FD_WATCHER_HPP_
#define ARGUS_CORE_FD_WATCHER_HPP_

#include <functional>

#include "argus/core/subscription.hpp"

namespace argus::core {

// IPoller 와 분리한 이유: EventLoop 는 process_events() 만 사용하고, 소켓을 여는 클래스는 등록만
// 사용한다. 소켓을 여는 클래스가 poller 의 구현 클래스에 의존하지 않는다.
class IFdWatcher {
public:
    using Handler = std::function<void()>;

    IFdWatcher() = default;
    virtual ~IFdWatcher() = default;
    IFdWatcher(const IFdWatcher&) = delete;
    IFdWatcher& operator=(const IFdWatcher&) = delete;
    IFdWatcher(IFdWatcher&&) = delete;
    IFdWatcher& operator=(IFdWatcher&&) = delete;

    // fd 에 읽을 데이터가 있는 동안 IPoller::process_events() 안에서 on_readable 을 호출한다
    // (level-triggered). on_readable 이 데이터를 모두 읽지 않은 경우 다시 호출하며, 같은
    // process_events() 안에서 다시 호출할지는 구현이 정한다.
    // fd 는 등록한 클래스가 소유하며, 반환된 Subscription 을 해제한 후에 닫는다.
    // 반환된 Subscription 이 소멸하면 등록이 해제된다. Subscription 은 이 객체보다 늦게 소멸하여도
    // 된다. 등록하지 못한 경우 빈 Subscription 을 반환한다
    [[nodiscard]] virtual Subscription add_fd(int fd, Handler on_readable) = 0;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_FD_WATCHER_HPP_
