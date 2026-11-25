// core::IPoller 의 poll(2) 구현. file descriptor 와 타이머를 함께 기다린다. 통신 라이브러리에
// 의존하지 않는다.
// 규약: docs/design/uml/class_mavlink_gateway.md 1절
// 사용: ZeroMQ 를 사용하지 않는 실행 파일의 main() 이 하나를 만들어 EventLoop(IPoller), 통신 구현과
// UDP 연결 클래스(IFdWatcher, ITimerScheduler)에 넘긴다. ZeroMQ 를 사용하는 실행 파일은
// bus_zmq 의 ZmqPoller 를 사용한다
//
// 규칙 (ZmqPoller 와 같음)
// - 한 스레드에서만 사용한다. 등록, 해제, process_events() 를 모두 같은 스레드에서 호출한다.
// - fd 의 handler 는 읽을 데이터가 남아 있는 동안 다시 호출된다. handler 는 한 번 호출될 때
//   데이터를 읽어야 한다. 읽지 않으면 budget 이 끝날 때까지 같은 handler 가 반복해서 호출된다.
// - handler 안에서 등록과 해제를 해도 된다 (자기 자신의 해제 포함).
// - 등록한 fd 는 Subscription 을 해제할 때까지 열려 있어야 한다. poller 는 fd 를 닫지 않는다.
#ifndef ARGUS_CORE_POLL_POLLER_HPP_
#define ARGUS_CORE_POLL_POLLER_HPP_

#include <poll.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "argus/core/clock.hpp"
#include "argus/core/fd_watcher.hpp"
#include "argus/core/poller.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core/timer_scheduler.hpp"

namespace argus::core {

class PollPoller : public IPoller, public IFdWatcher, public ITimerScheduler {
public:
    using Handler = std::function<void()>;

    // clock 은 budget 과 타이머 시각을 재는 데 사용한다. poller 보다 오래 유지되어야 한다
    explicit PollPoller(const IClock& clock);

    // fd 가 음수이거나 handler 가 비어 있으면 등록하지 않고 빈 Subscription 을 반환한다
    [[nodiscard]] Subscription add_fd(int fd, Handler on_readable) override;

    [[nodiscard]] Subscription add_timer_at(std::int64_t deadline_ns, Handler handler) override;

    // 이벤트가 없으면 최대 wait 동안 기다린다 (가장 이른 타이머 시각이 더 가까우면 그때까지).
    // 이벤트가 있으면 handler 를 차례로 호출하고, 남은 이벤트가 없거나 handler 를 호출한 시간이
    // budget 에 이르면 반환한다. 이벤트가 있으면 budget 과 관계없이 handler 를 적어도 하나
    // 호출한다. budget 때문에 호출하지 못한 handler 는 다음 호출에서 먼저 호출한다. 등록한
    // 시각(deadline_ns)이 지난 타이머는 budget 과 관계없이 반환 전에 시각 순서대로 모두 호출한다
    void process_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget) override;

private:
    struct FdEntry {
        int fd = -1;  // POSIX 의 file descriptor 형
        Handler handler;
    };

    struct Timer {
        std::int64_t deadline_ns = 0;
        Handler handler;
    };

    // Subscription 이 poller 보다 오래 유지되어도 안전하도록 등록 목록은 공유 객체에 두고,
    // Subscription 은 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    struct Registry {
        std::map<std::uint64_t, FdEntry> fds_by_id;
        std::map<std::uint64_t, Timer> timers_by_id;
        std::uint64_t next_id = 1;
        bool changed = true;  // 등록이나 해제가 있어 poll 목록을 다시 만들어야 함
    };

    void rebuild_poll_items();
    std::chrono::milliseconds limit_to_next_timer(std::chrono::milliseconds wait) const;
    void dispatch_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget);
    void run_due_timers();

    const IClock& clock_;
    std::shared_ptr<Registry> registry_;
    std::vector<pollfd> poll_items_;
    std::vector<std::uint64_t> poll_item_ids_;  // poll_items_ 와 같은 순서의 등록 번호
    std::vector<std::uint64_t> ready_ids_;

    // 마지막으로 handler 를 호출한 등록 번호. 다음에는 이 번호 다음 등록부터 호출한다 (budget 으로
    // 끊겨도 같은 fd 만 계속 처리되지 않게)
    std::uint64_t last_served_id_ = 0;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_POLL_POLLER_HPP_
