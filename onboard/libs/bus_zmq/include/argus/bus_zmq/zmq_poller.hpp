// core::IPoller 의 ZeroMQ 구현. zmq_poll 로 ZeroMQ 소켓과 file descriptor(UDP 소켓 등)를 함께
// 기다린다.
// 규약: docs/design/uml/class_onboard.md 1절
// 사용: 실행 파일의 main() 이 하나를 만들어 EventLoop, ZmqBus, UDP 연결 클래스에 넘긴다
//
// 규칙
// - 한 스레드에서만 사용한다. 등록, 해제, process_events() 를 모두 같은 스레드에서 부른다.
// - handler 는 읽을 데이터가 남아 있는 동안 다시 불린다. handler 는 한 번 불릴 때 메시지를 하나
//   이상 읽어야 한다. 읽지 않으면 budget 이 끝날 때까지 같은 handler 가 반복해서 불린다.
// - handler 안에서 등록과 해제를 해도 된다 (자기 자신의 해제 포함).
// - 타이머는 한 번만 불린다. 불린 뒤에는 등록이 없어진다.
// - 등록한 소켓과 fd 는 Subscription 을 해제할 때까지 열려 있어야 한다. poller 는 닫지 않는다.
#ifndef ARGUS_BUS_ZMQ_ZMQ_POLLER_HPP_
#define ARGUS_BUS_ZMQ_ZMQ_POLLER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <zmq.hpp>

#include "argus/core/clock.hpp"
#include "argus/core/poller.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus_zmq {

class ZmqPoller : public core::IPoller {
public:
    using Handler = std::function<void()>;

    // clock 은 budget 을 재는 데 쓴다. poller 보다 오래 살아야 한다
    explicit ZmqPoller(const core::IClock& clock);

    // socket 에 읽을 메시지가 있으면 handler 를 부른다. 반환된 Subscription 이 사라지면 등록이
    // 해제된다. socket 이 비어 있거나 handler 가 비어 있으면 등록하지 않고 빈 Subscription 을
    // 반환한다.
    [[nodiscard]] core::Subscription add_socket(zmq::socket_ref socket, Handler handler);

    // fd 에 읽을 데이터가 있으면 handler 를 부른다. fd 가 음수이거나 handler 가 비어 있으면
    // 등록하지 않고 빈 Subscription 을 반환한다.
    [[nodiscard]] core::Subscription add_fd(int fd, Handler handler);

    // clock 의 시각이 deadline_ns 이상이 되면 process_events() 안에서 handler 를 한 번 부른다.
    // deadline_ns 는 IClock::now_monotonic_ns() 와 같은 기준이다. 이미 지난 시각이면 다음
    // process_events() 에서 부른다. handler 가 비어 있으면 등록하지 않고 빈 Subscription 을
    // 반환한다.
    [[nodiscard]] core::Subscription add_timer_at(std::int64_t deadline_ns, Handler handler);

    // 이벤트가 없으면 최대 wait 동안 기다린다 (가장 이른 타이머 시각이 더 가까우면 그때까지).
    // 이벤트가 있으면 handler 를 차례로 부르고, 남은 이벤트가 없거나 handler 를 부른 시간이 budget
    // 에 이르면 반환한다. 이벤트가 있으면 budget 과 관계없이 handler 를 적어도 하나 부른다 (budget
    // 이 0 이어도 진행된다). budget 때문에 부르지 못한 handler 는 다음 호출에서 먼저 부른다. 때가
    // 된 타이머는 budget 과 관계없이 반환 전에 모두 부른다.
    void process_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget) override;

private:
    struct Entry {
        void* socket = nullptr;  // ZeroMQ 소켓. fd 등록이면 nullptr
        int fd = -1;             // fd 등록일 때만 사용
        Handler handler;
    };

    struct Timer {
        std::int64_t deadline_ns = 0;
        Handler handler;
    };

    // Subscription 이 poller 보다 오래 살아도 안전하도록 등록 목록은 공유 객체에 두고,
    // Subscription 은 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    struct Registry {
        std::map<std::uint64_t, Entry> by_id;
        std::map<std::uint64_t, Timer> timers_by_id;
        std::uint64_t next_id = 1;
        bool changed = true;  // 등록이나 해제가 있어 poll 목록을 다시 만들어야 함
    };

    core::Subscription add(Entry entry);
    void rebuild_poll_items();
    std::chrono::milliseconds limit_to_next_timer(std::chrono::milliseconds wait) const;
    void dispatch_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget);
    void run_due_timers();

    const core::IClock& clock_;
    std::shared_ptr<Registry> registry_;
    std::vector<zmq::pollitem_t> poll_items_;
    std::vector<std::uint64_t> poll_item_ids_;  // poll_items_ 와 같은 순서의 등록 번호
    std::vector<std::uint64_t> ready_ids_;

    // 마지막으로 handler 를 부른 등록 번호. 다음에는 이 번호 다음 등록부터 부른다 (budget 으로
    // 끊겨도 같은 소켓만 계속 처리되지 않게)
    std::uint64_t last_served_id_ = 0;
};

}  // namespace argus::bus_zmq

#endif  // ARGUS_BUS_ZMQ_ZMQ_POLLER_HPP_
