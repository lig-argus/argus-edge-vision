// 실행 파일 main() 의 공통 루프: 소켓 이벤트 처리, 주기 작업, 정지
// 규약: docs/design/uml/class_onboard.md 1절
#ifndef ARGUS_CORE_EVENT_LOOP_HPP_
#define ARGUS_CORE_EVENT_LOOP_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <utility>

#include "argus/core/clock.hpp"
#include "argus/core/poller.hpp"
#include "argus/core/subscription.hpp"

namespace argus::core {

// 주기 작업 한 건을 나타내는 RAII handle. 객체가 사라지면 그 주기 작업이 더 불리지 않는다.
class PeriodicTimer {
public:
    PeriodicTimer() = default;
    explicit PeriodicTimer(Subscription registration) : registration_(std::move(registration)) {}

    void cancel() { registration_.reset(); }
    bool active() const { return registration_.active(); }

private:
    Subscription registration_;
};

class EventLoop {
public:
    // budget: process_events() 한 번에 handler 를 부르는 최대 시간
    EventLoop(IPoller& poller, const IClock& clock, std::chrono::milliseconds budget);
    ~EventLoop();

    EventLoop(const EventLoop&) = delete;
    EventLoop& operator=(const EventLoop&) = delete;
    EventLoop(EventLoop&&) = delete;
    EventLoop& operator=(EventLoop&&) = delete;

    // period 마다 task 를 부른다. 첫 호출은 등록하고 period 가 지난 뒤.
    // task 가 오래 걸려 다음 시각을 놓치면 놓친 횟수만큼 몰아서 부르지 않고, 지금부터 period 뒤로
    // 미룬다. task 는 이 루프를 돌리는 스레드에서 불린다.
    [[nodiscard]] PeriodicTimer add_periodic(std::chrono::milliseconds period,
                                             std::function<void()> task);

    // stop() 이 불릴 때까지 run_once() 를 반복한다. 한 번 stop() 이 불리면 다시 run() 해도 바로
    // 반환한다.
    void run();

    // 한 번: 다음 주기 작업까지 남은 시간(주기 작업이 없으면 kMaxWait) 동안 poller 로 소켓 이벤트를
    // 처리하고, 때가 된 주기 작업을 부른다. 테스트는 이것으로 한 단계씩 진행한다.
    void run_once();

    // 루프를 멈춘다. handler, 주기 작업, 정지 신호 처리기에서 불러도 된다 (원자 변수 하나만
    // 바꾼다).
    void stop();
    bool stopped() const { return stop_requested_.load(); }

    // 주기 작업이 없을 때 한 번에 기다리는 최대 시간. 이 간격으로 stop() 여부를 다시 확인한다.
    static constexpr std::chrono::milliseconds kMaxWait{100};

private:
    struct Timer {
        std::int64_t period_ns = 0;
        std::int64_t next_due_ns = 0;
        std::function<void()> task;
    };
    // PeriodicTimer 가 루프보다 오래 살아도 안전하도록, 타이머 목록은 공유 객체에 두고
    // handle 은 이 객체를 약하게(weak_ptr) 가리킨다
    struct Timers {
        std::map<std::uint64_t, Timer> by_id;
        std::uint64_t next_id = 1;
    };

    void run_due_timers();

    IPoller& poller_;
    const IClock& clock_;
    std::chrono::milliseconds budget_;
    std::shared_ptr<Timers> timers_;
    std::atomic<bool> stop_requested_{false};
};

// SIGINT(Ctrl+C), SIGTERM(systemctl stop, kill) 을 받으면 loop.stop() 을 부르게 한다.
// 프로세스에 하나만 연결할 수 있다 (나중에 부르면 그 루프로 바뀐다). loop 가 사라지기 전에
// stop_on_signals(nullptr) 로 연결을 끊는다. EventLoop 소멸자도 자기가 연결돼 있으면 끊는다.
void stop_on_signals(EventLoop* loop);

}  // namespace argus::core

#endif  // ARGUS_CORE_EVENT_LOOP_HPP_
