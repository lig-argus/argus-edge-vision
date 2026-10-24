#include "argus/core/event_loop.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace argus::core {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;

// 정지 신호를 받을 루프. 신호 처리기 안에서는 원자 변수만 읽고 쓴다 (async-signal-safe)
std::atomic<EventLoop*> g_signal_loop{nullptr};

void on_stop_signal(int /*signal*/) {
    EventLoop* loop = g_signal_loop.load();
    if (loop != nullptr) {
        loop->stop();
    }
}

}  // namespace

EventLoop::EventLoop(IPoller& poller, const IClock& clock, std::chrono::milliseconds budget)
    : poller_(poller), clock_(clock), budget_(budget), timers_(std::make_shared<Timers>()) {}

EventLoop::~EventLoop() {
    EventLoop* self = this;
    g_signal_loop.compare_exchange_strong(self, nullptr);
}

PeriodicTimer EventLoop::add_periodic(std::chrono::milliseconds period,
                                      std::function<void()> task) {
    const std::uint64_t id = timers_->next_id++;
    const std::int64_t period_ns = std::max<std::int64_t>(period.count(), 1) * kNsPerMs;
    timers_->by_id[id] = Timer{period_ns, clock_.now_monotonic_ns() + period_ns, std::move(task)};

    std::weak_ptr<Timers> weak = timers_;
    return PeriodicTimer(Subscription([weak, id] {
        if (const auto timers = weak.lock()) {
            timers->by_id.erase(id);
        }
    }));
}

void EventLoop::run() {
    while (!stop_requested_.load()) {
        run_once();
    }
}

void EventLoop::run_once() {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    std::int64_t next_due_ns = std::numeric_limits<std::int64_t>::max();
    for (const auto& [id, timer] : timers_->by_id) {
        next_due_ns = std::min(next_due_ns, timer.next_due_ns);
    }
    const std::int64_t max_wait_ns = kMaxWait.count() * kNsPerMs;
    const std::int64_t wait_ns = std::clamp<std::int64_t>(
        next_due_ns == std::numeric_limits<std::int64_t>::max() ? max_wait_ns
                                                                : next_due_ns - now_ns,
        0, max_wait_ns);

    // 남은 시간을 ms 로 올림해 기다린다. 내리면 시각 직전에 깨어나 한 번 더 기다리게 된다
    poller_.process_events(std::chrono::milliseconds{(wait_ns + kNsPerMs - 1) / kNsPerMs}, budget_);

    if (!stop_requested_.load()) {
        run_due_timers();
    }
}

void EventLoop::run_due_timers() {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    std::vector<std::uint64_t> due;
    for (const auto& [id, timer] : timers_->by_id) {
        if (timer.next_due_ns <= now_ns) {
            due.push_back(id);
        }
    }
    for (const std::uint64_t id : due) {
        // 앞 작업이 이 타이머를 취소했을 수 있다
        const auto found = timers_->by_id.find(id);
        if (found == timers_->by_id.end()) {
            continue;
        }
        Timer& timer = found->second;
        timer.next_due_ns += timer.period_ns;
        if (timer.next_due_ns <= now_ns) {
            timer.next_due_ns = now_ns + timer.period_ns;  // 놓친 시각은 몰아서 부르지 않는다
        }
        // task 가 자기 타이머를 취소해도 안전하도록 복사해서 부른다
        const std::function<void()> task = timer.task;
        task();
    }
}

void EventLoop::stop() { stop_requested_.store(true); }

void stop_on_signals(EventLoop* loop) {
    g_signal_loop.store(loop);
    static std::once_flag install_once;
    std::call_once(install_once, [] {
        std::signal(SIGINT, on_stop_signal);
        std::signal(SIGTERM, on_stop_signal);
    });
}

}  // namespace argus::core
