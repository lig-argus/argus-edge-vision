// 테스트용 IPoller. 실제로 기다리지 않고 FakeClock 을 앞으로 움직인다.
// 다른 Fake(FakeBus, FakeMavlinkConnection)나 테스트가 처리 시각을 정해 이벤트를 예약하면,
// process_events() 가 FakeClock 을 그 시각으로 옮기고 이벤트를 처리한다.
// 실제 구현과 같이 handler 는 process_events() 안에서만 불린다.
#ifndef ARGUS_CORE_TESTING_FAKE_POLLER_HPP_
#define ARGUS_CORE_TESTING_FAKE_POLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "argus/core/poller.hpp"
#include "argus/core_testing/fake_clock.hpp"

namespace argus::core_testing {

class FakePoller : public core::IPoller {
public:
    explicit FakePoller(FakeClock& clock) : clock_(clock) {}

    // 다음 process_events() 호출에서 처리할 이벤트를 넣는다
    void post(std::function<void()> event) { post_at(clock_.now_monotonic_ns(), std::move(event)); }

    // FakeClock 의 현재 시각이 at_ns 이상이 되면 처리할 이벤트를 넣는다
    void post_at(std::int64_t at_ns, std::function<void()> event) {
        events_.emplace(at_ns, std::move(event));
    }

    void process_events(std::chrono::milliseconds wait,
                        std::chrono::milliseconds /*budget*/) override {
        waits_.push_back(wait);
        const std::int64_t now_ns = clock_.now_monotonic_ns();
        const std::int64_t deadline_ns = now_ns + wait.count() * kNsPerMs;
        if (events_.empty() || events_.begin()->first > deadline_ns) {
            clock_.advance_ns(deadline_ns -
                              now_ns);  // 아무것도 오지 않아 wait 만큼 기다린 것과 같음
            return;
        }
        // 예약 시각이 가장 이른 이벤트의 시각으로 FakeClock 을 옮기고,
        // 예약 시각이 그 시각 이하인 이벤트를 모두 처리한다
        const std::int64_t wake_ns = std::max(now_ns, events_.begin()->first);
        clock_.advance_ns(wake_ns - now_ns);
        while (!events_.empty() && events_.begin()->first <= wake_ns) {
            auto event = std::move(events_.begin()->second);
            events_.erase(events_.begin());
            event();
        }
    }

    // 테스트 확인용: process_events() 가 받은 wait 값들
    const std::vector<std::chrono::milliseconds>& waits() const { return waits_; }

private:
    static constexpr std::int64_t kNsPerMs = 1'000'000;

    FakeClock& clock_;
    std::multimap<std::int64_t, std::function<void()>> events_;
    std::vector<std::chrono::milliseconds> waits_;
};

}  // namespace argus::core_testing

#endif  // ARGUS_CORE_TESTING_FAKE_POLLER_HPP_
