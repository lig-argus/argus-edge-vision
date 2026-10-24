#include "argus/core/event_loop.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::core {
namespace {

using core_testing::FakeClock;
using core_testing::FakePoller;
using std::chrono::milliseconds;

constexpr std::int64_t kMs = 1'000'000;

class EventLoopTest : public ::testing::Test {
protected:
    FakeClock clock_;
    FakePoller poller_{clock_};
    EventLoop loop_{poller_, clock_, milliseconds{5}};
};

TEST_F(EventLoopTest, CallsPeriodicTaskEveryPeriod) {
    std::vector<std::int64_t> called_at_ns;
    const PeriodicTimer timer = loop_.add_periodic(
        milliseconds{50}, [&] { called_at_ns.push_back(clock_.now_monotonic_ns()); });

    for (int i = 0; i < 3; ++i) {
        loop_.run_once();
    }

    EXPECT_EQ(called_at_ns, (std::vector<std::int64_t>{50 * kMs, 100 * kMs, 150 * kMs}));
}

TEST_F(EventLoopTest, WaitsOnlyUntilNextTimer) {
    const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [] {});
    clock_.advance_ns(20 * kMs);

    loop_.run_once();

    ASSERT_FALSE(poller_.waits().empty());
    EXPECT_EQ(poller_.waits().front(), milliseconds{30});
}

TEST_F(EventLoopTest, WaitsMaxWaitWithoutTimers) {
    loop_.run_once();

    ASSERT_FALSE(poller_.waits().empty());
    EXPECT_EQ(poller_.waits().front(), EventLoop::kMaxWait);
}

TEST_F(EventLoopTest, HandlesInputBeforeTimer) {
    std::vector<int> order;
    const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [&] { order.push_back(2); });
    poller_.post_at(10 * kMs, [&] { order.push_back(1); });

    loop_.run_once();  // 10 ms 에 이벤트가 와서 깨어남
    loop_.run_once();  // 50 ms 까지 기다린 뒤 주기 작업

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(EventLoopTest, DestroyedTimerIsNotCalled) {
    int calls = 0;
    {
        const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [&] { ++calls; });
    }

    loop_.run_once();
    loop_.run_once();

    EXPECT_EQ(calls, 0);
}

TEST_F(EventLoopTest, LateTaskDoesNotBurst) {
    std::vector<std::int64_t> called_at_ns;
    const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [&] {
        called_at_ns.push_back(clock_.now_monotonic_ns());
        if (called_at_ns.size() == 1) {
            clock_.advance_ns(120 * kMs);  // 첫 처리가 120 ms 걸려 100 ms, 150 ms 시각을 놓침
        }
    });

    for (int i = 0; i < 3; ++i) {
        loop_.run_once();
    }

    // 놓친 두 번을 몰아서 부르지 않고 한 번만 늦게 부른 뒤, 그때부터 다시 50 ms 간격
    EXPECT_EQ(called_at_ns, (std::vector<std::int64_t>{50 * kMs, 170 * kMs, 220 * kMs}));
}

TEST_F(EventLoopTest, StopFromTaskEndsRun) {
    int calls = 0;
    const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [&] {
        if (++calls == 3) {
            loop_.stop();
        }
    });

    loop_.run();

    EXPECT_EQ(calls, 3);
    EXPECT_TRUE(loop_.stopped());
}

TEST_F(EventLoopTest, TimerOutlivingLoopIsSafe) {
    PeriodicTimer timer;
    {
        FakeClock clock;
        FakePoller poller(clock);
        EventLoop loop(poller, clock, milliseconds{5});
        timer = loop.add_periodic(milliseconds{50}, [] {});
    }
    timer.cancel();  // 루프가 이미 없어도 아무 일도 일어나지 않아야 한다
    EXPECT_FALSE(timer.active());
}

TEST_F(EventLoopTest, StopSignalEndsRun) {
    stop_on_signals(&loop_);
    const PeriodicTimer timer = loop_.add_periodic(milliseconds{50}, [] { std::raise(SIGTERM); });

    loop_.run();

    EXPECT_TRUE(loop_.stopped());
    stop_on_signals(nullptr);
}

}  // namespace
}  // namespace argus::core
