#include "argus/core_testing/fake_poller.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "argus/core_testing/fake_clock.hpp"

namespace argus::core_testing {
namespace {

using std::chrono::milliseconds;

constexpr std::int64_t kMs = 1'000'000;

TEST(FakePollerTest, AdvancesClockByWaitWhenNothingArrives) {
    FakeClock clock;
    FakePoller poller(clock);

    poller.process_events(milliseconds{30}, milliseconds{5});

    EXPECT_EQ(clock.now_monotonic_ns(), 30 * kMs);
}

TEST(FakePollerTest, WakesAtArrivalAndRunsEvent) {
    FakeClock clock;
    FakePoller poller(clock);
    int handled = 0;
    poller.post_at(10 * kMs, [&handled] { ++handled; });

    poller.process_events(milliseconds{30}, milliseconds{5});

    EXPECT_EQ(handled, 1);
    EXPECT_EQ(clock.now_monotonic_ns(), 10 * kMs);
}

TEST(FakePollerTest, RunsNothingOutsideProcessEvents) {
    FakeClock clock;
    FakePoller poller(clock);
    int handled = 0;

    poller.post([&handled] { ++handled; });

    EXPECT_EQ(handled, 0);
    poller.process_events(milliseconds{0}, milliseconds{5});
    EXPECT_EQ(handled, 1);
}

TEST(FakePollerTest, KeepsLaterEventsForNextCall) {
    FakeClock clock;
    FakePoller poller(clock);
    int handled = 0;
    poller.post_at(50 * kMs, [&handled] { ++handled; });

    poller.process_events(milliseconds{30}, milliseconds{5});

    EXPECT_EQ(handled, 0);
    EXPECT_EQ(clock.now_monotonic_ns(), 30 * kMs);
}

}  // namespace
}  // namespace argus::core_testing
