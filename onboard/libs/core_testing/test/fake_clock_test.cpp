#include "argus/core_testing/fake_clock.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace argus::core_testing {
namespace {

constexpr std::int64_t kMs = 1'000'000;

TEST(FakeClockTest, StartsAtGivenTime) {
    const FakeClock clock(5 * kMs);
    EXPECT_EQ(clock.now_monotonic_ns(), 5 * kMs);
}

TEST(FakeClockTest, AdvancesOnlyWhenTold) {
    FakeClock clock;
    EXPECT_EQ(clock.now_monotonic_ns(), 0);

    clock.advance_ns(3 * kMs);
    EXPECT_EQ(clock.now_monotonic_ns(), 3 * kMs);
}

TEST(FakeClockTest, NeverGoesBackward) {
    FakeClock clock(10 * kMs);

    clock.advance_ns(-5 * kMs);

    EXPECT_EQ(clock.now_monotonic_ns(), 10 * kMs);
}

}  // namespace
}  // namespace argus::core_testing
