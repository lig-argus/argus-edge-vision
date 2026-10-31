#include "argus/core/steady_clock.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace argus::core {
namespace {

TEST(SteadyClockTest, NeverGoesBackward) {
    const SteadyClock clock;
    std::int64_t previous = clock.now_monotonic_ns();
    for (int i = 0; i < 1000; ++i) {
        const std::int64_t now = clock.now_monotonic_ns();
        EXPECT_GE(now, previous);
        previous = now;
    }
}

TEST(SteadyClockTest, ReturnsPositiveValue) {
    const SteadyClock clock;
    EXPECT_GT(clock.now_monotonic_ns(), 0);
}

}  // namespace
}  // namespace argus::core
