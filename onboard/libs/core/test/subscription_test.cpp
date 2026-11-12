#include "argus/core/subscription.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>

namespace argus::core {
namespace {

TEST(SubscriptionTest, ReleasesWhenDestroyed) {
    std::int32_t released = 0;
    {
        const Subscription subscription([&released] { ++released; });
        EXPECT_EQ(released, 0);
    }
    EXPECT_EQ(released, 1);
}

TEST(SubscriptionTest, DefaultIsEmpty) {
    const Subscription subscription;
    EXPECT_FALSE(subscription.active());
}

TEST(SubscriptionTest, MoveTransfersRegistration) {
    std::int32_t released = 0;
    Subscription first([&released] { ++released; });
    Subscription second = std::move(first);

    EXPECT_TRUE(second.active());
    second.reset();
    EXPECT_EQ(released, 1);  // 해제는 한 번만 (first 는 빈 handle 이 됨)
}

TEST(SubscriptionTest, MoveAssignReleasesPreviousRegistration) {
    std::int32_t first_released = 0;
    std::int32_t second_released = 0;
    Subscription target([&first_released] { ++first_released; });

    target = Subscription([&second_released] { ++second_released; });

    EXPECT_EQ(first_released, 1);
    EXPECT_EQ(second_released, 0);
}

TEST(SubscriptionTest, ResetReleasesOnlyOnce) {
    std::int32_t released = 0;
    Subscription subscription([&released] { ++released; });

    subscription.reset();
    subscription.reset();

    EXPECT_EQ(released, 1);
}

}  // namespace
}  // namespace argus::core
