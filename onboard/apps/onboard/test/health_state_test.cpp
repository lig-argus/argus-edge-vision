#include "argus/onboard/health_state.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"

namespace argus::onboard {
namespace {

TEST(HealthStateTest, StartsWithAllBitsCleared) {
    const HealthState health;
    EXPECT_EQ(health.value(), 0);
    EXPECT_FALSE(health.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_FALSE(health.is_set(ARGUS_HEALTH_GCS_LINK));
}

TEST(HealthStateTest, SetsAndClearsEachBitIndependently) {
    HealthState health;
    health.set(ARGUS_HEALTH_FC_CONNECTED, true);
    health.set(ARGUS_HEALTH_GCS_LINK, true);
    EXPECT_EQ(health.value(), ARGUS_HEALTH_FC_CONNECTED | ARGUS_HEALTH_GCS_LINK);

    health.set(ARGUS_HEALTH_FC_CONNECTED, false);
    EXPECT_FALSE(health.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_TRUE(health.is_set(ARGUS_HEALTH_GCS_LINK));
    EXPECT_EQ(health.value(), ARGUS_HEALTH_GCS_LINK);
}

TEST(HealthStateTest, NotifiesOnlyWhenValueChanges) {
    HealthState health;
    std::vector<std::uint8_t> values;
    const core::Subscription subscription =
        health.subscribe_changes([&] { values.push_back(health.value()); });

    health.set(ARGUS_HEALTH_FC_CONNECTED, false);  // 이미 0
    health.set(ARGUS_HEALTH_FC_CONNECTED, true);
    health.set(ARGUS_HEALTH_FC_CONNECTED, true);  // 같은 값
    health.set(ARGUS_HEALTH_GCS_LINK, true);
    health.set(ARGUS_HEALTH_FC_CONNECTED, false);

    EXPECT_EQ(values, (std::vector<std::uint8_t>{1, 3, 2}));
}

TEST(HealthStateTest, DoesNotNotifyAfterSubscriptionIsReleased) {
    HealthState health;
    int count = 0;
    core::Subscription subscription = health.subscribe_changes([&count] { ++count; });
    subscription.reset();

    health.set(ARGUS_HEALTH_FC_CONNECTED, true);

    EXPECT_EQ(count, 0);
}

TEST(HealthStateTest, LaterHandlerReadsCurrentValueWhenEarlierHandlerChangesIt) {
    HealthState health;
    std::vector<std::uint8_t> seen_by_later;
    // 먼저 등록된 handler 가 FC_CONNECTED 가 설정된 경우 GCS_LINK 를 함께 설정한다
    const core::Subscription earlier = health.subscribe_changes([&health] {
        if (health.is_set(ARGUS_HEALTH_FC_CONNECTED)) {
            health.set(ARGUS_HEALTH_GCS_LINK, true);
        }
    });
    const core::Subscription later =
        health.subscribe_changes([&] { seen_by_later.push_back(health.value()); });

    health.set(ARGUS_HEALTH_FC_CONNECTED, true);

    ASSERT_FALSE(seen_by_later.empty());
    EXPECT_EQ(seen_by_later.back(), health.value());
    EXPECT_EQ(health.value(), ARGUS_HEALTH_FC_CONNECTED | ARGUS_HEALTH_GCS_LINK);
}

}  // namespace
}  // namespace argus::onboard
