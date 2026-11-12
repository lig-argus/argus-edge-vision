#include "argus/onboard/mavlink_handler_registry.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"

namespace argus::onboard {
namespace {

mavlink_message_t heartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t attitude() {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(1, 1, &message, 0, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    return message;
}

TEST(MavlinkHandlerRegistryTest, CallsOnlyHandlersOfMessageMsgid) {
    MavlinkHandlerRegistry registry;
    int heartbeat_count = 0;
    int attitude_count = 0;
    const core::Subscription first =
        registry.add(MAVLINK_MSG_ID_HEARTBEAT,
                     [&heartbeat_count](const mavlink_message_t&) { ++heartbeat_count; });
    const core::Subscription second = registry.add(
        MAVLINK_MSG_ID_ATTITUDE, [&attitude_count](const mavlink_message_t&) { ++attitude_count; });

    registry.dispatch(heartbeat());

    EXPECT_EQ(heartbeat_count, 1);
    EXPECT_EQ(attitude_count, 0);
}

TEST(MavlinkHandlerRegistryTest, DispatchWithoutHandlersDoesNothing) {
    const MavlinkHandlerRegistry registry;
    registry.dispatch(attitude());
    SUCCEED();
}

TEST(MavlinkHandlerRegistryTest, CallsHandlersInRegistrationOrder) {
    MavlinkHandlerRegistry registry;
    std::vector<int> order;
    const core::Subscription first = registry.add(
        MAVLINK_MSG_ID_HEARTBEAT, [&order](const mavlink_message_t&) { order.push_back(1); });
    const core::Subscription second = registry.add(
        MAVLINK_MSG_ID_HEARTBEAT, [&order](const mavlink_message_t&) { order.push_back(2); });

    registry.dispatch(heartbeat());

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST(MavlinkHandlerRegistryTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    MavlinkHandlerRegistry registry;
    int count = 0;
    core::Subscription subscription =
        registry.add(MAVLINK_MSG_ID_HEARTBEAT, [&count](const mavlink_message_t&) { ++count; });
    subscription.reset();

    registry.dispatch(heartbeat());

    EXPECT_EQ(count, 0);
}

TEST(MavlinkHandlerRegistryTest, DoesNotCallHandlerReleasedByEarlierHandler) {
    MavlinkHandlerRegistry registry;
    int second_count = 0;
    core::Subscription second;
    const core::Subscription first = registry.add(
        MAVLINK_MSG_ID_HEARTBEAT, [&second](const mavlink_message_t&) { second.reset(); });
    second = registry.add(MAVLINK_MSG_ID_HEARTBEAT,
                          [&second_count](const mavlink_message_t&) { ++second_count; });

    registry.dispatch(heartbeat());

    EXPECT_EQ(second_count, 0);
}

TEST(MavlinkHandlerRegistryTest, HandlerMayReleaseItsOwnSubscription) {
    MavlinkHandlerRegistry registry;
    int count = 0;
    core::Subscription subscription;
    subscription = registry.add(MAVLINK_MSG_ID_HEARTBEAT, [&](const mavlink_message_t&) {
        ++count;
        subscription.reset();
    });

    registry.dispatch(heartbeat());
    registry.dispatch(heartbeat());

    EXPECT_EQ(count, 1);
}

TEST(MavlinkHandlerRegistryTest, HandlerAddedDuringDispatchIsCalledFromNextMessage) {
    MavlinkHandlerRegistry registry;
    int added_count = 0;
    core::Subscription added;
    const core::Subscription first =
        registry.add(MAVLINK_MSG_ID_HEARTBEAT, [&](const mavlink_message_t&) {
            if (!added.active()) {
                added = registry.add(MAVLINK_MSG_ID_HEARTBEAT,
                                     [&added_count](const mavlink_message_t&) { ++added_count; });
            }
        });

    registry.dispatch(heartbeat());
    EXPECT_EQ(added_count, 0);

    registry.dispatch(heartbeat());
    EXPECT_EQ(added_count, 1);
}

TEST(MavlinkHandlerRegistryTest, SubscriptionReleasedAfterRegistryIsDestroyedIsSafe) {
    auto registry = std::make_unique<MavlinkHandlerRegistry>();
    core::Subscription subscription =
        registry->add(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t&) {});
    registry.reset();
    subscription.reset();
    SUCCEED();
}

TEST(MavlinkHandlerRegistryTest, HandlerMayDestroyRegistryDuringDispatch) {
    auto registry = std::make_unique<MavlinkHandlerRegistry>();
    int second_count = 0;
    const core::Subscription first = registry->add(
        MAVLINK_MSG_ID_HEARTBEAT, [&registry](const mavlink_message_t&) { registry.reset(); });
    const core::Subscription second = registry->add(
        MAVLINK_MSG_ID_HEARTBEAT, [&second_count](const mavlink_message_t&) { ++second_count; });

    registry->dispatch(heartbeat());

    EXPECT_EQ(registry, nullptr);
    EXPECT_EQ(second_count, 1);  // 목록은 dispatch 가 끝날 때까지 유지되며 등록도 해제되지 않았음
}

}  // namespace
}  // namespace argus::onboard
