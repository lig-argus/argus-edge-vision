#include "argus/onboard/fc_link_tracker.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/health_state.hpp"

namespace argus::onboard {
namespace {

constexpr std::int64_t kStartNs = 5'000'000'000;

class FcLinkTrackerTest : public ::testing::Test {
protected:
    void SetUp() override {
        subscription_ = tracker_.subscribe_epoch_changes(
            [this](std::uint64_t epoch) { epochs_.push_back(epoch); });
    }

    core_testing::FakeClock clock_{kStartNs};
    HealthState health_;
    FcLinkTracker tracker_{clock_, health_};
    std::vector<std::uint64_t> epochs_;
    core::Subscription subscription_;
};

TEST_F(FcLinkTrackerTest, StartsDisconnectedWithEpochZero) {
    tracker_.check_timeout();

    EXPECT_FALSE(tracker_.connected());
    EXPECT_EQ(tracker_.fc_epoch(), 0U);
    EXPECT_FALSE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_TRUE(epochs_.empty());
}

TEST_F(FcLinkTrackerTest, FirstMessageConnectsAndStartsEpochOne) {
    EXPECT_EQ(tracker_.on_fc_message(), kStartNs);

    EXPECT_TRUE(tracker_.connected());
    EXPECT_EQ(tracker_.fc_epoch(), 1U);
    EXPECT_TRUE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_EQ(epochs_, std::vector<std::uint64_t>{1});
}

TEST_F(FcLinkTrackerTest, ReturnsReceiveTimeOfEachMessage) {
    tracker_.on_fc_message();
    clock_.advance_ns(123);
    EXPECT_EQ(tracker_.on_fc_message(FcTimedMessage::Attitude, 1), kStartNs + 123);
}

TEST_F(FcLinkTrackerTest, EpochStaysWhileMessagesArriveWithinTimeout) {
    tracker_.on_fc_message();
    clock_.advance_ns(kFcLinkTimeoutNs);
    tracker_.on_fc_message();

    EXPECT_EQ(tracker_.fc_epoch(), 1U);
    EXPECT_EQ(epochs_.size(), 1U);
}

TEST_F(FcLinkTrackerTest, CheckTimeoutDetectsLossOnlyAfterTimeout) {
    tracker_.on_fc_message();

    clock_.advance_ns(kFcLinkTimeoutNs);
    tracker_.check_timeout();
    EXPECT_TRUE(tracker_.connected());

    clock_.advance_ns(1);
    tracker_.check_timeout();
    EXPECT_FALSE(tracker_.connected());
    EXPECT_FALSE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_EQ(tracker_.fc_epoch(), 1U);  // 두절만으로는 증가하지 않음
}

TEST_F(FcLinkTrackerTest, ConnectedReflectsTimeoutBeforeCheckTimeoutIsCalled) {
    tracker_.on_fc_message();
    clock_.advance_ns(kFcLinkTimeoutNs + 1);

    EXPECT_FALSE(tracker_.connected());
    // HealthState 는 check_timeout() 이 호출된 시점에 갱신된다
    EXPECT_TRUE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    tracker_.check_timeout();
    EXPECT_FALSE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
}

TEST_F(FcLinkTrackerTest, MessageAfterLossStartsNewEpoch) {
    tracker_.on_fc_message();
    clock_.advance_ns(kFcLinkTimeoutNs + 1);
    tracker_.check_timeout();

    tracker_.on_fc_message();

    EXPECT_EQ(tracker_.fc_epoch(), 2U);
    EXPECT_TRUE(tracker_.connected());
    EXPECT_TRUE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    EXPECT_EQ(epochs_, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(FcLinkTrackerTest, MessageAfterTimeoutStartsNewEpochWithoutCheckTimeout) {
    tracker_.on_fc_message();
    clock_.advance_ns(kFcLinkTimeoutNs + 1);

    tracker_.on_fc_message();

    EXPECT_EQ(tracker_.fc_epoch(), 2U);
}

TEST_F(FcLinkTrackerTest, SmallerFcTimeOfSameKindStartsNewEpoch) {
    tracker_.on_fc_message(FcTimedMessage::Attitude, 50'000);

    tracker_.on_fc_message(FcTimedMessage::Attitude, 300);

    EXPECT_EQ(tracker_.fc_epoch(), 2U);
    EXPECT_EQ(epochs_, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(FcLinkTrackerTest, EqualFcTimeDoesNotStartNewEpoch) {
    tracker_.on_fc_message(FcTimedMessage::Attitude, 1000);
    tracker_.on_fc_message(FcTimedMessage::Attitude, 1000);

    EXPECT_EQ(tracker_.fc_epoch(), 1U);
}

TEST_F(FcLinkTrackerTest, FcTimeIsNotComparedAcrossKinds) {
    tracker_.on_fc_message(FcTimedMessage::Attitude, 50'000);
    tracker_.on_fc_message(FcTimedMessage::LocalPosition, 49'990);

    EXPECT_EQ(tracker_.fc_epoch(), 1U);
}

TEST_F(FcLinkTrackerTest, RebootIsCountedOnceForAllKinds) {
    tracker_.on_fc_message(FcTimedMessage::Attitude, 50'000);
    tracker_.on_fc_message(FcTimedMessage::LocalPosition, 50'000);
    tracker_.on_fc_message(FcTimedMessage::Attitude, 300);

    tracker_.on_fc_message(FcTimedMessage::LocalPosition, 310);

    EXPECT_EQ(tracker_.fc_epoch(), 2U);
}

TEST_F(FcLinkTrackerTest, RebootLongerThanTimeoutStartsOneNewEpoch) {
    tracker_.on_fc_message(FcTimedMessage::Attitude, 50'000);
    clock_.advance_ns(kFcLinkTimeoutNs + 1);

    tracker_.on_fc_message(FcTimedMessage::Attitude, 300);

    EXPECT_EQ(tracker_.fc_epoch(), 2U);
    EXPECT_EQ(epochs_, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(FcLinkTrackerTest, EpochHandlerSeesNewEpochConnectedStateAndHealth) {
    std::uint64_t seen_epoch = 0;
    bool seen_connected = false;
    bool seen_health = false;
    const core::Subscription subscription = tracker_.subscribe_epoch_changes([&](std::uint64_t) {
        seen_epoch = tracker_.fc_epoch();
        seen_connected = tracker_.connected();
        seen_health = health_.is_set(ARGUS_HEALTH_FC_CONNECTED);
    });

    tracker_.on_fc_message();

    EXPECT_EQ(seen_epoch, 1U);
    EXPECT_TRUE(seen_connected);
    EXPECT_TRUE(seen_health);
}

TEST_F(FcLinkTrackerTest, DoesNotCallEpochHandlerAfterSubscriptionIsReleased) {
    subscription_.reset();
    tracker_.on_fc_message();
    EXPECT_TRUE(epochs_.empty());
}

}  // namespace
}  // namespace argus::onboard
