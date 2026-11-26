#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_follow_status_subscriber.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::FollowStatus;
using vehicle_api::MissionState;
using vehicle_api::ReceivedFollowStatus;

constexpr std::int64_t kMs = 1'000'000;

FollowStatus status_in(MissionState state) {
    FollowStatus status;
    status.mission_state = state;
    return status;
}

class FollowStatusTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 추종 제어의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
        publisher_ = bus::make_publisher<FollowStatus>(
            bus_, protocol::kFollowStatus, kFollowStatusType, protocol::kFollowStatusQueueDepth);
        subscriber_ = BusFollowStatusSubscriber::create(bus_, clock_);
        ASSERT_NE(publisher_, nullptr);
        ASSERT_NE(subscriber_, nullptr);
    }

    // 시각을 at_ns 로 옮기고 status 를 발행해 구독자에게 전달한다
    void deliver_at(std::int64_t at_ns, const FollowStatus& status) {
        clock_.advance_ns(at_ns - clock_.now_monotonic_ns());
        ASSERT_TRUE(publisher_->publish(status));
        poller_.process_events(std::chrono::milliseconds{0}, kBudget);
    }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<bus::Publisher<FollowStatus>> publisher_;
    std::unique_ptr<BusFollowStatusSubscriber> subscriber_;
};

TEST_F(FollowStatusTest, LatestIsEmptyUntilFirstStatus) {
    EXPECT_FALSE(subscriber_->latest().has_value());
}

TEST_F(FollowStatusTest, LatestKeepsLastStatusWithArrivalTimeFromClock) {
    FollowStatus first = status_in(MissionState::Preparing);
    first.received_monotonic_ns = 1;  // 발행자가 채운 값은 받은 시각으로 사용하지 않는다
    deliver_at(100 * kMs, first);
    deliver_at(250 * kMs, status_in(MissionState::FollowActive));

    ASSERT_TRUE(subscriber_->latest().has_value());
    const ReceivedFollowStatus latest = subscriber_->latest().value_or(ReceivedFollowStatus{});
    EXPECT_EQ(latest.status.mission_state, MissionState::FollowActive);
    EXPECT_EQ(latest.arrived_monotonic_ns, 250 * kMs);
}

TEST_F(FollowStatusTest, EveryHandlerIsCalledForEachStatusUntilUnsubscribed) {
    std::vector<MissionState> first;
    std::vector<std::int64_t> second;
    const core::Subscription first_subscription =
        subscriber_->subscribe([&first](const ReceivedFollowStatus& received) {
            first.push_back(received.status.mission_state);
        });
    core::Subscription second_subscription =
        subscriber_->subscribe([&second](const ReceivedFollowStatus& received) {
            second.push_back(received.arrived_monotonic_ns);
        });

    deliver_at(10 * kMs, status_in(MissionState::ScanActive));
    second_subscription.reset();  // 등록을 풀면 더 받지 않는다
    deliver_at(20 * kMs, status_in(MissionState::Idle));

    EXPECT_EQ(first, (std::vector<MissionState>{MissionState::ScanActive, MissionState::Idle}));
    EXPECT_EQ(second, std::vector<std::int64_t>{10 * kMs});
}

TEST_F(FollowStatusTest, HandlerMayUnsubscribeItselfWhileBeingCalled) {
    std::int32_t calls = 0;
    core::Subscription subscription;
    subscription = subscriber_->subscribe([&](const ReceivedFollowStatus& /*received*/) {
        ++calls;
        subscription.reset();
    });

    deliver_at(10 * kMs, status_in(MissionState::Idle));
    deliver_at(20 * kMs, status_in(MissionState::Idle));

    EXPECT_EQ(calls, 1);
}

TEST_F(FollowStatusTest, EmptyHandlerIsNotRegistered) {
    const core::Subscription subscription = subscriber_->subscribe(nullptr);

    deliver_at(10 * kMs, status_in(MissionState::Idle));  // 빈 handler 를 부르지 않는다

    EXPECT_TRUE(subscriber_->latest().has_value());
}

TEST_F(FollowStatusTest, SubscriptionOutlivingTheSubscriberIsSafeToRelease) {
    core::Subscription subscription =
        subscriber_->subscribe([](const ReceivedFollowStatus& /*received*/) {});

    subscriber_.reset();
    deliver_at(10 * kMs, status_in(MissionState::Idle));  // 구독이 해제되어 아무도 받지 않는다
    subscription.reset();
}

}  // namespace
}  // namespace argus::vehicle_api_bus
