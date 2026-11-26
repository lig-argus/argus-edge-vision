#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/bus_tracking_status_subscriber.hpp"
#include "argus/perception_api_bus/message_types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {
namespace {

using perception_api::ReceivedTrackingStatus;
using perception_api::TrackingStatus;

constexpr std::int64_t kMs = 1'000'000;

TrackingStatus status_of(const std::string& source, std::uint32_t generation) {
    TrackingStatus status;
    status.source = source;
    status.generation = generation;
    return status;
}

class TrackingStatusSubscriberTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 추적기의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
        publisher_ = bus::make_publisher<TrackingStatus>(bus_, protocol::kTrackingStatus,
                                                         kTrackingStatusType,
                                                         protocol::kTrackingStatusQueueDepth);
        subscriber_ = BusTrackingStatusSubscriber::create(bus_, clock_);
        ASSERT_NE(publisher_, nullptr);
        ASSERT_NE(subscriber_, nullptr);
    }

    // 시각을 at_ns 로 옮기고 status 를 발행해 구독자에게 전달한다
    void deliver_at(std::int64_t at_ns, const TrackingStatus& status) {
        clock_.advance_ns(at_ns - clock_.now_monotonic_ns());
        ASSERT_TRUE(publisher_->publish(status));
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<bus::Publisher<TrackingStatus>> publisher_;
    std::unique_ptr<BusTrackingStatusSubscriber> subscriber_;
};

TEST_F(TrackingStatusSubscriberTest, LatestIsEmptyUntilThatCameraReports) {
    deliver_at(10 * kMs, status_of("ir", 1));

    EXPECT_FALSE(subscriber_->latest("eo").has_value());
}

TEST_F(TrackingStatusSubscriberTest, KeepsLatestStatusPerCameraWithArrivalTime) {
    TrackingStatus first = status_of("ir", 1);
    first.published_monotonic_ns = 1;  // 발행자가 채운 값은 받은 시각으로 사용하지 않는다
    deliver_at(10 * kMs, first);
    deliver_at(20 * kMs, status_of("eo", 5));
    deliver_at(30 * kMs, status_of("ir", 2));

    const ReceivedTrackingStatus ir = subscriber_->latest("ir").value_or(ReceivedTrackingStatus{});
    const ReceivedTrackingStatus eo = subscriber_->latest("eo").value_or(ReceivedTrackingStatus{});
    EXPECT_EQ(ir.status.generation, 2U);
    EXPECT_EQ(ir.arrived_monotonic_ns, 30 * kMs);
    EXPECT_EQ(eo.status.generation, 5U);
    EXPECT_EQ(eo.arrived_monotonic_ns, 20 * kMs);
}

TEST_F(TrackingStatusSubscriberTest, EveryHandlerIsCalledForEachStatusUntilUnsubscribed) {
    std::vector<std::string> first;
    std::vector<std::int64_t> second;
    const core::Subscription first_subscription =
        subscriber_->subscribe([&first](const ReceivedTrackingStatus& received) {
            first.push_back(received.status.source);
        });
    core::Subscription second_subscription =
        subscriber_->subscribe([&second](const ReceivedTrackingStatus& received) {
            second.push_back(received.arrived_monotonic_ns);
        });

    deliver_at(10 * kMs, status_of("ir", 1));
    second_subscription.reset();  // 등록을 풀면 더 받지 않는다
    deliver_at(20 * kMs, status_of("eo", 1));

    EXPECT_EQ(first, (std::vector<std::string>{"ir", "eo"}));
    EXPECT_EQ(second, std::vector<std::int64_t>{10 * kMs});
}

TEST_F(TrackingStatusSubscriberTest, HandlerSeesUpdatedLatest) {
    std::optional<std::uint32_t> seen;
    const core::Subscription subscription =
        subscriber_->subscribe([this, &seen](const ReceivedTrackingStatus& received) {
            seen = subscriber_->latest(received.status.source)
                       .value_or(ReceivedTrackingStatus{})
                       .status.generation;
        });

    deliver_at(10 * kMs, status_of("ir", 7));

    EXPECT_EQ(seen, std::optional<std::uint32_t>{7});
}

TEST_F(TrackingStatusSubscriberTest, EmptyHandlerIsNotRegistered) {
    const core::Subscription subscription = subscriber_->subscribe(nullptr);

    deliver_at(10 * kMs, status_of("ir", 1));  // 빈 handler 를 부르지 않는다

    EXPECT_TRUE(subscriber_->latest("ir").has_value());
}

TEST_F(TrackingStatusSubscriberTest, SubscriptionOutlivingTheSubscriberIsSafeToRelease) {
    core::Subscription subscription =
        subscriber_->subscribe([](const ReceivedTrackingStatus& /*received*/) {});

    subscriber_.reset();
    deliver_at(10 * kMs, status_of("ir", 1));  // 구독이 해제되어 아무도 받지 않는다
    subscription.reset();
}

}  // namespace
}  // namespace argus::perception_api_bus
