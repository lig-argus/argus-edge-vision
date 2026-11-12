#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "argus/bus/bus.hpp"
#include "argus/bus/message_type.hpp"
#include "argus/bus/typed.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::VelocitySetpoint;

constexpr std::int32_t kDepth = 4;

VelocitySetpoint sample_setpoint() {
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.right_m_s = -0.25;
    setpoint.down_m_s = 0.125;
    setpoint.yaw_rate_rad_s = 0.1745;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;
    return setpoint;
}

// 발행자와 구독자를 만들지 못하는 bus (생성 실패 테스트용)
class FailingBus : public bus::IBus {
public:
    std::unique_ptr<bus::ITopicPublisher> create_publisher(std::string_view /*topic*/,
                                                           const bus::MessageType& /*type*/,
                                                           std::int32_t /*queue_depth*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(
        std::string_view /*topic*/, const bus::MessageType& /*type*/,
        std::int32_t /*queue_depth*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::IRequester> create_requester(std::string_view /*name*/,
                                                      const bus::MessageType& /*request*/,
                                                      const bus::MessageType& /*reply*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::IReplier> create_replier(std::string_view /*name*/,
                                                  const bus::MessageType& /*request*/,
                                                  const bus::MessageType& /*reply*/) override {
        return nullptr;
    }
};

class VelocitySetpointTest : public ::testing::Test {
protected:
    void process() { poller_.process_events(std::chrono::milliseconds{0}, kBudget); }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
};

TEST_F(VelocitySetpointTest, SubscriberReceivesTheValuesThePublisherSent) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<VelocitySetpoint> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint& s) { received.push_back(s); });

    publisher->send(sample_setpoint());
    process();

    ASSERT_EQ(received.size(), 1U);
    EXPECT_DOUBLE_EQ(received[0].forward_m_s, 1.5);
    EXPECT_DOUBLE_EQ(received[0].right_m_s, -0.25);
    EXPECT_DOUBLE_EQ(received[0].down_m_s, 0.125);
    EXPECT_DOUBLE_EQ(received[0].yaw_rate_rad_s, 0.1745);
    EXPECT_EQ(received[0].valid_until_monotonic_ns, 123'456'789'000);
}

TEST_F(VelocitySetpointTest, PublishesOnControlSetpointTopic) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    ASSERT_NE(publisher, nullptr);

    publisher->send(sample_setpoint());

    EXPECT_EQ(bus_.published_count(), 1U);
    const std::vector<VelocitySetpoint> published =
        bus_.published<VelocitySetpoint>(protocol::kControlSetpoint);
    ASSERT_EQ(published.size(), 1U);
    EXPECT_DOUBLE_EQ(published[0].forward_m_s, 1.5);
}

TEST_F(VelocitySetpointTest, SequenceIncreasesByOneForEachSend) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    ASSERT_NE(publisher, nullptr);
    const auto raw_subscriber = bus::make_subscriber<VelocitySetpoint>(
        bus_, protocol::kControlSetpoint, kControlSetpointType, kDepth);
    ASSERT_NE(raw_subscriber, nullptr);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription =
        raw_subscriber->subscribe([&](const VelocitySetpoint&, const bus::MessageInfo& info) {
            sequences.push_back(info.sequence);
        });

    publisher->send(sample_setpoint());
    process();
    publisher->send(sample_setpoint());
    process();

    EXPECT_EQ(sequences, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(VelocitySetpointTest, CountsMessagesLostBetweenReceivedSequences) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    publisher->send(sample_setpoint());  // 번호 1
    process();
    EXPECT_EQ(subscriber->lost_count(), 0U);

    // 번호 2, 3 이 도착하지 않고 4 가 도착한 경우
    bus_.simulate_message_received(protocol::kControlSetpoint, sample_setpoint(),
                                   bus::MessageInfo{4});
    process();

    EXPECT_EQ(subscriber->lost_count(), 2U);
}

TEST_F(VelocitySetpointTest, CountsFailedSendAsLoss) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    publisher->send(sample_setpoint());  // 번호 1
    bus_.simulate_publish_failure(true);
    publisher->send(sample_setpoint());  // 번호 2. 보내지 못함
    bus_.simulate_publish_failure(false);
    publisher->send(sample_setpoint());  // 번호 3
    process();

    EXPECT_EQ(subscriber->lost_count(), 1U);
}

TEST_F(VelocitySetpointTest, DoesNotCountPublisherRestartAsLoss) {
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(subscriber, nullptr);

    bus_.simulate_message_received(protocol::kControlSetpoint, sample_setpoint(),
                                   bus::MessageInfo{50});
    bus_.simulate_message_received(protocol::kControlSetpoint, sample_setpoint(),
                                   bus::MessageInfo{1});  // 발행자가 다시 시작
    process();

    EXPECT_EQ(subscriber->lost_count(), 0U);
}

TEST_F(VelocitySetpointTest, HandlerIsNotCalledAfterSubscriptionIsReleased) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::int32_t received = 0;
    core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint&) { ++received; });

    subscription.reset();
    publisher->send(sample_setpoint());
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(VelocitySetpointTest, SubscribeRejectsEmptyHandler) {
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(subscriber, nullptr);

    const core::Subscription subscription = subscriber->subscribe(nullptr);

    EXPECT_FALSE(subscription.active());
}

TEST_F(VelocitySetpointTest, CreateReturnsNullWhenBusCannotCreateTheEntity) {
    FailingBus failing_bus;

    EXPECT_EQ(BusVelocitySetpointPublisher::create(failing_bus), nullptr);
    EXPECT_EQ(BusVelocitySetpointSubscriber::create(failing_bus), nullptr);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
