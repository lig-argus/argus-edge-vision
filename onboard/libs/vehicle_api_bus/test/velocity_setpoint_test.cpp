#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "argus/bus/bus.hpp"
#include "argus/bus/message.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"
#include "argus_msg/control_setpoint.pb.h"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::VelocitySetpoint;

VelocitySetpoint sample_setpoint() {
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.right_m_s = -0.25;
    setpoint.down_m_s = 0.125;
    setpoint.yaw_rate_rad_s = 0.1745;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;
    return setpoint;
}

// seq 만 지정한 control/setpoint 본문
bus::Bytes body_with_seq(std::uint64_t seq) {
    msg::ControlSetpoint message;
    message.set_seq(seq);
    bus::Bytes body(message.ByteSizeLong());
    EXPECT_TRUE(message.SerializeToArray(body.data(), static_cast<int>(body.size())));
    return body;
}

// 발행자와 구독자를 만들지 못하는 bus (생성 실패 테스트용)
class FailingBus : public bus::IBus {
public:
    std::unique_ptr<bus::ITopicPublisher> create_publisher(std::string_view /*topic*/,
                                                           int /*queue_depth*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(std::string_view /*topic*/,
                                                             int /*queue_depth*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::IRequester> create_requester(std::string_view /*name*/) override {
        return nullptr;
    }
    std::unique_ptr<bus::IReplier> create_replier(std::string_view /*name*/) override {
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

TEST_F(VelocitySetpointTest, PublishesOnControlSetpointTopicWithoutBinary) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    ASSERT_NE(publisher, nullptr);

    publisher->send(sample_setpoint());

    ASSERT_EQ(bus_.published().size(), 1U);
    EXPECT_EQ(bus_.published()[0].topic, protocol::kControlSetpoint);
    EXPECT_TRUE(bus_.published()[0].binary.empty());
}

TEST_F(VelocitySetpointTest, SeqIncreasesByOneForEachSend) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    ASSERT_NE(publisher, nullptr);

    publisher->send(sample_setpoint());
    process();
    publisher->send(sample_setpoint());
    process();

    ASSERT_EQ(bus_.published().size(), 2U);
    msg::ControlSetpoint first;
    msg::ControlSetpoint second;
    const bus::Bytes& first_body = bus_.published()[0].body;
    const bus::Bytes& second_body = bus_.published()[1].body;
    ASSERT_TRUE(first.ParseFromArray(first_body.data(), static_cast<int>(first_body.size())));
    ASSERT_TRUE(second.ParseFromArray(second_body.data(), static_cast<int>(second_body.size())));
    EXPECT_EQ(first.seq(), 1U);
    EXPECT_EQ(second.seq(), 2U);
}

TEST_F(VelocitySetpointTest, SubscriberDropsMessageThatCannotBeParsed) {
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(subscriber, nullptr);
    int received = 0;
    const core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint&) { ++received; });

    // 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없음
    bus_.simulate_message_received({std::string(protocol::kControlSetpoint), {0x0A, 0x05}, {}});
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(VelocitySetpointTest, SubscriberDropsMessageWithBinary) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    int received = 0;
    const core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint&) { ++received; });
    publisher->send(sample_setpoint());
    const bus::Bytes valid_body = bus_.published()[0].body;
    process();
    ASSERT_EQ(received, 1);

    bus_.simulate_message_received({std::string(protocol::kControlSetpoint), valid_body, {1}});
    process();

    EXPECT_EQ(received, 1);
}

TEST_F(VelocitySetpointTest, CountsMessagesLostBetweenReceivedSeqs) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    publisher->send(sample_setpoint());  // seq 1
    process();
    EXPECT_EQ(subscriber->lost_count(), 0U);

    // seq 2, 3 이 도착하지 않고 seq 4 가 도착한 경우
    bus_.simulate_message_received({std::string(protocol::kControlSetpoint), body_with_seq(4), {}});
    process();

    EXPECT_EQ(subscriber->lost_count(), 2U);
}

TEST_F(VelocitySetpointTest, DoesNotCountPublisherRestartAsLoss) {
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(subscriber, nullptr);

    bus_.simulate_message_received(
        {std::string(protocol::kControlSetpoint), body_with_seq(50), {}});
    bus_.simulate_message_received(
        {std::string(protocol::kControlSetpoint), body_with_seq(1), {}});  // 발행자가 다시 시작
    process();

    EXPECT_EQ(subscriber->lost_count(), 0U);
}

TEST_F(VelocitySetpointTest, HandlerIsNotCalledAfterSubscriptionIsReleased) {
    const auto publisher = BusVelocitySetpointPublisher::create(bus_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    int received = 0;
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
