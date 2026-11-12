#include "argus/bus_testing/fake_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "argus/bus/message.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::bus_testing {
namespace {

using bus::BusMessage;
using bus::Bytes;
using std::chrono::milliseconds;

constexpr std::int64_t kMs = 1'000'000;
constexpr int kDepth = 4;

BusMessage message(const std::string& topic, std::uint8_t value) { return {topic, {value}, {}}; }

class FakeBusTest : public ::testing::Test {
protected:
    void process(milliseconds wait = milliseconds{0}) {
        poller_.process_events(wait, milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeBus bus_{poller_, clock_};
};

TEST_F(FakeBusTest, DeliversPublishedMessageOnlyInsideProcessEvents) {
    const auto publisher = bus_.create_publisher("fc/attitude", kDepth);
    const auto subscriber = bus_.create_subscriber("fc/attitude", kDepth);
    ASSERT_TRUE(publisher && subscriber);
    EXPECT_EQ(publisher->topic(), "fc/attitude");
    EXPECT_EQ(subscriber->topic(), "fc/attitude");

    std::vector<BusMessage> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage& m) { received.push_back(m); });

    EXPECT_TRUE(publisher->publish({1}, {}));
    EXPECT_TRUE(received.empty());

    process();
    EXPECT_EQ(received, (std::vector<BusMessage>{message("fc/attitude", 1)}));
    EXPECT_EQ(bus_.published().size(), 1U);
}

TEST_F(FakeBusTest, DeliversBinaryWithBody) {
    const auto publisher = bus_.create_publisher("vision/frame/gray8", kDepth);
    const auto subscriber = bus_.create_subscriber("vision/frame/gray8", kDepth);
    std::vector<BusMessage> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage& m) { received.push_back(m); });

    EXPECT_TRUE(publisher->publish({1, 2}, {9, 9, 9}));
    process();

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].body, (Bytes{1, 2}));
    EXPECT_EQ(received[0].binary, (Bytes{9, 9, 9}));
}

TEST_F(FakeBusTest, PublishSucceedsBeyondQueueDepth) {
    const auto publisher = bus_.create_publisher("t", 2);
    const auto subscriber = bus_.create_subscriber("t", 2);

    EXPECT_TRUE(publisher->publish({1}, {}));
    EXPECT_TRUE(publisher->publish({2}, {}));
    EXPECT_TRUE(publisher->publish({3}, {}));  // 큐 길이 2 를 넘겨도 발행은 성공
    EXPECT_EQ(bus_.published().size(), 3U);
}

TEST_F(FakeBusTest, DeliversOnlyLatestQueueDepthMessagesInOrder) {
    const auto publisher = bus_.create_publisher("t", kDepth);
    const auto subscriber = bus_.create_subscriber("t", 2);
    std::vector<BusMessage> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage& m) { received.push_back(m); });

    // 구독자가 이벤트를 처리하지 않는 동안 queue_depth 보다 많이 발행한다
    for (std::uint8_t value = 1; value <= 5; ++value) {
        EXPECT_TRUE(publisher->publish({value}, {}));
    }
    process();

    EXPECT_EQ(received, (std::vector<BusMessage>{message("t", 4), message("t", 5)}));

    // 전달한 뒤에는 큐가 비어 다시 queue_depth 개까지 보관한다
    received.clear();
    EXPECT_TRUE(publisher->publish({6}, {}));
    process();
    EXPECT_EQ(received, (std::vector<BusMessage>{message("t", 6)}));
}

TEST_F(FakeBusTest, EachSubscriberAppliesItsOwnQueueDepth) {
    const auto shallow_subscriber = bus_.create_subscriber("t", 1);
    const auto deep_subscriber = bus_.create_subscriber("t", 3);
    std::vector<BusMessage> shallow;
    std::vector<BusMessage> deep;
    const core::Subscription first =
        shallow_subscriber->subscribe([&](const BusMessage& m) { shallow.push_back(m); });
    const core::Subscription second =
        deep_subscriber->subscribe([&](const BusMessage& m) { deep.push_back(m); });

    for (std::uint8_t value = 1; value <= 3; ++value) {
        bus_.simulate_message_received(message("t", value));
    }
    process();

    EXPECT_EQ(shallow, (std::vector<BusMessage>{message("t", 3)}));
    EXPECT_EQ(deep, (std::vector<BusMessage>{message("t", 1), message("t", 2), message("t", 3)}));
}

TEST_F(FakeBusTest, PublishFailsWhileFailureIsSimulated) {
    const auto publisher = bus_.create_publisher("t", kDepth);
    const auto subscriber = bus_.create_subscriber("t", kDepth);
    int received = 0;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage&) { ++received; });

    bus_.simulate_publish_failure(true);
    EXPECT_FALSE(publisher->publish({1}, {}));
    process();
    EXPECT_EQ(received, 0);
    EXPECT_TRUE(bus_.published().empty());

    bus_.simulate_publish_failure(false);
    EXPECT_TRUE(publisher->publish({2}, {}));
    process();
    EXPECT_EQ(received, 1);
}

TEST_F(FakeBusTest, DestroyedSubscriberDropsUndeliveredMessages) {
    auto subscriber = bus_.create_subscriber("t", kDepth);
    int received = 0;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage&) { ++received; });

    bus_.simulate_message_received(message("t", 1));
    subscriber.reset();
    process();

    EXPECT_EQ(received, 0);
}

// 거부 테스트는 원인마다 하나. 같은 값으로 성공하는 호출을 먼저 두어 다른 입력은 정상임을 보인다
TEST_F(FakeBusTest, RejectsEmptyTopicName) {
    EXPECT_NE(bus_.create_publisher("t", kDepth), nullptr);
    EXPECT_EQ(bus_.create_publisher("", kDepth), nullptr);
    EXPECT_EQ(bus_.create_subscriber("", kDepth), nullptr);
}

TEST_F(FakeBusTest, RejectsNonPositiveQueueDepth) {
    EXPECT_NE(bus_.create_publisher("t", 1), nullptr);
    EXPECT_EQ(bus_.create_publisher("t", 0), nullptr);
    EXPECT_EQ(bus_.create_subscriber("t", -1), nullptr);
}

TEST_F(FakeBusTest, RejectsEmptyRequestName) {
    EXPECT_NE(bus_.create_requester("r"), nullptr);
    EXPECT_EQ(bus_.create_requester(""), nullptr);
    EXPECT_EQ(bus_.create_replier(""), nullptr);
}

TEST_F(FakeBusTest, MatchesTopicExactly) {
    const auto subscriber = bus_.create_subscriber("fc/state", kDepth);
    int received = 0;
    const core::Subscription subscription =
        subscriber->subscribe([&](const BusMessage&) { ++received; });

    bus_.simulate_message_received(message("fc/state_extra", 1));
    bus_.simulate_message_received(message("fc", 1));
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(FakeBusTest, CallsEverySubscriberInOrder) {
    const auto first_subscriber = bus_.create_subscriber("t", kDepth);
    const auto second_subscriber = bus_.create_subscriber("t", kDepth);
    std::vector<int> order;
    const core::Subscription first =
        first_subscriber->subscribe([&](const BusMessage&) { order.push_back(1); });
    const core::Subscription second =
        second_subscriber->subscribe([&](const BusMessage&) { order.push_back(2); });

    bus_.simulate_message_received(message("t", 0));
    process();

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(FakeBusTest, StopsDeliveringAfterSubscriptionReleased) {
    const auto subscriber = bus_.create_subscriber("t", kDepth);
    int received = 0;
    core::Subscription subscription = subscriber->subscribe([&](const BusMessage&) { ++received; });

    subscription.reset();
    bus_.simulate_message_received(message("t", 0));
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(FakeBusTest, SubscriptionOutlivesSubscriberSafely) {
    core::Subscription subscription;
    {
        const auto subscriber = bus_.create_subscriber("t", kDepth);
        subscription = subscriber->subscribe([](const BusMessage&) {});
    }
    subscription.reset();  // subscriber 가 먼저 사라져도 해제가 안전함
    SUCCEED();
}

TEST_F(FakeBusTest, RequestGetsReplyFromReplier) {
    const auto replier = bus_.create_replier("control/mode_request");
    const auto requester = bus_.create_requester("control/mode_request");
    ASSERT_TRUE(replier && requester);
    EXPECT_EQ(requester->name(), "control/mode_request");

    const core::Subscription served = replier->serve([](const BusMessage& request) {
        EXPECT_EQ(request.topic, "control/mode_request");
        return Bytes{42};
    });
    std::optional<BusMessage> reply;
    const core::Subscription pending = requester->request(
        {1}, milliseconds{100}, [&](std::optional<BusMessage> r) { reply = std::move(r); });

    EXPECT_FALSE(reply.has_value());  // 바로 반환하고, 응답은 process_events 에서
    process();

    EXPECT_EQ(reply, std::optional<BusMessage>(message("control/mode_request", 42)));
}

TEST_F(FakeBusTest, RequestTimesOutWithoutReplier) {
    const auto requester = bus_.create_requester("follow/command");
    bool called = false;
    std::optional<BusMessage> reply = message("unused", 0);
    const core::Subscription pending =
        requester->request({1}, milliseconds{100}, [&](std::optional<BusMessage> r) {
            called = true;
            reply = std::move(r);
        });

    process(milliseconds{50});
    EXPECT_FALSE(called);  // 아직 시간 초과 전

    process(milliseconds{60});
    EXPECT_TRUE(called);
    EXPECT_FALSE(reply.has_value());
    EXPECT_EQ(clock_.now_monotonic_ns(), 100 * kMs);
}

TEST_F(FakeBusTest, ReleasedRequestDoesNotReply) {
    const auto replier = bus_.create_replier("r");
    const auto requester = bus_.create_requester("r");
    const core::Subscription served =
        replier->serve([](const BusMessage& request) { return request.body; });
    bool called = false;
    core::Subscription pending = requester->request(
        {1}, milliseconds{100}, [&](const std::optional<BusMessage>&) { called = true; });

    pending.reset();
    process();

    EXPECT_FALSE(called);
}

TEST_F(FakeBusTest, LaterServeReplacesEarlier) {
    const auto replier = bus_.create_replier("r");
    const auto requester = bus_.create_requester("r");
    const core::Subscription first = replier->serve([](const BusMessage&) { return Bytes{1}; });
    const core::Subscription second = replier->serve([](const BusMessage&) { return Bytes{2}; });
    std::optional<BusMessage> reply;
    const core::Subscription pending = requester->request(
        {0}, milliseconds{100}, [&](std::optional<BusMessage> r) { reply = std::move(r); });

    process();

    EXPECT_EQ(reply, std::optional<BusMessage>(message("r", 2)));
}

}  // namespace
}  // namespace argus::bus_testing
