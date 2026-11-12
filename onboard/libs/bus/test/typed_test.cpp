#include "argus/bus/typed.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "argus/bus/bus.hpp"
#include "argus/bus/message_type.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::bus {
namespace {

using std::chrono::milliseconds;

struct Ping {
    int value = 0;
};

struct Pong {
    std::string text;
};

constexpr int kDepth = 4;
constexpr milliseconds kTimeout{100};

MessageType ping_type() { return message_type<Ping>("test.Ping"); }
MessageType pong_type() { return message_type<Pong>("test.Pong"); }

// create_*() 가 불린 횟수를 세고 항상 nullptr 를 반환하는 bus
class CountingBus : public IBus {
public:
    std::unique_ptr<ITopicPublisher> create_publisher(std::string_view /*topic*/,
                                                      const MessageType& /*type*/,
                                                      int /*queue_depth*/) override {
        ++calls;
        return nullptr;
    }
    std::unique_ptr<ITopicSubscriber> create_subscriber(std::string_view /*topic*/,
                                                        const MessageType& /*type*/,
                                                        int /*queue_depth*/) override {
        ++calls;
        return nullptr;
    }
    std::unique_ptr<IRequester> create_requester(std::string_view /*name*/,
                                                 const MessageType& /*request*/,
                                                 const MessageType& /*reply*/) override {
        ++calls;
        return nullptr;
    }
    std::unique_ptr<IReplier> create_replier(std::string_view /*name*/,
                                             const MessageType& /*request*/,
                                             const MessageType& /*reply*/) override {
        ++calls;
        return nullptr;
    }

    int calls = 0;
};

class TypedTest : public ::testing::Test {
protected:
    void process(milliseconds wait = milliseconds{0}) {
        poller_.process_events(wait, milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
};

TEST_F(TypedTest, SubscriberReceivesTheValueAndInfoThePublisherSent) {
    const auto publisher = make_publisher<Ping>(bus_, "t", ping_type(), kDepth);
    const auto subscriber = make_subscriber<Ping>(bus_, "t", ping_type(), kDepth);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    EXPECT_EQ(publisher->topic(), "t");
    EXPECT_EQ(subscriber->topic(), "t");
    std::vector<int> values;
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription =
        subscriber->subscribe([&](const Ping& ping, const MessageInfo& info) {
            values.push_back(ping.value);
            sequences.push_back(info.sequence);
        });

    EXPECT_TRUE(publisher->publish(Ping{5}));
    EXPECT_TRUE(publisher->publish(Ping{6}));
    process();

    EXPECT_EQ(values, (std::vector<int>{5, 6}));
    EXPECT_EQ(sequences, (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(TypedTest, PublishReturnsFalseWhenBusFailsToPublish) {
    const auto publisher = make_publisher<Ping>(bus_, "t", ping_type(), kDepth);
    ASSERT_NE(publisher, nullptr);

    bus_.simulate_publish_failure(true);

    EXPECT_FALSE(publisher->publish(Ping{1}));
}

TEST_F(TypedTest, SubscribeWithEmptyHandlerRegistersNothing) {
    const auto subscriber = make_subscriber<Ping>(bus_, "t", ping_type(), kDepth);
    ASSERT_NE(subscriber, nullptr);

    const core::Subscription subscription = subscriber->subscribe(nullptr);
    bus_.simulate_message_received("t", Ping{1});
    process();  // 빈 handler 가 등록되었다면 여기서 호출되어 예외가 난다

    EXPECT_FALSE(subscription.active());
}

TEST_F(TypedTest, RequesterReceivesTheReplyTheReplierReturned) {
    const auto replier = make_replier<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    const auto requester = make_requester<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    ASSERT_NE(replier, nullptr);
    ASSERT_NE(requester, nullptr);
    EXPECT_EQ(replier->name(), "r");
    EXPECT_EQ(requester->name(), "r");
    const core::Subscription serving =
        replier->serve([](const Ping& ping) { return Pong{std::to_string(ping.value)}; });
    int calls = 0;
    std::optional<Pong> reply;

    const core::Subscription pending =
        requester->request(Ping{42}, kTimeout, [&](std::optional<Pong> pong) {
            ++calls;
            reply = std::move(pong);
        });
    process();

    EXPECT_EQ(calls, 1);
    ASSERT_TRUE(reply.has_value());
    EXPECT_EQ(reply.value_or(Pong{}).text, "42");
}

TEST_F(TypedTest, RequesterReceivesNulloptOnTimeout) {
    const auto requester = make_requester<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    ASSERT_NE(requester, nullptr);
    int calls = 0;
    std::optional<Pong> reply = Pong{"unused"};

    const core::Subscription pending =
        requester->request(Ping{1}, kTimeout, [&](std::optional<Pong> pong) {
            ++calls;
            reply = std::move(pong);
        });
    process(kTimeout);

    EXPECT_EQ(calls, 1);
    EXPECT_FALSE(reply.has_value());
}

TEST_F(TypedTest, ReplierStopsServingWhenSubscriptionIsReleased) {
    const auto replier = make_replier<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    const auto requester = make_requester<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    ASSERT_NE(replier, nullptr);
    ASSERT_NE(requester, nullptr);
    core::Subscription serving = replier->serve([](const Ping&) { return Pong{"served"}; });
    serving.reset();
    std::optional<Pong> reply = Pong{"unused"};

    const core::Subscription pending = requester->request(
        Ping{1}, kTimeout, [&](std::optional<Pong> pong) { reply = std::move(pong); });
    process(kTimeout);

    EXPECT_FALSE(reply.has_value());
}

TEST_F(TypedTest, RequestWithEmptyHandlerSendsNothing) {
    const auto replier = make_replier<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    const auto requester = make_requester<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    ASSERT_NE(replier, nullptr);
    ASSERT_NE(requester, nullptr);
    int served = 0;
    const core::Subscription serving = replier->serve([&](const Ping&) {
        ++served;
        return Pong{};
    });

    const core::Subscription pending = requester->request(Ping{1}, kTimeout, nullptr);
    process(kTimeout);

    EXPECT_FALSE(pending.active());
    EXPECT_EQ(served, 0);
}

TEST_F(TypedTest, ServeWithEmptyHandlerKeepsTheEarlierRegistration) {
    const auto replier = make_replier<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    const auto requester = make_requester<Ping, Pong>(bus_, "r", ping_type(), pong_type());
    ASSERT_NE(replier, nullptr);
    ASSERT_NE(requester, nullptr);
    const core::Subscription serving = replier->serve([](const Ping&) { return Pong{"first"}; });
    std::optional<Pong> reply;

    const core::Subscription empty = replier->serve(nullptr);
    const core::Subscription pending = requester->request(
        Ping{1}, kTimeout, [&](std::optional<Pong> pong) { reply = std::move(pong); });
    process();

    EXPECT_FALSE(empty.active());
    ASSERT_TRUE(reply.has_value());
    EXPECT_EQ(reply.value_or(Pong{}).text, "first");
}

// 타입 불일치 테스트는 함수마다 하나. bus 의 create_*() 를 부르지 않고 nullptr 를 반환해야 한다
TEST(TypedMakeTest, MakePublisherReturnsNullWhenTypeIsNotT) {
    CountingBus bus;

    EXPECT_EQ(make_publisher<Ping>(bus, "t", pong_type(), kDepth), nullptr);
    EXPECT_EQ(bus.calls, 0);
}

TEST(TypedMakeTest, MakeSubscriberReturnsNullWhenTypeIsNotT) {
    CountingBus bus;

    EXPECT_EQ(make_subscriber<Ping>(bus, "t", pong_type(), kDepth), nullptr);
    EXPECT_EQ(bus.calls, 0);
}

TEST(TypedMakeTest, MakeRequesterReturnsNullWhenEitherTypeDiffers) {
    CountingBus bus;

    EXPECT_EQ((make_requester<Ping, Pong>(bus, "r", pong_type(), pong_type())), nullptr);
    EXPECT_EQ((make_requester<Ping, Pong>(bus, "r", ping_type(), ping_type())), nullptr);
    EXPECT_EQ(bus.calls, 0);
}

TEST(TypedMakeTest, MakeReplierReturnsNullWhenEitherTypeDiffers) {
    CountingBus bus;

    EXPECT_EQ((make_replier<Ping, Pong>(bus, "r", pong_type(), pong_type())), nullptr);
    EXPECT_EQ((make_replier<Ping, Pong>(bus, "r", ping_type(), ping_type())), nullptr);
    EXPECT_EQ(bus.calls, 0);
}

TEST(TypedMakeTest, MakeReturnsNullWhenBusCannotCreateTheEntity) {
    CountingBus bus;

    EXPECT_EQ(make_publisher<Ping>(bus, "t", ping_type(), kDepth), nullptr);
    EXPECT_EQ(make_subscriber<Ping>(bus, "t", ping_type(), kDepth), nullptr);
    EXPECT_EQ((make_requester<Ping, Pong>(bus, "r", ping_type(), pong_type())), nullptr);
    EXPECT_EQ((make_replier<Ping, Pong>(bus, "r", ping_type(), pong_type())), nullptr);
    EXPECT_EQ(bus.calls, 4);
}

}  // namespace
}  // namespace argus::bus
