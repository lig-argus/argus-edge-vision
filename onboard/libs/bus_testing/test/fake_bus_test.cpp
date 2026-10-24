#include "argus/bus_testing/fake_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
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
using std::chrono::milliseconds;

constexpr std::int64_t kMs = 1'000'000;

BusMessage message(const std::string& topic, std::uint8_t value) { return {topic, {{value}}}; }

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
    std::vector<BusMessage> received;
    const core::Subscription subscription =
        bus_.subscribe("fc/attitude", [&](const BusMessage& m) { received.push_back(m); });

    bus_.publish(message("fc/attitude", 1));
    EXPECT_TRUE(received.empty());

    process();
    EXPECT_EQ(received, (std::vector<BusMessage>{message("fc/attitude", 1)}));
    EXPECT_EQ(bus_.published().size(), 1U);
}

TEST_F(FakeBusTest, MatchesTopicExactly) {
    int received = 0;
    const core::Subscription subscription =
        bus_.subscribe("fc/state", [&](const BusMessage&) { ++received; });

    bus_.inject(message("fc/state_extra", 1));  // 앞부분만 같은 토픽
    bus_.inject(message("fc", 1));
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(FakeBusTest, CallsEverySubscriberInOrder) {
    std::vector<int> order;
    const core::Subscription first =
        bus_.subscribe("t", [&](const BusMessage&) { order.push_back(1); });
    const core::Subscription second =
        bus_.subscribe("t", [&](const BusMessage&) { order.push_back(2); });

    bus_.inject(message("t", 0));
    process();

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(FakeBusTest, StopsDeliveringAfterSubscriptionReleased) {
    int received = 0;
    core::Subscription subscription = bus_.subscribe("t", [&](const BusMessage&) { ++received; });

    subscription.reset();
    bus_.inject(message("t", 0));
    process();

    EXPECT_EQ(received, 0);
}

TEST_F(FakeBusTest, CallGetsReplyFromServer) {
    const core::Subscription server =
        bus_.serve("control/mode_request",
                   [](const BusMessage& request) { return BusMessage{request.topic, {{42}}}; });
    std::optional<BusMessage> reply;
    const core::Subscription call =
        bus_.call(message("control/mode_request", 1), milliseconds{100},
                  [&](std::optional<BusMessage> r) { reply = std::move(r); });

    EXPECT_FALSE(reply.has_value());  // 바로 반환하고, 응답은 process_events 에서
    process();

    EXPECT_EQ(reply, std::optional<BusMessage>(BusMessage{"control/mode_request", {{42}}}));
}

TEST_F(FakeBusTest, CallTimesOutWithoutServer) {
    bool called = false;
    std::optional<BusMessage> reply = message("unused", 0);
    const core::Subscription call = bus_.call(message("follow/command", 1), milliseconds{100},
                                              [&](std::optional<BusMessage> r) {
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

TEST_F(FakeBusTest, ReleasedCallDoesNotReply) {
    const core::Subscription server =
        bus_.serve("r", [](const BusMessage& request) { return request; });
    bool called = false;
    core::Subscription call = bus_.call(message("r", 1), milliseconds{100},
                                        [&](const std::optional<BusMessage>&) { called = true; });

    call.reset();
    process();

    EXPECT_FALSE(called);
}

TEST_F(FakeBusTest, LaterServeReplacesEarlier) {
    const core::Subscription first =
        bus_.serve("r", [](const BusMessage&) { return BusMessage{"r", {{1}}}; });
    const core::Subscription second =
        bus_.serve("r", [](const BusMessage&) { return BusMessage{"r", {{2}}}; });
    std::optional<BusMessage> reply;
    const core::Subscription call =
        bus_.call(message("r", 0), milliseconds{100},
                  [&](std::optional<BusMessage> r) { reply = std::move(r); });

    process();

    EXPECT_EQ(reply, std::optional<BusMessage>(BusMessage{"r", {{2}}}));
}

}  // namespace
}  // namespace argus::bus_testing
