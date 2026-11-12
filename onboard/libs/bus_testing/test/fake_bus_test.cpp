#include "argus/bus_testing/fake_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "argus/bus/message_type.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::bus_testing {
namespace {

using bus::MessageInfo;
using std::chrono::milliseconds;

constexpr std::int64_t kMs = 1'000'000;
constexpr int kDepth = 4;

// 토픽과 요청이 나르는 값
struct Sample {
    int value = 0;

    friend bool operator==(const Sample& lhs, const Sample& rhs) { return lhs.value == rhs.value; }
};

// Sample 과 타입만 다른 값
struct Other {
    int value = 0;
};

// 응답이 나르는 값
struct Answer {
    std::string text;
};

bus::MessageType sample_type() { return bus::message_type<Sample>("test.Sample"); }
bus::MessageType other_type() { return bus::message_type<Other>("test.Other"); }
bus::MessageType answer_type() { return bus::message_type<Answer>("test.Answer"); }

std::vector<Sample> samples(const std::vector<int>& values) {
    std::vector<Sample> result;
    result.reserve(values.size());
    for (const int value : values) {
        result.push_back(Sample{value});
    }
    return result;
}

class FakeBusTest : public ::testing::Test {
protected:
    void process(milliseconds wait = milliseconds{0}) {
        poller_.process_events(wait, milliseconds{5});
    }

    std::unique_ptr<bus::Publisher<Sample>> publisher(const std::string& topic, int depth) {
        return bus::make_publisher<Sample>(bus_, topic, sample_type(), depth);
    }
    std::unique_ptr<bus::Subscriber<Sample>> subscriber(const std::string& topic, int depth) {
        return bus::make_subscriber<Sample>(bus_, topic, sample_type(), depth);
    }
    std::unique_ptr<bus::Requester<Sample, Answer>> requester(const std::string& name) {
        return bus::make_requester<Sample, Answer>(bus_, name, sample_type(), answer_type());
    }
    std::unique_ptr<bus::Replier<Sample, Answer>> replier(const std::string& name) {
        return bus::make_replier<Sample, Answer>(bus_, name, sample_type(), answer_type());
    }

    // 받은 값을 received 에 기록하는 handler
    static bus::Subscriber<Sample>::Handler record_into(std::vector<Sample>& received) {
        return
            [&received](const Sample& sample, const MessageInfo&) { received.push_back(sample); };
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeBus bus_{poller_, clock_};
};

TEST_F(FakeBusTest, DeliversPublishedMessageOnlyInsideProcessEvents) {
    const auto pub = publisher("fc/attitude", kDepth);
    const auto sub = subscriber("fc/attitude", kDepth);
    ASSERT_TRUE(pub && sub);
    EXPECT_EQ(pub->topic(), "fc/attitude");
    EXPECT_EQ(sub->topic(), "fc/attitude");
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));

    EXPECT_TRUE(pub->publish(Sample{1}));
    EXPECT_TRUE(received.empty());

    process();
    EXPECT_EQ(received, samples({1}));
    EXPECT_EQ(bus_.published<Sample>("fc/attitude"), samples({1}));
    EXPECT_EQ(bus_.published_count(), 1U);
}

TEST_F(FakeBusTest, DeliversCopyTakenAtPublish) {
    const auto pub = publisher("t", kDepth);
    const auto sub = subscriber("t", kDepth);
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));
    Sample sample{1};

    EXPECT_TRUE(pub->publish(sample));
    sample.value = 2;  // 발행한 뒤에 바꾼 값은 전달되지 않는다
    process();

    EXPECT_EQ(received, samples({1}));
}

TEST_F(FakeBusTest, PublishSucceedsBeyondQueueDepth) {
    const auto pub = publisher("t", 2);
    const auto sub = subscriber("t", 2);

    EXPECT_TRUE(pub->publish(Sample{1}));
    EXPECT_TRUE(pub->publish(Sample{2}));
    EXPECT_TRUE(pub->publish(Sample{3}));  // 큐 길이 2 를 넘겨도 발행은 성공
    EXPECT_EQ(bus_.published_count(), 3U);
}

TEST_F(FakeBusTest, DeliversOnlyLatestQueueDepthMessagesInOrder) {
    const auto pub = publisher("t", kDepth);
    const auto sub = subscriber("t", 2);
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));

    // 구독자가 이벤트를 처리하지 않는 동안 queue_depth 보다 많이 발행한다
    for (int value = 1; value <= 5; ++value) {
        EXPECT_TRUE(pub->publish(Sample{value}));
    }
    process();

    EXPECT_EQ(received, samples({4, 5}));

    // 전달한 뒤에는 큐가 비어 다시 queue_depth 개까지 보관한다
    received.clear();
    EXPECT_TRUE(pub->publish(Sample{6}));
    process();
    EXPECT_EQ(received, samples({6}));
}

TEST_F(FakeBusTest, EachSubscriberAppliesItsOwnQueueDepth) {
    const auto shallow_subscriber = subscriber("t", 1);
    const auto deep_subscriber = subscriber("t", 3);
    std::vector<Sample> shallow;
    std::vector<Sample> deep;
    const core::Subscription first = shallow_subscriber->subscribe(record_into(shallow));
    const core::Subscription second = deep_subscriber->subscribe(record_into(deep));

    for (int value = 1; value <= 3; ++value) {
        bus_.simulate_message_received("t", Sample{value});
    }
    process();

    EXPECT_EQ(shallow, samples({3}));
    EXPECT_EQ(deep, samples({1, 2, 3}));
}

TEST_F(FakeBusTest, PublishFailsWhileFailureIsSimulated) {
    const auto pub = publisher("t", kDepth);
    const auto sub = subscriber("t", kDepth);
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));

    bus_.simulate_publish_failure(true);
    EXPECT_FALSE(pub->publish(Sample{1}));
    process();
    EXPECT_TRUE(received.empty());
    EXPECT_EQ(bus_.published_count(), 0U);

    bus_.simulate_publish_failure(false);
    EXPECT_TRUE(pub->publish(Sample{2}));
    process();
    EXPECT_EQ(received, samples({2}));
}

TEST_F(FakeBusTest, DestroyedSubscriberDropsUndeliveredMessages) {
    auto sub = subscriber("t", kDepth);
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));

    bus_.simulate_message_received("t", Sample{1});
    sub.reset();
    process();

    EXPECT_TRUE(received.empty());
}

// 거부 테스트는 원인마다 하나. 같은 값으로 성공하는 호출을 먼저 두어 다른 입력은 정상임을 보인다
TEST_F(FakeBusTest, RejectsEmptyTopicName) {
    EXPECT_NE(bus_.create_publisher("t", sample_type(), kDepth), nullptr);
    EXPECT_EQ(bus_.create_publisher("", sample_type(), kDepth), nullptr);
    EXPECT_EQ(bus_.create_subscriber("", sample_type(), kDepth), nullptr);
}

TEST_F(FakeBusTest, RejectsNonPositiveQueueDepth) {
    EXPECT_NE(bus_.create_publisher("t", sample_type(), 1), nullptr);
    EXPECT_EQ(bus_.create_publisher("t", sample_type(), 0), nullptr);
    EXPECT_EQ(bus_.create_subscriber("t", sample_type(), -1), nullptr);
}

TEST_F(FakeBusTest, RejectsEmptyRequestName) {
    EXPECT_NE(bus_.create_requester("r", sample_type(), answer_type()), nullptr);
    EXPECT_EQ(bus_.create_requester("", sample_type(), answer_type()), nullptr);
    EXPECT_EQ(bus_.create_replier("", sample_type(), answer_type()), nullptr);
}

TEST_F(FakeBusTest, MatchesTopicExactly) {
    const auto sub = subscriber("fc/state", kDepth);
    std::vector<Sample> received;
    const core::Subscription subscription = sub->subscribe(record_into(received));

    bus_.simulate_message_received("fc/state_extra", Sample{1});
    bus_.simulate_message_received("fc", Sample{1});
    process();

    EXPECT_TRUE(received.empty());
}

TEST_F(FakeBusTest, DoesNotDeliverToSubscriberOfDifferentType) {
    const auto pub = publisher("t", kDepth);
    const auto sample_subscriber = subscriber("t", kDepth);
    const auto other_subscriber = bus::make_subscriber<Other>(bus_, "t", other_type(), kDepth);
    ASSERT_NE(other_subscriber, nullptr);
    std::vector<Sample> received;
    int other_received = 0;
    const core::Subscription first = sample_subscriber->subscribe(record_into(received));
    const core::Subscription second =
        other_subscriber->subscribe([&](const Other&, const MessageInfo&) { ++other_received; });

    EXPECT_TRUE(pub->publish(Sample{1}));
    bus_.simulate_message_received("t", Sample{2});
    process();

    EXPECT_EQ(received, samples({1, 2}));
    EXPECT_EQ(other_received, 0);
    EXPECT_TRUE(bus_.published<Other>("t").empty());  // 타입이 다르면 발행 기록에서도 빠진다
}

TEST_F(FakeBusTest, SequenceStartsAtOneForEachPublisher) {
    const auto first_publisher = publisher("t", kDepth);
    const auto second_publisher = publisher("t", kDepth);
    const auto sub = subscriber("t", kDepth);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription = sub->subscribe(
        [&](const Sample&, const MessageInfo& info) { sequences.push_back(info.sequence); });

    EXPECT_TRUE(first_publisher->publish(Sample{0}));
    EXPECT_TRUE(first_publisher->publish(Sample{0}));
    EXPECT_TRUE(second_publisher->publish(Sample{0}));
    EXPECT_TRUE(first_publisher->publish(Sample{0}));
    process();

    EXPECT_EQ(sequences, (std::vector<std::uint64_t>{1, 2, 1, 3}));
}

TEST_F(FakeBusTest, FailedPublishUsesSequenceNumber) {
    const auto pub = publisher("t", kDepth);
    const auto sub = subscriber("t", kDepth);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription = sub->subscribe(
        [&](const Sample&, const MessageInfo& info) { sequences.push_back(info.sequence); });

    EXPECT_TRUE(pub->publish(Sample{0}));
    bus_.simulate_publish_failure(true);
    EXPECT_FALSE(pub->publish(Sample{0}));
    bus_.simulate_publish_failure(false);
    EXPECT_TRUE(pub->publish(Sample{0}));
    process();

    EXPECT_EQ(sequences, (std::vector<std::uint64_t>{1, 3}));  // 받는 쪽이 2 를 유실로 센다
}

TEST_F(FakeBusTest, SimulatedMessageCarriesGivenInfo) {
    const auto sub = subscriber("t", kDepth);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription = sub->subscribe(
        [&](const Sample&, const MessageInfo& info) { sequences.push_back(info.sequence); });

    bus_.simulate_message_received("t", Sample{0}, MessageInfo{9});
    bus_.simulate_message_received("t", Sample{0});
    process();

    EXPECT_EQ(sequences, (std::vector<std::uint64_t>{9, 0}));
    EXPECT_EQ(bus_.published_count(), 0U);  // 도착한 것으로 넣은 메시지는 발행 기록이 아니다
}

TEST_F(FakeBusTest, CallsEverySubscriberInOrder) {
    const auto first_subscriber = subscriber("t", kDepth);
    const auto second_subscriber = subscriber("t", kDepth);
    std::vector<int> order;
    const core::Subscription first =
        first_subscriber->subscribe([&](const Sample&, const MessageInfo&) { order.push_back(1); });
    const core::Subscription second = second_subscriber->subscribe(
        [&](const Sample&, const MessageInfo&) { order.push_back(2); });

    bus_.simulate_message_received("t", Sample{0});
    process();

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(FakeBusTest, CallsEveryHandlerOfOneSubscriberInOrder) {
    const auto sub = subscriber("t", kDepth);
    std::vector<int> order;
    const core::Subscription first =
        sub->subscribe([&](const Sample&, const MessageInfo&) { order.push_back(1); });
    const core::Subscription second =
        sub->subscribe([&](const Sample&, const MessageInfo&) { order.push_back(2); });

    bus_.simulate_message_received("t", Sample{0});
    process();

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(FakeBusTest, StopsDeliveringAfterSubscriptionReleased) {
    const auto sub = subscriber("t", kDepth);
    std::vector<Sample> received;
    core::Subscription subscription = sub->subscribe(record_into(received));

    subscription.reset();
    bus_.simulate_message_received("t", Sample{0});
    process();

    EXPECT_TRUE(received.empty());
}

TEST_F(FakeBusTest, SubscriptionOutlivesSubscriberSafely) {
    core::Subscription subscription;
    {
        const auto sub = subscriber("t", kDepth);
        subscription = sub->subscribe([](const Sample&, const MessageInfo&) {});
    }
    subscription.reset();  // subscriber 가 먼저 사라져도 해제가 안전함
    SUCCEED();
}

TEST_F(FakeBusTest, RequestGetsReplyFromReplier) {
    const auto rep = replier("control/mode_request");
    const auto req = requester("control/mode_request");
    ASSERT_TRUE(rep && req);
    EXPECT_EQ(rep->name(), "control/mode_request");
    EXPECT_EQ(req->name(), "control/mode_request");

    const core::Subscription served = rep->serve(
        [](const Sample& request) { return Answer{"re:" + std::to_string(request.value)}; });
    int calls = 0;
    std::optional<Answer> reply;
    const core::Subscription pending =
        req->request(Sample{42}, milliseconds{100}, [&](std::optional<Answer> answer) {
            ++calls;
            reply = std::move(answer);
        });

    EXPECT_EQ(calls, 0);  // 바로 반환하고, 응답은 process_events 에서
    process();

    EXPECT_EQ(calls, 1);
    ASSERT_TRUE(reply.has_value());
    EXPECT_EQ(reply.value_or(Answer{}).text, "re:42");
}

TEST_F(FakeBusTest, RequestTimesOutWithoutReplier) {
    const auto req = requester("follow/command");
    bool called = false;
    std::optional<Answer> reply = Answer{"unused"};
    const core::Subscription pending =
        req->request(Sample{1}, milliseconds{100}, [&](std::optional<Answer> answer) {
            called = true;
            reply = std::move(answer);
        });

    process(milliseconds{50});
    EXPECT_FALSE(called);  // 아직 시간 초과 전

    process(milliseconds{60});
    EXPECT_TRUE(called);
    EXPECT_FALSE(reply.has_value());
    EXPECT_EQ(clock_.now_monotonic_ns(), 100 * kMs);
}

TEST_F(FakeBusTest, RequestTimesOutWhenReplierHasDifferentTypes) {
    // 응답하는 쪽은 요청을 Other 로 받는다
    const auto rep = bus::make_replier<Other, Answer>(bus_, "r", other_type(), answer_type());
    const auto req = requester("r");
    ASSERT_TRUE(rep && req);
    int served = 0;
    const core::Subscription serving = rep->serve([&](const Other&) {
        ++served;
        return Answer{};
    });
    bool called = false;
    std::optional<Answer> reply = Answer{"unused"};
    const core::Subscription pending =
        req->request(Sample{1}, milliseconds{100}, [&](std::optional<Answer> answer) {
            called = true;
            reply = std::move(answer);
        });

    process(milliseconds{100});

    EXPECT_EQ(served, 0);
    EXPECT_TRUE(called);
    EXPECT_FALSE(reply.has_value());
}

TEST_F(FakeBusTest, ReleasedRequestDoesNotReply) {
    const auto rep = replier("r");
    const auto req = requester("r");
    const core::Subscription served = rep->serve([](const Sample&) { return Answer{}; });
    bool called = false;
    core::Subscription pending = req->request(Sample{1}, milliseconds{100},
                                              [&](const std::optional<Answer>&) { called = true; });

    pending.reset();
    process();

    EXPECT_FALSE(called);
}

TEST_F(FakeBusTest, LaterServeReplacesEarlier) {
    const auto rep = replier("r");
    const auto req = requester("r");
    const core::Subscription first = rep->serve([](const Sample&) { return Answer{"first"}; });
    const core::Subscription second = rep->serve([](const Sample&) { return Answer{"second"}; });
    std::optional<Answer> reply;
    const core::Subscription pending =
        req->request(Sample{0}, milliseconds{100},
                     [&](std::optional<Answer> answer) { reply = std::move(answer); });

    process();

    ASSERT_TRUE(reply.has_value());
    EXPECT_EQ(reply.value_or(Answer{}).text, "second");
}

}  // namespace
}  // namespace argus::bus_testing
