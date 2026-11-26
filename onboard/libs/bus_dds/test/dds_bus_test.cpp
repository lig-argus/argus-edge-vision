// DdsBus 테스트: 같은 프로세스 안 전달 (Fast DDS 의 기본 설정). 발행하는 쪽과 받는 쪽은 서로 다른
// DdsBus(DomainParticipant)다. 실제 시계와 PollPoller 로 Fast DDS 의 스레드가 전달하는 표본을
// 처리한다
#include "argus/bus_dds/dds_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <fastdds/dds/core/ReturnCode.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/core/poll_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "dds_test_support.hpp"

namespace argus::bus_dds {
namespace {

using std::chrono::milliseconds;

constexpr std::int32_t kDepth = 4;
constexpr milliseconds kTimeout{2000};

class DdsBusTest : public ::testing::Test {
protected:
    void SetUp() override {
        sender_ = DdsBus::create(poller_, poller_, clock_, kTestDomain, test_types(), {});
        receiver_ = DdsBus::create(poller_, poller_, clock_, kTestDomain, test_types(), {});
        ASSERT_NE(sender_, nullptr);
        ASSERT_NE(receiver_, nullptr);
    }

    // 테스트마다 다른 토픽 이름 (앞 테스트의 엔티티와 섞이지 않게)
    std::string name() const {
        return std::string("test/") +
               ::testing::UnitTest::GetInstance()->current_test_info()->name();
    }

    core::SteadyClock clock_;
    core::PollPoller poller_{clock_};
    std::unique_ptr<DdsBus> sender_;
    std::unique_ptr<DdsBus> receiver_;
};

TEST_F(DdsBusTest, DeliversPublishedValueInsideProcessEventsOnThisThread) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*receiver_, name(), kTextType, kDepth);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    const std::thread::id this_thread = std::this_thread::get_id();
    std::vector<Text> received;
    std::vector<std::uint64_t> sequences;
    bool other_thread = false;
    const core::Subscription subscription =
        subscriber->subscribe([&](const Text& text, const bus::MessageInfo& info) {
            other_thread = other_thread || std::this_thread::get_id() != this_thread;
            received.push_back(text);
            sequences.push_back(info.sequence);
        });

    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"hello", 7}); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].text, "hello");
    EXPECT_EQ(received[0].number, 7);
    EXPECT_GE(sequences[0], 1U);  // DDS 의 순번은 1 부터
    EXPECT_FALSE(other_thread);
}

TEST_F(DdsBusTest, SequenceIncreasesByOnePerPublish) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*receiver_, name(), kTextType, kDepth);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription =
        subscriber->subscribe([&](const Text& /*text*/, const bus::MessageInfo& info) {
            sequences.push_back(info.sequence);
        });
    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"connect", 0}); },
        [&] { return !sequences.empty(); }));
    const std::uint64_t first = sequences.back();

    sequences.clear();
    publisher->publish(Text{"a", 1});
    publisher->publish(Text{"b", 2});
    ASSERT_TRUE(pump_until(poller_, nullptr, [&] { return sequences.size() >= 2; }));

    EXPECT_EQ(sequences[sequences.size() - 2] + 1, sequences.back());
    EXPECT_GT(sequences.back(), first);
}

TEST_F(DdsBusTest, KeepsOnlyLatestQueueDepthValuesInOrder) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, 16);
    const auto subscriber = bus::make_subscriber<Text>(*receiver_, name(), kTextType, 3);
    std::vector<std::int32_t> numbers;
    const core::Subscription subscription =
        subscriber->subscribe([&](const Text& text, const bus::MessageInfo& /*info*/) {
            numbers.push_back(text.number);
        });
    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"connect", 0}); },
        [&] { return !numbers.empty(); }));
    // 연결 확인에 보낸 값이 더 도착하지 않을 때까지 처리한다
    process_for(poller_, milliseconds{300});
    numbers.clear();

    // 이벤트를 처리하지 않는 동안 10개를 보내면 구독자는 최근 3개만 남긴다
    for (std::int32_t number = 1; number <= 10; ++number) {
        publisher->publish(Text{"burst", number});
    }
    std::this_thread::sleep_for(milliseconds{200});
    pump_until(poller_, nullptr, [&] { return numbers.size() >= 3; });

    EXPECT_EQ(numbers, (std::vector<std::int32_t>{8, 9, 10}));
}

TEST_F(DdsBusTest, RejectsInvalidCreation) {
    EXPECT_EQ(sender_->create_publisher("", kTextType, kDepth), nullptr);
    EXPECT_EQ(sender_->create_publisher(name(), kTextType, 0), nullptr);
    EXPECT_EQ(sender_->create_publisher(name(), kUnregisteredType, kDepth), nullptr);
    EXPECT_EQ(sender_->create_subscriber(name(), kUnregisteredType, kDepth), nullptr);
    EXPECT_EQ(sender_->create_requester(name(), kTextType, kUnregisteredType), nullptr);
    EXPECT_EQ(sender_->create_replier(name(), kUnregisteredType, kTextType), nullptr);
}

TEST_F(DdsBusTest, PublisherAndSubscriberOfSameTopicShareOneBus) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*sender_, name(), kTextType, kDepth);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<Text> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const Text& text, const bus::MessageInfo& /*info*/) { received.push_back(text); });

    EXPECT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"same bus", 1}); },
        [&] { return !received.empty(); }));
}

TEST_F(DdsBusTest, ReleasedSubscriptionAndDestroyedSubscriberStopDelivery) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, kDepth);
    auto subscriber = bus::make_subscriber<Text>(*receiver_, name(), kTextType, kDepth);
    std::int32_t first_calls = 0;
    std::int32_t second_calls = 0;
    core::Subscription first = subscriber->subscribe(
        [&](const Text& /*text*/, const bus::MessageInfo& /*info*/) { ++first_calls; });
    const core::Subscription second = subscriber->subscribe(
        [&](const Text& /*text*/, const bus::MessageInfo& /*info*/) { ++second_calls; });
    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"connect", 0}); },
        [&] { return second_calls > 0; }));

    first.reset();
    const std::int32_t first_before = first_calls;
    const std::int32_t second_before = second_calls;
    publisher->publish(Text{"after release", 1});
    ASSERT_TRUE(pump_until(poller_, nullptr, [&] { return second_calls > second_before; }));
    EXPECT_EQ(first_calls, first_before);

    subscriber.reset();
    const std::int32_t second_after_destroy = second_calls;
    publisher->publish(Text{"after destroy", 2});
    process_for(poller_, milliseconds{300});
    EXPECT_EQ(second_calls, second_after_destroy);
}

class DdsRequestReplyTest : public DdsBusTest {
protected:
    // 응답하는 쪽: 요청의 number 에 100 을 더하고 text 앞에 "reply:" 를 붙인다
    core::Subscription serve_echo(bus::Replier<Text, Text>& replier) {
        return replier.serve([](const Text& request) {
            return Text{"reply:" + request.text, request.number + 100};
        });
    }
};

TEST_F(DdsRequestReplyTest, RequesterReceivesReplyToItsRequest) {
    const auto requester = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    const auto replier = bus::make_replier<Text, Text>(*receiver_, name(), kTextType, kTextType);
    ASSERT_NE(requester, nullptr);
    ASSERT_NE(replier, nullptr);
    const core::Subscription serving = serve_echo(*replier);
    std::optional<Text> reply;
    std::vector<core::Subscription> pending;

    // 발견이 끝나기 전의 요청은 전달되지 않으므로 응답을 받을 때까지 다시 보낸다
    ASSERT_TRUE(pump_until(
        poller_,
        [&] {
            pending.push_back(requester->request(Text{"ping", 1}, kTimeout,
                                                 [&](const std::optional<Text>& value) {
                                                     if (value.has_value()) {
                                                         reply = value;
                                                     }
                                                 }));
        },
        [&] { return reply.has_value(); }));

    EXPECT_EQ(reply.value_or(Text{}).text, "reply:ping");
    EXPECT_EQ(reply.value_or(Text{}).number, 101);
}

TEST_F(DdsRequestReplyTest, TwoRequestersReceiveOnlyTheirOwnReplies) {
    const auto first = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    const auto second = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    const auto replier = bus::make_replier<Text, Text>(*receiver_, name(), kTextType, kTextType);
    const core::Subscription serving = serve_echo(*replier);
    std::vector<std::int32_t> first_replies;
    std::vector<std::int32_t> second_replies;
    std::vector<core::Subscription> pending;

    ASSERT_TRUE(pump_until(
        poller_,
        [&] {
            pending.push_back(
                first->request(Text{"first", 1}, kTimeout, [&](const std::optional<Text>& value) {
                    if (value.has_value()) {
                        first_replies.push_back(value.value_or(Text{}).number);
                    }
                }));
            pending.push_back(
                second->request(Text{"second", 2}, kTimeout, [&](const std::optional<Text>& value) {
                    if (value.has_value()) {
                        second_replies.push_back(value.value_or(Text{}).number);
                    }
                }));
        },
        [&] { return !first_replies.empty() && !second_replies.empty(); }));

    for (const std::int32_t number : first_replies) {
        EXPECT_EQ(number, 101);
    }
    for (const std::int32_t number : second_replies) {
        EXPECT_EQ(number, 102);
    }
}

TEST_F(DdsRequestReplyTest, CallsHandlerWithNullAfterTimeoutWithoutReplier) {
    const auto requester = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    bool called = false;
    bool got_null = false;
    const auto started = std::chrono::steady_clock::now();
    const core::Subscription pending = requester->request(Text{"nobody", 1}, milliseconds{200},
                                                          [&](const std::optional<Text>& value) {
                                                              called = true;
                                                              got_null = !value.has_value();
                                                          });

    ASSERT_TRUE(pump_until(poller_, nullptr, [&] { return called; }));

    EXPECT_TRUE(got_null);
    EXPECT_GE(std::chrono::steady_clock::now() - started, milliseconds{190});
}

TEST_F(DdsRequestReplyTest, ReleasedRequestDoesNotCallHandler) {
    const auto requester = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    bool called = false;
    core::Subscription pending =
        requester->request(Text{"released", 1}, milliseconds{100},
                           [&](const std::optional<Text>& /*value*/) { called = true; });

    pending.reset();
    std::this_thread::sleep_for(milliseconds{150});
    poller_.process_events(milliseconds{10}, milliseconds{50});

    EXPECT_FALSE(called);
}

// Fast DDS 의 요청-응답 API 가 지원으로 바뀌면 알기 위한 테스트. 지원으로 바뀌면 DdsBus 의
// 요청-응답을 그 API 로 바꿀지 검토한다
TEST(FastDdsServiceApiTest, CreateServiceIsUnsupported) {
    namespace dds = eprosima::fastdds::dds;
    dds::DomainParticipant* const participant =
        dds::DomainParticipantFactory::get_instance()->create_participant(
            kTestDomain, dds::PARTICIPANT_QOS_DEFAULT);
    ASSERT_NE(participant, nullptr);
    dds::ReturnCode_t code = dds::RETCODE_OK;

    EXPECT_EQ(participant->create_service("test_Service", "TestText", code), nullptr);
    EXPECT_EQ(code, dds::RETCODE_UNSUPPORTED);

    dds::DomainParticipantFactory::get_instance()->delete_participant(participant);
}

}  // namespace
}  // namespace argus::bus_dds
