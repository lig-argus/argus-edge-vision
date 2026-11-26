// DdsBus 테스트: 전송 계층을 거치는 전달. Fast DDS 의 같은 프로세스 안 전달을 끄면, 같은 프로세스의
// DomainParticipant 끼리도 전송(같은 컴퓨터에서는 공유 메모리 전송)을 거친다. 이 설정은 프로세스
// 전체에 적용되므로 dds_bus_test 와 다른 실행 파일로 둔다
#include <gtest/gtest.h>

#include <chrono>
#include <fastdds/LibrarySettings.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/core/poll_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "dds_test_support.hpp"

namespace argus::bus_dds {
namespace {

using std::chrono::milliseconds;

constexpr std::int32_t kDepth = 4;

class DdsBusTransportTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        eprosima::fastdds::LibrarySettings settings;
        settings.intraprocess_delivery = eprosima::fastdds::INTRAPROCESS_OFF;
        ASSERT_EQ(
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->set_library_settings(
                settings),
            eprosima::fastdds::dds::RETCODE_OK);
    }

    void SetUp() override {
        sender_ = DdsBus::create(poller_, poller_, clock_, kTestDomain, test_types(), {});
        receiver_ = DdsBus::create(poller_, poller_, clock_, kTestDomain, test_types(), {});
        ASSERT_NE(sender_, nullptr);
        ASSERT_NE(receiver_, nullptr);
    }

    std::string name() const {
        return std::string("transport/") +
               ::testing::UnitTest::GetInstance()->current_test_info()->name();
    }

    core::SteadyClock clock_;
    core::PollPoller poller_{clock_};
    std::unique_ptr<DdsBus> sender_;
    std::unique_ptr<DdsBus> receiver_;
};

TEST_F(DdsBusTransportTest, DeliversTopicValue) {
    const auto publisher = bus::make_publisher<Text>(*sender_, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*receiver_, name(), kTextType, kDepth);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<Text> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const Text& text, const bus::MessageInfo& /*info*/) { received.push_back(text); });

    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"over transport", 3}); },
        [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].text, "over transport");
    EXPECT_EQ(received[0].number, 3);
}

TEST_F(DdsBusTransportTest, RequesterReceivesReply) {
    const auto requester = bus::make_requester<Text, Text>(*sender_, name(), kTextType, kTextType);
    const auto replier = bus::make_replier<Text, Text>(*receiver_, name(), kTextType, kTextType);
    ASSERT_NE(requester, nullptr);
    ASSERT_NE(replier, nullptr);
    const core::Subscription serving =
        replier->serve([](const Text& request) { return Text{request.text, request.number * 2}; });
    std::optional<Text> reply;
    std::vector<core::Subscription> pending;

    ASSERT_TRUE(pump_until(
        poller_,
        [&] {
            pending.push_back(requester->request(Text{"double", 21}, milliseconds{2000},
                                                 [&](const std::optional<Text>& value) {
                                                     if (value.has_value()) {
                                                         reply = value;
                                                     }
                                                 }));
        },
        [&] { return reply.has_value(); }));

    EXPECT_EQ(reply.value_or(Text{}).number, 42);
}

}  // namespace
}  // namespace argus::bus_dds
