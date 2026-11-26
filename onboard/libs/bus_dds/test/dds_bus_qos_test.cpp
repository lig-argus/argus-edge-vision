// DdsBus 의 토픽별 발행자 QoS (Deadline) 와 위반 로그 테스트.
// 로그는 Fast DDS 의 스레드가 std::cerr 에 남기므로 표준 오류를 가로채 확인한다.
// QoS 불일치는 DdsBus 를 사용하지 않는 상대(Fast DDS 의 엔티티를 직접 생성)로 만든다.
// 프로세스 사이 통신과 같게 같은 프로세스 안 전달을 끄고 전송 계층을 거친다 (프로세스 전체에
// 적용되므로 따로 실행한다)
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <fastdds/LibrarySettings.hpp>
#include <fastdds/dds/core/ReturnCode.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <memory>
#include <string>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/bus_dds/dds_topic_qos.hpp"
#include "argus/core/poll_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "dds_test_support.hpp"
#include "test_textPubSubTypes.hpp"

namespace argus::bus_dds {
namespace {

namespace dds = eprosima::fastdds::dds;
using std::chrono::milliseconds;

constexpr std::int32_t kDepth = 4;

// DdsBus 를 사용하지 않는 상대 프로그램. 같은 domain 에 Fast DDS 의 엔티티를 직접 만든다
class RawParticipant {
public:
    explicit RawParticipant(const std::string& topic_name) {
        participant_ = dds::DomainParticipantFactory::get_instance()->create_participant(
            kTestDomain, dds::PARTICIPANT_QOS_DEFAULT);
        support_.register_type(participant_);
        topic_ = participant_->create_topic(topic_name, support_.get_type_name(),
                                            dds::TOPIC_QOS_DEFAULT);
        subscriber_ = participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT);
    }

    ~RawParticipant() {
        participant_->delete_contained_entities();
        dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    RawParticipant(const RawParticipant&) = delete;
    RawParticipant& operator=(const RawParticipant&) = delete;
    RawParticipant(RawParticipant&&) = delete;
    RawParticipant& operator=(RawParticipant&&) = delete;

    // deadline 안에 표본을 요구하는 구독자
    dds::DataReader* deadline_reader(milliseconds deadline) {
        dds::DataReaderQos qos = dds::DATAREADER_QOS_DEFAULT;
        qos.reliability().kind = dds::RELIABLE_RELIABILITY_QOS;
        qos.deadline().period = dds::Duration_t(
            0, static_cast<std::uint32_t>(std::chrono::nanoseconds(deadline).count()));
        return subscriber_->create_datareader(topic_, qos);
    }

private:
    dds::TypeSupport support_{new bus_dds_test::TestTextPubSubType()};
    dds::DomainParticipant* participant_ = nullptr;
    dds::Topic* topic_ = nullptr;
    dds::Subscriber* subscriber_ = nullptr;
};

class DdsBusQosTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        eprosima::fastdds::LibrarySettings settings;
        settings.intraprocess_delivery = eprosima::fastdds::INTRAPROCESS_OFF;
        ASSERT_EQ(dds::DomainParticipantFactory::get_instance()->set_library_settings(settings),
                  dds::RETCODE_OK);
    }

    void TearDown() override {
        if (capturing_) {
            ::testing::internal::GetCapturedStderr();
        }
    }

    std::unique_ptr<DdsBus> make_bus(DdsTopicQosTable qos) {
        return DdsBus::create(poller_, poller_, clock_, kTestDomain, test_types(), std::move(qos));
    }

    // 테스트마다 다른 토픽 이름 (앞 테스트의 엔티티와 섞이지 않게)
    std::string name() const {
        return std::string("test/qos/") +
               ::testing::UnitTest::GetInstance()->current_test_info()->name();
    }

    void start_capture() {
        ::testing::internal::CaptureStderr();
        capturing_ = true;
    }

    std::string stop_capture() {
        capturing_ = false;
        return ::testing::internal::GetCapturedStderr();
    }

    // duration 동안 이벤트를 처리한다
    void pump_for(milliseconds duration) {
        const auto until = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < until) {
            poller_.process_events(milliseconds{10}, milliseconds{50});
        }
    }

    core::SteadyClock clock_;
    core::PollPoller poller_{clock_};
    bool capturing_ = false;
};

TEST_F(DdsBusQosTest, PublisherLogsDeadlineMissedWhenNotPublishingInTime) {
    const auto sender = make_bus({{name(), {milliseconds{50}}}});
    const auto receiver = make_bus({});
    const auto publisher = bus::make_publisher<Text>(*sender, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*receiver, name(), kTextType, kDepth);
    bool received = false;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const Text& /*text*/, const bus::MessageInfo& /*info*/) { received = true; });
    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"connect", 0}); }, [&] { return received; }));

    start_capture();
    pump_for(milliseconds{300});
    const std::string log = stop_capture();

    EXPECT_NE(log.find("Deadline 초과 (발행 지연): " + name()), std::string::npos) << log;
}

TEST_F(DdsBusQosTest, PublisherLogsIncompatibleSubscriberQos) {
    const auto sender = make_bus({{name(), {milliseconds{500}}}});
    const auto publisher = bus::make_publisher<Text>(*sender, name(), kTextType, kDepth);
    RawParticipant other(name());

    start_capture();
    ASSERT_NE(other.deadline_reader(milliseconds{100}),
              nullptr);  // 500 ms 를 제공하는 발행자와 불일치
    pump_for(milliseconds{1000});
    const std::string log = stop_capture();

    EXPECT_NE(log.find("구독자와 QoS 불일치: " + name()), std::string::npos) << log;
}

TEST_F(DdsBusQosTest, TopicWithoutQosLogsNothing) {
    auto sender = make_bus({});
    const auto receiver = make_bus({});
    auto publisher = bus::make_publisher<Text>(*sender, name(), kTextType, kDepth);
    const auto subscriber = bus::make_subscriber<Text>(*receiver, name(), kTextType, kDepth);
    bool received = false;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const Text& /*text*/, const bus::MessageInfo& /*info*/) { received = true; });
    ASSERT_TRUE(pump_until(
        poller_, [&] { publisher->publish(Text{"connect", 0}); }, [&] { return received; }));

    start_capture();
    pump_for(milliseconds{300});
    publisher.reset();
    sender.reset();
    pump_for(milliseconds{500});
    const std::string log = stop_capture();

    EXPECT_EQ(log.find("[bus_dds]"), std::string::npos) << log;
}

}  // namespace
}  // namespace argus::bus_dds
