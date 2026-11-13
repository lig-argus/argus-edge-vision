#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>

#include "argus/bus/message.hpp"
#include "argus/bus/replier.hpp"
#include "argus/bus/requester.hpp"
#include "argus/core/subscription.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_server.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus_msg/follow_command.pb.h"
#include "request_test_support.hpp"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::FollowOutcome;
using vehicle_api::FollowResult;
using vehicle_api_testing::FollowCommandKind;

// on_result 가 받은 것
struct ResultRecord {
    int calls = 0;
    std::optional<FollowResult> result;
};

bus::Bytes to_body(const msg::FollowCommand& message) {
    bus::Bytes body(message.ByteSizeLong());
    EXPECT_TRUE(message.SerializeToArray(body.data(), static_cast<int>(body.size())));
    return body;
}

class FollowControlTest : public RequestTest {
protected:
    void SetUp() override {
        client_ = BusFollowControlClient::create(bus_);
        ASSERT_NE(client_, nullptr);
    }

    void start_server() {
        server_ = BusFollowControlServer::create(bus_, control_);
        ASSERT_NE(server_, nullptr);
    }

    vehicle_api::IFollowControlClient::ResultHandler record_into(ResultRecord& record) {
        return [&record](std::optional<FollowResult> result) {
            ++record.calls;
            record.result = std::move(result);
        };
    }

    // 본문을 직접 보내고 받은 응답을 FollowResult 메시지로 파싱해 반환한다
    std::optional<msg::FollowResult> send_raw(const bus::Bytes& body) {
        const auto requester = bus_.create_requester(protocol::kFollowCommand);
        std::optional<bus::BusMessage> raw_reply;
        const core::Subscription request = requester->request(
            body, kRequestTimeout,
            [&](std::optional<bus::BusMessage> reply) { raw_reply = std::move(reply); });
        process();
        if (!raw_reply) {
            return std::nullopt;
        }
        msg::FollowResult result;
        if (!result.ParseFromArray(raw_reply->body.data(),
                                   static_cast<int>(raw_reply->body.size()))) {
            return std::nullopt;
        }
        return result;
    }

    vehicle_api_testing::FakeFollowControl control_;
    std::unique_ptr<BusFollowControlClient> client_;
    std::unique_ptr<BusFollowControlServer> server_;
};

TEST_F(FollowControlTest, TrackStartReachesControlAndResultReturns) {
    start_server();
    vehicle_api::TrackTarget target;
    target.sensor = vehicle_api::Sensor::Ir;
    target.generation = 5;
    target.class_id = 2;
    target.track_id = 17;
    target.label = "person";
    ResultRecord record;

    const core::Subscription request =
        client_->track_start(sample_header(), target, kRequestTimeout, record_into(record));
    process();

    ASSERT_EQ(control_.commands().size(), 1U);
    EXPECT_EQ(control_.commands()[0].kind, FollowCommandKind::TrackStart);
    EXPECT_EQ(control_.commands()[0].header.id, sample_header().id);
    EXPECT_EQ(control_.commands()[0].header.valid_until_monotonic_ns, 123'456'789'000);
    const std::optional<vehicle_api::TrackTarget> received = control_.commands()[0].target;
    ASSERT_TRUE(received.has_value());
    const vehicle_api::TrackTarget received_value = received.value_or(vehicle_api::TrackTarget{});
    EXPECT_EQ(received_value.sensor, vehicle_api::Sensor::Ir);
    EXPECT_EQ(received_value.generation, 5U);
    EXPECT_EQ(received_value.class_id, 2);
    EXPECT_EQ(received_value.track_id, 17U);
    EXPECT_EQ(received_value.label, "person");
    ASSERT_EQ(record.calls, 1);
    ASSERT_TRUE(record.result.has_value());
    EXPECT_EQ(record.result.value_or(FollowResult{}).id, sample_header().id);
    EXPECT_EQ(record.result.value_or(FollowResult{}).outcome, FollowOutcome::InProgress);
    EXPECT_TRUE(record.result.value_or(FollowResult{}).request_accepted);
}

TEST_F(FollowControlTest, StopCarriesTheReason) {
    start_server();
    ResultRecord record;

    const core::Subscription request = client_->stop(
        sample_header(), vehicle_api::StopReason::LinkLoss, kRequestTimeout, record_into(record));
    process();

    ASSERT_EQ(control_.commands().size(), 1U);
    EXPECT_EQ(control_.commands()[0].kind, FollowCommandKind::Stop);
    EXPECT_EQ(control_.commands()[0].reason,
              std::optional<vehicle_api::StopReason>{vehicle_api::StopReason::LinkLoss});
    EXPECT_EQ(record.calls, 1);
}

TEST_F(FollowControlTest, ScanStartKeepsEmptyItemsEmpty) {
    start_server();
    vehicle_api::ScanRequest scan;
    scan.yaw_rate_rad_s = 0.3;  // 나머지 2개는 비어 있음
    ResultRecord record;

    const core::Subscription request =
        client_->scan_start(sample_header(), scan, kRequestTimeout, record_into(record));
    process();

    ASSERT_EQ(control_.commands().size(), 1U);
    EXPECT_EQ(control_.commands()[0].kind, FollowCommandKind::ScanStart);
    const std::optional<vehicle_api::ScanRequest> received = control_.commands()[0].scan;
    ASSERT_TRUE(received.has_value());
    const vehicle_api::ScanRequest received_value = received.value_or(vehicle_api::ScanRequest{});
    EXPECT_EQ(received_value.yaw_range_rad, std::nullopt);
    const std::optional<double> yaw_rate_rad_s = received_value.yaw_rate_rad_s;
    ASSERT_TRUE(yaw_rate_rad_s.has_value());
    EXPECT_DOUBLE_EQ(yaw_rate_rad_s.value_or(0.0), 0.3);
    EXPECT_EQ(received_value.timeout_s, std::nullopt);
}

TEST_F(FollowControlTest, RejectionAndReasonReturnToTheClient) {
    start_server();
    FollowResult rejection;
    rejection.outcome = FollowOutcome::Rejected;
    rejection.request_accepted = false;
    rejection.reason = "target not found";
    control_.set_result(rejection);
    ResultRecord record;

    const core::Subscription request = client_->track_start(
        sample_header(), vehicle_api::TrackTarget{}, kRequestTimeout, record_into(record));
    process();

    ASSERT_TRUE(record.result.has_value());
    EXPECT_EQ(record.result.value_or(FollowResult{}).outcome, FollowOutcome::Rejected);
    EXPECT_FALSE(record.result.value_or(FollowResult{}).request_accepted);
    EXPECT_EQ(record.result.value_or(FollowResult{}).reason, "target not found");
}

TEST_F(FollowControlTest, ResultIsEmptyWhenNoServerAnswersWithinTimeout) {
    ResultRecord record;

    const core::Subscription request = client_->stop(
        sample_header(), vehicle_api::StopReason::Operator, kRequestTimeout, record_into(record));
    process();
    EXPECT_EQ(record.calls, 0);
    wait_past_timeout();

    EXPECT_EQ(record.calls, 1);
    EXPECT_EQ(record.result, std::nullopt);
}

TEST_F(FollowControlTest, HandlerIsNotCalledAfterRequestIsReleased) {
    start_server();
    ResultRecord record;

    core::Subscription request = client_->stop(sample_header(), vehicle_api::StopReason::Operator,
                                               kRequestTimeout, record_into(record));
    request.reset();
    process();
    wait_past_timeout();

    EXPECT_EQ(record.calls, 0);
}

TEST_F(FollowControlTest, ClientRejectsEmptyHandler) {
    const core::Subscription request =
        client_->stop(sample_header(), vehicle_api::StopReason::Operator, kRequestTimeout, nullptr);

    EXPECT_FALSE(request.active());
}

TEST_F(FollowControlTest, ServerRejectsRequestThatCannotBeParsedWithoutCallingControl) {
    start_server();

    const std::optional<msg::FollowResult> result = send_raw(broken_body());

    EXPECT_TRUE(control_.commands().empty());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value_or(msg::FollowResult{}).outcome(), msg::FOLLOW_OUTCOME_REJECTED);
    EXPECT_FALSE(result.value_or(msg::FollowResult{}).request_accepted());
    EXPECT_EQ(result.value_or(msg::FollowResult{}).reason(), "malformed request");
}

TEST_F(FollowControlTest, ServerRejectsStopWithUnspecifiedReason) {
    start_server();
    msg::FollowCommand command;
    command.mutable_header()->mutable_id()->set_command_id(1);
    command.mutable_stop();  // reason 을 지정하지 않음

    const std::optional<msg::FollowResult> result = send_raw(to_body(command));

    EXPECT_TRUE(control_.commands().empty());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value_or(msg::FollowResult{}).outcome(), msg::FOLLOW_OUTCOME_REJECTED);
}

TEST_F(FollowControlTest, ClientTreatsResultWithoutOutcomeAsNoReply) {
    const auto replier = bus_.create_replier(protocol::kFollowCommand);
    ASSERT_NE(replier, nullptr);
    const core::Subscription serving = replier->serve([](const bus::BusMessage&) {
        msg::FollowResult without_outcome;
        without_outcome.mutable_id()->set_command_id(42);
        bus::Bytes body(without_outcome.ByteSizeLong());
        EXPECT_TRUE(without_outcome.SerializeToArray(body.data(), static_cast<int>(body.size())));
        return body;
    });
    ResultRecord record;

    const core::Subscription request = client_->stop(
        sample_header(), vehicle_api::StopReason::Operator, kRequestTimeout, record_into(record));
    process();

    EXPECT_EQ(record.calls, 1);
    EXPECT_EQ(record.result, std::nullopt);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
