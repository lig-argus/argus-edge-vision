#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "argus/bus/message.hpp"
#include "argus/bus/replier.hpp"
#include "argus/bus/requester.hpp"
#include "argus/core/subscription.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control.hpp"
#include "argus_msg/control_mode_request.pb.h"
#include "request_test_support.hpp"

namespace argus::vehicle_api_bus {
namespace {

using vehicle_api::FlightModeReply;
using vehicle_api_testing::FlightModeRequestKind;

// on_reply 가 받은 것. 불리지 않았으면 called 가 false
struct ReplyRecord {
    int calls = 0;
    std::optional<FlightModeReply> reply;
};

class FlightModeControlTest : public RequestTest {
protected:
    void SetUp() override {
        client_ = BusFlightModeControlClient::create(bus_);
        ASSERT_NE(client_, nullptr);
    }

    void start_server() {
        server_ = BusFlightModeControlServer::create(bus_, control_);
        ASSERT_NE(server_, nullptr);
    }

    vehicle_api::IFlightModeControlClient::ReplyHandler record_into(ReplyRecord& record) {
        return [&record](std::optional<FlightModeReply> reply) {
            ++record.calls;
            record.reply = std::move(reply);
        };
    }

    vehicle_api_testing::FakeFlightModeControl control_;
    std::unique_ptr<BusFlightModeControlClient> client_;
    std::unique_ptr<BusFlightModeControlServer> server_;
};

TEST_F(FlightModeControlTest, OffboardRequestReachesControlAndReplyReturns) {
    start_server();
    ReplyRecord record;

    const core::Subscription request =
        client_->request_offboard(sample_header(), kRequestTimeout, record_into(record));
    process();

    ASSERT_EQ(control_.requests().size(), 1U);
    EXPECT_EQ(control_.requests()[0].kind, FlightModeRequestKind::Offboard);
    EXPECT_EQ(control_.requests()[0].header.id, sample_header().id);
    EXPECT_EQ(control_.requests()[0].header.valid_until_monotonic_ns, 123'456'789'000);
    ASSERT_EQ(record.calls, 1);
    ASSERT_TRUE(record.reply.has_value());
    EXPECT_TRUE(record.reply.value_or(FlightModeReply{}).accepted);
    EXPECT_EQ(record.reply.value_or(FlightModeReply{}).id,
              std::optional<vehicle_api::RequestId>{sample_header().id});
}

TEST_F(FlightModeControlTest, HoldRequestReachesControl) {
    start_server();
    ReplyRecord record;

    const core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, record_into(record));
    process();

    ASSERT_EQ(control_.requests().size(), 1U);
    EXPECT_EQ(control_.requests()[0].kind, FlightModeRequestKind::Hold);
    EXPECT_EQ(record.calls, 1);
}

TEST_F(FlightModeControlTest, RejectionAndReasonReturnToTheClient) {
    start_server();
    FlightModeReply rejection;
    rejection.accepted = false;
    rejection.reason = "another writer owns FC";
    control_.set_reply(rejection);
    ReplyRecord record;

    const core::Subscription request =
        client_->request_offboard(sample_header(), kRequestTimeout, record_into(record));
    process();

    ASSERT_TRUE(record.reply.has_value());
    EXPECT_FALSE(record.reply.value_or(FlightModeReply{}).accepted);
    EXPECT_EQ(record.reply.value_or(FlightModeReply{}).reason, "another writer owns FC");
}

TEST_F(FlightModeControlTest, ReplyIsEmptyWhenNoServerAnswersWithinTimeout) {
    ReplyRecord record;

    const core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, record_into(record));
    process();
    EXPECT_EQ(record.calls, 0);
    wait_past_timeout();

    EXPECT_EQ(record.calls, 1);
    EXPECT_EQ(record.reply, std::nullopt);
}

TEST_F(FlightModeControlTest, HandlerIsNotCalledAfterRequestIsReleased) {
    start_server();
    ReplyRecord record;

    core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, record_into(record));
    request.reset();
    process();
    wait_past_timeout();

    EXPECT_EQ(record.calls, 0);
}

TEST_F(FlightModeControlTest, ClientRejectsEmptyHandler) {
    const core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, nullptr);

    EXPECT_FALSE(request.active());
}

TEST_F(FlightModeControlTest, ServerRejectsRequestThatCannotBeParsedWithoutCallingControl) {
    start_server();
    const auto requester = bus_.create_requester(protocol::kControlModeRequest);
    ASSERT_NE(requester, nullptr);
    std::optional<bus::BusMessage> raw_reply;

    const core::Subscription request = requester->request(
        broken_body(), kRequestTimeout,
        [&](std::optional<bus::BusMessage> reply) { raw_reply = std::move(reply); });
    process();

    EXPECT_TRUE(control_.requests().empty());
    ASSERT_TRUE(raw_reply.has_value());
    msg::ControlModeReply reply;
    ASSERT_TRUE(
        reply.ParseFromArray(raw_reply.value_or(bus::BusMessage{}).body.data(),
                             static_cast<int>(raw_reply.value_or(bus::BusMessage{}).body.size())));
    EXPECT_FALSE(reply.accepted());
    EXPECT_FALSE(reply.has_id());
    EXPECT_EQ(reply.reason(), "malformed request");
}

TEST_F(FlightModeControlTest, ServerRejectsRequestWithoutAction) {
    start_server();
    const auto requester = bus_.create_requester(protocol::kControlModeRequest);
    ASSERT_NE(requester, nullptr);
    msg::ControlModeRequest without_action;
    without_action.mutable_header()->mutable_id()->set_command_id(1);
    bus::Bytes body(without_action.ByteSizeLong());
    EXPECT_TRUE(without_action.SerializeToArray(body.data(), static_cast<int>(body.size())));
    std::optional<bus::BusMessage> raw_reply;

    const core::Subscription request = requester->request(
        body, kRequestTimeout,
        [&](std::optional<bus::BusMessage> reply) { raw_reply = std::move(reply); });
    process();

    EXPECT_TRUE(control_.requests().empty());
    EXPECT_TRUE(raw_reply.has_value());
}

TEST_F(FlightModeControlTest, ClientTreatsReplyThatCannotBeParsedAsNoReply) {
    const auto replier = bus_.create_replier(protocol::kControlModeRequest);
    ASSERT_NE(replier, nullptr);
    const core::Subscription serving =
        replier->serve([](const bus::BusMessage&) { return broken_body(); });
    ReplyRecord record;

    const core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, record_into(record));
    process();

    EXPECT_EQ(record.calls, 1);
    EXPECT_EQ(record.reply, std::nullopt);
}

TEST_F(FlightModeControlTest, ServerStopsAnsweringAfterItIsDestroyed) {
    start_server();
    server_.reset();
    ReplyRecord record;

    const core::Subscription request =
        client_->request_hold(sample_header(), kRequestTimeout, record_into(record));
    process();
    wait_past_timeout();

    EXPECT_TRUE(control_.requests().empty());
    EXPECT_EQ(record.calls, 1);
    EXPECT_EQ(record.reply, std::nullopt);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
