// 어댑터를 ZmqBus 와 zmq_codecs() 에 붙여, 값이 오가는지와 전송 바이트(multipart 구성과 protobuf
// 본문)를 확인한다. 전송 바이트는 ZmqBus 를 거치지 않는 소켓으로 직접 주고받아 확인한다
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <typeindex>
#include <utility>
#include <vector>
#include <zmq.hpp>

#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_server.hpp"
#include "argus/vehicle_api_bus/bus_follow_status_subscriber.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus/vehicle_api_bus/zmq_codecs.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus_msg/control_mode_request.pb.h"
#include "argus_msg/control_setpoint.pb.h"
#include "argus_msg/follow_command.pb.h"

namespace argus::vehicle_api_bus {
namespace {

using std::chrono::milliseconds;
using vehicle_api::FlightModeReply;
using vehicle_api::FollowResult;
using vehicle_api::VelocitySetpoint;

constexpr milliseconds kBudget{1000};
constexpr milliseconds kTimeout{1000};

// 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없는 본문 (파싱 실패)
std::string_view broken_body() { return "\x0A\x05"; }

// zmq_codecs() 의 표에서 요청 타입 T 의 encode 만, 파싱할 수 없는 본문을 만드는 함수로 바꾼 표.
// 어댑터로는 깨진 요청을 보낼 수 없으므로 요청하는 쪽의 bus 에 이 표를 준다
template <typename T>
bus_zmq::ZmqCodecTable with_broken_request() {
    bus_zmq::ZmqCodecTable table = zmq_codecs();
    table.at(std::type_index(typeid(T))).encode =
        [](const void* /*value*/, std::uint64_t /*sequence*/, bus_zmq::Bytes& body,
           bus_zmq::Bytes& /*binary*/) {
            const std::string_view broken = broken_body();
            body.assign(broken.begin(), broken.end());
            return true;
        };
    return table;
}

vehicle_api::RequestHeader sample_header() {
    vehicle_api::RequestHeader header;
    header.id.command_id.value = 42;
    header.id.session_id = "session-a";
    header.id.seq = 7;
    header.id.fc_epoch = 3;
    header.valid_until_monotonic_ns = 123'456'789'000;
    return header;
}

// 시각은 FakeClock 으로 정한다. 테스트가 시각을 옮기지 않으므로 요청의 timeout 은 지나지 않는다
class ZmqEndToEndTest : public ::testing::Test {
protected:
    // 테스트마다 다른 inproc 엔드포인트를 만든다 (같은 context 안에서 이름이 겹치지 않게)
    std::string endpoint(const std::string& label) const {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        return std::string("inproc://") + info->name() + "_" + label;
    }

    // condition 이 참이 될 때까지 이벤트를 처리한다. 끝까지 참이 되지 않으면 false
    bool pump_until(const std::function<bool()>& condition) {
        for (std::int32_t i = 0; i < 400 && !condition(); ++i) {
            poller_.process_events(milliseconds{5}, kBudget);
        }
        return condition();
    }

    // 요청-응답 이름의 엔드포인트에 REQ 소켓으로 본문을 직접 보내고 응답의 프레임들을 받는다.
    // 응답이 오지 않으면 빈 목록
    std::vector<std::string> request_raw(const std::string& address, std::string_view name,
                                         std::string_view body) {
        zmq::socket_t raw(context_, zmq::socket_type::req);
        raw.set(zmq::sockopt::linger, 0);
        raw.set(zmq::sockopt::rcvtimeo, 0);
        raw.connect(address);
        raw.send(zmq::buffer(name), zmq::send_flags::sndmore);
        raw.send(zmq::buffer(body), zmq::send_flags::none);
        // 응답을 다 읽은 뒤에는 다시 읽지 않는다 (REQ 소켓은 응답을 받은 뒤에 다시 읽으면 오류)
        std::vector<std::string> frames;
        bool complete = false;
        pump_until([&] {
            zmq::message_t frame;
            while (!complete && raw.recv(frame).has_value()) {
                frames.push_back(frame.to_string());
                complete = !frame.more();
            }
            return complete;
        });
        return complete ? frames : std::vector<std::string>{};
    }

    zmq::context_t context_;
    core_testing::FakeClock clock_;
    bus_zmq::ZmqPoller poller_{clock_};
};

TEST_F(ZmqEndToEndTest, SetpointReachesSubscriberWithoutLoss) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kControlSetpoint), endpoint("pub")}}, zmq_codecs());
    const auto publisher = BusVelocitySetpointPublisher::create(bus);
    const auto subscriber = BusVelocitySetpointSubscriber::create(bus);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<VelocitySetpoint> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint& s) { received.push_back(s); });
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;

    // 연결이 끝나기 전의 메시지는 도착하지 않으므로 3개를 받을 때까지 보낸다
    for (std::int32_t i = 0; i < 400 && received.size() < 3; ++i) {
        publisher->send(setpoint);
        poller_.process_events(milliseconds{5}, kBudget);
    }

    ASSERT_GE(received.size(), 3U);
    EXPECT_DOUBLE_EQ(received[0].forward_m_s, 1.5);
    EXPECT_EQ(received[0].valid_until_monotonic_ns, 123'456'789'000);
    // 처음 받은 뒤로는 번호가 이어지므로 손실이 없다
    EXPECT_EQ(subscriber->lost_count(), 0U);
}

TEST_F(ZmqEndToEndTest, SetpointIsSentAsTopicNameAndControlSetpointBody) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kControlSetpoint), endpoint("pub")}}, zmq_codecs());
    const auto publisher = BusVelocitySetpointPublisher::create(bus);
    ASSERT_NE(publisher, nullptr);
    zmq::socket_t raw(context_, zmq::socket_type::sub);
    raw.set(zmq::sockopt::subscribe, "");
    raw.set(zmq::sockopt::rcvtimeo, 5);
    raw.connect(endpoint("pub"));
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.yaw_rate_rad_s = -0.5;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;

    std::vector<std::string> frames;
    std::uint64_t sent = 0;
    for (std::int32_t i = 0; i < 400 && frames.empty(); ++i) {
        publisher->send(setpoint);
        ++sent;
        zmq::message_t frame;
        while (raw.recv(frame).has_value()) {
            frames.push_back(frame.to_string());
            if (!frame.more()) {
                break;
            }
        }
    }

    // 본문은 발행 번호를 seq 에 실은 ControlSetpoint 메시지와 같은 바이트다
    msg::ControlSetpoint expected;
    expected.set_seq(sent);
    expected.set_forward_m_s(1.5);
    expected.set_yaw_rate_rad_s(-0.5);
    expected.set_valid_until_monotonic_ns(123'456'789'000);
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], protocol::kControlSetpoint);
    EXPECT_EQ(frames[1], expected.SerializeAsString());
}

TEST_F(ZmqEndToEndTest, FollowStatusReachesSubscriberWithArrivalTime) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kFollowStatus), endpoint("pub")}}, zmq_codecs());
    // 추종 제어의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
    const auto publisher = bus::make_publisher<vehicle_api::FollowStatus>(
        bus, protocol::kFollowStatus, kFollowStatusType, protocol::kFollowStatusQueueDepth);
    const auto subscriber = BusFollowStatusSubscriber::create(bus, clock_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    clock_.advance_ns(5'000'000);
    vehicle_api::FollowStatus status;
    status.mission_state = vehicle_api::MissionState::FollowActive;
    status.controller_session = "controller-f";

    // 연결이 끝나기 전의 메시지는 도착하지 않으므로 받을 때까지 보낸다
    for (std::int32_t i = 0; i < 400 && !subscriber->latest(); ++i) {
        publisher->publish(status);
        poller_.process_events(milliseconds{5}, kBudget);
    }

    ASSERT_TRUE(subscriber->latest().has_value());
    const vehicle_api::ReceivedFollowStatus latest =
        subscriber->latest().value_or(vehicle_api::ReceivedFollowStatus{});
    EXPECT_EQ(latest.status.mission_state, vehicle_api::MissionState::FollowActive);
    EXPECT_EQ(latest.status.controller_session, "controller-f");
    EXPECT_EQ(latest.arrived_monotonic_ns, 5'000'000);
}

TEST_F(ZmqEndToEndTest, FlightModeRequestReachesControlAndReplyReturns) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kControlModeRequest), endpoint("rep")}},
                        zmq_codecs());
    vehicle_api_testing::FakeFlightModeControl control;
    const auto server = BusFlightModeControlServer::create(bus, control);
    const auto client = BusFlightModeControlClient::create(bus);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FlightModeReply> reply;

    const core::Subscription request = client->request_offboard(
        sample_header(), kTimeout, [&](std::optional<FlightModeReply> received) {
            ++calls;
            reply = std::move(received);
        });
    ASSERT_TRUE(pump_until([&] { return calls > 0; }));

    ASSERT_EQ(control.requests().size(), 1U);
    EXPECT_EQ(control.requests()[0].kind, vehicle_api_testing::FlightModeRequestKind::Offboard);
    EXPECT_EQ(control.requests()[0].header.id, sample_header().id);
    ASSERT_TRUE(reply.has_value());
    EXPECT_TRUE(reply.value_or(FlightModeReply{}).accepted);
    EXPECT_EQ(reply.value_or(FlightModeReply{}).id,
              std::optional<vehicle_api::RequestId>{sample_header().id});
}

// 해석하지 못한 요청: control 을 부르지 않고 빈 본문으로 응답한다 (ZmqBus 의 규칙)
TEST_F(ZmqEndToEndTest, FlightModeServerRepliesEmptyBodyToRequestThatCannotBeParsed) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kControlModeRequest), endpoint("rep")}},
                        zmq_codecs());
    vehicle_api_testing::FakeFlightModeControl control;
    const auto server = BusFlightModeControlServer::create(bus, control);
    ASSERT_NE(server, nullptr);

    const std::vector<std::string> frames =
        request_raw(endpoint("rep"), protocol::kControlModeRequest, broken_body());

    EXPECT_TRUE(control.requests().empty());
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], protocol::kControlModeRequest);
    EXPECT_TRUE(frames[1].empty());
}

// 요청하는 쪽은 timeout 을 기다리지 않고 "응답 없음" 을 받는다 (빈 본문의 응답은 id 가 없어 해석
// 실패). 요청하는 쪽의 bus 는 요청을 깨진 본문으로 보내는 변환 표를 쓴다
TEST_F(ZmqEndToEndTest, FlightModeClientReportsNoReplyWhenServerCannotParseTheRequest) {
    const bus_zmq::ZmqEndpointTable endpoints{
        {std::string(protocol::kControlModeRequest), endpoint("rep")}};
    bus_zmq::ZmqBus server_bus(context_, poller_, clock_, endpoints, zmq_codecs());
    bus_zmq::ZmqBus client_bus(context_, poller_, clock_, endpoints,
                               with_broken_request<FlightModeRequest>());
    vehicle_api_testing::FakeFlightModeControl control;
    const auto server = BusFlightModeControlServer::create(server_bus, control);
    const auto client = BusFlightModeControlClient::create(client_bus);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FlightModeReply> reply;

    const core::Subscription request = client->request_offboard(
        sample_header(), kTimeout, [&](std::optional<FlightModeReply> received) {
            ++calls;
            reply = std::move(received);
        });
    // 시각을 옮기지 않으므로 여기서 불린 것은 timeout 때문이 아니다
    ASSERT_TRUE(pump_until([&] { return calls > 0; }));

    EXPECT_TRUE(control.requests().empty());
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(reply, std::nullopt);
}

TEST_F(ZmqEndToEndTest, FlightModeClientReportsNoReplyForReplyThatCannotBeParsed) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kControlModeRequest), endpoint("rep")}},
                        zmq_codecs());
    // 응답하는 쪽은 ZmqBus 를 거치지 않는 REP 소켓이다
    zmq::socket_t raw(context_, zmq::socket_type::rep);
    raw.set(zmq::sockopt::linger, 0);
    raw.set(zmq::sockopt::rcvtimeo, 0);
    raw.bind(endpoint("rep"));
    const auto client = BusFlightModeControlClient::create(bus);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FlightModeReply> reply = FlightModeReply{};

    const core::Subscription request = client->request_hold(
        sample_header(), kTimeout, [&](std::optional<FlightModeReply> received) {
            ++calls;
            reply = std::move(received);
        });
    // 요청을 읽어 프레임 구성과 본문을 확인하고, 파싱할 수 없는 본문으로 응답한다
    std::vector<std::string> request_frames;
    bool replied = false;
    ASSERT_TRUE(pump_until([&] {
        zmq::message_t frame;
        while (!replied && raw.recv(frame).has_value()) {
            request_frames.push_back(frame.to_string());
            if (!frame.more()) {
                raw.send(zmq::buffer(protocol::kControlModeRequest), zmq::send_flags::sndmore);
                raw.send(zmq::buffer(broken_body()), zmq::send_flags::none);
                replied = true;
            }
        }
        return calls > 0;
    }));

    msg::ControlModeRequest expected;
    msg::RequestId& id = *expected.mutable_header()->mutable_id();
    id.set_command_id(42);
    id.set_session_id("session-a");
    id.set_seq(7);
    id.set_fc_epoch(3);
    expected.mutable_header()->set_valid_until_monotonic_ns(123'456'789'000);
    expected.mutable_hold();
    ASSERT_EQ(request_frames.size(), 2U);
    EXPECT_EQ(request_frames[0], protocol::kControlModeRequest);
    EXPECT_EQ(request_frames[1], expected.SerializeAsString());
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(reply, std::nullopt);  // timeout 을 기다리지 않고 응답이 없는 것으로 알린다
}

TEST_F(ZmqEndToEndTest, FollowCommandReachesControlAndResultReturns) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kFollowCommand), endpoint("rep")}}, zmq_codecs());
    vehicle_api_testing::FakeFollowControl control;
    const auto server = BusFollowControlServer::create(bus, control);
    const auto client = BusFollowControlClient::create(bus);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FollowResult> result;

    const core::Subscription request =
        client->stop(sample_header(), vehicle_api::StopReason::LinkLoss, kTimeout,
                     [&](std::optional<FollowResult> received) {
                         ++calls;
                         result = std::move(received);
                     });
    ASSERT_TRUE(pump_until([&] { return calls > 0; }));

    ASSERT_EQ(control.commands().size(), 1U);
    EXPECT_EQ(control.commands()[0].kind, vehicle_api_testing::FollowCommandKind::Stop);
    EXPECT_EQ(control.commands()[0].reason,
              std::optional<vehicle_api::StopReason>{vehicle_api::StopReason::LinkLoss});
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value_or(FollowResult{}).id, sample_header().id);
    EXPECT_EQ(result.value_or(FollowResult{}).outcome, vehicle_api::FollowOutcome::InProgress);
}

TEST_F(ZmqEndToEndTest, FollowServerRepliesEmptyBodyToRequestThatCannotBeParsed) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kFollowCommand), endpoint("rep")}}, zmq_codecs());
    vehicle_api_testing::FakeFollowControl control;
    const auto server = BusFollowControlServer::create(bus, control);
    ASSERT_NE(server, nullptr);

    const std::vector<std::string> frames =
        request_raw(endpoint("rep"), protocol::kFollowCommand, broken_body());

    EXPECT_TRUE(control.commands().empty());
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], protocol::kFollowCommand);
    EXPECT_TRUE(frames[1].empty());
}

TEST_F(ZmqEndToEndTest, FollowServerRepliesEmptyBodyToStopWithUnspecifiedReason) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kFollowCommand), endpoint("rep")}}, zmq_codecs());
    vehicle_api_testing::FakeFollowControl control;
    const auto server = BusFollowControlServer::create(bus, control);
    ASSERT_NE(server, nullptr);
    msg::FollowCommand command;
    command.mutable_header()->mutable_id()->set_command_id(1);
    command.mutable_stop();  // reason 을 지정하지 않음

    const std::vector<std::string> frames =
        request_raw(endpoint("rep"), protocol::kFollowCommand, command.SerializeAsString());

    EXPECT_TRUE(control.commands().empty());
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0], protocol::kFollowCommand);
    EXPECT_TRUE(frames[1].empty());
}

// 요청하는 쪽은 빈 본문 응답을 해석하지 못하므로 timeout 을 기다리지 않고 응답이 없는 것으로 안다
TEST_F(ZmqEndToEndTest, FollowClientReportsNoReplyWhenServerCannotParseTheRequest) {
    const bus_zmq::ZmqEndpointTable endpoints{
        {std::string(protocol::kFollowCommand), endpoint("rep")}};
    bus_zmq::ZmqBus server_bus(context_, poller_, clock_, endpoints, zmq_codecs());
    bus_zmq::ZmqBus client_bus(context_, poller_, clock_, endpoints,
                               with_broken_request<FollowCommandRequest>());
    vehicle_api_testing::FakeFollowControl control;
    const auto server = BusFollowControlServer::create(server_bus, control);
    const auto client = BusFollowControlClient::create(client_bus);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FollowResult> result = FollowResult{};

    const core::Subscription request =
        client->stop(sample_header(), vehicle_api::StopReason::Operator, kTimeout,
                     [&](std::optional<FollowResult> received) {
                         ++calls;
                         result = std::move(received);
                     });
    // 시각을 옮기지 않으므로 여기서 불린 것은 timeout 때문이 아니다
    ASSERT_TRUE(pump_until([&] { return calls > 0; }));

    EXPECT_TRUE(control.commands().empty());
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(result, std::nullopt);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
