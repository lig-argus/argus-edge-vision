// 어댑터를 DdsBus 와 dds_types() 에 붙여, 값이 IDL 타입 토픽으로 오가는지 확인한다. 보내는 쪽과
// 받는 쪽은 서로 다른 DdsBus(DomainParticipant)다. 실제 시계와 PollPoller 로 Fast DDS 의 스레드가
// 전달하는 표본을 처리한다
#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <typeindex>
#include <utility>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/core/poll_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_server.hpp"
#include "argus/vehicle_api_bus/bus_follow_status_subscriber.hpp"
#include "argus/vehicle_api_bus/bus_vehicle_telemetry_publisher.hpp"
#include "argus/vehicle_api_bus/bus_vehicle_telemetry_subscriber.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"
#include "argus/vehicle_api_bus/dds_types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus_msg/control_mode_request.hpp"

namespace argus::vehicle_api_bus {
namespace {

using std::chrono::milliseconds;
using vehicle_api::FlightModeReply;
using vehicle_api::FollowResult;
using vehicle_api::VelocitySetpoint;

// 같은 컴퓨터의 다른 프로그램, bus_dds 테스트와 섞이지 않도록 기본값(0)이 아닌 domain 을 사용한다
constexpr std::uint32_t kTestDomain = 78;
// 해석하지 못한 요청에는 응답이 없으므로 요청하는 쪽은 timeout 뒤에 응답 없음을 받는다
constexpr milliseconds kShortTimeout{300};

vehicle_api::RequestHeader sample_header() {
    vehicle_api::RequestHeader header;
    header.id.command_id.value = 42;
    header.id.session_id = "session-a";
    header.id.seq = 7;
    header.id.fc_epoch = 3;
    header.valid_until_monotonic_ns = 123'456'789'000;
    return header;
}

// dds_types() 의 표에서 control/mode_request 요청의 to_idl 만, action 을 UNSPECIFIED 로 보내는
// 함수로 바꾼 표. 어댑터로는 해석할 수 없는 요청을 보낼 수 없으므로 요청하는 쪽의 bus 에 이 표를
// 준다
bus_dds::DdsTypeTable with_unspecified_mode_action() {
    bus_dds::DdsTypeTable table = dds_types();
    table.at(std::type_index(typeid(FlightModeRequest))).to_idl = [](const void* /*value*/,
                                                                     void* sample) {
        static_cast<idl::ControlModeRequest*>(sample)->action(
            idl::ModeAction::MODE_ACTION_UNSPECIFIED);
        return true;
    };
    return table;
}

class DdsEndToEndTest : public ::testing::Test {
protected:
    void SetUp() override {
        sender_ = bus_dds::DdsBus::create(poller_, poller_, clock_, kTestDomain, dds_types(), {});
        receiver_ = bus_dds::DdsBus::create(poller_, poller_, clock_, kTestDomain, dds_types(), {});
        ASSERT_NE(sender_, nullptr);
        ASSERT_NE(receiver_, nullptr);
    }

    // condition 이 참이 될 때까지 step 과 이벤트 처리를 반복한다 (최대 약 4 초). DDS 의 발견이
    // 끝나기 전에 보낸 표본은 전달되지 않으므로, 보내는 동작을 step 에 넣어 받을 때까지 다시 보낸다
    bool pump_until(const std::function<void()>& step, const std::function<bool()>& condition) {
        constexpr std::int32_t kMaxRounds = 400;
        for (std::int32_t round = 0; round < kMaxRounds && !condition(); ++round) {
            if (step) {
                step();
            }
            poller_.process_events(milliseconds{10}, milliseconds{50});
        }
        return condition();
    }

    core::SteadyClock clock_;
    core::PollPoller poller_{clock_};
    std::unique_ptr<bus_dds::DdsBus> sender_;
    std::unique_ptr<bus_dds::DdsBus> receiver_;
};

TEST_F(DdsEndToEndTest, SetpointReachesSubscriber) {
    const auto publisher = BusVelocitySetpointPublisher::create(*sender_);
    const auto subscriber = BusVelocitySetpointSubscriber::create(*receiver_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<VelocitySetpoint> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const VelocitySetpoint& s) { received.push_back(s); });
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.yaw_rate_rad_s = -0.5;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;

    ASSERT_TRUE(pump_until([&] { publisher->send(setpoint); }, [&] { return !received.empty(); }));

    EXPECT_DOUBLE_EQ(received[0].forward_m_s, 1.5);
    EXPECT_DOUBLE_EQ(received[0].yaw_rate_rad_s, -0.5);
    EXPECT_EQ(received[0].valid_until_monotonic_ns, 123'456'789'000);
}

TEST_F(DdsEndToEndTest, TelemetryReachesSubscriber) {
    const auto publisher = BusVehicleTelemetryPublisher::create(*sender_);
    const auto subscriber = BusVehicleTelemetrySubscriber::create(*receiver_, clock_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<vehicle_api::FlightModeEvent> events;
    const core::Subscription subscription = subscriber->subscribe_flight_mode_events(
        [&](const vehicle_api::FlightModeEvent& event) { events.push_back(event); });
    vehicle_api::Attitude attitude;
    attitude.yaw_rad = 1.5;
    attitude.fc_time_monotonic_ns = 4'900'000'000;
    vehicle_api::VehicleState state;
    vehicle_api::FlightStatus status;
    status.mode = vehicle_api::FlightMode::Offboard;
    status.armed = true;
    state.status = status;
    state.downward_distance = vehicle_api::DownwardDistance{2.5, 11, 2};
    vehicle_api::FlightModeEvent event;
    event.mode = vehicle_api::FlightMode::Hold;
    event.request_command_id = vehicle_api::CommandId{42};

    const auto received_all = [&] {
        const std::optional<vehicle_api::VehicleState> latest = subscriber->latest();
        return latest.has_value() && latest->attitude.has_value() && latest->status.has_value() &&
               !events.empty();
    };
    ASSERT_TRUE(pump_until(
        [&] {
            publisher->publish_attitude(attitude);
            publisher->publish_state(state);
            publisher->publish_mode_event(event);
        },
        received_all));

    const vehicle_api::VehicleState latest =
        subscriber->latest().value_or(vehicle_api::VehicleState{});
    const vehicle_api::Attitude received_attitude =
        latest.attitude.value_or(vehicle_api::Attitude{});
    EXPECT_DOUBLE_EQ(received_attitude.yaw_rad, 1.5);
    EXPECT_EQ(received_attitude.fc_time_monotonic_ns, std::optional<std::int64_t>{4'900'000'000});
    const vehicle_api::FlightStatus received_status =
        latest.status.value_or(vehicle_api::FlightStatus{});
    EXPECT_EQ(received_status.mode, vehicle_api::FlightMode::Offboard);
    EXPECT_TRUE(received_status.armed);
    ASSERT_TRUE(latest.downward_distance.has_value());
    EXPECT_DOUBLE_EQ(latest.downward_distance.value_or(vehicle_api::DownwardDistance{}).distance_m,
                     2.5);
    EXPECT_FALSE(latest.local.has_value());
    EXPECT_EQ(events[0].mode, vehicle_api::FlightMode::Hold);
    EXPECT_EQ(events[0].request_command_id,
              std::optional<vehicle_api::CommandId>{vehicle_api::CommandId{42}});
}

TEST_F(DdsEndToEndTest, FollowStatusReachesSubscriber) {
    // 추종 제어의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
    const auto publisher = bus::make_publisher<vehicle_api::FollowStatus>(
        *sender_, protocol::kFollowStatus, kFollowStatusType, protocol::kFollowStatusQueueDepth);
    const auto subscriber = BusFollowStatusSubscriber::create(*receiver_, clock_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<vehicle_api::ReceivedFollowStatus> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const vehicle_api::ReceivedFollowStatus& r) { received.push_back(r); });
    vehicle_api::FollowStatus status;
    status.mission_state = vehicle_api::MissionState::ScanActive;
    vehicle_api::FollowCommandResult result;
    result.command_id = vehicle_api::CommandId{42};
    result.action = vehicle_api::FollowAction::Search;
    result.outcome = vehicle_api::FollowOutcome::InProgress;
    status.command_results.push_back(result);
    vehicle_api::ScanProgress scan;
    scan.progress_deg = 30.0;
    status.scan = scan;
    const std::int64_t sent_after_ns = clock_.now_monotonic_ns();

    ASSERT_TRUE(pump_until([&] { publisher->publish(status); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].status.mission_state, vehicle_api::MissionState::ScanActive);
    ASSERT_EQ(received[0].status.command_results.size(), 1U);
    EXPECT_EQ(received[0].status.command_results[0].command_id, vehicle_api::CommandId{42});
    ASSERT_TRUE(received[0].status.scan.has_value());
    EXPECT_FALSE(
        received[0].status.scan.value_or(vehicle_api::ScanProgress{}).remaining_s.has_value());
    EXPECT_GE(received[0].arrived_monotonic_ns, sent_after_ns);
    EXPECT_EQ(
        subscriber->latest().value_or(vehicle_api::ReceivedFollowStatus{}).arrived_monotonic_ns,
        received.back().arrived_monotonic_ns);
}

TEST_F(DdsEndToEndTest, FlightModeRequestReachesControlAndReplyReturns) {
    vehicle_api_testing::FakeFlightModeControl control;
    const auto server = BusFlightModeControlServer::create(*receiver_, control);
    const auto client = BusFlightModeControlClient::create(*sender_);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    std::int32_t calls = 0;
    std::optional<FlightModeReply> reply;

    // 발견이 끝나기 전에 보낸 요청은 응답 없이 timeout 이 되므로, 응답을 받을 때까지 다시 보낸다
    std::vector<core::Subscription> requests;
    ASSERT_TRUE(pump_until(
        [&] {
            if (requests.size() <= static_cast<std::size_t>(calls)) {
                requests.push_back(client->request_offboard(
                    sample_header(), kShortTimeout, [&](std::optional<FlightModeReply> received) {
                        ++calls;
                        reply = std::move(received);
                    }));
            }
        },
        [&] { return reply.has_value(); }));

    ASSERT_FALSE(control.requests().empty());
    EXPECT_EQ(control.requests()[0].kind, vehicle_api_testing::FlightModeRequestKind::Offboard);
    EXPECT_EQ(control.requests()[0].header.id, sample_header().id);
    EXPECT_TRUE(reply.value_or(FlightModeReply{}).accepted);
    EXPECT_EQ(reply.value_or(FlightModeReply{}).id,
              std::optional<vehicle_api::RequestId>{sample_header().id});
}

TEST_F(DdsEndToEndTest, FollowCommandReachesControlAndResultReturns) {
    vehicle_api_testing::FakeFollowControl control;
    const auto server = BusFollowControlServer::create(*receiver_, control);
    const auto client = BusFollowControlClient::create(*sender_);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    vehicle_api::TrackTarget target;
    target.sensor = vehicle_api::Sensor::Ir;
    target.generation = 3;
    target.class_id = 1;
    target.track_id = 77;
    target.label = "person";
    std::int32_t calls = 0;
    std::optional<FollowResult> result;

    std::vector<core::Subscription> requests;
    ASSERT_TRUE(pump_until(
        [&] {
            if (requests.size() <= static_cast<std::size_t>(calls)) {
                requests.push_back(client->track_start(sample_header(), target, kShortTimeout,
                                                       [&](std::optional<FollowResult> received) {
                                                           ++calls;
                                                           result = std::move(received);
                                                       }));
            }
        },
        [&] { return result.has_value(); }));

    ASSERT_FALSE(control.commands().empty());
    EXPECT_EQ(control.commands()[0].kind, vehicle_api_testing::FollowCommandKind::TrackStart);
    const vehicle_api::TrackTarget received_target =
        control.commands()[0].target.value_or(vehicle_api::TrackTarget{});
    EXPECT_EQ(received_target.track_id, 77U);
    EXPECT_EQ(received_target.label, "person");
    EXPECT_EQ(result.value_or(FollowResult{}).id, sample_header().id);
    EXPECT_EQ(result.value_or(FollowResult{}).outcome, vehicle_api::FollowOutcome::InProgress);
}

// 해석하지 못한 요청: control 을 부르지 않고 응답하지 않는다 (DdsBus 는 변환에 실패한 표본을
// 폐기한다). 요청하는 쪽은 timeout 뒤에 응답 없음을 받는다. 발견이 끝나기 전의 요청만 보고
// 통과하지 않도록 요청을 3번 보낸다 (3번의 timeout 동안 발견이 끝난다)
TEST_F(DdsEndToEndTest, FlightModeRequestThatCannotBeConvertedGetsNoReply) {
    const auto broken_sender = bus_dds::DdsBus::create(poller_, poller_, clock_, kTestDomain,
                                                       with_unspecified_mode_action(), {});
    ASSERT_NE(broken_sender, nullptr);
    vehicle_api_testing::FakeFlightModeControl control;
    const auto server = BusFlightModeControlServer::create(*receiver_, control);
    const auto client = BusFlightModeControlClient::create(*broken_sender);
    ASSERT_NE(server, nullptr);
    ASSERT_NE(client, nullptr);
    constexpr std::size_t kRequests = 3;
    std::vector<std::optional<FlightModeReply>> replies;

    std::vector<core::Subscription> requests;
    ASSERT_TRUE(pump_until(
        [&] {
            if (requests.size() == replies.size() && requests.size() < kRequests) {
                requests.push_back(client->request_hold(
                    sample_header(), kShortTimeout, [&](std::optional<FlightModeReply> received) {
                        replies.push_back(std::move(received));
                    }));
            }
        },
        [&] { return replies.size() == kRequests; }));

    EXPECT_TRUE(control.requests().empty());
    for (const std::optional<FlightModeReply>& reply : replies) {
        EXPECT_EQ(reply, std::nullopt);
    }
}

}  // namespace
}  // namespace argus::vehicle_api_bus
