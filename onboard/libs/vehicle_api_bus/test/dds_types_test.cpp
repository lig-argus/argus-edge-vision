// dds_types() 의 변환 테스트. IDL 타입이 .proto 의 필드를 빠짐없이 나르는지 확인한다: 값을 IDL
// 표본으로 바꿨다가 되돌린 결과를 zmq_codecs() 로 protobuf 본문으로 만들어, 원래 값의 본문과
// 바이트가 같은지 비교한다
#include "argus/vehicle_api_bus/dds_types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <typeindex>
#include <variant>

#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus/vehicle_api_bus/zmq_codecs.hpp"
#include "argus_msg/control_mode_request.hpp"
#include "argus_msg/control_setpoint.hpp"
#include "argus_msg/fc_attitude.hpp"
#include "argus_msg/fc_mode_event.hpp"
#include "argus_msg/fc_state.hpp"
#include "argus_msg/follow_command.hpp"

namespace argus::vehicle_api_bus {
namespace {

using bus_zmq::Bytes;
using vehicle_api::FollowOutcome;
using vehicle_api::FollowResult;

class DdsTypesTest : public ::testing::Test {
protected:
    // value 를 IDL 표본(Idl)으로 바꿨다가 되돌린다. 어느 쪽이든 실패하면 nullopt
    template <typename T, typename Idl>
    std::optional<T> through_idl(const T& value) const {
        const bus_dds::DdsType& type = dds_.at(std::type_index(typeid(T)));
        Idl sample;
        if (!type.to_idl(&value, &sample)) {
            return std::nullopt;
        }
        return from_sample<T>(sample);
    }

    // 표본을 값으로 바꾼다. 실패하면 nullopt
    template <typename T, typename Idl>
    std::optional<T> from_sample(const Idl& sample) const {
        const bus_dds::DdsType& type = dds_.at(std::type_index(typeid(T)));
        T value{};
        if (!type.from_idl(&sample, &value)) {
            return std::nullopt;
        }
        return value;
    }

    // value 의 protobuf 본문 (발행 번호 1)
    template <typename T>
    Bytes proto_body(const T& value) const {
        Bytes body;
        Bytes binary;
        EXPECT_TRUE(zmq_.at(std::type_index(typeid(T))).encode(&value, 1, body, binary));
        return body;
    }

    // value 를 IDL 로 왕복시킨 값의 본문이 value 의 본문과 같은지 확인한다
    template <typename T, typename Idl>
    void expect_same_proto_body(const T& value) const {
        const std::optional<T> returned = through_idl<T, Idl>(value);
        ASSERT_TRUE(returned.has_value());
        EXPECT_EQ(proto_body(returned.value_or(T{})), proto_body(value));
    }

    bus_dds::DdsTypeTable dds_ = dds_types();
    bus_zmq::ZmqCodecTable zmq_ = zmq_codecs();
};

vehicle_api::RequestHeader sample_header() {
    vehicle_api::RequestHeader header;
    header.id.command_id.value = 42;
    header.id.session_id = "session-a";
    header.id.seq = 7;
    header.id.fc_epoch = 3;
    header.valid_until_monotonic_ns = 123'456'789'000;
    return header;
}

TEST_F(DdsTypesTest, AttitudeKeepsEveryField) {
    vehicle_api::Attitude attitude;
    attitude.roll_rad = 0.1;
    attitude.pitch_rad = -0.2;
    attitude.yaw_rad = 1.5;
    attitude.roll_rate_rad_s = 0.01;
    attitude.pitch_rate_rad_s = -0.02;
    attitude.yaw_rate_rad_s = 0.03;
    attitude.fc_time_boot_ms = 12'345;
    attitude.received_monotonic_ns = 5'000'000'000;
    attitude.fc_time_monotonic_ns = 4'900'000'000;
    attitude.timesync_valid = true;
    attitude.fc_epoch = 2;

    expect_same_proto_body<vehicle_api::Attitude, idl::FcAttitude>(attitude);
}

TEST_F(DdsTypesTest, AttitudeKeepsEmptyFcTime) {
    vehicle_api::Attitude attitude;
    attitude.roll_rad = 0.1;

    const std::optional<vehicle_api::Attitude> returned =
        through_idl<vehicle_api::Attitude, idl::FcAttitude>(attitude);

    ASSERT_TRUE(returned.has_value());
    EXPECT_EQ(returned.value_or(vehicle_api::Attitude{}).fc_time_monotonic_ns, std::nullopt);
}

TEST_F(DdsTypesTest, VehicleStateKeepsEveryPartButAttitude) {
    vehicle_api::VehicleState state;
    state.attitude = vehicle_api::Attitude{};
    state.local = vehicle_api::LocalPosition{1.0, 2.0, -3.0, 0.1, 0.2, -0.3, 10, 2};
    state.downward_distance = vehicle_api::DownwardDistance{2.5, 11, 2};
    state.global = vehicle_api::GlobalPosition{37.4, 127.1, 50.0, 12, 2};
    vehicle_api::FlightStatus status;
    status.mode = vehicle_api::FlightMode::Offboard;
    status.armed = true;
    status.in_air = true;
    status.offboard_hover = true;
    status.received_monotonic_ns = 13;
    status.fc_epoch = 2;
    state.status = status;

    expect_same_proto_body<vehicle_api::VehicleState, idl::FcState>(state);
    const std::optional<vehicle_api::VehicleState> returned =
        through_idl<vehicle_api::VehicleState, idl::FcState>(state);
    ASSERT_TRUE(returned.has_value());
    EXPECT_FALSE(returned.value_or(vehicle_api::VehicleState{}).attitude.has_value());
}

TEST_F(DdsTypesTest, VehicleStateKeepsEmptyParts) {
    vehicle_api::VehicleState state;
    state.downward_distance = vehicle_api::DownwardDistance{2.5, 11, 2};

    const std::optional<vehicle_api::VehicleState> returned =
        through_idl<vehicle_api::VehicleState, idl::FcState>(state);

    ASSERT_TRUE(returned.has_value());
    const vehicle_api::VehicleState value = returned.value_or(vehicle_api::VehicleState{});
    EXPECT_FALSE(value.local.has_value());
    EXPECT_TRUE(value.downward_distance.has_value());
    EXPECT_FALSE(value.global.has_value());
    EXPECT_FALSE(value.status.has_value());
}

TEST_F(DdsTypesTest, EveryFlightModeKeepsItsValue) {
    using vehicle_api::FlightMode;
    for (const FlightMode mode :
         {FlightMode::Unknown, FlightMode::Manual, FlightMode::Stabilized, FlightMode::Altitude,
          FlightMode::Position, FlightMode::Offboard, FlightMode::Hold, FlightMode::Return,
          FlightMode::Land, FlightMode::Takeoff, FlightMode::Mission}) {
        vehicle_api::FlightModeEvent event;
        event.mode = mode;
        expect_same_proto_body<vehicle_api::FlightModeEvent, idl::FcModeEvent>(event);
    }
}

TEST_F(DdsTypesTest, FlightModeEventKeepsEveryField) {
    vehicle_api::FlightModeEvent event;
    event.mode = vehicle_api::FlightMode::Hold;
    event.request_command_id = vehicle_api::CommandId{42};
    event.exit_cause = vehicle_api::OffboardExitCause::PilotSwitch;
    event.received_monotonic_ns = 5'000'000'000;

    expect_same_proto_body<vehicle_api::FlightModeEvent, idl::FcModeEvent>(event);
}

TEST_F(DdsTypesTest, VelocitySetpointKeepsEveryField) {
    vehicle_api::VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.right_m_s = -0.5;
    setpoint.down_m_s = 0.2;
    setpoint.yaw_rate_rad_s = -0.3;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;

    expect_same_proto_body<vehicle_api::VelocitySetpoint, idl::ControlSetpoint>(setpoint);
}

TEST_F(DdsTypesTest, FlightModeRequestKeepsHeaderAndAction) {
    for (const FlightModeRequest::Action action :
         {FlightModeRequest::Action::Offboard, FlightModeRequest::Action::Hold}) {
        FlightModeRequest request;
        request.header = sample_header();
        request.action = action;
        expect_same_proto_body<FlightModeRequest, idl::ControlModeRequest>(request);
    }
}

TEST_F(DdsTypesTest, FlightModeRequestWithUnspecifiedActionCannotBeConverted) {
    idl::ControlModeRequest sample;
    sample.action(idl::ModeAction::MODE_ACTION_UNSPECIFIED);

    EXPECT_EQ(from_sample<FlightModeRequest>(sample), std::nullopt);
}

TEST_F(DdsTypesTest, FlightModeReplyKeepsEveryField) {
    vehicle_api::FlightModeReply reply;
    reply.id = sample_header().id;
    reply.accepted = true;
    reply.reason = "ok";

    expect_same_proto_body<vehicle_api::FlightModeReply, idl::ControlModeReply>(reply);
}

TEST_F(DdsTypesTest, FlightModeReplyWithoutIdCannotBeConverted) {
    vehicle_api::FlightModeReply reply;
    reply.accepted = true;

    EXPECT_EQ((through_idl<vehicle_api::FlightModeReply, idl::ControlModeReply>(reply)),
              std::nullopt);
}

TEST_F(DdsTypesTest, FollowCommandKeepsEveryActionKind) {
    FollowCommandRequest track_start;
    track_start.header = sample_header();
    vehicle_api::TrackTarget target;
    target.sensor = vehicle_api::Sensor::Ir;
    target.generation = 3;
    target.class_id = 1;
    target.track_id = 77;
    target.label = "person";
    track_start.action = target;

    FollowCommandRequest stop;
    stop.header = sample_header();
    stop.action = vehicle_api::StopReason::LinkLoss;

    FollowCommandRequest search;
    search.header = sample_header();
    vehicle_api::ScanRequest scan;
    scan.yaw_range_rad = 3.0;
    scan.timeout_s = 20.0;  // yaw_rate_rad_s 는 비워 둔다 (NaN 으로 보냄)
    search.action = scan;

    for (const FollowCommandRequest& request : {track_start, stop, search}) {
        expect_same_proto_body<FollowCommandRequest, idl::FollowCommand>(request);
    }
}

TEST_F(DdsTypesTest, ScanRequestKeepsEmptyItems) {
    FollowCommandRequest request;
    vehicle_api::ScanRequest scan;
    scan.yaw_range_rad = 3.0;
    request.action = scan;

    const std::optional<FollowCommandRequest> returned =
        through_idl<FollowCommandRequest, idl::FollowCommand>(request);

    ASSERT_TRUE(returned.has_value());
    const FollowCommandRequest value = returned.value_or(FollowCommandRequest{});
    const auto* returned_scan = std::get_if<vehicle_api::ScanRequest>(&value.action);
    ASSERT_NE(returned_scan, nullptr);
    EXPECT_EQ(returned_scan->yaw_range_rad, std::optional<double>{3.0});
    EXPECT_EQ(returned_scan->yaw_rate_rad_s, std::nullopt);
    EXPECT_EQ(returned_scan->timeout_s, std::nullopt);
}

TEST_F(DdsTypesTest, StopWithUnspecifiedReasonCannotBeConverted) {
    idl::FollowCommand sample;
    idl::FollowCommand_Stop stop;
    stop.reason(idl::StopReason::STOP_REASON_UNSPECIFIED);
    sample.action().stop(stop);

    EXPECT_EQ(from_sample<FollowCommandRequest>(sample), std::nullopt);
}

TEST_F(DdsTypesTest, FollowResultKeepsEveryOutcome) {
    for (const FollowOutcome outcome : {FollowOutcome::InProgress, FollowOutcome::Succeeded,
                                        FollowOutcome::Rejected, FollowOutcome::Failed}) {
        FollowResult result;
        result.id = sample_header().id;
        result.outcome = outcome;
        result.request_accepted = true;
        result.reason = "reason";
        expect_same_proto_body<FollowResult, idl::FollowResult>(result);
    }
}

TEST_F(DdsTypesTest, FollowResultWithUnspecifiedOutcomeCannotBeConverted) {
    idl::FollowResult sample;
    sample.outcome(idl::FollowOutcome::FOLLOW_OUTCOME_UNSPECIFIED);

    EXPECT_EQ(from_sample<FollowResult>(sample), std::nullopt);
}

}  // namespace
}  // namespace argus::vehicle_api_bus
