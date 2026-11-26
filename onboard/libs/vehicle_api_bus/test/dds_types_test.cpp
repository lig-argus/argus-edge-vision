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
#include "argus_msg/follow_status.hpp"

namespace argus::vehicle_api_bus {
namespace {

using bus_zmq::Bytes;
using vehicle_api::FollowOutcome;
using vehicle_api::FollowResult;
using vehicle_api::FollowStatus;

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

vehicle_api::TrackTarget sample_target(std::uint32_t track_id) {
    vehicle_api::TrackTarget target;
    target.sensor = vehicle_api::Sensor::Eo;
    target.generation = 3;
    target.class_id = 2;
    target.track_id = track_id;
    target.label = "person";
    return target;
}

TEST_F(DdsTypesTest, FollowStatusKeepsEveryField) {
    FollowStatus status;
    status.session_id = "session-f";
    status.controller_session = "controller-f";
    status.client_session = "client-f";
    status.onboard_session = "onboard-g";
    status.uid = "uid-1";
    status.fc_epoch = 4;
    status.received_monotonic_ns = 9'000'000;
    status.mission_state = vehicle_api::MissionState::HoldTransition;
    status.target_state = vehicle_api::TargetState::Predicted;
    status.selected = sample_target(17);
    vehicle_api::FollowCommandResult track_start;
    track_start.command_id = vehicle_api::CommandId{42};
    track_start.action = vehicle_api::FollowAction::TrackStart;
    track_start.outcome = FollowOutcome::Succeeded;
    track_start.request_accepted = true;
    track_start.target = sample_target(17);
    vehicle_api::FollowCommandResult search;
    search.command_id = vehicle_api::CommandId{43};
    search.action = vehicle_api::FollowAction::Search;
    search.outcome = FollowOutcome::Failed;
    search.request_accepted = true;
    search.reason = "timeout";
    status.command_results = {track_start, search};
    status.command_epoch = 5;
    status.command_sent = true;
    vehicle_api::ScanProgress scan;
    scan.yaw_rate_rad_s = 0.3;
    scan.sweep_rad = 6.0;
    scan.timeout_s = 30.0;
    scan.progress_deg = 90.0;
    scan.remaining_s = 15.0;
    status.scan = scan;

    expect_same_proto_body<FollowStatus, idl::FollowStatus>(status);
}

TEST_F(DdsTypesTest, FollowStatusKeepsEmptyOptionalValues) {
    FollowStatus status;
    status.mission_state = vehicle_api::MissionState::Idle;
    status.scan = vehicle_api::ScanProgress{};  // remaining_s 없음

    const std::optional<FollowStatus> returned =
        through_idl<FollowStatus, idl::FollowStatus>(status);

    ASSERT_TRUE(returned.has_value());
    const FollowStatus received = returned.value_or(FollowStatus{});
    EXPECT_FALSE(received.selected.has_value());
    EXPECT_TRUE(received.command_results.empty());
    ASSERT_TRUE(received.scan.has_value());
    EXPECT_FALSE(received.scan.value_or(vehicle_api::ScanProgress{}).remaining_s.has_value());
    expect_same_proto_body<FollowStatus, idl::FollowStatus>(status);
}

TEST_F(DdsTypesTest, EveryMissionAndTargetStateKeepsItsValue) {
    for (const vehicle_api::MissionState state :
         {vehicle_api::MissionState::Idle, vehicle_api::MissionState::Preparing,
          vehicle_api::MissionState::FollowActive, vehicle_api::MissionState::HoldTransition,
          vehicle_api::MissionState::HoldWait, vehicle_api::MissionState::ScanActive,
          vehicle_api::MissionState::Manual, vehicle_api::MissionState::Fault,
          vehicle_api::MissionState::Unknown}) {
        FollowStatus status;
        status.mission_state = state;
        expect_same_proto_body<FollowStatus, idl::FollowStatus>(status);
    }
    for (const vehicle_api::TargetState state :
         {vehicle_api::TargetState::None, vehicle_api::TargetState::Confirmed,
          vehicle_api::TargetState::Predicted, vehicle_api::TargetState::Lost,
          vehicle_api::TargetState::Unknown}) {
        FollowStatus status;
        status.target_state = state;
        expect_same_proto_body<FollowStatus, idl::FollowStatus>(status);
    }
}

TEST_F(DdsTypesTest, FollowStatusUnspecifiedEnumValuesBecomeUnknown) {
    idl::FollowStatus sample;
    sample.mission_state(idl::MissionState::MISSION_STATE_UNSPECIFIED);
    sample.target_state(idl::TargetState::TARGET_STATE_UNSPECIFIED);
    idl::FollowStatus_CommandResult entry;
    entry.action(idl::FollowAction::FOLLOW_ACTION_UNSPECIFIED);
    entry.outcome(idl::FollowOutcome::FOLLOW_OUTCOME_SUCCEEDED);
    sample.command_results().push_back(entry);

    const std::optional<FollowStatus> status = from_sample<FollowStatus>(sample);

    ASSERT_TRUE(status.has_value());
    const FollowStatus received = status.value_or(FollowStatus{});
    EXPECT_EQ(received.mission_state, vehicle_api::MissionState::Unknown);
    EXPECT_EQ(received.target_state, vehicle_api::TargetState::Unknown);
    ASSERT_EQ(received.command_results.size(), 1U);
    EXPECT_EQ(received.command_results[0].action, vehicle_api::FollowAction::Unknown);
}

TEST_F(DdsTypesTest, FollowStatusDropsCommandResultWithUnspecifiedOutcome) {
    idl::FollowStatus sample;
    idl::FollowStatus_CommandResult dropped;
    dropped.command_id(1);
    dropped.outcome(idl::FollowOutcome::FOLLOW_OUTCOME_UNSPECIFIED);
    idl::FollowStatus_CommandResult kept;
    kept.command_id(2);
    kept.outcome(idl::FollowOutcome::FOLLOW_OUTCOME_IN_PROGRESS);
    sample.command_results().push_back(dropped);
    sample.command_results().push_back(kept);

    const std::optional<FollowStatus> status = from_sample<FollowStatus>(sample);

    ASSERT_TRUE(status.has_value());
    const FollowStatus received = status.value_or(FollowStatus{});
    ASSERT_EQ(received.command_results.size(), 1U);
    EXPECT_EQ(received.command_results[0].command_id, vehicle_api::CommandId{2});
}

}  // namespace
}  // namespace argus::vehicle_api_bus
