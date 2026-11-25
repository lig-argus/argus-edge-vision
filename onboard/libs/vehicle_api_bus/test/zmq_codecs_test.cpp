#include "argus/vehicle_api_bus/zmq_codecs.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <typeindex>
#include <variant>

#include "argus/bus/message_type.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus_msg/control_mode_request.pb.h"
#include "argus_msg/control_setpoint.pb.h"
#include "argus_msg/fc_attitude.pb.h"
#include "argus_msg/fc_mode_event.pb.h"
#include "argus_msg/fc_state.pb.h"
#include "argus_msg/follow_command.pb.h"

namespace argus::vehicle_api_bus {
namespace {

using bus_zmq::Bytes;
using vehicle_api::Attitude;
using vehicle_api::FlightMode;
using vehicle_api::FlightModeEvent;
using vehicle_api::FlightModeReply;
using vehicle_api::FollowOutcome;
using vehicle_api::FollowResult;
using vehicle_api::VehicleState;
using vehicle_api::VelocitySetpoint;

constexpr std::int64_t kMs = 1'000'000;

// 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없는 본문 (파싱 실패)
Bytes broken_body() { return {0x0A, 0x05}; }

template <typename Message>
Bytes to_body(const Message& message) {
    const std::string serialized = message.SerializeAsString();
    return {serialized.begin(), serialized.end()};
}

template <typename Message>
Message parsed(const Bytes& body) {
    Message message;
    EXPECT_TRUE(message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size())));
    return message;
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

class ZmqCodecsTest : public ::testing::Test {
protected:
    // 값을 변환 표의 encode 로 바꾼다. 바이너리는 binary_ 에 남는다
    template <typename T>
    std::optional<Bytes> encode(const T& value, std::uint64_t sequence = 0) {
        Bytes body;
        binary_.clear();
        const bus_zmq::ZmqCodec& codec = table_.at(std::type_index(typeid(T)));
        if (!codec.encode(&value, sequence, body, binary_)) {
            return std::nullopt;
        }
        return body;
    }

    // 본문을 변환 표의 decode 로 바꾼다. 본문에 실린 번호는 sequence_ 에 남는다
    template <typename T>
    std::optional<T> decode(const Bytes& body, const Bytes& binary = {}) {
        T value;
        sequence_ = 0;
        const bus_zmq::ZmqCodec& codec = table_.at(std::type_index(typeid(T)));
        if (!codec.decode(body, binary, &value, sequence_)) {
            return std::nullopt;
        }
        return value;
    }

    // encode 한 뒤 decode 한 값
    template <typename T>
    std::optional<T> round_trip(const T& value, std::uint64_t sequence = 0) {
        const std::optional<Bytes> body = encode(value, sequence);
        if (!body) {
            return std::nullopt;
        }
        return decode<T>(*body, binary_);
    }

    bus_zmq::ZmqCodecTable table_ = zmq_codecs();
    Bytes binary_;
    std::uint64_t sequence_ = 0;
};

TEST_F(ZmqCodecsTest, HasCodecForEveryMessageType) {
    for (const bus::MessageType* type :
         {&kFcAttitudeType, &kFcStateType, &kFcModeEventType, &kControlSetpointType,
          &kControlModeRequestType, &kControlModeReplyType, &kFollowCommandType,
          &kFollowResultType}) {
        const auto found = table_.find(type->type);
        ASSERT_NE(found, table_.end()) << type->name;
        EXPECT_TRUE(found->second.encode) << type->name;
        EXPECT_TRUE(found->second.decode) << type->name;
    }
    EXPECT_EQ(table_.size(), 8U);
}

// ------------------------------------------------------------------------------- control/setpoint

TEST_F(ZmqCodecsTest, ControlSetpointBodyIsTheMessageWithSequenceInSeq) {
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.right_m_s = -0.25;
    setpoint.down_m_s = 0.125;
    setpoint.yaw_rate_rad_s = 0.1745;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;
    msg::ControlSetpoint expected;
    expected.set_seq(9);
    expected.set_forward_m_s(1.5);
    expected.set_right_m_s(-0.25);
    expected.set_down_m_s(0.125);
    expected.set_yaw_rate_rad_s(0.1745);
    expected.set_valid_until_monotonic_ns(123'456'789'000);

    const std::optional<Bytes> body = encode(setpoint, 9);

    EXPECT_EQ(body, std::optional<Bytes>{to_body(expected)});
    EXPECT_TRUE(binary_.empty());
}

TEST_F(ZmqCodecsTest, ControlSetpointRoundTripKeepsValuesAndSequence) {
    VelocitySetpoint setpoint;
    setpoint.forward_m_s = 1.5;
    setpoint.right_m_s = -0.25;
    setpoint.down_m_s = 0.125;
    setpoint.yaw_rate_rad_s = 0.1745;
    setpoint.valid_until_monotonic_ns = 123'456'789'000;

    const VelocitySetpoint received = round_trip(setpoint, 4).value_or(VelocitySetpoint{});

    EXPECT_EQ(sequence_, 4U);
    EXPECT_DOUBLE_EQ(received.forward_m_s, 1.5);
    EXPECT_DOUBLE_EQ(received.right_m_s, -0.25);
    EXPECT_DOUBLE_EQ(received.down_m_s, 0.125);
    EXPECT_DOUBLE_EQ(received.yaw_rate_rad_s, 0.1745);
    EXPECT_EQ(received.valid_until_monotonic_ns, 123'456'789'000);
}

// ------------------------------------------------------------------- fc/attitude, state,
// mode_event

TEST_F(ZmqCodecsTest, FcAttitudeRoundTripKeepsValuesAndSequence) {
    Attitude sent;
    sent.roll_rad = 0.1;
    sent.pitch_rad = -0.2;
    sent.yaw_rad = 1.5;
    sent.roll_rate_rad_s = 0.01;
    sent.pitch_rate_rad_s = 0.02;
    sent.yaw_rate_rad_s = 0.03;
    sent.fc_time_boot_ms = 123'456;
    sent.received_monotonic_ns = 5 * kMs;
    sent.fc_time_monotonic_ns = 4 * kMs;
    sent.timesync_valid = true;
    sent.fc_epoch = 7;

    const std::optional<Bytes> body = encode(sent, 11);
    ASSERT_TRUE(body.has_value());
    EXPECT_EQ(parsed<msg::FcAttitude>(body.value_or(Bytes{})).seq(), 11U);
    const Attitude received = decode<Attitude>(body.value_or(Bytes{})).value_or(Attitude{});

    EXPECT_EQ(sequence_, 11U);
    EXPECT_DOUBLE_EQ(received.roll_rad, 0.1);
    EXPECT_DOUBLE_EQ(received.pitch_rad, -0.2);
    EXPECT_DOUBLE_EQ(received.yaw_rad, 1.5);
    EXPECT_DOUBLE_EQ(received.roll_rate_rad_s, 0.01);
    EXPECT_DOUBLE_EQ(received.pitch_rate_rad_s, 0.02);
    EXPECT_DOUBLE_EQ(received.yaw_rate_rad_s, 0.03);
    EXPECT_EQ(received.fc_time_boot_ms, 123'456U);
    EXPECT_EQ(received.received_monotonic_ns, 5 * kMs);
    EXPECT_EQ(received.fc_time_monotonic_ns, std::optional<std::int64_t>{4 * kMs});
    EXPECT_TRUE(received.timesync_valid);
    EXPECT_EQ(received.fc_epoch, 7U);
}

TEST_F(ZmqCodecsTest, FcAttitudeWithoutFcTimeArrivesWithoutFcTime) {
    const std::optional<Attitude> received = round_trip(Attitude{}, 1);

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received.value_or(Attitude{}).fc_time_monotonic_ns, std::nullopt);
}

TEST_F(ZmqCodecsTest, FcStateRoundTripKeepsValuesAndSequence) {
    VehicleState sent;
    sent.local = vehicle_api::LocalPosition{1.0, 2.0, -3.0, 0.1, 0.2, -0.3, 10 * kMs, 7};
    sent.downward_distance = vehicle_api::DownwardDistance{12.5, 11 * kMs, 7};
    sent.global = vehicle_api::GlobalPosition{37.5, 127.0, 80.0, 12 * kMs, 7};
    sent.status = vehicle_api::FlightStatus{FlightMode::Hold, true, true, true, 13 * kMs, 7};

    const std::optional<Bytes> body = encode(sent, 21);
    ASSERT_TRUE(body.has_value());
    EXPECT_EQ(parsed<msg::FcState>(body.value_or(Bytes{})).seq(), 21U);
    const VehicleState received =
        decode<VehicleState>(body.value_or(Bytes{})).value_or(VehicleState{});

    EXPECT_EQ(sequence_, 21U);
    const vehicle_api::LocalPosition local = received.local.value_or(vehicle_api::LocalPosition{});
    const vehicle_api::DownwardDistance distance =
        received.downward_distance.value_or(vehicle_api::DownwardDistance{});
    const vehicle_api::GlobalPosition global =
        received.global.value_or(vehicle_api::GlobalPosition{});
    const vehicle_api::FlightStatus status = received.status.value_or(vehicle_api::FlightStatus{});
    EXPECT_DOUBLE_EQ(local.north_m, 1.0);
    EXPECT_DOUBLE_EQ(local.east_m, 2.0);
    EXPECT_DOUBLE_EQ(local.down_m, -3.0);
    EXPECT_DOUBLE_EQ(local.vel_north_m_s, 0.1);
    EXPECT_DOUBLE_EQ(local.vel_east_m_s, 0.2);
    EXPECT_DOUBLE_EQ(local.vel_down_m_s, -0.3);
    EXPECT_EQ(local.received_monotonic_ns, 10 * kMs);
    EXPECT_EQ(local.fc_epoch, 7U);
    EXPECT_DOUBLE_EQ(distance.distance_m, 12.5);
    EXPECT_EQ(distance.received_monotonic_ns, 11 * kMs);
    EXPECT_EQ(distance.fc_epoch, 7U);
    EXPECT_DOUBLE_EQ(global.lat_deg, 37.5);
    EXPECT_DOUBLE_EQ(global.lon_deg, 127.0);
    EXPECT_DOUBLE_EQ(global.alt_m, 80.0);
    EXPECT_EQ(global.received_monotonic_ns, 12 * kMs);
    EXPECT_EQ(status.mode, FlightMode::Hold);
    EXPECT_TRUE(status.armed);
    EXPECT_TRUE(status.in_air);
    EXPECT_TRUE(status.offboard_hover);
    EXPECT_EQ(status.received_monotonic_ns, 13 * kMs);
}

TEST_F(ZmqCodecsTest, FcStateDoesNotCarryAttitudeAndKeepsMissingValuesEmpty) {
    VehicleState sent;
    sent.attitude = Attitude{};  // fc/state 는 자세를 싣지 않는다
    sent.downward_distance = vehicle_api::DownwardDistance{12.5, 1 * kMs, 7};

    const std::optional<VehicleState> received = round_trip(sent, 1);

    ASSERT_TRUE(received.has_value());
    const VehicleState state = received.value_or(VehicleState{});
    EXPECT_FALSE(state.attitude.has_value());
    EXPECT_FALSE(state.local.has_value());
    EXPECT_TRUE(state.downward_distance.has_value());
    EXPECT_FALSE(state.global.has_value());
    EXPECT_FALSE(state.status.has_value());
}

TEST_F(ZmqCodecsTest, FcModeEventRoundTripKeepsValuesAndSequence) {
    FlightModeEvent sent;
    sent.mode = FlightMode::Position;
    sent.request_command_id = vehicle_api::CommandId{42};
    sent.exit_cause = vehicle_api::OffboardExitCause::PilotSwitch;
    sent.received_monotonic_ns = 9 * kMs;

    const std::optional<Bytes> body = encode(sent, 31);
    ASSERT_TRUE(body.has_value());
    EXPECT_EQ(parsed<msg::FcModeEvent>(body.value_or(Bytes{})).seq(), 31U);
    const FlightModeEvent received =
        decode<FlightModeEvent>(body.value_or(Bytes{})).value_or(FlightModeEvent{});

    EXPECT_EQ(sequence_, 31U);
    EXPECT_EQ(received.mode, FlightMode::Position);
    EXPECT_EQ(received.request_command_id,
              std::optional<vehicle_api::CommandId>{vehicle_api::CommandId{42}});
    EXPECT_EQ(received.exit_cause, vehicle_api::OffboardExitCause::PilotSwitch);
    EXPECT_EQ(received.received_monotonic_ns, 9 * kMs);
}

TEST_F(ZmqCodecsTest, FcModeEventWithoutRequestArrivesWithoutCommandId) {
    const std::optional<FlightModeEvent> received = round_trip(FlightModeEvent{}, 1);

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received.value_or(FlightModeEvent{}).request_command_id, std::nullopt);
}

TEST_F(ZmqCodecsTest, TopicDecodeFailsForBodyThatCannotBeParsed) {
    EXPECT_FALSE(decode<Attitude>(broken_body()).has_value());
    EXPECT_FALSE(decode<VehicleState>(broken_body()).has_value());
    EXPECT_FALSE(decode<FlightModeEvent>(broken_body()).has_value());
    EXPECT_FALSE(decode<VelocitySetpoint>(broken_body()).has_value());
}

TEST_F(ZmqCodecsTest, TopicDecodeFailsForMessageWithBinary) {
    const Bytes binary{1};
    // 같은 본문이 바이너리 없이는 성공한다
    EXPECT_TRUE(decode<VelocitySetpoint>(Bytes{}).has_value());

    EXPECT_FALSE(decode<Attitude>(Bytes{}, binary).has_value());
    EXPECT_FALSE(decode<VehicleState>(Bytes{}, binary).has_value());
    EXPECT_FALSE(decode<FlightModeEvent>(Bytes{}, binary).has_value());
    EXPECT_FALSE(decode<VelocitySetpoint>(Bytes{}, binary).has_value());
}

// --------------------------------------------------------------------------- control/mode_request

TEST_F(ZmqCodecsTest, FlightModeRequestBodyIsTheMessageWithHeaderAndAction) {
    msg::ControlModeRequest expected;
    msg::RequestId& id = *expected.mutable_header()->mutable_id();
    id.set_command_id(42);
    id.set_session_id("session-a");
    id.set_seq(7);
    id.set_fc_epoch(3);
    expected.mutable_header()->set_valid_until_monotonic_ns(123'456'789'000);
    expected.mutable_offboard();

    const std::optional<Bytes> body =
        encode(FlightModeRequest{sample_header(), FlightModeRequest::Action::Offboard});

    EXPECT_EQ(body, std::optional<Bytes>{to_body(expected)});
    EXPECT_TRUE(binary_.empty());
}

TEST_F(ZmqCodecsTest, FlightModeRequestRoundTripKeepsHeaderAndAction) {
    const FlightModeRequest offboard =
        round_trip(FlightModeRequest{sample_header(), FlightModeRequest::Action::Offboard})
            .value_or(FlightModeRequest{});
    const FlightModeRequest hold =
        round_trip(FlightModeRequest{sample_header(), FlightModeRequest::Action::Hold})
            .value_or(FlightModeRequest{});

    EXPECT_EQ(offboard.action, FlightModeRequest::Action::Offboard);
    EXPECT_EQ(offboard.header.id, sample_header().id);
    EXPECT_EQ(offboard.header.valid_until_monotonic_ns, 123'456'789'000);
    EXPECT_EQ(hold.action, FlightModeRequest::Action::Hold);
}

// 해석하지 못한 요청은 decode 가 실패한다
TEST_F(ZmqCodecsTest, FlightModeRequestThatCannotBeParsedFailsToDecode) {
    EXPECT_FALSE(decode<FlightModeRequest>(broken_body()).has_value());
}

TEST_F(ZmqCodecsTest, FlightModeRequestWithoutActionFailsToDecode) {
    msg::ControlModeRequest without_action;
    without_action.mutable_header()->mutable_id()->set_command_id(1);

    EXPECT_FALSE(decode<FlightModeRequest>(to_body(without_action)).has_value());
}

TEST_F(ZmqCodecsTest, FlightModeRequestWithoutHeaderFailsToDecode) {
    msg::ControlModeRequest without_header;
    without_header.mutable_hold();

    EXPECT_FALSE(decode<FlightModeRequest>(to_body(without_header)).has_value());
}

TEST_F(ZmqCodecsTest, FlightModeRequestWithBinaryFailsToDecode) {
    const std::optional<Bytes> body =
        encode(FlightModeRequest{sample_header(), FlightModeRequest::Action::Hold});
    ASSERT_TRUE(body.has_value());

    EXPECT_FALSE(decode<FlightModeRequest>(body.value_or(Bytes{}), Bytes{1}).has_value());
}

// 빈 본문은 header 가 없는 메시지로 파싱되므로 실패한다 (ZmqBus 가 해석하지 못한 요청에 빈 본문으로
// 응답하므로, 빈 본문이 요청으로 되돌아와도 처리되지 않아야 한다)
TEST_F(ZmqCodecsTest, EmptyRequestBodyFailsToDecode) {
    EXPECT_FALSE(decode<FlightModeRequest>(Bytes{}).has_value());
    EXPECT_FALSE(decode<FollowCommandRequest>(Bytes{}).has_value());
}

TEST_F(ZmqCodecsTest, FlightModeReplyBodyIsTheMessage) {
    FlightModeReply reply;
    reply.reason = "not armed";
    msg::ControlModeReply expected;
    expected.set_accepted(false);
    expected.set_reason("not armed");

    EXPECT_EQ(encode(reply), std::optional<Bytes>{to_body(expected)});
    EXPECT_TRUE(binary_.empty());
}

TEST_F(ZmqCodecsTest, FlightModeReplyRoundTripKeepsValues) {
    FlightModeReply sent;
    sent.id = sample_header().id;
    sent.accepted = true;
    sent.reason = "ok";

    const std::optional<FlightModeReply> received = round_trip(sent);

    ASSERT_TRUE(received.has_value());
    const FlightModeReply reply = received.value_or(FlightModeReply{});
    EXPECT_EQ(reply.id, std::optional<vehicle_api::RequestId>{sample_header().id});
    EXPECT_TRUE(reply.accepted);
    EXPECT_EQ(reply.reason, "ok");
}

// id 가 없는 응답은 어느 요청의 것인지 맞출 수 없으므로 받지 않는다
TEST_F(ZmqCodecsTest, FlightModeReplyWithoutIdFailsToDecode) {
    EXPECT_FALSE(round_trip(FlightModeReply{}).has_value());
    EXPECT_FALSE(decode<FlightModeReply>(Bytes{}).has_value());
}

TEST_F(ZmqCodecsTest, FlightModeReplyDecodeFailsForBrokenBodyOrBinary) {
    EXPECT_FALSE(decode<FlightModeReply>(broken_body()).has_value());
    EXPECT_FALSE(decode<FlightModeReply>(Bytes{}, Bytes{1}).has_value());
}

// --------------------------------------------------------------------------------- follow/command

TEST_F(ZmqCodecsTest, TrackStartRoundTripKeepsHeaderAndTarget) {
    vehicle_api::TrackTarget target;
    target.sensor = vehicle_api::Sensor::Ir;
    target.generation = 5;
    target.class_id = 2;
    target.track_id = 17;
    target.label = "person";

    const std::optional<Bytes> body = encode(FollowCommandRequest{sample_header(), target});
    ASSERT_TRUE(body.has_value());
    EXPECT_TRUE(parsed<msg::FollowCommand>(body.value_or(Bytes{})).has_track_start());
    const FollowCommandRequest received =
        decode<FollowCommandRequest>(body.value_or(Bytes{})).value_or(FollowCommandRequest{});

    EXPECT_EQ(received.header.id, sample_header().id);
    EXPECT_EQ(received.header.valid_until_monotonic_ns, 123'456'789'000);
    ASSERT_TRUE(std::holds_alternative<vehicle_api::TrackTarget>(received.action));
    const auto& received_target = std::get<vehicle_api::TrackTarget>(received.action);
    EXPECT_EQ(received_target.sensor, vehicle_api::Sensor::Ir);
    EXPECT_EQ(received_target.generation, 5U);
    EXPECT_EQ(received_target.class_id, 2);
    EXPECT_EQ(received_target.track_id, 17U);
    EXPECT_EQ(received_target.label, "person");
}

TEST_F(ZmqCodecsTest, StopRoundTripKeepsTheReason) {
    const FollowCommandRequest received =
        round_trip(FollowCommandRequest{sample_header(), vehicle_api::StopReason::LinkLoss})
            .value_or(FollowCommandRequest{});

    ASSERT_TRUE(std::holds_alternative<vehicle_api::StopReason>(received.action));
    EXPECT_EQ(std::get<vehicle_api::StopReason>(received.action),
              vehicle_api::StopReason::LinkLoss);
}

TEST_F(ZmqCodecsTest, ScanStartSendsEmptyItemsAsNanAndKeepsThemEmpty) {
    vehicle_api::ScanRequest scan;
    scan.yaw_rate_rad_s = 0.3;  // 나머지 2개는 비어 있음

    const std::optional<Bytes> body = encode(FollowCommandRequest{sample_header(), scan});
    ASSERT_TRUE(body.has_value());
    const auto message = parsed<msg::FollowCommand>(body.value_or(Bytes{}));
    const FollowCommandRequest received =
        decode<FollowCommandRequest>(body.value_or(Bytes{})).value_or(FollowCommandRequest{});

    ASSERT_TRUE(message.has_search());
    EXPECT_TRUE(std::isnan(message.search().yaw_range_rad()));
    EXPECT_DOUBLE_EQ(message.search().yaw_rate_rad_s(), 0.3);
    EXPECT_TRUE(std::isnan(message.search().timeout_s()));
    ASSERT_TRUE(std::holds_alternative<vehicle_api::ScanRequest>(received.action));
    const auto& received_scan = std::get<vehicle_api::ScanRequest>(received.action);
    EXPECT_EQ(received_scan.yaw_range_rad, std::nullopt);
    EXPECT_DOUBLE_EQ(received_scan.yaw_rate_rad_s.value_or(0.0), 0.3);
    EXPECT_EQ(received_scan.timeout_s, std::nullopt);
}

// 해석하지 못한 요청은 decode 가 실패한다
TEST_F(ZmqCodecsTest, FollowCommandThatCannotBeParsedFailsToDecode) {
    EXPECT_FALSE(decode<FollowCommandRequest>(broken_body()).has_value());
}

TEST_F(ZmqCodecsTest, StopWithUnspecifiedReasonFailsToDecode) {
    msg::FollowCommand command;
    command.mutable_header()->mutable_id()->set_command_id(1);
    command.mutable_stop();  // reason 을 지정하지 않음

    EXPECT_FALSE(decode<FollowCommandRequest>(to_body(command)).has_value());
}

TEST_F(ZmqCodecsTest, FollowCommandWithoutHeaderFailsToDecode) {
    msg::FollowCommand without_header;
    without_header.mutable_stop()->set_reason(msg::STOP_REASON_OPERATOR);

    EXPECT_FALSE(decode<FollowCommandRequest>(to_body(without_header)).has_value());
}

TEST_F(ZmqCodecsTest, FollowCommandWithoutActionOrWithBinaryFailsToDecode) {
    msg::FollowCommand without_action;
    without_action.mutable_header()->mutable_id()->set_command_id(1);
    const std::optional<Bytes> valid =
        encode(FollowCommandRequest{sample_header(), vehicle_api::StopReason::Operator});
    ASSERT_TRUE(valid.has_value());

    EXPECT_FALSE(decode<FollowCommandRequest>(to_body(without_action)).has_value());
    EXPECT_FALSE(decode<FollowCommandRequest>(valid.value_or(Bytes{}), Bytes{1}).has_value());
}

TEST_F(ZmqCodecsTest, FollowResultBodyIsTheMessage) {
    FollowResult result;
    result.outcome = FollowOutcome::Rejected;
    result.reason = "no such track";
    msg::FollowResult expected;
    expected.mutable_id()->set_command_id(0);
    expected.mutable_id()->set_session_id("");
    expected.set_outcome(msg::FOLLOW_OUTCOME_REJECTED);
    expected.set_request_accepted(false);
    expected.set_reason("no such track");

    EXPECT_EQ(encode(result), std::optional<Bytes>{to_body(expected)});
    EXPECT_TRUE(binary_.empty());
}

TEST_F(ZmqCodecsTest, FollowResultRoundTripKeepsValues) {
    FollowResult sent;
    sent.id = sample_header().id;
    sent.outcome = FollowOutcome::InProgress;
    sent.request_accepted = true;
    sent.reason = "tracking";

    const std::optional<FollowResult> received = round_trip(sent);

    ASSERT_TRUE(received.has_value());
    const FollowResult result = received.value_or(FollowResult{});
    EXPECT_EQ(result.id, sample_header().id);
    EXPECT_EQ(result.outcome, FollowOutcome::InProgress);
    EXPECT_TRUE(result.request_accepted);
    EXPECT_EQ(result.reason, "tracking");
}

TEST_F(ZmqCodecsTest, FollowResultDecodeFailsWithoutOutcome) {
    msg::FollowResult without_outcome;
    without_outcome.mutable_id()->set_command_id(42);

    EXPECT_FALSE(decode<FollowResult>(to_body(without_outcome)).has_value());
}

TEST_F(ZmqCodecsTest, FollowResultDecodeFailsWithoutId) {
    msg::FollowResult without_id;
    without_id.set_outcome(msg::FOLLOW_OUTCOME_SUCCEEDED);

    EXPECT_FALSE(decode<FollowResult>(to_body(without_id)).has_value());
}

TEST_F(ZmqCodecsTest, FollowResultDecodeFailsForBrokenBodyOrBinary) {
    const std::optional<Bytes> valid = encode(FollowResult{});
    ASSERT_TRUE(valid.has_value());
    EXPECT_TRUE(decode<FollowResult>(valid.value_or(Bytes{})).has_value());

    EXPECT_FALSE(decode<FollowResult>(broken_body()).has_value());
    EXPECT_FALSE(decode<FollowResult>(valid.value_or(Bytes{}), Bytes{1}).has_value());
}

}  // namespace
}  // namespace argus::vehicle_api_bus
