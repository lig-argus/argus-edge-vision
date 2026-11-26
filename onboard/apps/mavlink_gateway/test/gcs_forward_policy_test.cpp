#include "argus/mavlink_gateway/gcs_forward_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>

#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr float kCustom = static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// 제어권을 가진 통제기, Position 모드, 시동 켜짐, mavlink_gateway 는 setpoint 를 보내지 않음
ForwardObservation flying_controller() {
    ForwardObservation observation;
    observation.from_controller = true;
    observation.fc_status_known = true;
    observation.fc_mode = FlightMode::Position;
    observation.armed = true;
    return observation;
}

ForwardObservation grounded_controller() {
    ForwardObservation observation = flying_controller();
    observation.armed = false;
    return observation;
}

GcsMessageRequest command(std::uint16_t mav_cmd, float param1 = 0.0F, float param2 = 0.0F,
                          float param3 = 0.0F) {
    return GcsMessageRequest{GcsMessageKind::CommandLong, mav_cmd, param1, param2, param3};
}

GcsMessageRequest set_mode(float main_mode, float sub_mode = 0.0F) {
    return command(MAV_CMD_DO_SET_MODE, kCustom, main_mode, sub_mode);
}

GcsMessageRequest message(GcsMessageKind kind) { return GcsMessageRequest{kind, 0, 0, 0, 0}; }

void expect_forward(const ForwardDecision& decision) {
    EXPECT_EQ(decision.action, ForwardAction::Forward);
}

void expect_reply(const ForwardDecision& decision, MAV_RESULT result, const char* reason) {
    EXPECT_EQ(decision.action, ForwardAction::Reply);
    EXPECT_EQ(decision.result, result);
    EXPECT_EQ(decision.reason, reason);
}

TEST(SetModeRequestTest, ReadsPx4MainAndSubMode) {
    EXPECT_EQ(set_mode_request(kCustom, 3.0F, 0.0F), FlightMode::Position);
    EXPECT_EQ(set_mode_request(kCustom, 4.0F, 3.0F), FlightMode::Hold);
    EXPECT_EQ(set_mode_request(kCustom, 6.0F, 0.0F), FlightMode::Offboard);
    EXPECT_EQ(set_mode_request(kCustom, 99.0F, 0.0F), FlightMode::Unknown);
}

TEST(SetModeRequestTest, RejectsMissingCustomFlagAndNonIntegerValues) {
    EXPECT_EQ(set_mode_request(0.0F, 3.0F, 0.0F), std::nullopt);
    EXPECT_EQ(set_mode_request(kCustom, 3.5F, 0.0F), std::nullopt);
    EXPECT_EQ(set_mode_request(kCustom, kNan, 0.0F), std::nullopt);
    EXPECT_EQ(set_mode_request(kCustom, 256.0F, 0.0F), std::nullopt);
}

TEST(GcsForwardPolicyTest, CommandFromNonControllerIsRejectedWithReason) {
    ForwardObservation observation = flying_controller();
    observation.from_controller = false;

    expect_reply(decide_forward(observation, command(MAV_CMD_NAV_LAND)),
                 MAV_RESULT_TEMPORARILY_REJECTED, "제어권 없음");
}

TEST(GcsForwardPolicyTest, MessagesFromNonControllerAreDropped) {
    ForwardObservation observation = grounded_controller();
    observation.from_controller = false;

    for (const GcsMessageKind kind : {GcsMessageKind::Heartbeat, GcsMessageKind::ManualControl,
                                      GcsMessageKind::ParamRequestRead, GcsMessageKind::ParamSet}) {
        EXPECT_EQ(decide_forward(observation, message(kind)).action, ForwardAction::Drop);
    }
}

TEST(GcsForwardPolicyTest, CommandIntIsAnsweredCommandLongOnly) {
    expect_reply(decide_forward(flying_controller(),
                                GcsMessageRequest{GcsMessageKind::CommandInt, MAV_CMD_NAV_LAND}),
                 MAV_RESULT_COMMAND_LONG_ONLY, "");
}

TEST(GcsForwardPolicyTest, LandIsForwardedInEveryState) {
    ForwardObservation observation = flying_controller();
    observation.sending_setpoints = true;
    observation.landing_locked = true;
    observation.pilot_in_control = true;
    observation.fc_status_known = false;

    expect_forward(decide_forward(observation, command(MAV_CMD_NAV_LAND)));
}

TEST(GcsForwardPolicyTest, UnlistedCommandIsDenied) {
    expect_reply(decide_forward(flying_controller(), command(MAV_CMD_REQUEST_MESSAGE)),
                 MAV_RESULT_DENIED, "지원하지 않는 명령");
}

TEST(GcsForwardPolicyTest, ForceArmIsDenied) {
    expect_forward(decide_forward(grounded_controller(), command(MAV_CMD_COMPONENT_ARM_DISARM, 1)));
    expect_reply(
        decide_forward(grounded_controller(), command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196.0F)),
        MAV_RESULT_DENIED, "강제 시동 거부");
    expect_reply(
        decide_forward(grounded_controller(), command(MAV_CMD_COMPONENT_ARM_DISARM, 0, kNan)),
        MAV_RESULT_DENIED, "강제 시동 거부");
}

TEST(GcsForwardPolicyTest, ArmTakeoffReturnAreRejectedWhileSendingSetpoints) {
    ForwardObservation observation = flying_controller();
    observation.sending_setpoints = true;

    for (const std::uint16_t mav_cmd : {static_cast<std::uint16_t>(MAV_CMD_COMPONENT_ARM_DISARM),
                                        static_cast<std::uint16_t>(MAV_CMD_NAV_TAKEOFF),
                                        static_cast<std::uint16_t>(MAV_CMD_NAV_RETURN_TO_LAUNCH)}) {
        expect_reply(decide_forward(observation, command(mav_cmd)), MAV_RESULT_TEMPORARILY_REJECTED,
                     "setpoint 송신 중");
    }
    expect_forward(decide_forward(flying_controller(), command(MAV_CMD_NAV_TAKEOFF)));
    expect_forward(decide_forward(flying_controller(), command(MAV_CMD_NAV_RETURN_TO_LAUNCH)));
}

TEST(GcsForwardPolicyTest, CommandsExceptLandAreRejectedInLandingLockAndPilotControl) {
    ForwardObservation locked = flying_controller();
    locked.landing_locked = true;
    expect_reply(decide_forward(locked, command(MAV_CMD_NAV_TAKEOFF)),
                 MAV_RESULT_TEMPORARILY_REJECTED, "착륙 잠금");

    ForwardObservation pilot = flying_controller();
    pilot.pilot_in_control = true;
    expect_reply(decide_forward(pilot, set_mode(3)), MAV_RESULT_TEMPORARILY_REJECTED,
                 "조종사 개입 중");
}

TEST(GcsForwardPolicyTest, ValueConditionIsCheckedBeforeStateCondition) {
    ForwardObservation locked = flying_controller();
    locked.landing_locked = true;

    expect_reply(decide_forward(locked, command(MAV_CMD_REQUEST_MESSAGE)), MAV_RESULT_DENIED,
                 "지원하지 않는 명령");
}

TEST(GcsForwardPolicyTest, PilotModesAreForwardedAndOthersDenied) {
    for (const float main_mode : {2.0F, 3.0F, 7.0F}) {  // Altitude, Position, Stabilized
        expect_forward(decide_forward(flying_controller(), set_mode(main_mode)));
    }
    expect_reply(decide_forward(flying_controller(), set_mode(6)), MAV_RESULT_DENIED,
                 "허용하지 않는 모드");  // Offboard
    expect_reply(decide_forward(flying_controller(), set_mode(4, 6)), MAV_RESULT_DENIED,
                 "허용하지 않는 모드");  // Land
    expect_reply(decide_forward(flying_controller(), set_mode(1)), MAV_RESULT_DENIED,
                 "허용하지 않는 모드");  // Manual
    expect_reply(decide_forward(flying_controller(), command(MAV_CMD_DO_SET_MODE, 0, 3)),
                 MAV_RESULT_DENIED, "허용하지 않는 모드");  // custom 아님
}

TEST(GcsForwardPolicyTest, PilotModeIsForwardedWhileSendingSetpoints) {
    ForwardObservation observation = flying_controller();
    observation.sending_setpoints = true;
    observation.follow_in_progress = true;

    // FC 가 새 모드를 보고하면 FcControl 이 setpoint 송신을 멈춘다 (icd 5.2)
    expect_forward(decide_forward(observation, set_mode(3)));
}

TEST(GcsForwardPolicyTest, HoldIsForwardedWhenOnboardIsNotControlling) {
    expect_forward(decide_forward(flying_controller(), set_mode(4, 3)));
}

TEST(GcsForwardPolicyTest, HoldWhileFollowingIsRejected) {
    ForwardObservation observation = flying_controller();
    observation.sending_setpoints = true;
    observation.follow_in_progress = true;
    expect_reply(decide_forward(observation, set_mode(4, 3)), MAV_RESULT_TEMPORARILY_REJECTED,
                 "추종 중: 추종 정지 사용");

    observation.follow_in_progress = false;
    observation.command_in_progress = true;
    expect_reply(decide_forward(observation, set_mode(4, 3)), MAV_RESULT_TEMPORARILY_REJECTED,
                 "추종 중: 추종 정지 사용");
}

TEST(GcsForwardPolicyTest, HoldWhileHoveringStopsOnboard) {
    ForwardObservation observation = flying_controller();
    observation.sending_setpoints = true;

    EXPECT_EQ(decide_forward(observation, set_mode(4, 3)).action, ForwardAction::Stop);
}

TEST(GcsForwardPolicyTest, HomeAndRebootRequireDisarmed) {
    for (const std::uint16_t mav_cmd :
         {static_cast<std::uint16_t>(MAV_CMD_DO_SET_HOME),
          static_cast<std::uint16_t>(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)}) {
        expect_forward(decide_forward(grounded_controller(), command(mav_cmd)));
        expect_reply(decide_forward(flying_controller(), command(mav_cmd)),
                     MAV_RESULT_TEMPORARILY_REJECTED, "시동 켜짐");
        ForwardObservation unknown = grounded_controller();
        unknown.fc_status_known = false;
        expect_reply(decide_forward(unknown, command(mav_cmd)), MAV_RESULT_TEMPORARILY_REJECTED,
                     "FC 상태 모름");
    }
}

TEST(GcsForwardPolicyTest, HeartbeatAndParamReadAreForwardedEvenInLandingLock) {
    ForwardObservation observation = flying_controller();
    observation.landing_locked = true;

    expect_forward(decide_forward(observation, message(GcsMessageKind::Heartbeat)));
    expect_forward(decide_forward(observation, message(GcsMessageKind::ParamRequestRead)));
}

TEST(GcsForwardPolicyTest, ParamSetRequiresDisarmedAndNoLandingLock) {
    expect_forward(decide_forward(grounded_controller(), message(GcsMessageKind::ParamSet)));
    EXPECT_EQ(decide_forward(flying_controller(), message(GcsMessageKind::ParamSet)).action,
              ForwardAction::Drop);
    ForwardObservation locked = grounded_controller();
    locked.landing_locked = true;
    EXPECT_EQ(decide_forward(locked, message(GcsMessageKind::ParamSet)).action,
              ForwardAction::Drop);
}

TEST(GcsForwardPolicyTest, ManualControlConditions) {
    expect_forward(decide_forward(flying_controller(), message(GcsMessageKind::ManualControl)));

    const auto dropped = [](const ForwardObservation& observation) {
        return decide_forward(observation, message(GcsMessageKind::ManualControl)).action ==
               ForwardAction::Drop;
    };
    ForwardObservation hold = flying_controller();
    hold.fc_mode = FlightMode::Hold;
    EXPECT_TRUE(dropped(hold));
    ForwardObservation sending = flying_controller();
    sending.sending_setpoints = true;
    EXPECT_TRUE(dropped(sending));
    ForwardObservation locked = flying_controller();
    locked.landing_locked = true;
    EXPECT_TRUE(dropped(locked));
    ForwardObservation pilot = flying_controller();
    pilot.pilot_in_control = true;
    EXPECT_TRUE(dropped(pilot));
    ForwardObservation unknown = flying_controller();
    unknown.fc_status_known = false;
    EXPECT_TRUE(dropped(unknown));
}

}  // namespace
}  // namespace argus::mavlink_gateway
