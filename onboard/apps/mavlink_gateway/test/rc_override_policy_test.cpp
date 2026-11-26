#include "argus/mavlink_gateway/rc_override_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/mavlink_gateway/fc_rc_status_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::int64_t kNowNs = 100'000'000'000;
constexpr std::int64_t kSecond = 1'000'000'000;
const RcOverrideConfig kConfig;

// 기본 설정의 4축(채널 1, 2, 4 는 중앙 1500, 채널 3 스로틀은 중앙 1500)이 모두 중앙인 값
RcChannels centered() {
    RcChannels channels;
    channels.channel_count = 8;
    channels.raw_us.fill(UINT16_MAX);
    for (std::size_t index = 0; index < 8; ++index) {
        channels.raw_us[index] = 1500;
    }
    channels.raw_us[2] = 1500;  // 스로틀 중앙 (MIN 1000, MAX 2000 의 가운데)
    return channels;
}

ReturnObservation ready() {
    ReturnObservation observation;
    observation.now_ns = kNowNs;
    observation.pilot_in_control = true;
    vehicle_api::FlightStatus status;
    status.mode = FlightMode::Position;
    status.armed = true;
    status.received_monotonic_ns = kNowNs - 100'000'000;
    observation.status = status;
    observation.neutral_since_ns = kNowNs - kSecond;
    FcSensorHealth health;
    health.rc_receiver_ok = true;
    health.local_position_ok = true;
    health.attitude_ok = true;
    health.received_monotonic_ns = kNowNs - 100'000'000;
    observation.sensor_health = health;
    return observation;
}

// observation 의 비행 상태와 SYS_STATUS 를 바꾼다
void change_status(ReturnObservation& observation,
                   const std::function<void(vehicle_api::FlightStatus&)>& change) {
    vehicle_api::FlightStatus status = observation.status.value_or(vehicle_api::FlightStatus{});
    change(status);
    observation.status = status;
}

void change_health(ReturnObservation& observation,
                   const std::function<void(FcSensorHealth&)>& change) {
    FcSensorHealth health = observation.sensor_health.value_or(FcSensorHealth{});
    change(health);
    observation.sensor_health = health;
}

TEST(RcOverridePolicyTest, NormalizesAroundTrimOrMidpoint) {
    const StickCalibration roll{1, 1000, 2000, 1500, false, false};
    EXPECT_DOUBLE_EQ(normalized_stick(1500, roll).value_or(9.0), 0.0);
    EXPECT_DOUBLE_EQ(normalized_stick(2000, roll).value_or(9.0), 1.0);
    EXPECT_DOUBLE_EQ(normalized_stick(1000, roll).value_or(9.0), -1.0);
    EXPECT_DOUBLE_EQ(normalized_stick(1750, roll).value_or(9.0), 0.5);

    // TRIM 이 중앙이 아니면 양쪽을 따로 정규화한다
    const StickCalibration offset{1, 1000, 2000, 1600, false, false};
    EXPECT_DOUBLE_EQ(normalized_stick(1300, offset).value_or(9.0), -0.5);
    EXPECT_DOUBLE_EQ(normalized_stick(1800, offset).value_or(9.0), 0.5);

    const StickCalibration reversed{1, 1000, 2000, 1500, true, false};
    EXPECT_DOUBLE_EQ(normalized_stick(2000, reversed).value_or(9.0), -1.0);

    // 스로틀: TRIM 은 아래 끝값이지만 중앙은 MIN, MAX 의 가운데
    const StickCalibration throttle{3, 1000, 2000, 1000, false, true};
    EXPECT_DOUBLE_EQ(normalized_stick(1500, throttle).value_or(9.0), 0.0);
}

TEST(RcOverridePolicyTest, RejectsMissingValueOrBadCalibration) {
    const StickCalibration roll{1, 1000, 2000, 1500, false, false};
    EXPECT_FALSE(normalized_stick(0, roll).has_value());
    EXPECT_FALSE(normalized_stick(UINT16_MAX, roll).has_value());
    EXPECT_FALSE(normalized_stick(1500, StickCalibration{1, 2000, 1000, 1500, false, false}));
    EXPECT_FALSE(normalized_stick(1500, StickCalibration{1, 1000, 2000, 1000, false, false}));
}

TEST(RcOverridePolicyTest, SticksAreNeutralOnlyWithinBandOnAllAxes) {
    EXPECT_TRUE(sticks_neutral(centered(), kConfig));

    RcChannels roll_at_band = centered();
    roll_at_band.raw_us[0] = 1550;  // 0.1
    EXPECT_TRUE(sticks_neutral(roll_at_band, kConfig));

    RcChannels roll_moved = centered();
    roll_moved.raw_us[0] = 1551;
    EXPECT_FALSE(sticks_neutral(roll_moved, kConfig));

    RcChannels throttle_low = centered();
    throttle_low.raw_us[2] = 1000;  // 스로틀이 아래 끝 (TRIM 값) 이면 중립이 아님
    EXPECT_FALSE(sticks_neutral(throttle_low, kConfig));

    RcChannels no_signal = centered();
    no_signal.channel_count = 0;
    EXPECT_FALSE(sticks_neutral(no_signal, kConfig));

    RcChannels few_channels = centered();
    few_channels.channel_count = 3;  // 요(채널 4) 없음
    EXPECT_FALSE(sticks_neutral(few_channels, kConfig));
}

TEST(RcOverridePolicyTest, ManualModeChangeWithoutGcsCommandIsTakeover) {
    for (const FlightMode mode :
         {FlightMode::Position, FlightMode::Altitude, FlightMode::Stabilized, FlightMode::Manual}) {
        EXPECT_TRUE(is_pilot_takeover(ModeChange{mode, kNowNs, true}, std::nullopt, kConfig));
    }
    // failsafe 가 고르는 자동 모드와 Offboard 는 개입이 아니다
    for (const FlightMode mode :
         {FlightMode::Hold, FlightMode::Return, FlightMode::Land, FlightMode::Offboard}) {
        EXPECT_FALSE(is_pilot_takeover(ModeChange{mode, kNowNs, true}, std::nullopt, kConfig));
    }
    // 시동이 꺼져 있으면 개입이 아니다
    EXPECT_FALSE(
        is_pilot_takeover(ModeChange{FlightMode::Position, kNowNs, false}, std::nullopt, kConfig));
}

TEST(RcOverridePolicyTest, GcsModeCommandWithinWindowIsNotTakeover) {
    const ModeChange change{FlightMode::Position, kNowNs, true};
    const std::int64_t window = kConfig.gcs_mode_command_window_ns;

    EXPECT_FALSE(
        is_pilot_takeover(change, GcsModeCommand{FlightMode::Position, kNowNs - window}, kConfig));
    EXPECT_TRUE(is_pilot_takeover(change, GcsModeCommand{FlightMode::Position, kNowNs - window - 1},
                                  kConfig));
    // 다른 모드로 바꾸는 명령이었으면 개입이다
    EXPECT_TRUE(
        is_pilot_takeover(change, GcsModeCommand{FlightMode::Altitude, kNowNs - 1}, kConfig));
    // 바뀐 모드를 받은 뒤에 보낸 명령은 이 변경의 원인이 아니다
    EXPECT_TRUE(
        is_pilot_takeover(change, GcsModeCommand{FlightMode::Position, kNowNs + 1}, kConfig));
}

TEST(RcOverridePolicyTest, ReturnConditionsAreCheckedInOrder) {
    EXPECT_EQ(check_return_conditions(ready(), kConfig), ReturnBlock::None);

    ReturnObservation observation = ready();
    observation.pilot_in_control = false;
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::NotPilot);

    for (const FlightMode mode : {FlightMode::Land, FlightMode::Altitude, FlightMode::Offboard}) {
        observation = ready();
        change_status(observation, [&](vehicle_api::FlightStatus& status) { status.mode = mode; });
        EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::FcMode);
    }
    observation = ready();
    change_status(observation,
                  [&](vehicle_api::FlightStatus& status) { status.mode = FlightMode::Hold; });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::None);
    change_status(observation, [&](vehicle_api::FlightStatus& status) {
        status.received_monotonic_ns = kNowNs - kConfig.status_max_age_ns - 1;
    });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::FcMode);

    observation = ready();
    observation.neutral_since_ns = kNowNs - kConfig.neutral_hold_ns + 1;
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::Sticks);
    observation.neutral_since_ns.reset();
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::Sticks);

    observation = ready();
    change_health(observation, [&](FcSensorHealth& health) { health.rc_receiver_ok = false; });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::RcReceiver);
    observation = ready();
    change_health(observation, [&](FcSensorHealth& health) {
        health.received_monotonic_ns = kNowNs - kConfig.sensor_max_age_ns - 1;
    });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::RcReceiver);

    observation = ready();
    change_health(observation, [&](FcSensorHealth& health) { health.local_position_ok = false; });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::Estimate);
    observation = ready();
    change_health(observation, [&](FcSensorHealth& health) { health.attitude_ok = false; });
    EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::Estimate);

    for (bool ReturnObservation::* busy :
         {&ReturnObservation::command_in_progress, &ReturnObservation::follow_in_progress,
          &ReturnObservation::fc_entering}) {
        observation = ready();
        observation.*busy = true;
        EXPECT_EQ(check_return_conditions(observation, kConfig), ReturnBlock::Busy);
    }
}

TEST(RcOverridePolicyTest, ProcedureIgnoresItsOwnActivityAndHoldTime) {
    ReturnObservation observation = ready();
    observation.command_in_progress = true;
    observation.fc_entering = true;
    observation.neutral_since_ns = kNowNs;  // 지금 중립이면 유지 시간은 보지 않는다
    EXPECT_TRUE(return_procedure_allowed(observation, kConfig));

    observation.neutral_since_ns.reset();
    EXPECT_FALSE(return_procedure_allowed(observation, kConfig));

    observation = ready();
    observation.pilot_in_control = false;
    EXPECT_FALSE(return_procedure_allowed(observation, kConfig));

    observation = ready();
    change_health(observation, [&](FcSensorHealth& health) { health.rc_receiver_ok = false; });
    EXPECT_FALSE(return_procedure_allowed(observation, kConfig));
}

TEST(RcOverridePolicyTest, HasTextForEveryBlock) {
    EXPECT_TRUE(to_text(ReturnBlock::None).empty());
    for (std::int32_t value = 1; value <= static_cast<std::int32_t>(ReturnBlock::Busy); ++value) {
        EXPECT_FALSE(to_text(static_cast<ReturnBlock>(value)).empty()) << value;
    }
}

}  // namespace
}  // namespace argus::mavlink_gateway
