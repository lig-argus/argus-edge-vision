#include "argus/mavlink_gateway/argus_status_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/follow_activity.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::MissionState;

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kStart = 10 * kSecond;
constexpr std::int64_t kNow = 100 * kSecond;

vehicle_api::TrackTarget target() {
    vehicle_api::TrackTarget value;
    value.sensor = vehicle_api::Sensor::Ir;
    value.generation = 3;
    value.track_id = 17;
    return value;
}

// now 에서 age_ns 전에 받은 follow/status
vehicle_api::ReceivedFollowStatus received(MissionState state, std::int64_t age_ns = 0) {
    vehicle_api::ReceivedFollowStatus value;
    value.status.mission_state = state;
    value.status.target_state = vehicle_api::TargetState::Confirmed;
    value.status.selected = target();
    value.arrived_monotonic_ns = kNow - age_ns;
    return value;
}

StatusObservation observing(const vehicle_api::ReceivedFollowStatus& follow) {
    StatusObservation observation;
    observation.now_ns = kNow;
    observation.started_ns = kStart;
    observation.follow = follow;
    return observation;
}

StatusObservation with_status(MissionState state, std::int64_t age_ns = 0) {
    return observing(received(state, age_ns));
}

TEST(ArgusStatusPolicyTest, CopiesFollowStatusAndCommonValues) {
    StatusObservation observation = with_status(MissionState::FollowActive);
    observation.controller = 251;
    observation.health = ARGUS_HEALTH_FC_CONNECTED;
    observation.cpu_celsius = 52.5;

    const mavlink_argus_status_t status = compose_argus_status(observation);

    EXPECT_EQ(status.mission_state, ARGUS_MISSION_STATE_FOLLOW_ACTIVE);
    EXPECT_EQ(status.target_state, ARGUS_TARGET_STATE_CONFIRMED);
    EXPECT_EQ(status.target_id, 17U);
    EXPECT_EQ(status.target_generation, 3U);
    EXPECT_EQ(status.active_sensor, ARGUS_SENSOR_IR);
    EXPECT_EQ(status.control_sysid, 251);
    EXPECT_EQ(status.health, ARGUS_HEALTH_FC_CONNECTED);
    EXPECT_FLOAT_EQ(status.cpu_temp, 52.5F);
    EXPECT_EQ(status.time_boot_ms, 90'000U);
    EXPECT_TRUE(std::isnan(status.scan_yaw_done_rad));
    EXPECT_TRUE(std::isnan(status.scan_remaining_s));
    EXPECT_EQ(follow_fault(observation), FollowFault::None);
}

TEST(ArgusStatusPolicyTest, ActiveSensorOnlyWhilePreparingOrFollowing) {
    EXPECT_EQ(compose_argus_status(with_status(MissionState::Preparing)).active_sensor,
              ARGUS_SENSOR_IR);
    for (const MissionState state :
         {MissionState::Idle, MissionState::HoldTransition, MissionState::HoldWait,
          MissionState::ScanActive, MissionState::Manual}) {
        EXPECT_EQ(compose_argus_status(with_status(state)).active_sensor, ARGUS_SENSOR_UNKNOWN);
    }
}

TEST(ArgusStatusPolicyTest, NoControllerAndUnknownTemperature) {
    const mavlink_argus_status_t status = compose_argus_status(with_status(MissionState::Idle));

    EXPECT_EQ(status.control_sysid, 0);
    EXPECT_TRUE(std::isnan(status.cpu_temp));
}

TEST(ArgusStatusPolicyTest, ScanValuesWhileScanning) {
    vehicle_api::ReceivedFollowStatus follow = received(MissionState::ScanActive);
    vehicle_api::ScanProgress scan;
    scan.progress_deg = 90.0;
    scan.remaining_s = 12.0;
    follow.status.scan = scan;

    const mavlink_argus_status_t status = compose_argus_status(observing(follow));

    EXPECT_NEAR(status.scan_yaw_done_rad, 1.5707963F, 1e-6);
    EXPECT_FLOAT_EQ(status.scan_remaining_s, 12.0F);

    scan.remaining_s.reset();
    follow.status.scan = scan;
    EXPECT_TRUE(std::isnan(compose_argus_status(observing(follow)).scan_remaining_s));
}

TEST(ArgusStatusPolicyTest, IdleBeforeFirstStatusUntilTimeout) {
    StatusObservation observation;
    observation.started_ns = kStart;
    observation.now_ns = kStart + 3 * kSecond;
    EXPECT_EQ(compose_argus_status(observation).mission_state, ARGUS_MISSION_STATE_IDLE);

    observation.now_ns += 1;
    EXPECT_EQ(follow_fault(observation), FollowFault::StatusLost);
    EXPECT_EQ(compose_argus_status(observation).mission_state, ARGUS_MISSION_STATE_FAULT);
}

TEST(ArgusStatusPolicyTest, LostStatusIsFaultWithoutTarget) {
    const StatusObservation observation = with_status(MissionState::FollowActive, 3 * kSecond + 1);

    const mavlink_argus_status_t status = compose_argus_status(observation);

    EXPECT_EQ(follow_fault(observation), FollowFault::StatusLost);
    EXPECT_EQ(status.mission_state, ARGUS_MISSION_STATE_FAULT);
    EXPECT_EQ(status.target_state, ARGUS_TARGET_STATE_NONE);
    EXPECT_EQ(status.target_id, 0U);
    EXPECT_EQ(status.active_sensor, ARGUS_SENSOR_UNKNOWN);
}

TEST(ArgusStatusPolicyTest, SetpointTimeoutIsFaultUntilFollowStatusLeavesActiveState) {
    constexpr std::int64_t kTimeout = kNow - 500'000'000;
    StatusObservation before = with_status(MissionState::FollowActive, kSecond);  // 정지 전에 받음
    before.setpoint_timeout_ns = kTimeout;
    EXPECT_EQ(follow_fault(before), FollowFault::SetpointTimeout);
    EXPECT_FALSE(setpoint_timeout_cleared(before));

    StatusObservation still_active = with_status(MissionState::FollowActive);  // 정지 뒤에 받음
    still_active.setpoint_timeout_ns = kTimeout;
    EXPECT_EQ(follow_fault(still_active), FollowFault::SetpointTimeout);
    const mavlink_argus_status_t status = compose_argus_status(still_active);
    EXPECT_EQ(status.mission_state, ARGUS_MISSION_STATE_FAULT);
    EXPECT_EQ(status.target_id, 17U);  // 표적 값은 그대로

    StatusObservation stopped = with_status(MissionState::HoldWait);
    stopped.setpoint_timeout_ns = kTimeout;
    EXPECT_EQ(follow_fault(stopped), FollowFault::None);
    EXPECT_TRUE(setpoint_timeout_cleared(stopped));
}

TEST(ArgusStatusPolicyTest, UnknownMissionStateIsFault) {
    vehicle_api::ReceivedFollowStatus follow = received(MissionState::Unknown);
    follow.status.target_state = vehicle_api::TargetState::Unknown;
    const StatusObservation observation = observing(follow);

    const mavlink_argus_status_t status = compose_argus_status(observation);

    EXPECT_EQ(follow_fault(observation), FollowFault::UnknownState);
    EXPECT_EQ(status.mission_state, ARGUS_MISSION_STATE_FAULT);
    EXPECT_EQ(status.target_state, ARGUS_TARGET_STATE_NONE);
}

TEST(ArgusStatusPolicyTest, WasFollowingLooksOnlyAtState) {
    EXPECT_TRUE(was_following(with_status(MissionState::ScanActive, 10 * kSecond).follow));
    EXPECT_FALSE(was_following(with_status(MissionState::HoldWait).follow));
    EXPECT_FALSE(was_following(std::nullopt));
}

TEST(ArgusStatusPolicyTest, ControlFlagsShowLandingLockAndPilot) {
    StatusObservation observation = with_status(MissionState::Idle);
    EXPECT_EQ(compose_argus_status(observation).control_flags, 0);

    observation.landing_locked = true;
    EXPECT_EQ(compose_argus_status(observation).control_flags, ARGUS_CONTROL_FLAGS_LANDING_LOCKED);

    observation.pilot_in_control = true;
    EXPECT_EQ(compose_argus_status(observation).control_flags,
              ARGUS_CONTROL_FLAGS_LANDING_LOCKED | ARGUS_CONTROL_FLAGS_PILOT_IN_CONTROL);

    // follow/status 두절(FAULT)에서도 표시한다
    StatusObservation lost = with_status(MissionState::Idle, kFollowStatusTimeoutNs + 1);
    lost.pilot_in_control = true;
    EXPECT_EQ(compose_argus_status(lost).control_flags, ARGUS_CONTROL_FLAGS_PILOT_IN_CONTROL);
}

}  // namespace
}  // namespace argus::mavlink_gateway
