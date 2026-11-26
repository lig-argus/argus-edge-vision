#include "argus/mavlink_gateway/control_authority_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kNow = 100'000'000'000;
constexpr std::uint8_t kOwner = 250;
constexpr std::uint8_t kOther = 251;

ControlObservation free_observation() {
    ControlObservation observation;
    observation.now_ns = kNow;
    return observation;
}

ControlObservation owned_by(std::uint8_t owner, std::int64_t heartbeat_age_ns) {
    ControlObservation observation = free_observation();
    observation.controller = owner;
    observation.controller_last_heartbeat_ns = kNow - heartbeat_age_ns;
    return observation;
}

ControlDecision decide(const ControlObservation& observation, ControlEventKind kind,
                       std::uint8_t sysid) {
    return decide_control(observation, ControlEvent{kind, sysid});
}

// ------------------------------------------------------------------------- Heartbeat (부여)

TEST(ControlAuthorityPolicyTest, FirstHeartbeatGetsControlWhenNothingIsInProgress) {
    EXPECT_EQ(decide(free_observation(), ControlEventKind::Heartbeat, kOther).controller,
              std::optional<std::uint8_t>{kOther});
}

TEST(ControlAuthorityPolicyTest, HeartbeatDoesNotChangeExistingController) {
    const ControlObservation observation = owned_by(kOwner, 10'000'000'000);

    EXPECT_EQ(decide(observation, ControlEventKind::Heartbeat, kOther).controller,
              std::optional<std::uint8_t>{kOwner});
}

TEST(ControlAuthorityPolicyTest, HeartbeatGrantsNothingWhileBusy) {
    for (const auto busy :
         {&ControlObservation::command_in_progress, &ControlObservation::follow_in_progress,
          &ControlObservation::sending_setpoints}) {
        ControlObservation observation = free_observation();
        observation.*busy = true;
        EXPECT_EQ(decide(observation, ControlEventKind::Heartbeat, kOther).controller,
                  std::nullopt);
    }
}

TEST(ControlAuthorityPolicyTest, LandingLockDoesNotBlockFirstHeartbeat) {
    ControlObservation observation = free_observation();
    observation.landing_locked = true;  // icd 5.1 의 부여 조건에 착륙 잠금은 없다

    EXPECT_EQ(decide(observation, ControlEventKind::Heartbeat, kOther).controller,
              std::optional<std::uint8_t>{kOther});
}

// ------------------------------------------------------------------------------------- Request

TEST(ControlAuthorityPolicyTest, RequestFromControllerIsAccepted) {
    ControlObservation observation = owned_by(kOwner, 0);
    observation.follow_in_progress = true;

    const ControlDecision decision = decide(observation, ControlEventKind::Request, kOwner);

    EXPECT_EQ(decision.result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(decision.controller, std::optional<std::uint8_t>{kOwner});
}

TEST(ControlAuthorityPolicyTest, RequestIsRejectedWhileAliveControllerIsFlying) {
    ControlObservation observation = owned_by(kOwner, kGcsHeartbeatTimeoutNs);
    observation.flying = true;

    const ControlDecision decision = decide(observation, ControlEventKind::Request, kOther);

    EXPECT_EQ(decision.result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(decision.controller, std::optional<std::uint8_t>{kOwner});
}

TEST(ControlAuthorityPolicyTest, RequestTakesControlFromLostControllerWhileFlying) {
    ControlObservation observation = owned_by(kOwner, kGcsHeartbeatTimeoutNs + 1);
    observation.flying = true;

    const ControlDecision decision = decide(observation, ControlEventKind::Request, kOther);

    EXPECT_EQ(decision.result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(decision.controller, std::optional<std::uint8_t>{kOther});
}

TEST(ControlAuthorityPolicyTest, RequestTakesControlFromAliveControllerOnGround) {
    const ControlObservation observation = owned_by(kOwner, 0);

    EXPECT_EQ(decide(observation, ControlEventKind::Request, kOther).controller,
              std::optional<std::uint8_t>{kOther});
}

TEST(ControlAuthorityPolicyTest, RequestIsRejectedWhileCommandFollowOrLandingLock) {
    for (const auto condition :
         {&ControlObservation::command_in_progress, &ControlObservation::follow_in_progress,
          &ControlObservation::landing_locked}) {
        ControlObservation observation = free_observation();
        observation.*condition = true;
        const ControlDecision decision = decide(observation, ControlEventKind::Request, kOther);
        EXPECT_EQ(decision.result, MAV_RESULT_TEMPORARILY_REJECTED);
        EXPECT_EQ(decision.controller, std::nullopt);
    }
}

TEST(ControlAuthorityPolicyTest, SetpointSendingDoesNotRejectRequest) {
    ControlObservation observation = free_observation();
    observation.sending_setpoints = true;  // icd 5.1 의 거부 조건에 없다

    EXPECT_EQ(decide(observation, ControlEventKind::Request, kOther).result, MAV_RESULT_ACCEPTED);
}

// ------------------------------------------------------------------------------------- Release

TEST(ControlAuthorityPolicyTest, ReleaseByControllerClearsControl) {
    const ControlDecision decision = decide(owned_by(kOwner, 0), ControlEventKind::Release, kOwner);

    EXPECT_EQ(decision.result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(decision.controller, std::nullopt);
}

TEST(ControlAuthorityPolicyTest, ReleaseByOtherIsDeniedAndKeepsControl) {
    const ControlDecision decision = decide(owned_by(kOwner, 0), ControlEventKind::Release, kOther);

    EXPECT_EQ(decision.result, MAV_RESULT_DENIED);
    EXPECT_EQ(decision.controller, std::optional<std::uint8_t>{kOwner});
}

}  // namespace
}  // namespace argus::mavlink_gateway
