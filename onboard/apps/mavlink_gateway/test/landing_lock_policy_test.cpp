#include "argus/mavlink_gateway/landing_lock_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kStartNs = 100'000'000'000;

LandingLockStatus locked() { return on_land_forwarded(LandingLockStatus{}, kStartNs).status; }

LandingLockObservation status_at(std::int64_t received_ns, bool armed) {
    LandingLockObservation observation;
    vehicle_api::FlightStatus status;
    status.armed = armed;
    status.received_monotonic_ns = received_ns;
    observation.status = status;
    return observation;
}

TEST(LandingLockPolicyTest, ForwardedLandStartsLockOnce) {
    const LandingLockStep first = on_land_forwarded(LandingLockStatus{}, kStartNs);
    EXPECT_TRUE(first.status.active);
    EXPECT_TRUE(first.started);
    EXPECT_EQ(first.status.started_ns, kStartNs);

    const LandingLockStep again = on_land_forwarded(first.status, kStartNs + 1000);
    EXPECT_TRUE(again.status.active);
    EXPECT_FALSE(again.started);
    EXPECT_EQ(again.status.started_ns, kStartNs + 1000);
}

TEST(LandingLockPolicyTest, OnlyFinalRejectionFromFcReleasesLock) {
    EXPECT_TRUE(on_land_answered(locked(), LandAnswer::InProgress).active);
    EXPECT_TRUE(on_land_answered(locked(), LandAnswer::Accepted).active);
    EXPECT_FALSE(on_land_answered(locked(), LandAnswer::Rejected).active);
    // 잠금이 아닌 때의 응답은 아무것도 바꾸지 않는다
    EXPECT_FALSE(on_land_answered(LandingLockStatus{}, LandAnswer::Accepted).active);
}

TEST(LandingLockPolicyTest, DisarmReceivedAfterStartReleasesLock) {
    EXPECT_TRUE(on_landing_lock_tick(locked(), status_at(kStartNs + 1, true)).active);
    // 시작 전에 받은 비행 상태는 쓰지 않는다
    EXPECT_TRUE(on_landing_lock_tick(locked(), status_at(kStartNs, false)).active);
    EXPECT_FALSE(on_landing_lock_tick(locked(), status_at(kStartNs + 1, false)).active);
    EXPECT_TRUE(on_landing_lock_tick(locked(), LandingLockObservation{}).active);
}

TEST(LandingLockPolicyTest, ReturnAfterPilotTakeoverReleasesLock) {
    LandingLockObservation pilot = status_at(kStartNs + 1, true);
    pilot.pilot_in_control = true;
    const LandingLockStatus seen = on_landing_lock_tick(locked(), pilot);
    EXPECT_TRUE(seen.active);
    EXPECT_TRUE(seen.pilot_seen);

    pilot.pilot_in_control = false;
    EXPECT_FALSE(on_landing_lock_tick(seen, pilot).active);
    // 개입을 본 적이 없으면 개입이 없다는 것만으로 풀지 않는다
    EXPECT_TRUE(on_landing_lock_tick(locked(), pilot).active);
}

}  // namespace
}  // namespace argus::mavlink_gateway
