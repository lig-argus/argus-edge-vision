#include "argus/mavlink_gateway/landing_lock_policy.hpp"

#include <cstdint>

namespace argus::mavlink_gateway {

LandingLockStep on_land_forwarded(const LandingLockStatus& status, std::int64_t now_ns) {
    LandingLockStep step;
    step.started = !status.active;
    step.status.active = true;
    step.status.started_ns = now_ns;
    step.status.pilot_seen = status.active && status.pilot_seen;
    return step;
}

LandingLockStatus on_land_answered(const LandingLockStatus& status, LandAnswer answer) {
    if (!status.active || answer != LandAnswer::Rejected) {
        return status;
    }
    return LandingLockStatus{};
}

LandingLockStatus on_landing_lock_tick(const LandingLockStatus& status,
                                       const LandingLockObservation& observation) {
    if (!status.active) {
        return status;
    }
    const bool disarmed = observation.status.has_value() && !observation.status->armed &&
                          observation.status->received_monotonic_ns > status.started_ns;
    if (disarmed || (status.pilot_seen && !observation.pilot_in_control)) {
        return LandingLockStatus{};
    }
    LandingLockStatus next = status;
    next.pilot_seen = status.pilot_seen || observation.pilot_in_control;
    return next;
}

}  // namespace argus::mavlink_gateway
