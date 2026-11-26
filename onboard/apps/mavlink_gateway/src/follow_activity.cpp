#include "argus/mavlink_gateway/follow_activity.hpp"

namespace argus::mavlink_gateway {

bool follow_in_progress(const std::optional<vehicle_api::ReceivedFollowStatus>& latest,
                        std::int64_t now_ns) {
    if (!latest.has_value() || now_ns - latest->arrived_monotonic_ns > kFollowStatusTimeoutNs) {
        return false;
    }
    switch (latest->status.mission_state) {
        case vehicle_api::MissionState::Preparing:
        case vehicle_api::MissionState::FollowActive:
        case vehicle_api::MissionState::HoldTransition:
        case vehicle_api::MissionState::ScanActive:
            return true;
        case vehicle_api::MissionState::Idle:
        case vehicle_api::MissionState::HoldWait:
        case vehicle_api::MissionState::Manual:
        case vehicle_api::MissionState::Fault:
        case vehicle_api::MissionState::Unknown:
            return false;
    }
    return false;
}

}  // namespace argus::mavlink_gateway
