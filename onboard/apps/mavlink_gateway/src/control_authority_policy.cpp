#include "argus/mavlink_gateway/control_authority_policy.hpp"

namespace argus::mavlink_gateway {
namespace {

bool controller_alive(const ControlObservation& observation) {
    return observation.controller.has_value() &&
           observation.controller_last_heartbeat_ns.has_value() &&
           observation.now_ns - *observation.controller_last_heartbeat_ns <= kGcsHeartbeatTimeoutNs;
}

ControlDecision on_heartbeat(const ControlObservation& observation, std::uint8_t sysid) {
    const bool busy = observation.command_in_progress || observation.follow_in_progress ||
                      observation.sending_setpoints;
    if (observation.controller.has_value() || busy) {
        return {observation.controller, MAV_RESULT_ACCEPTED};
    }
    return {sysid, MAV_RESULT_ACCEPTED};
}

ControlDecision on_request(const ControlObservation& observation, std::uint8_t sysid) {
    if (observation.controller == sysid) {
        return {observation.controller, MAV_RESULT_ACCEPTED};
    }
    const bool denied = (controller_alive(observation) && observation.flying) ||
                        observation.command_in_progress || observation.follow_in_progress ||
                        observation.landing_locked;
    if (denied) {
        return {observation.controller, MAV_RESULT_TEMPORARILY_REJECTED};
    }
    return {sysid, MAV_RESULT_ACCEPTED};
}

ControlDecision on_release(const ControlObservation& observation, std::uint8_t sysid) {
    if (observation.controller != sysid) {
        return {observation.controller, MAV_RESULT_DENIED};
    }
    return {std::nullopt, MAV_RESULT_ACCEPTED};
}

}  // namespace

ControlDecision decide_control(const ControlObservation& observation, const ControlEvent& event) {
    switch (event.kind) {
        case ControlEventKind::Heartbeat:
            return on_heartbeat(observation, event.sysid);
        case ControlEventKind::Request:
            return on_request(observation, event.sysid);
        case ControlEventKind::Release:
            return on_release(observation, event.sysid);
    }
    return {observation.controller, MAV_RESULT_FAILED};
}

}  // namespace argus::mavlink_gateway
