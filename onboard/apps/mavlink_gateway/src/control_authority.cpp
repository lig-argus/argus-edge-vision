#include "argus/mavlink_gateway/control_authority.hpp"

#include <cstdint>
#include <optional>

#include "argus/mavlink_gateway/follow_activity.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

ControlAuthority::ControlAuthority(const ControlAuthorityInputs& inputs) : inputs_(inputs) {
    heartbeat_subscription_ = inputs_.messages.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this](const mavlink_message_t& /*message*/, const GcsPeer& sender) {
            apply(ControlEventKind::Heartbeat, sender.sysid);
        });
}

MAV_RESULT ControlAuthority::request(const GcsPeer& sender, ControlAction action) {
    const ControlEventKind kind =
        action == ControlAction::Request ? ControlEventKind::Request : ControlEventKind::Release;
    return apply(kind, sender.sysid).result;
}

ControlObservation ControlAuthority::observe() const {
    ControlObservation observation;
    observation.now_ns = inputs_.clock.now_monotonic_ns();
    observation.controller = controller_;
    if (controller_.has_value()) {
        observation.controller_last_heartbeat_ns = inputs_.presence.last_heartbeat_ns(*controller_);
    }
    const vehicle_api::VehicleState vehicle = inputs_.fc_state.latest();
    observation.flying = vehicle.status.has_value() && vehicle.status->in_air;
    observation.command_in_progress = inputs_.commands.command_in_progress();
    observation.follow_in_progress =
        follow_in_progress(inputs_.follow_status.latest(), observation.now_ns);
    observation.sending_setpoints = inputs_.fc_command_status.sending_setpoints();
    observation.landing_locked = inputs_.landing_lock.active();
    return observation;
}

ControlDecision ControlAuthority::apply(ControlEventKind kind, std::uint8_t sysid) {
    const ControlDecision decision = decide_control(observe(), ControlEvent{kind, sysid});
    controller_ = decision.controller;
    return decision;
}

}  // namespace argus::mavlink_gateway
