#include "argus/mavlink_gateway/rc_override_monitor.hpp"

#include <cstdint>
#include <optional>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/follow_activity.hpp"

namespace argus::mavlink_gateway {

RcOverrideMonitor::RcOverrideMonitor(const RcOverrideMonitorInputs& inputs, RcOverrideConfig config)
    : inputs_(inputs), config_(config) {
    forwarded_subscription_ = inputs_.forwarded_commands.subscribe(
        [this](const ForwardedCommandEvent& event) { on_forwarded_command(event); });
}

void RcOverrideMonitor::on_forwarded_command(const ForwardedCommandEvent& event) {
    if (event.command != MAV_CMD_DO_SET_MODE || event.stage != ForwardStage::Forwarded ||
        !event.requested_mode.has_value()) {
        return;
    }
    last_gcs_mode_command_ =
        GcsModeCommand{*event.requested_mode, inputs_.clock.now_monotonic_ns()};
}

std::optional<vehicle_api::FlightStatus> RcOverrideMonitor::current_status() const {
    const vehicle_api::VehicleState vehicle = inputs_.fc_state.latest();
    if (!vehicle.status.has_value() || vehicle.status->fc_epoch != inputs_.fc_state.fc_epoch()) {
        return std::nullopt;
    }
    return vehicle.status;
}

std::optional<FcSensorHealth> RcOverrideMonitor::current_sensor_health() const {
    const std::optional<FcSensorHealth> health = inputs_.rc_status.sensor_health();
    if (!health.has_value() || health->fc_epoch != inputs_.fc_state.fc_epoch()) {
        return std::nullopt;
    }
    return health;
}

void RcOverrideMonitor::tick() {
    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();

    const std::optional<RcChannels> channels = inputs_.rc_status.rc_channels();
    const bool channels_fresh = channels.has_value() &&
                                channels->fc_epoch == inputs_.fc_state.fc_epoch() &&
                                now_ns - channels->received_monotonic_ns <= config_.rc_max_age_ns;
    if (channels_fresh && sticks_neutral(*channels, config_)) {
        if (!neutral_since_ns_.has_value()) {
            neutral_since_ns_ = channels->received_monotonic_ns;
        }
    } else {
        neutral_since_ns_.reset();
    }

    const std::optional<vehicle_api::FlightStatus> status = current_status();
    if (!status.has_value()) {
        return;
    }
    if (!status->armed) {
        pilot_in_control_ = false;
    }
    const std::uint64_t epoch = status->fc_epoch;
    const bool changed =
        last_mode_.has_value() && last_mode_epoch_ == epoch && *last_mode_ != status->mode;
    if (changed &&
        is_pilot_takeover(ModeChange{status->mode, status->received_monotonic_ns, status->armed},
                          last_gcs_mode_command_, config_)) {
        pilot_in_control_ = true;
    }
    last_mode_ = status->mode;
    last_mode_epoch_ = epoch;
}

ReturnObservation RcOverrideMonitor::observe_return() const {
    ReturnObservation observation;
    observation.now_ns = inputs_.clock.now_monotonic_ns();
    observation.pilot_in_control = pilot_in_control_;
    observation.status = current_status();
    observation.neutral_since_ns = neutral_since_ns_;
    observation.sensor_health = current_sensor_health();
    observation.command_in_progress = inputs_.commands.command_in_progress();
    observation.follow_in_progress =
        follow_in_progress(inputs_.follow_status.latest(), observation.now_ns);
    observation.fc_entering = inputs_.fc_command_status.entering();
    return observation;
}

ReturnCheck RcOverrideMonitor::check_return() const {
    const ReturnObservation observation = observe_return();
    ReturnCheck check;
    check.block = check_return_conditions(observation, config_);
    if (observation.status.has_value()) {
        check.fc_mode = observation.status->mode;
    }
    return check;
}

bool RcOverrideMonitor::return_still_allowed() const {
    return return_procedure_allowed(observe_return(), config_);
}

}  // namespace argus::mavlink_gateway
