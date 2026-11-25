#include "fc_telemetry_idl.hpp"

#include "idl_common.hpp"

namespace argus::vehicle_api_bus {
namespace {

idl::OffboardExitCause to_idl(vehicle_api::OffboardExitCause cause) {
    using vehicle_api::OffboardExitCause;
    switch (cause) {
        case OffboardExitCause::None:
            return idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_NONE;
        case OffboardExitCause::PilotSwitch:
            return idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_PILOT_SWITCH;
        case OffboardExitCause::Failsafe:
            return idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_FAILSAFE;
        case OffboardExitCause::Unknown:
            return idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_UNKNOWN;
    }
    return idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_UNKNOWN;
}

// 이 코드에 정의되지 않은 값(표본을 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::OffboardExitCause from_idl(idl::OffboardExitCause cause) {
    using vehicle_api::OffboardExitCause;
    switch (cause) {
        case idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_NONE:
            return OffboardExitCause::None;
        case idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_PILOT_SWITCH:
            return OffboardExitCause::PilotSwitch;
        case idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_FAILSAFE:
            return OffboardExitCause::Failsafe;
        case idl::OffboardExitCause::OFFBOARD_EXIT_CAUSE_UNKNOWN:
            return OffboardExitCause::Unknown;
    }
    return OffboardExitCause::Unknown;
}

}  // namespace

void to_idl_fc_attitude(const vehicle_api::Attitude& attitude, idl::FcAttitude& sample) {
    sample.roll_rad(attitude.roll_rad);
    sample.pitch_rad(attitude.pitch_rad);
    sample.yaw_rad(attitude.yaw_rad);
    sample.roll_rate_rad_s(attitude.roll_rate_rad_s);
    sample.pitch_rate_rad_s(attitude.pitch_rate_rad_s);
    sample.yaw_rate_rad_s(attitude.yaw_rate_rad_s);
    sample.fc_time_boot_ms(attitude.fc_time_boot_ms);
    sample.received_monotonic_ns(attitude.received_monotonic_ns);
    sample.fc_time_monotonic_ns().reset();
    if (attitude.fc_time_monotonic_ns) {
        sample.fc_time_monotonic_ns(*attitude.fc_time_monotonic_ns);
    }
    sample.timesync_valid(attitude.timesync_valid);
    sample.fc_epoch(attitude.fc_epoch);
}

bool from_idl_fc_attitude(const idl::FcAttitude& sample, vehicle_api::Attitude& attitude) {
    attitude.roll_rad = sample.roll_rad();
    attitude.pitch_rad = sample.pitch_rad();
    attitude.yaw_rad = sample.yaw_rad();
    attitude.roll_rate_rad_s = sample.roll_rate_rad_s();
    attitude.pitch_rate_rad_s = sample.pitch_rate_rad_s();
    attitude.yaw_rate_rad_s = sample.yaw_rate_rad_s();
    attitude.fc_time_boot_ms = sample.fc_time_boot_ms();
    attitude.received_monotonic_ns = sample.received_monotonic_ns();
    attitude.fc_time_monotonic_ns.reset();
    if (sample.fc_time_monotonic_ns().has_value()) {
        attitude.fc_time_monotonic_ns = *sample.fc_time_monotonic_ns();
    }
    attitude.timesync_valid = sample.timesync_valid();
    attitude.fc_epoch = sample.fc_epoch();
    return true;
}

void to_idl_fc_state(const vehicle_api::VehicleState& state, idl::FcState& sample) {
    sample.local_position().reset();
    if (state.local) {
        idl::LocalPosition local;
        local.north_m(state.local->north_m);
        local.east_m(state.local->east_m);
        local.down_m(state.local->down_m);
        local.vel_north_m_s(state.local->vel_north_m_s);
        local.vel_east_m_s(state.local->vel_east_m_s);
        local.vel_down_m_s(state.local->vel_down_m_s);
        local.received_monotonic_ns(state.local->received_monotonic_ns);
        local.fc_epoch(state.local->fc_epoch);
        sample.local_position(local);
    }
    sample.downward_distance().reset();
    if (state.downward_distance) {
        idl::DownwardDistance distance;
        distance.distance_m(state.downward_distance->distance_m);
        distance.received_monotonic_ns(state.downward_distance->received_monotonic_ns);
        distance.fc_epoch(state.downward_distance->fc_epoch);
        sample.downward_distance(distance);
    }
    sample.global_position().reset();
    if (state.global) {
        idl::GlobalPosition global;
        global.lat_deg(state.global->lat_deg);
        global.lon_deg(state.global->lon_deg);
        global.alt_m(state.global->alt_m);
        global.received_monotonic_ns(state.global->received_monotonic_ns);
        global.fc_epoch(state.global->fc_epoch);
        sample.global_position(global);
    }
    sample.status().reset();
    if (state.status) {
        idl::FlightStatus status;
        status.mode(to_idl(state.status->mode));
        status.armed(state.status->armed);
        status.in_air(state.status->in_air);
        status.offboard_hover(state.status->offboard_hover);
        status.received_monotonic_ns(state.status->received_monotonic_ns);
        status.fc_epoch(state.status->fc_epoch);
        sample.status(status);
    }
}

bool from_idl_fc_state(const idl::FcState& sample, vehicle_api::VehicleState& state) {
    state = vehicle_api::VehicleState{};
    if (sample.local_position().has_value()) {
        const idl::LocalPosition& received = *sample.local_position();
        vehicle_api::LocalPosition local;
        local.north_m = received.north_m();
        local.east_m = received.east_m();
        local.down_m = received.down_m();
        local.vel_north_m_s = received.vel_north_m_s();
        local.vel_east_m_s = received.vel_east_m_s();
        local.vel_down_m_s = received.vel_down_m_s();
        local.received_monotonic_ns = received.received_monotonic_ns();
        local.fc_epoch = received.fc_epoch();
        state.local = local;
    }
    if (sample.downward_distance().has_value()) {
        const idl::DownwardDistance& received = *sample.downward_distance();
        vehicle_api::DownwardDistance distance;
        distance.distance_m = received.distance_m();
        distance.received_monotonic_ns = received.received_monotonic_ns();
        distance.fc_epoch = received.fc_epoch();
        state.downward_distance = distance;
    }
    if (sample.global_position().has_value()) {
        const idl::GlobalPosition& received = *sample.global_position();
        vehicle_api::GlobalPosition global;
        global.lat_deg = received.lat_deg();
        global.lon_deg = received.lon_deg();
        global.alt_m = received.alt_m();
        global.received_monotonic_ns = received.received_monotonic_ns();
        global.fc_epoch = received.fc_epoch();
        state.global = global;
    }
    if (sample.status().has_value()) {
        const idl::FlightStatus& received = *sample.status();
        vehicle_api::FlightStatus status;
        status.mode = from_idl(received.mode());
        status.armed = received.armed();
        status.in_air = received.in_air();
        status.offboard_hover = received.offboard_hover();
        status.received_monotonic_ns = received.received_monotonic_ns();
        status.fc_epoch = received.fc_epoch();
        state.status = status;
    }
    return true;
}

void to_idl_fc_mode_event(const vehicle_api::FlightModeEvent& event, idl::FcModeEvent& sample) {
    sample.mode(to_idl(event.mode));
    sample.request_command_id().reset();
    if (event.request_command_id) {
        sample.request_command_id(event.request_command_id->value);
    }
    sample.exit_cause(to_idl(event.exit_cause));
    sample.received_monotonic_ns(event.received_monotonic_ns);
}

bool from_idl_fc_mode_event(const idl::FcModeEvent& sample, vehicle_api::FlightModeEvent& event) {
    event.mode = from_idl(sample.mode());
    event.request_command_id.reset();
    if (sample.request_command_id().has_value()) {
        event.request_command_id = vehicle_api::CommandId{*sample.request_command_id()};
    }
    event.exit_cause = from_idl(sample.exit_cause());
    event.received_monotonic_ns = sample.received_monotonic_ns();
    return true;
}

}  // namespace argus::vehicle_api_bus
