#include "fc_telemetry_codec.hpp"

#include "argus_msg/fc_attitude.pb.h"
#include "argus_msg/fc_mode_event.pb.h"
#include "argus_msg/fc_state.pb.h"

namespace argus::vehicle_api_bus {
namespace {

msg::OffboardExitCause to_message(vehicle_api::OffboardExitCause cause) {
    using vehicle_api::OffboardExitCause;
    switch (cause) {
        case OffboardExitCause::None:
            return msg::OFFBOARD_EXIT_CAUSE_NONE;
        case OffboardExitCause::PilotSwitch:
            return msg::OFFBOARD_EXIT_CAUSE_PILOT_SWITCH;
        case OffboardExitCause::Failsafe:
            return msg::OFFBOARD_EXIT_CAUSE_FAILSAFE;
        case OffboardExitCause::Unknown:
            return msg::OFFBOARD_EXIT_CAUSE_UNKNOWN;
    }
    return msg::OFFBOARD_EXIT_CAUSE_UNKNOWN;
}

// 이 코드에 정의되지 않은 값(메시지를 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::OffboardExitCause from_message(msg::OffboardExitCause cause) {
    using vehicle_api::OffboardExitCause;
    switch (cause) {
        case msg::OFFBOARD_EXIT_CAUSE_NONE:
            return OffboardExitCause::None;
        case msg::OFFBOARD_EXIT_CAUSE_PILOT_SWITCH:
            return OffboardExitCause::PilotSwitch;
        case msg::OFFBOARD_EXIT_CAUSE_FAILSAFE:
            return OffboardExitCause::Failsafe;
        default:
            return OffboardExitCause::Unknown;
    }
}

}  // namespace

bus::Bytes encode_fc_attitude(const vehicle_api::Attitude& attitude, std::uint64_t seq) {
    msg::FcAttitude message;
    message.set_seq(seq);
    message.set_roll_rad(attitude.roll_rad);
    message.set_pitch_rad(attitude.pitch_rad);
    message.set_yaw_rad(attitude.yaw_rad);
    message.set_roll_rate_rad_s(attitude.roll_rate_rad_s);
    message.set_pitch_rate_rad_s(attitude.pitch_rate_rad_s);
    message.set_yaw_rate_rad_s(attitude.yaw_rate_rad_s);
    message.set_fc_time_boot_ms(attitude.fc_time_boot_ms);
    message.set_received_monotonic_ns(attitude.received_monotonic_ns);
    if (attitude.fc_time_monotonic_ns) {
        message.set_fc_time_monotonic_ns(*attitude.fc_time_monotonic_ns);
    }
    message.set_timesync_valid(attitude.timesync_valid);
    message.set_fc_epoch(attitude.fc_epoch);
    return to_bytes(message);
}

std::optional<Decoded<vehicle_api::Attitude>> decode_fc_attitude(const bus::Bytes& body) {
    msg::FcAttitude message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    Decoded<vehicle_api::Attitude> decoded;
    decoded.seq = message.seq();
    vehicle_api::Attitude& attitude = decoded.value;
    attitude.roll_rad = message.roll_rad();
    attitude.pitch_rad = message.pitch_rad();
    attitude.yaw_rad = message.yaw_rad();
    attitude.roll_rate_rad_s = message.roll_rate_rad_s();
    attitude.pitch_rate_rad_s = message.pitch_rate_rad_s();
    attitude.yaw_rate_rad_s = message.yaw_rate_rad_s();
    attitude.fc_time_boot_ms = message.fc_time_boot_ms();
    attitude.received_monotonic_ns = message.received_monotonic_ns();
    if (message.has_fc_time_monotonic_ns()) {
        attitude.fc_time_monotonic_ns = message.fc_time_monotonic_ns();
    }
    attitude.timesync_valid = message.timesync_valid();
    attitude.fc_epoch = message.fc_epoch();
    return decoded;
}

bus::Bytes encode_fc_state(const vehicle_api::VehicleState& state, std::uint64_t seq) {
    msg::FcState message;
    message.set_seq(seq);
    if (state.local) {
        msg::FcState::LocalPosition& local = *message.mutable_local();
        local.set_north_m(state.local->north_m);
        local.set_east_m(state.local->east_m);
        local.set_down_m(state.local->down_m);
        local.set_vel_north_m_s(state.local->vel_north_m_s);
        local.set_vel_east_m_s(state.local->vel_east_m_s);
        local.set_vel_down_m_s(state.local->vel_down_m_s);
        local.set_received_monotonic_ns(state.local->received_monotonic_ns);
        local.set_fc_epoch(state.local->fc_epoch);
    }
    if (state.downward_distance) {
        msg::FcState::DownwardDistance& distance = *message.mutable_downward_distance();
        distance.set_distance_m(state.downward_distance->distance_m);
        distance.set_received_monotonic_ns(state.downward_distance->received_monotonic_ns);
        distance.set_fc_epoch(state.downward_distance->fc_epoch);
    }
    if (state.global) {
        msg::FcState::GlobalPosition& global = *message.mutable_global();
        global.set_lat_deg(state.global->lat_deg);
        global.set_lon_deg(state.global->lon_deg);
        global.set_alt_m(state.global->alt_m);
        global.set_received_monotonic_ns(state.global->received_monotonic_ns);
        global.set_fc_epoch(state.global->fc_epoch);
    }
    if (state.status) {
        msg::FcState::FlightStatus& status = *message.mutable_status();
        status.set_mode(to_message(state.status->mode));
        status.set_armed(state.status->armed);
        status.set_in_air(state.status->in_air);
        status.set_offboard_hover(state.status->offboard_hover);
        status.set_received_monotonic_ns(state.status->received_monotonic_ns);
        status.set_fc_epoch(state.status->fc_epoch);
    }
    return to_bytes(message);
}

std::optional<Decoded<vehicle_api::VehicleState>> decode_fc_state(const bus::Bytes& body) {
    msg::FcState message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    Decoded<vehicle_api::VehicleState> decoded;
    decoded.seq = message.seq();
    vehicle_api::VehicleState& state = decoded.value;
    if (message.has_local()) {
        vehicle_api::LocalPosition local;
        local.north_m = message.local().north_m();
        local.east_m = message.local().east_m();
        local.down_m = message.local().down_m();
        local.vel_north_m_s = message.local().vel_north_m_s();
        local.vel_east_m_s = message.local().vel_east_m_s();
        local.vel_down_m_s = message.local().vel_down_m_s();
        local.received_monotonic_ns = message.local().received_monotonic_ns();
        local.fc_epoch = message.local().fc_epoch();
        state.local = local;
    }
    if (message.has_downward_distance()) {
        vehicle_api::DownwardDistance distance;
        distance.distance_m = message.downward_distance().distance_m();
        distance.received_monotonic_ns = message.downward_distance().received_monotonic_ns();
        distance.fc_epoch = message.downward_distance().fc_epoch();
        state.downward_distance = distance;
    }
    if (message.has_global()) {
        vehicle_api::GlobalPosition global;
        global.lat_deg = message.global().lat_deg();
        global.lon_deg = message.global().lon_deg();
        global.alt_m = message.global().alt_m();
        global.received_monotonic_ns = message.global().received_monotonic_ns();
        global.fc_epoch = message.global().fc_epoch();
        state.global = global;
    }
    if (message.has_status()) {
        vehicle_api::FlightStatus status;
        status.mode = from_message(message.status().mode());
        status.armed = message.status().armed();
        status.in_air = message.status().in_air();
        status.offboard_hover = message.status().offboard_hover();
        status.received_monotonic_ns = message.status().received_monotonic_ns();
        status.fc_epoch = message.status().fc_epoch();
        state.status = status;
    }
    return decoded;
}

bus::Bytes encode_fc_mode_event(const vehicle_api::FlightModeEvent& event, std::uint64_t seq) {
    msg::FcModeEvent message;
    message.set_seq(seq);
    message.set_mode(to_message(event.mode));
    if (event.request_command_id) {
        message.set_request_command_id(event.request_command_id->value);
    }
    message.set_exit_cause(to_message(event.exit_cause));
    message.set_received_monotonic_ns(event.received_monotonic_ns);
    return to_bytes(message);
}

std::optional<Decoded<vehicle_api::FlightModeEvent>> decode_fc_mode_event(const bus::Bytes& body) {
    msg::FcModeEvent message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    Decoded<vehicle_api::FlightModeEvent> decoded;
    decoded.seq = message.seq();
    vehicle_api::FlightModeEvent& event = decoded.value;
    event.mode = from_message(message.mode());
    if (message.has_request_command_id()) {
        event.request_command_id = vehicle_api::CommandId{message.request_command_id()};
    }
    event.exit_cause = from_message(message.exit_cause());
    event.received_monotonic_ns = message.received_monotonic_ns();
    return decoded;
}

}  // namespace argus::vehicle_api_bus
