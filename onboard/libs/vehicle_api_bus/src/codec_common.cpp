#include "codec_common.hpp"

namespace argus::vehicle_api_bus {

void to_message(const vehicle_api::RequestId& id, msg::RequestId& message) {
    message.set_command_id(id.command_id.value);
    message.set_session_id(id.session_id);
    message.set_seq(id.seq);
    message.set_fc_epoch(id.fc_epoch);
}

vehicle_api::RequestId from_message(const msg::RequestId& message) {
    vehicle_api::RequestId id;
    id.command_id.value = message.command_id();
    id.session_id = message.session_id();
    id.seq = message.seq();
    id.fc_epoch = message.fc_epoch();
    return id;
}

void to_message(const vehicle_api::RequestHeader& header, msg::RequestHeader& message) {
    to_message(header.id, *message.mutable_id());
    message.set_valid_until_monotonic_ns(header.valid_until_monotonic_ns);
}

vehicle_api::RequestHeader from_message(const msg::RequestHeader& message) {
    vehicle_api::RequestHeader header;
    header.id = from_message(message.id());
    header.valid_until_monotonic_ns = message.valid_until_monotonic_ns();
    return header;
}

msg::FlightMode to_message(vehicle_api::FlightMode mode) {
    using vehicle_api::FlightMode;
    switch (mode) {
        case FlightMode::Unknown:
            return msg::FLIGHT_MODE_UNKNOWN;
        case FlightMode::Manual:
            return msg::FLIGHT_MODE_MANUAL;
        case FlightMode::Stabilized:
            return msg::FLIGHT_MODE_STABILIZED;
        case FlightMode::Altitude:
            return msg::FLIGHT_MODE_ALTITUDE;
        case FlightMode::Position:
            return msg::FLIGHT_MODE_POSITION;
        case FlightMode::Offboard:
            return msg::FLIGHT_MODE_OFFBOARD;
        case FlightMode::Hold:
            return msg::FLIGHT_MODE_HOLD;
        case FlightMode::Return:
            return msg::FLIGHT_MODE_RETURN;
        case FlightMode::Land:
            return msg::FLIGHT_MODE_LAND;
        case FlightMode::Takeoff:
            return msg::FLIGHT_MODE_TAKEOFF;
        case FlightMode::Mission:
            return msg::FLIGHT_MODE_MISSION;
    }
    return msg::FLIGHT_MODE_UNKNOWN;
}

vehicle_api::FlightMode from_message(msg::FlightMode mode) {
    using vehicle_api::FlightMode;
    switch (mode) {
        case msg::FLIGHT_MODE_MANUAL:
            return FlightMode::Manual;
        case msg::FLIGHT_MODE_STABILIZED:
            return FlightMode::Stabilized;
        case msg::FLIGHT_MODE_ALTITUDE:
            return FlightMode::Altitude;
        case msg::FLIGHT_MODE_POSITION:
            return FlightMode::Position;
        case msg::FLIGHT_MODE_OFFBOARD:
            return FlightMode::Offboard;
        case msg::FLIGHT_MODE_HOLD:
            return FlightMode::Hold;
        case msg::FLIGHT_MODE_RETURN:
            return FlightMode::Return;
        case msg::FLIGHT_MODE_LAND:
            return FlightMode::Land;
        case msg::FLIGHT_MODE_TAKEOFF:
            return FlightMode::Takeoff;
        case msg::FLIGHT_MODE_MISSION:
            return FlightMode::Mission;
        default:
            return FlightMode::Unknown;
    }
}

}  // namespace argus::vehicle_api_bus
