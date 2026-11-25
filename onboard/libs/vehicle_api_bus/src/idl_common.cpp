#include "idl_common.hpp"

namespace argus::vehicle_api_bus {

void to_idl(const vehicle_api::RequestId& id, idl::RequestId& sample) {
    sample.command_id(id.command_id.value);
    sample.session_id(id.session_id);
    sample.seq(id.seq);
    sample.fc_epoch(id.fc_epoch);
}

vehicle_api::RequestId from_idl(const idl::RequestId& sample) {
    vehicle_api::RequestId id;
    id.command_id.value = sample.command_id();
    id.session_id = sample.session_id();
    id.seq = sample.seq();
    id.fc_epoch = sample.fc_epoch();
    return id;
}

void to_idl(const vehicle_api::RequestHeader& header, idl::RequestHeader& sample) {
    to_idl(header.id, sample.id());
    sample.valid_until_monotonic_ns(header.valid_until_monotonic_ns);
}

vehicle_api::RequestHeader from_idl(const idl::RequestHeader& sample) {
    vehicle_api::RequestHeader header;
    header.id = from_idl(sample.id());
    header.valid_until_monotonic_ns = sample.valid_until_monotonic_ns();
    return header;
}

idl::FlightMode to_idl(vehicle_api::FlightMode mode) {
    using vehicle_api::FlightMode;
    switch (mode) {
        case FlightMode::Unknown:
            return idl::FlightMode::FLIGHT_MODE_UNKNOWN;
        case FlightMode::Manual:
            return idl::FlightMode::FLIGHT_MODE_MANUAL;
        case FlightMode::Stabilized:
            return idl::FlightMode::FLIGHT_MODE_STABILIZED;
        case FlightMode::Altitude:
            return idl::FlightMode::FLIGHT_MODE_ALTITUDE;
        case FlightMode::Position:
            return idl::FlightMode::FLIGHT_MODE_POSITION;
        case FlightMode::Offboard:
            return idl::FlightMode::FLIGHT_MODE_OFFBOARD;
        case FlightMode::Hold:
            return idl::FlightMode::FLIGHT_MODE_HOLD;
        case FlightMode::Return:
            return idl::FlightMode::FLIGHT_MODE_RETURN;
        case FlightMode::Land:
            return idl::FlightMode::FLIGHT_MODE_LAND;
        case FlightMode::Takeoff:
            return idl::FlightMode::FLIGHT_MODE_TAKEOFF;
        case FlightMode::Mission:
            return idl::FlightMode::FLIGHT_MODE_MISSION;
    }
    return idl::FlightMode::FLIGHT_MODE_UNKNOWN;
}

vehicle_api::FlightMode from_idl(idl::FlightMode mode) {
    using vehicle_api::FlightMode;
    switch (mode) {
        case idl::FlightMode::FLIGHT_MODE_UNKNOWN:
            return FlightMode::Unknown;
        case idl::FlightMode::FLIGHT_MODE_MANUAL:
            return FlightMode::Manual;
        case idl::FlightMode::FLIGHT_MODE_STABILIZED:
            return FlightMode::Stabilized;
        case idl::FlightMode::FLIGHT_MODE_ALTITUDE:
            return FlightMode::Altitude;
        case idl::FlightMode::FLIGHT_MODE_POSITION:
            return FlightMode::Position;
        case idl::FlightMode::FLIGHT_MODE_OFFBOARD:
            return FlightMode::Offboard;
        case idl::FlightMode::FLIGHT_MODE_HOLD:
            return FlightMode::Hold;
        case idl::FlightMode::FLIGHT_MODE_RETURN:
            return FlightMode::Return;
        case idl::FlightMode::FLIGHT_MODE_LAND:
            return FlightMode::Land;
        case idl::FlightMode::FLIGHT_MODE_TAKEOFF:
            return FlightMode::Takeoff;
        case idl::FlightMode::FLIGHT_MODE_MISSION:
            return FlightMode::Mission;
    }
    return FlightMode::Unknown;
}

}  // namespace argus::vehicle_api_bus
