#include "argus/vehicle_api_bus/bus_vehicle_telemetry_publisher.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVehicleTelemetryPublisher> BusVehicleTelemetryPublisher::create(bus::IBus& bus) {
    Topic<vehicle_api::Attitude> attitude{bus::make_publisher<vehicle_api::Attitude>(
        bus, protocol::kFcAttitude, kFcAttitudeType, protocol::kFcAttitudeQueueDepth)};
    Topic<vehicle_api::VehicleState> state{bus::make_publisher<vehicle_api::VehicleState>(
        bus, protocol::kFcState, kFcStateType, protocol::kFcStateQueueDepth)};
    Topic<vehicle_api::FlightModeEvent> mode_event{
        bus::make_publisher<vehicle_api::FlightModeEvent>(
            bus, protocol::kFcModeEvent, kFcModeEventType, protocol::kFcModeEventQueueDepth)};
    if (!attitude.publisher || !state.publisher || !mode_event.publisher) {
        return nullptr;
    }
    return std::unique_ptr<BusVehicleTelemetryPublisher>(new BusVehicleTelemetryPublisher(
        std::move(attitude), std::move(state), std::move(mode_event)));
}

BusVehicleTelemetryPublisher::BusVehicleTelemetryPublisher(
    Topic<vehicle_api::Attitude> attitude, Topic<vehicle_api::VehicleState> state,
    Topic<vehicle_api::FlightModeEvent> mode_event)
    : attitude_(std::move(attitude)),
      state_(std::move(state)),
      mode_event_(std::move(mode_event)) {}

bool BusVehicleTelemetryPublisher::publish_attitude(const vehicle_api::Attitude& attitude) {
    return publish(attitude_, attitude);
}

bool BusVehicleTelemetryPublisher::publish_state(const vehicle_api::VehicleState& state) {
    return publish(state_, state);
}

bool BusVehicleTelemetryPublisher::publish_mode_event(const vehicle_api::FlightModeEvent& event) {
    return publish(mode_event_, event);
}

template <typename T>
bool BusVehicleTelemetryPublisher::publish(Topic<T>& topic, const T& message) {
    const bool published = topic.publisher->publish(message);
    detail::record_publish_result(topic.publisher->topic(), published, topic.failing);
    return published;
}

}  // namespace argus::vehicle_api_bus
