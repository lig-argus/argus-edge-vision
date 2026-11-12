#include "argus/vehicle_api_bus/bus_vehicle_telemetry_publisher.hpp"

#include <utility>

#include "argus/bus/message.hpp"
#include "argus/protocol/topics.hpp"
#include "detail.hpp"
#include "fc_telemetry_codec.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVehicleTelemetryPublisher> BusVehicleTelemetryPublisher::create(bus::IBus& bus) {
    Topic attitude{bus.create_publisher(protocol::kFcAttitude, protocol::kFcAttitudeQueueDepth)};
    Topic state{bus.create_publisher(protocol::kFcState, protocol::kFcStateQueueDepth)};
    Topic mode_event{
        bus.create_publisher(protocol::kFcModeEvent, protocol::kFcModeEventQueueDepth)};
    if (!attitude.publisher || !state.publisher || !mode_event.publisher) {
        return nullptr;
    }
    return std::unique_ptr<BusVehicleTelemetryPublisher>(new BusVehicleTelemetryPublisher(
        std::move(attitude), std::move(state), std::move(mode_event)));
}

BusVehicleTelemetryPublisher::BusVehicleTelemetryPublisher(Topic attitude, Topic state,
                                                           Topic mode_event)
    : attitude_(std::move(attitude)),
      state_(std::move(state)),
      mode_event_(std::move(mode_event)) {}

bool BusVehicleTelemetryPublisher::publish_attitude(const vehicle_api::Attitude& attitude) {
    return publish(attitude_, encode_fc_attitude(attitude, attitude_.next_seq));
}

bool BusVehicleTelemetryPublisher::publish_state(const vehicle_api::VehicleState& state) {
    return publish(state_, encode_fc_state(state, state_.next_seq));
}

bool BusVehicleTelemetryPublisher::publish_mode_event(const vehicle_api::FlightModeEvent& event) {
    return publish(mode_event_, encode_fc_mode_event(event, mode_event_.next_seq));
}

bool BusVehicleTelemetryPublisher::publish(Topic& topic, const bus::Bytes& body) {
    const bool published = topic.publisher->publish(body, {});
    ++topic.next_seq;
    detail::record_publish_result(topic.publisher->topic(), published, topic.failing);
    return published;
}

}  // namespace argus::vehicle_api_bus
