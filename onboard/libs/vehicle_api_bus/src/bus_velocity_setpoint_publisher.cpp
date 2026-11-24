#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVelocitySetpointPublisher> BusVelocitySetpointPublisher::create(bus::IBus& bus) {
    auto publisher = bus::make_publisher<vehicle_api::VelocitySetpoint>(
        bus, protocol::kControlSetpoint, kControlSetpointType,
        protocol::kControlSetpointQueueDepth);
    if (!publisher) {
        return nullptr;
    }
    return std::unique_ptr<BusVelocitySetpointPublisher>(
        new BusVelocitySetpointPublisher(std::move(publisher)));
}

BusVelocitySetpointPublisher::BusVelocitySetpointPublisher(std::unique_ptr<Publisher> publisher)
    : publisher_(std::move(publisher)) {}

void BusVelocitySetpointPublisher::send(const vehicle_api::VelocitySetpoint& setpoint) {
    const bool published = publisher_->publish(setpoint);
    detail::record_publish_result(protocol::kControlSetpoint, published, publish_failing_);
}

}  // namespace argus::vehicle_api_bus
