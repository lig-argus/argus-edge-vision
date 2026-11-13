#include "argus/vehicle_api_bus/bus_velocity_setpoint_publisher.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "control_setpoint_codec.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVelocitySetpointPublisher> BusVelocitySetpointPublisher::create(bus::IBus& bus) {
    auto publisher =
        bus.create_publisher(protocol::kControlSetpoint, protocol::kControlSetpointQueueDepth);
    if (!publisher) {
        return nullptr;
    }
    return std::unique_ptr<BusVelocitySetpointPublisher>(
        new BusVelocitySetpointPublisher(std::move(publisher)));
}

BusVelocitySetpointPublisher::BusVelocitySetpointPublisher(
    std::unique_ptr<bus::ITopicPublisher> publisher)
    : publisher_(std::move(publisher)) {}

void BusVelocitySetpointPublisher::send(const vehicle_api::VelocitySetpoint& setpoint) {
    const bool published = publisher_->publish(encode_control_setpoint(setpoint, next_seq_), {});
    ++next_seq_;
    detail::record_publish_result(protocol::kControlSetpoint, published, publish_failing_);
}

}  // namespace argus::vehicle_api_bus
