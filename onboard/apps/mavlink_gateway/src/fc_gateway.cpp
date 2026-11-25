#include "argus/mavlink_gateway/fc_gateway.hpp"

#include <memory>

namespace argus::mavlink_gateway {

std::unique_ptr<FcGateway> FcGateway::create(bus::IBus& bus, IMavlinkConnection& connection,
                                             const core::IClock& clock, core::EventLoop& loop,
                                             HealthState& health) {
    // BusFlightModeControlServer 는 주소가 고정된 FcControl 이 있어야 만들 수 있으므로 이 객체를
    // 먼저 만든 뒤 bus 어댑터를 만든다
    std::unique_ptr<FcGateway> gateway(new FcGateway(connection, clock, health));
    if (!gateway->create_bus_adapters(bus)) {
        return nullptr;
    }
    gateway->connect_to_bus();
    gateway->start_periodic_tasks(loop);
    return gateway;
}

FcGateway::FcGateway(IMavlinkConnection& connection, const core::IClock& clock, HealthState& health)
    : link_(clock, health),
      time_sync_(connection, clock, link_),
      telemetry_(connection, link_, time_sync_),
      state_view_(link_, telemetry_),
      control_(connection, state_view_, clock) {}

bool FcGateway::create_bus_adapters(bus::IBus& bus) {
    telemetry_publisher_ = vehicle_api_bus::BusVehicleTelemetryPublisher::create(bus);
    setpoint_subscriber_ = vehicle_api_bus::BusVelocitySetpointSubscriber::create(bus);
    mode_control_server_ = vehicle_api_bus::BusFlightModeControlServer::create(bus, control_);
    return telemetry_publisher_ != nullptr && setpoint_subscriber_ != nullptr &&
           mode_control_server_ != nullptr;
}

void FcGateway::connect_to_bus() {
    attitude_subscription_ =
        telemetry_.subscribe_attitude([this](const vehicle_api::Attitude& attitude) {
            telemetry_publisher_->publish_attitude(attitude);
        });
    state_subscription_ = telemetry_.subscribe_state(
        [this](const vehicle_api::VehicleState& state) { publish_state(state); });
    mode_event_subscription_ =
        control_.subscribe_mode_events([this](const vehicle_api::FlightModeEvent& event) {
            telemetry_publisher_->publish_mode_event(event);
        });
    setpoint_subscription_ = setpoint_subscriber_->subscribe(
        [this](const vehicle_api::VelocitySetpoint& setpoint) { control_.send(setpoint); });
}

void FcGateway::start_periodic_tasks(core::EventLoop& loop) {
    control_tick_timer_ = loop.add_periodic(kFcControlTickPeriod, [this] { control_.tick(); });
    link_check_timer_ = loop.add_periodic(kFcLinkCheckPeriod, [this] { link_.check_timeout(); });
    // 송신 실패(FC 의 메시지를 아직 받지 못해 상대 주소가 없음)는 다음 주기에 다시 보내므로 결과를
    // 사용하지 않는다
    time_sync_timer_ = loop.add_periodic(kTimeSyncPeriod, [this] { time_sync_.send_request(); });
}

void FcGateway::publish_state(const vehicle_api::VehicleState& state) {
    vehicle_api::VehicleState published = state;
    if (published.status.has_value()) {
        published.status->offboard_hover = control_.offboard_hover();
    }
    telemetry_publisher_->publish_state(published);
}

}  // namespace argus::mavlink_gateway
