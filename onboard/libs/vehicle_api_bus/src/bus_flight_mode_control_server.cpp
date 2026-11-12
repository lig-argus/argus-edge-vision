#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"

namespace argus::vehicle_api_bus {
namespace {

vehicle_api::FlightModeReply handle(vehicle_api::IFlightModeControl& control,
                                    const FlightModeRequest& request) {
    switch (request.action) {
        case FlightModeRequest::Action::Offboard:
            return control.request_offboard(request.header);
        case FlightModeRequest::Action::Hold:
            break;
    }
    return control.request_hold(request.header);
}

}  // namespace

std::unique_ptr<BusFlightModeControlServer> BusFlightModeControlServer::create(
    bus::IBus& bus, vehicle_api::IFlightModeControl& control) {
    auto replier = bus::make_replier<FlightModeRequest, vehicle_api::FlightModeReply>(
        bus, protocol::kControlModeRequest, kControlModeRequestType, kControlModeReplyType);
    if (!replier) {
        return nullptr;
    }
    core::Subscription serving = replier->serve(
        [&control](const FlightModeRequest& request) { return handle(control, request); });
    return std::unique_ptr<BusFlightModeControlServer>(
        new BusFlightModeControlServer(std::move(replier), std::move(serving)));
}

BusFlightModeControlServer::BusFlightModeControlServer(std::unique_ptr<Replier> replier,
                                                       core::Subscription serving)
    : replier_(std::move(replier)), serving_(std::move(serving)) {}

}  // namespace argus::vehicle_api_bus
