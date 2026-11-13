#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"

#include <optional>
#include <utility>

#include "argus/bus/message.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "control_mode_request_codec.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {
namespace {

bus::Bytes handle(vehicle_api::IFlightModeControl& control, const bus::BusMessage& message) {
    // 요청에는 바이너리가 없다
    const std::optional<FlightModeRequest> request =
        message.binary.empty() ? decode_flight_mode_request(message.body) : std::nullopt;
    if (!request) {
        detail::log_error("control/mode_request: 형식이 맞지 않는 요청 거부");
        vehicle_api::FlightModeReply rejected;
        rejected.reason = "malformed request";
        return encode_flight_mode_reply(rejected);
    }
    switch (request->action) {
        case FlightModeRequest::Action::Offboard:
            return encode_flight_mode_reply(control.request_offboard(request->header));
        case FlightModeRequest::Action::Hold:
            return encode_flight_mode_reply(control.request_hold(request->header));
    }
    return encode_flight_mode_reply({});
}

}  // namespace

std::unique_ptr<BusFlightModeControlServer> BusFlightModeControlServer::create(
    bus::IBus& bus, vehicle_api::IFlightModeControl& control) {
    auto replier = bus.create_replier(protocol::kControlModeRequest);
    if (!replier) {
        return nullptr;
    }
    core::Subscription serving = replier->serve(
        [&control](const bus::BusMessage& message) { return handle(control, message); });
    return std::unique_ptr<BusFlightModeControlServer>(
        new BusFlightModeControlServer(std::move(replier), std::move(serving)));
}

BusFlightModeControlServer::BusFlightModeControlServer(std::unique_ptr<bus::IReplier> replier,
                                                       core::Subscription serving)
    : replier_(std::move(replier)), serving_(std::move(serving)) {}

}  // namespace argus::vehicle_api_bus
