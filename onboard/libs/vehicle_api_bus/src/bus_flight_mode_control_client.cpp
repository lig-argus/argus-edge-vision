#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusFlightModeControlClient> BusFlightModeControlClient::create(bus::IBus& bus) {
    auto requester = bus::make_requester<FlightModeRequest, vehicle_api::FlightModeReply>(
        bus, protocol::kControlModeRequest, kControlModeRequestType, kControlModeReplyType);
    if (!requester) {
        return nullptr;
    }
    return std::unique_ptr<BusFlightModeControlClient>(
        new BusFlightModeControlClient(std::move(requester)));
}

BusFlightModeControlClient::BusFlightModeControlClient(std::unique_ptr<Requester> requester)
    : requester_(std::move(requester)) {}

core::Subscription BusFlightModeControlClient::request_offboard(
    const vehicle_api::RequestHeader& header, std::chrono::milliseconds timeout,
    ReplyHandler on_reply) {
    return send({header, FlightModeRequest::Action::Offboard}, timeout, std::move(on_reply));
}

core::Subscription BusFlightModeControlClient::request_hold(
    const vehicle_api::RequestHeader& header, std::chrono::milliseconds timeout,
    ReplyHandler on_reply) {
    return send({header, FlightModeRequest::Action::Hold}, timeout, std::move(on_reply));
}

core::Subscription BusFlightModeControlClient::send(const FlightModeRequest& request,
                                                    std::chrono::milliseconds timeout,
                                                    ReplyHandler on_reply) {
    if (!on_reply) {
        detail::log_error("BusFlightModeControlClient: 빈 on_reply");
        return {};
    }
    // 응답은 값 그대로, timeout 은 nullopt 로 전달된다
    return requester_->request(request, timeout, std::move(on_reply));
}

}  // namespace argus::vehicle_api_bus
