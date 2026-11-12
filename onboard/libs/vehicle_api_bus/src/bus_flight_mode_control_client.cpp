#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"

#include <optional>
#include <utility>

#include "argus/protocol/topics.hpp"
#include "control_mode_request_codec.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusFlightModeControlClient> BusFlightModeControlClient::create(bus::IBus& bus) {
    auto requester = bus.create_requester(protocol::kControlModeRequest);
    if (!requester) {
        return nullptr;
    }
    return std::unique_ptr<BusFlightModeControlClient>(
        new BusFlightModeControlClient(std::move(requester)));
}

BusFlightModeControlClient::BusFlightModeControlClient(std::unique_ptr<bus::IRequester> requester)
    : requester_(std::move(requester)) {}

core::Subscription BusFlightModeControlClient::request_offboard(
    const vehicle_api::RequestHeader& header, std::chrono::milliseconds timeout,
    ReplyHandler on_reply) {
    return send(encode_flight_mode_request({header, FlightModeRequest::Action::Offboard}), timeout,
                std::move(on_reply));
}

core::Subscription BusFlightModeControlClient::request_hold(
    const vehicle_api::RequestHeader& header, std::chrono::milliseconds timeout,
    ReplyHandler on_reply) {
    return send(encode_flight_mode_request({header, FlightModeRequest::Action::Hold}), timeout,
                std::move(on_reply));
}

core::Subscription BusFlightModeControlClient::send(const bus::Bytes& request,
                                                    std::chrono::milliseconds timeout,
                                                    ReplyHandler on_reply) {
    if (!on_reply) {
        detail::log_error("BusFlightModeControlClient: 빈 on_reply");
        return {};
    }
    return requester_->request(
        request, timeout,
        [on_reply = std::move(on_reply)](const std::optional<bus::BusMessage>& message) {
            if (!message) {
                on_reply(std::nullopt);  // timeout
                return;
            }
            // 응답에는 바이너리가 없다
            std::optional<vehicle_api::FlightModeReply> reply =
                message->binary.empty() ? decode_flight_mode_reply(message->body) : std::nullopt;
            if (!reply) {
                detail::log_error("control/mode_request: 형식이 맞지 않는 응답 폐기");
            }
            on_reply(std::move(reply));
        });
}

}  // namespace argus::vehicle_api_bus
