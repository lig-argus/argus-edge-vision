#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusFollowControlClient> BusFollowControlClient::create(bus::IBus& bus) {
    auto requester = bus::make_requester<FollowCommandRequest, vehicle_api::FollowResult>(
        bus, protocol::kFollowCommand, kFollowCommandType, kFollowResultType);
    if (!requester) {
        return nullptr;
    }
    return std::unique_ptr<BusFollowControlClient>(
        new BusFollowControlClient(std::move(requester)));
}

BusFollowControlClient::BusFollowControlClient(std::unique_ptr<Requester> requester)
    : requester_(std::move(requester)) {}

core::Subscription BusFollowControlClient::track_start(const vehicle_api::RequestHeader& header,
                                                       const vehicle_api::TrackTarget& target,
                                                       std::chrono::milliseconds timeout,
                                                       ResultHandler on_result) {
    return send({header, target}, timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::stop(const vehicle_api::RequestHeader& header,
                                                vehicle_api::StopReason reason,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) {
    return send({header, reason}, timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::scan_start(const vehicle_api::RequestHeader& header,
                                                      const vehicle_api::ScanRequest& scan,
                                                      std::chrono::milliseconds timeout,
                                                      ResultHandler on_result) {
    return send({header, scan}, timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::send(const FollowCommandRequest& request,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) {
    if (!on_result) {
        detail::log_error("BusFollowControlClient: 빈 on_result");
        return {};
    }
    // 응답은 값 그대로, timeout 은 nullopt 로 전달된다
    return requester_->request(request, timeout, std::move(on_result));
}

}  // namespace argus::vehicle_api_bus
