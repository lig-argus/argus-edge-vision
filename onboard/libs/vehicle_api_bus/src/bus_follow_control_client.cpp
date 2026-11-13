#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"

#include <optional>
#include <utility>

#include "argus/protocol/topics.hpp"
#include "detail.hpp"
#include "follow_command_codec.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusFollowControlClient> BusFollowControlClient::create(bus::IBus& bus) {
    auto requester = bus.create_requester(protocol::kFollowCommand);
    if (!requester) {
        return nullptr;
    }
    return std::unique_ptr<BusFollowControlClient>(
        new BusFollowControlClient(std::move(requester)));
}

BusFollowControlClient::BusFollowControlClient(std::unique_ptr<bus::IRequester> requester)
    : requester_(std::move(requester)) {}

core::Subscription BusFollowControlClient::track_start(const vehicle_api::RequestHeader& header,
                                                       const vehicle_api::TrackTarget& target,
                                                       std::chrono::milliseconds timeout,
                                                       ResultHandler on_result) {
    return send(encode_follow_command({header, target}), timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::stop(const vehicle_api::RequestHeader& header,
                                                vehicle_api::StopReason reason,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) {
    return send(encode_follow_command({header, reason}), timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::scan_start(const vehicle_api::RequestHeader& header,
                                                      const vehicle_api::ScanRequest& scan,
                                                      std::chrono::milliseconds timeout,
                                                      ResultHandler on_result) {
    return send(encode_follow_command({header, scan}), timeout, std::move(on_result));
}

core::Subscription BusFollowControlClient::send(const bus::Bytes& request,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) {
    if (!on_result) {
        detail::log_error("BusFollowControlClient: 빈 on_result");
        return {};
    }
    return requester_->request(
        request, timeout,
        [on_result = std::move(on_result)](const std::optional<bus::BusMessage>& message) {
            if (!message) {
                on_result(std::nullopt);  // timeout
                return;
            }
            // 응답에는 바이너리가 없다
            std::optional<vehicle_api::FollowResult> result =
                message->binary.empty() ? decode_follow_result(message->body) : std::nullopt;
            if (!result) {
                detail::log_error("follow/command: 형식이 맞지 않는 응답 폐기");
            }
            on_result(std::move(result));
        });
}

}  // namespace argus::vehicle_api_bus
