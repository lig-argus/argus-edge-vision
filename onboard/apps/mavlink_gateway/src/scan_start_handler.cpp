#include "argus/mavlink_gateway/scan_start_handler.hpp"

#include <utility>
#include <variant>

#include "argus/mavlink_gateway/follow_command_policy.hpp"

namespace argus::mavlink_gateway {

void ScanStartHandler::handle(const CommandRequest& request, CommandResponder responder) {
    const std::variant<vehicle_api::ScanRequest, CommandRejection> parsed =
        parse_scan_start(request.command);
    if (const auto* rejection = std::get_if<CommandRejection>(&parsed)) {
        send_rejection(responder, *rejection);
        return;
    }
    const auto& scan = std::get<vehicle_api::ScanRequest>(parsed);
    const vehicle_api::RequestHeader header =
        inputs_.request_headers.next(kFollowCommandReplyTimeout);
    vehicle_api::IFollowControlClient* const follow_control = &inputs_.follow_control;
    inputs_.tracker.track(
        header.id.command_id, scan.yaw_range_rad, std::move(responder),
        [follow_control, &header, &scan](vehicle_api::IFollowControlClient::ResultHandler handler) {
            return follow_control->scan_start(header, scan, kFollowCommandReplyTimeout,
                                              std::move(handler));
        });
}

}  // namespace argus::mavlink_gateway
