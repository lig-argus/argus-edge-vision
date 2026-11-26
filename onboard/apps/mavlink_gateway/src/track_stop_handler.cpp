#include "argus/mavlink_gateway/track_stop_handler.hpp"

#include <optional>
#include <utility>

#include "argus/mavlink_gateway/follow_command_policy.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {

void TrackStopHandler::handle(const CommandRequest& /*request*/, CommandResponder responder) {
    if (!inputs_.lock_release.release_lock()) {
        detail::log_error("TRACK_STOP: FC 명령 Locked 해제 실패 (FC 비행 상태가 최신이 아님)");
    }
    if (!follow_stop_needed(inputs_.follow_status.latest(), inputs_.clock.now_monotonic_ns())) {
        responder.send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, ""});
        return;
    }
    const vehicle_api::RequestHeader header =
        inputs_.request_headers.next(kFollowCommandReplyTimeout);
    vehicle_api::IFollowControlClient* const follow_control = &inputs_.follow_control;
    inputs_.tracker.track(
        header.id.command_id, std::nullopt, std::move(responder),
        [follow_control, &header](vehicle_api::IFollowControlClient::ResultHandler handler) {
            return follow_control->stop(header, vehicle_api::StopReason::Operator,
                                        kFollowCommandReplyTimeout, std::move(handler));
        });
}

}  // namespace argus::mavlink_gateway
