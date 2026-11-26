#include "argus/mavlink_gateway/track_start_handler.hpp"

#include <cstdint>
#include <optional>
#include <utility>
#include <variant>

#include "argus/mavlink_gateway/follow_command_policy.hpp"
#include "argus/mavlink_gateway/sensor_status_policy.hpp"

namespace argus::mavlink_gateway {

void TrackStartHandler::handle(const CommandRequest& request, CommandResponder responder) {
    const std::variant<TrackStartParams, CommandRejection> parsed =
        parse_track_start(request.command);
    if (const auto* rejection = std::get_if<CommandRejection>(&parsed)) {
        send_rejection(responder, *rejection);
        return;
    }
    const auto& params = std::get<TrackStartParams>(parsed);
    const std::optional<SensorSource> sensor = find_sensor(inputs_.sensors, params.sensor);
    if (!sensor.has_value()) {
        send_rejection(responder,
                       CommandRejection{MAV_RESULT_TEMPORARILY_REJECTED, "센서 설정 없음"});
        return;
    }

    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();
    SensorObservation observation;
    observation.now_ns = now_ns;
    observation.sensor = sensor.value();
    observation.vision = inputs_.vision_status.latest(sensor->source);
    observation.tracking = inputs_.tracking_status.latest(sensor->source);
    const std::uint32_t flags = compose_sensor_status(observation).flags;

    TrackSelection selection;
    selection.params = params;
    selection.followable = (flags & static_cast<std::uint32_t>(ARGUS_SENSOR_FLAGS_FOLLOWABLE)) != 0;
    selection.tracks = inputs_.tracks.latest(sensor->source);
    selection.received_ns = now_ns;
    const std::variant<vehicle_api::TrackTarget, CommandRejection> selected =
        select_track_target(selection);
    if (const auto* rejection = std::get_if<CommandRejection>(&selected)) {
        send_rejection(responder, *rejection);
        return;
    }

    const vehicle_api::RequestHeader header =
        inputs_.request_headers.next(kFollowCommandReplyTimeout);
    vehicle_api::IFollowControlClient* const follow_control = &inputs_.follow_control;
    const auto& target = std::get<vehicle_api::TrackTarget>(selected);
    inputs_.tracker.track(header.id.command_id, std::nullopt, std::move(responder),
                          [follow_control, &header,
                           &target](vehicle_api::IFollowControlClient::ResultHandler handler) {
                              return follow_control->track_start(
                                  header, target, kFollowCommandReplyTimeout, std::move(handler));
                          });
}

}  // namespace argus::mavlink_gateway
