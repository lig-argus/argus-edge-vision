#include "argus/mavlink_gateway/gcs_fc_forwarder.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "argus/mavlink_gateway/follow_activity.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/vehicle_api/types.hpp"
#include "gcs_message_encoding.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {
namespace {

// HOLD 를 받았으나 정지를 시작하지 못한 경우의 이유. 외부의 모드 변경이 기록되어 있어 운용자의
// TRACK_STOP 으로 기록을 해제해야 한다 (icd 4.2)
constexpr const char* kStopFailedText = "정지할 수 없음: 추종 정지 필요";

bool is_for_fc(std::uint8_t target_system, std::uint8_t target_component) {
    return target_system == kVehicleSysid && target_component == kFcCompid;
}

// message 에서 판단에 쓰는 값. FC 가 대상이 아니면 nullopt (COMMAND_LONG 의 대상 195 는
// MavlinkCommandReceiver 가, 그 밖의 대상은 어느 쪽도 처리하지 않는다. icd 4.1)
std::optional<GcsMessageRequest> request_of(const mavlink_message_t& message) {
    GcsMessageRequest request;
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            request.kind = GcsMessageKind::Heartbeat;
            return request;
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t command{};
            mavlink_msg_command_long_decode(&message, &command);
            if (!is_for_fc(command.target_system, command.target_component)) {
                return std::nullopt;
            }
            request.kind = GcsMessageKind::CommandLong;
            request.command = command.command;
            request.param1 = command.param1;
            request.param2 = command.param2;
            request.param3 = command.param3;
            return request;
        }
        case MAVLINK_MSG_ID_COMMAND_INT: {
            mavlink_command_int_t command{};
            mavlink_msg_command_int_decode(&message, &command);
            if (!is_for_fc(command.target_system, command.target_component)) {
                return std::nullopt;
            }
            request.kind = GcsMessageKind::CommandInt;
            request.command = command.command;
            return request;
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            if (mavlink_msg_manual_control_get_target(&message) != kVehicleSysid) {
                return std::nullopt;
            }
            request.kind = GcsMessageKind::ManualControl;
            return request;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            if (!is_for_fc(mavlink_msg_param_request_read_get_target_system(&message),
                           mavlink_msg_param_request_read_get_target_component(&message))) {
                return std::nullopt;
            }
            request.kind = GcsMessageKind::ParamRequestRead;
            return request;
        case MAVLINK_MSG_ID_PARAM_SET:
            if (!is_for_fc(mavlink_msg_param_set_get_target_system(&message),
                           mavlink_msg_param_set_get_target_component(&message))) {
                return std::nullopt;
            }
            request.kind = GcsMessageKind::ParamSet;
            return request;
        default:
            return std::nullopt;
    }
}

}  // namespace

GcsFcForwarder::GcsFcForwarder(const GcsFcForwarderInputs& inputs) : inputs_(inputs) {
    const auto on_message = [this](const mavlink_message_t& message, const GcsPeer& sender) {
        on_gcs_message(message, sender);
    };
    heartbeat_subscription_ = inputs_.messages.subscribe(MAVLINK_MSG_ID_HEARTBEAT, on_message);
    command_long_subscription_ =
        inputs_.messages.subscribe(MAVLINK_MSG_ID_COMMAND_LONG, on_message);
    command_int_subscription_ = inputs_.messages.subscribe(MAVLINK_MSG_ID_COMMAND_INT, on_message);
    manual_control_subscription_ =
        inputs_.messages.subscribe(MAVLINK_MSG_ID_MANUAL_CONTROL, on_message);
    param_request_read_subscription_ =
        inputs_.messages.subscribe(MAVLINK_MSG_ID_PARAM_REQUEST_READ, on_message);
    param_set_subscription_ = inputs_.messages.subscribe(MAVLINK_MSG_ID_PARAM_SET, on_message);
    command_ack_subscription_ = inputs_.connection.subscribe(
        MAVLINK_MSG_ID_COMMAND_ACK,
        [this](const mavlink_message_t& message) { on_fc_command_ack(message); });
}

core::Subscription GcsFcForwarder::subscribe(Handler handler) {
    return handlers_.subscribe(std::move(handler));
}

void GcsFcForwarder::on_gcs_message(const mavlink_message_t& message, const GcsPeer& sender) {
    const std::optional<GcsMessageRequest> request = request_of(message);
    if (!request.has_value()) {
        return;
    }
    execute(decide_forward(observe(sender), *request), message, sender, *request);
}

ForwardObservation GcsFcForwarder::observe(const GcsPeer& sender) const {
    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();
    const vehicle_api::VehicleState vehicle = inputs_.fc_state.latest();
    ForwardObservation observation;
    observation.from_controller = inputs_.authority.is_controller(sender.sysid);
    observation.fc_status_known =
        inputs_.fc_state.fc_connected() && vehicle.status.has_value() &&
        now_ns - vehicle.status->received_monotonic_ns <= kFcStatusMaxAgeNs;
    if (observation.fc_status_known) {
        observation.fc_mode = vehicle.status->mode;
        observation.armed = vehicle.status->armed;
    }
    observation.sending_setpoints =
        inputs_.fc_command_status.sending_setpoints() || inputs_.fc_command_status.entering();
    observation.follow_in_progress = follow_in_progress(inputs_.follow_status.latest(), now_ns);
    observation.command_in_progress = inputs_.commands.command_in_progress();
    observation.landing_locked = inputs_.landing_lock.active();
    observation.pilot_in_control = inputs_.rc_override.pilot_in_control();
    return observation;
}

void GcsFcForwarder::execute(const ForwardDecision& decision, const mavlink_message_t& message,
                             const GcsPeer& sender, const GcsMessageRequest& request) {
    switch (decision.action) {
        case ForwardAction::Forward: {
            if (!inputs_.connection.send(message)) {
                detail::log_error("통제기 메시지를 FC 로 보내지 못함 (msgid " +
                                  std::to_string(message.msgid) + ")");
                return;
            }
            if (request.kind == GcsMessageKind::CommandLong) {
                ForwardedCommandEvent event;
                event.sender = sender;
                event.command = request.command;
                event.stage = ForwardStage::Forwarded;
                if (request.command == MAV_CMD_DO_SET_MODE) {
                    event.requested_mode =
                        set_mode_request(request.param1, request.param2, request.param3);
                }
                handlers_.notify(event);
            }
            return;
        }
        case ForwardAction::Reply:
            reply(sender, request.command, decision.result, decision.reason);
            return;
        case ForwardAction::Stop:
            if (inputs_.stop_requester.stop_from_onboard()) {
                reply(sender, request.command, MAV_RESULT_ACCEPTED, "");
            } else {
                reply(sender, request.command, MAV_RESULT_TEMPORARILY_REJECTED, kStopFailedText);
            }
            return;
        case ForwardAction::Drop:
            return;
    }
}

void GcsFcForwarder::reply(const GcsPeer& sender, std::uint16_t command, MAV_RESULT result,
                           const std::string& reason) {
    inputs_.link.send_to(
        sender, detail::encode_command_ack(command, result, 0, sender.sysid, sender.compid));
    if (!reason.empty()) {
        inputs_.link.send_to(sender, detail::encode_statustext(MAV_SEVERITY_WARNING, reason));
    }
}

void GcsFcForwarder::on_fc_command_ack(const mavlink_message_t& message) {
    if (!is_from_fc(message)) {
        return;
    }
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    if (!is_gcs_sysid(ack.target_system) || ack.target_component != kGcsCompid) {
        return;  // mavlink_gateway 자신의 명령(compid 195)이나 통제기가 아닌 노드에 대한 응답
    }
    const std::optional<GcsPeer> peer = inputs_.presence.peer(ack.target_system);
    if (!peer.has_value()) {
        return;
    }
    inputs_.link.send_to(*peer, message);
    ForwardedCommandEvent event;
    event.sender = *peer;
    event.command = ack.command;
    event.stage = ForwardStage::Answered;
    event.result = static_cast<MAV_RESULT>(ack.result);
    handlers_.notify(event);
}

}  // namespace argus::mavlink_gateway
