#include "argus/mavlink_gateway/request_operator_control_handler.hpp"

#include <cstdint>
#include <optional>
#include <utility>

namespace argus::mavlink_gateway {
namespace {

std::optional<ControlAction> action_of(float param1) {
    if (param1 == 0.0F) {
        return ControlAction::Release;
    }
    if (param1 == 1.0F) {
        return ControlAction::Request;
    }
    return std::nullopt;
}

// p4 가 0 이거나 보낸 통제기의 sysid 이면 true (icd 4.4)
bool names_sender(float param4, std::uint8_t sender_sysid) {
    return param4 == 0.0F || param4 == static_cast<float>(sender_sysid);
}

}  // namespace

void RequestOperatorControlHandler::handle(const CommandRequest& request,
                                           CommandResponder responder) {
    const std::optional<ControlAction> action = action_of(request.command.param1);
    if (!action.has_value()) {
        responder.send(CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "p1 must be 0 or 1"});
        return;
    }
    if (!names_sender(request.command.param4, request.sender.sysid)) {
        responder.send(
            CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "p4 must be 0 or sender sysid"});
        return;
    }
    responder.send(CommandAck{requests_.request(request.sender, *action), kProgressUnknown, {}});
}

}  // namespace argus::mavlink_gateway
