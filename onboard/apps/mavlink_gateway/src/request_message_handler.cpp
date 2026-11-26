#include "argus/mavlink_gateway/request_message_handler.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

namespace argus::mavlink_gateway {
namespace {

// MAVLink 2 의 메시지 번호는 24 비트다
constexpr float kMaxMessageId = 16'777'215.0F;

std::optional<std::uint32_t> message_id_of(float param1) {
    if (!std::isfinite(param1) || param1 < 0.0F || param1 > kMaxMessageId ||
        std::trunc(param1) != param1) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(param1);
}

}  // namespace

bool RequestMessageHandler::add_message(std::uint32_t message_id, MessageBuilder builder) {
    if (!builder) {
        return false;
    }
    return builders_.emplace(message_id, std::move(builder)).second;
}

void RequestMessageHandler::handle(const CommandRequest& request, CommandResponder responder) {
    const std::optional<std::uint32_t> message_id = message_id_of(request.command.param1);
    if (!message_id.has_value()) {
        responder.send(CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "p1 must be message id"});
        return;
    }
    const auto found = builders_.find(*message_id);
    if (found == builders_.end()) {
        responder.send(CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "message not supported"});
        return;
    }
    const RequestedMessages built = found->second(request);
    responder.send(CommandAck{built.result, kProgressUnknown, built.reason});
    if (built.result != MAV_RESULT_ACCEPTED) {
        return;
    }
    for (const mavlink_message_t& message : built.messages) {
        link_.send_to(request.sender, message);
    }
}

}  // namespace argus::mavlink_gateway
