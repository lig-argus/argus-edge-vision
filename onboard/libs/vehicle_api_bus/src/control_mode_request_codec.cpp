#include "control_mode_request_codec.hpp"

#include "argus_msg/control_mode_request.pb.h"
#include "codec_common.hpp"

namespace argus::vehicle_api_bus {

bus_zmq::Bytes encode_flight_mode_request(const FlightModeRequest& request) {
    msg::ControlModeRequest message;
    to_message(request.header, *message.mutable_header());
    switch (request.action) {
        case FlightModeRequest::Action::Offboard:
            message.mutable_offboard();
            break;
        case FlightModeRequest::Action::Hold:
            message.mutable_hold();
            break;
    }
    return to_bytes(message);
}

std::optional<FlightModeRequest> decode_flight_mode_request(const bus_zmq::Bytes& body) {
    msg::ControlModeRequest message;
    if (!parse(body, message) || !message.has_header()) {
        return std::nullopt;
    }
    FlightModeRequest request;
    request.header = from_message(message.header());
    switch (message.action_case()) {
        case msg::ControlModeRequest::kOffboard:
            request.action = FlightModeRequest::Action::Offboard;
            break;
        case msg::ControlModeRequest::kHold:
            request.action = FlightModeRequest::Action::Hold;
            break;
        case msg::ControlModeRequest::ACTION_NOT_SET:
            return std::nullopt;
    }
    return request;
}

bus_zmq::Bytes encode_flight_mode_reply(const vehicle_api::FlightModeReply& reply) {
    msg::ControlModeReply message;
    if (reply.id) {
        to_message(*reply.id, *message.mutable_id());
    }
    message.set_accepted(reply.accepted);
    message.set_reason(reply.reason);
    return to_bytes(message);
}

std::optional<vehicle_api::FlightModeReply> decode_flight_mode_reply(const bus_zmq::Bytes& body) {
    msg::ControlModeReply message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    // 응답하는 쪽은 요청의 id 를 항상 그대로 넣는다. id 가 없는 응답(빈 본문 포함)은 요청과 맞출 수
    // 없으므로 해석 실패로 본다
    if (!message.has_id()) {
        return std::nullopt;
    }
    vehicle_api::FlightModeReply reply;
    reply.id = from_message(message.id());
    reply.accepted = message.accepted();
    reply.reason = message.reason();
    return reply;
}

}  // namespace argus::vehicle_api_bus
