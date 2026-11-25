#include "control_mode_request_idl.hpp"

#include "idl_common.hpp"

namespace argus::vehicle_api_bus {

void to_idl_flight_mode_request(const FlightModeRequest& request, idl::ControlModeRequest& sample) {
    to_idl(request.header, sample.header());
    switch (request.action) {
        case FlightModeRequest::Action::Offboard:
            sample.action(idl::ModeAction::MODE_ACTION_OFFBOARD);
            break;
        case FlightModeRequest::Action::Hold:
            sample.action(idl::ModeAction::MODE_ACTION_HOLD);
            break;
    }
}

bool from_idl_flight_mode_request(const idl::ControlModeRequest& sample,
                                  FlightModeRequest& request) {
    switch (sample.action()) {
        case idl::ModeAction::MODE_ACTION_OFFBOARD:
            request.action = FlightModeRequest::Action::Offboard;
            request.header = from_idl(sample.header());
            return true;
        case idl::ModeAction::MODE_ACTION_HOLD:
            request.action = FlightModeRequest::Action::Hold;
            request.header = from_idl(sample.header());
            return true;
        case idl::ModeAction::MODE_ACTION_UNSPECIFIED:
            return false;
    }
    // 이 코드에 정의되지 않은 값
    return false;
}

void to_idl_flight_mode_reply(const vehicle_api::FlightModeReply& reply,
                              idl::ControlModeReply& sample) {
    sample.id().reset();
    if (reply.id) {
        idl::RequestId id;
        to_idl(*reply.id, id);
        sample.id(id);
    }
    sample.accepted(reply.accepted);
    sample.reason(reply.reason);
}

bool from_idl_flight_mode_reply(const idl::ControlModeReply& sample,
                                vehicle_api::FlightModeReply& reply) {
    // 응답하는 쪽은 요청의 id 를 항상 넣는다. id 가 없는 응답은 요청과 맞출 수 없으므로 해석 실패로
    // 본다
    if (!sample.id().has_value()) {
        return false;
    }
    reply.id = from_idl(*sample.id());
    reply.accepted = sample.accepted();
    reply.reason = sample.reason();
    return true;
}

}  // namespace argus::vehicle_api_bus
