#include "argus/mavlink_gateway/end_rc_override_handler.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {

void EndRcOverrideHandler::handle(const CommandRequest& /*request*/, CommandResponder responder) {
    const ReturnCheck check = rc_return_.check_return();
    if (check.block != ReturnBlock::None) {
        responder.send(CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown,
                                  std::string(to_text(check.block))});
        return;
    }
    if (check.fc_mode == vehicle_api::FlightMode::Hold) {
        // Hold 는 정지 상태이므로 FC 로 보낼 것이 없다 (icd 4.2 의 정지 상태)
        rc_return_.end_override();
        responder.send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}});
        return;
    }

    // done 이 나중에 불리므로 responder 를 함께 들고 있는다
    auto shared = std::make_shared<CommandResponder>(std::move(responder));
    IRcOverrideReturn* const rc_return = &rc_return_;
    const std::string_view denial = hover_entry_.request_hover_entry(
        [rc_return] { return rc_return->return_still_allowed(); },
        [rc_return, shared](bool entered) {
            if (!entered) {
                shared->send(
                    CommandAck{MAV_RESULT_FAILED, kProgressUnknown, "제자리 유지 진입 실패"});
                return;
            }
            rc_return->end_override();
            shared->send(CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, {}});
        });
    if (!denial.empty()) {
        detail::log_error("제자리 유지 진입을 시작하지 못함: " + std::string(denial));
        shared->send(
            CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown, "제자리 유지 시작 불가"});
        return;
    }
    shared->send(CommandAck{MAV_RESULT_IN_PROGRESS, kProgressUnknown, {}});
}

}  // namespace argus::mavlink_gateway
