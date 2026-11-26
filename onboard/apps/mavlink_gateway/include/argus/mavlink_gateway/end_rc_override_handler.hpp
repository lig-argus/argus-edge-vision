// MAV_CMD_ARGUS_END_RC_OVERRIDE (45011) 의 처리: 복귀 조건 확인 뒤 제자리 유지 시작
// 규약: docs/spec/icd.md 4.4, 5.4
// 사용: mavlink_gateway 의 main 이 생성하여 MavlinkCommandReceiver 에 kRules 로 등록
#ifndef ARGUS_ONBOARD_END_RC_OVERRIDE_HANDLER_HPP_
#define ARGUS_ONBOARD_END_RC_OVERRIDE_HANDLER_HPP_

#include <cstdint>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_hover_entry.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/rc_override_return.hpp"

namespace argus::mavlink_gateway {

// - 복귀 조건(IRcOverrideReturn::check_return)을 충족하지 않으면 TEMPORARILY_REJECTED 와 불충족
//   조건의 STATUSTEXT
// - FC 모드가 Hold: FC 로 아무것도 보내지 않고 개입을 끝낸 뒤 ACCEPTED
// - FC 모드가 Position: IN_PROGRESS 를 보내고 IFcHoverEntry 로 Offboard 제자리 유지에 들어간다.
//   들어가면 개입을 끝낸 뒤 ACCEPTED, 실패하면 FAILED (개입은 유지). 시작하지 못하면
//   TEMPORARILY_REJECTED
class EndRcOverrideHandler : public IMavlinkCommandHandler {
public:
    // 명령 번호 규칙과 제어권이 필요하고, 착륙 잠금이나 조종사 개입 중에도 받는다 (이 명령의 목적)
    static constexpr CommandRules kRules{true, true, CommandKind::Other, true};

    // 인자로 받은 객체들은 이 객체보다 오래 유지되어야 한다
    EndRcOverrideHandler(IRcOverrideReturn& rc_return, IFcHoverEntry& hover_entry)
        : rc_return_(rc_return), hover_entry_(hover_entry) {}

    std::uint16_t command() const override { return MAV_CMD_ARGUS_END_RC_OVERRIDE; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    IRcOverrideReturn& rc_return_;
    IFcHoverEntry& hover_entry_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_END_RC_OVERRIDE_HANDLER_HPP_
