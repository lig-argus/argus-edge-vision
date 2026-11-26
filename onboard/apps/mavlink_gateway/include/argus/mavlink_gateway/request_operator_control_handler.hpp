// MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL (45010) 의 처리: 매개변수 확인 뒤 제어권 요청 또는 해제
// 규약: docs/spec/icd.md 4.4, 5.1
// 사용: mavlink_gateway 의 main 이 생성하여 MavlinkCommandReceiver 에 kRules 로 등록
#ifndef ARGUS_ONBOARD_REQUEST_OPERATOR_CONTROL_HANDLER_HPP_
#define ARGUS_ONBOARD_REQUEST_OPERATOR_CONTROL_HANDLER_HPP_

#include <cstdint>

#include "argus/mavlink_gateway/control_requests.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"

namespace argus::mavlink_gateway {

// p1 이 0(해제) 또는 1(요청)이 아니거나, p4 가 0 도 보낸 통제기의 sysid 도 아니면 DENIED.
// 그 밖에는 IControlRequests 의 결과를 최종 결과로 바로 보낸다
class RequestOperatorControlHandler : public IMavlinkCommandHandler {
public:
    // 명령 번호 규칙을 적용하고, 제어권이 없는 통제기도 보낼 수 있다 (요청이 이 명령의 목적)
    static constexpr CommandRules kRules{true, false, CommandKind::Other};

    // requests 는 이 객체보다 오래 유지되어야 한다
    explicit RequestOperatorControlHandler(IControlRequests& requests) : requests_(requests) {}

    std::uint16_t command() const override { return MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    IControlRequests& requests_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_REQUEST_OPERATOR_CONTROL_HANDLER_HPP_
