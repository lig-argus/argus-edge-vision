// MAV_CMD_ARGUS_SCAN_START (45002) 의 처리: 기본값 보완과 범위 검사 뒤 follow/command search 요청
// 규약: docs/spec/icd.md 4.2
// 사용: mavlink_gateway 의 main 이 생성하여 MavlinkCommandReceiver 에 kRules 로 등록
#ifndef ARGUS_ONBOARD_SCAN_START_HANDLER_HPP_
#define ARGUS_ONBOARD_SCAN_START_HANDLER_HPP_

#include <cstdint>

#include "argus/mavlink_gateway/follow_command_tracker.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"

namespace argus::mavlink_gateway {

// 모두 ScanStartHandler 보다 오래 유지되어야 한다
struct ScanStartHandlerInputs {
    RequestHeaderFactory& request_headers;
    vehicle_api::IFollowControlClient& follow_control;
    FollowCommandTracker& tracker;
};

// - NaN 매개변수는 기본값으로 보완. 범위 밖 → DENIED (parse_scan_start)
// - 통과하면 FollowCommandTracker 로 IN_PROGRESS(progress 0) 송신, 보완한 값으로 search 요청,
//   진행률과 결과 전달
// ② ③ 은 추종 제어가 검사한다 (icd 4.1)
class ScanStartHandler : public IMavlinkCommandHandler {
public:
    static constexpr CommandRules kRules{true, true, CommandKind::Start};

    explicit ScanStartHandler(const ScanStartHandlerInputs& inputs) : inputs_(inputs) {}

    std::uint16_t command() const override { return MAV_CMD_ARGUS_SCAN_START; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    ScanStartHandlerInputs inputs_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SCAN_START_HANDLER_HPP_
