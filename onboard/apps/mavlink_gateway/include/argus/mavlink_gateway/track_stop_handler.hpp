// MAV_CMD_ARGUS_TRACK_STOP (45001) 의 처리: FC 명령 Locked 해제와 follow/command stop 요청
// 규약: docs/spec/icd.md 4.2 (TRACK_STOP, 운용자의 명시적 TRACK_STOP), 6장
// 사용: mavlink_gateway 의 main 이 생성하여 MavlinkCommandReceiver 에 kRules 로 등록
#ifndef ARGUS_ONBOARD_TRACK_STOP_HANDLER_HPP_
#define ARGUS_ONBOARD_TRACK_STOP_HANDLER_HPP_

#include <cstdint>

#include "argus/core/clock.hpp"
#include "argus/mavlink_gateway/fc_command_lock_release.hpp"
#include "argus/mavlink_gateway/follow_command_tracker.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// 모두 TrackStopHandler 보다 오래 유지되어야 한다
struct TrackStopHandlerInputs {
    const core::IClock& clock;
    IFcCommandLockRelease& lock_release;
    const vehicle_api::IFollowStatusSource& follow_status;  // mission_state
    RequestHeaderFactory& request_headers;
    vehicle_api::IFollowControlClient& follow_control;
    FollowCommandTracker& tracker;
};

// 1. FC 명령 Locked 해제 (해제하지 못하면 로그)
// 2. follow_stop_needed 이면(추종, 스캔 중이거나 MANUAL, FAULT) FollowCommandTracker 로
//    IN_PROGRESS 송신, stop(Operator) 요청, 결과 전달. 그 밖에는 즉시 ACCEPTED
class TrackStopHandler : public IMavlinkCommandHandler {
public:
    static constexpr CommandRules kRules{true, true, CommandKind::Stop};

    explicit TrackStopHandler(const TrackStopHandlerInputs& inputs) : inputs_(inputs) {}

    std::uint16_t command() const override { return MAV_CMD_ARGUS_TRACK_STOP; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    TrackStopHandlerInputs inputs_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TRACK_STOP_HANDLER_HPP_
