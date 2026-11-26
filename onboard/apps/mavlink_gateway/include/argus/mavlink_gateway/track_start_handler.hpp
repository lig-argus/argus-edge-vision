// MAV_CMD_ARGUS_TRACK_START (45000) 의 처리: 매개변수와 ③ ④ 검사 뒤 follow/command track_start 요청
// 규약: docs/spec/icd.md 4.2, docs/spec/process_interfaces.md 5.2 (label 보완)
// 사용: mavlink_gateway 의 main 이 생성하여 MavlinkCommandReceiver 에 kRules 로 등록
#ifndef ARGUS_ONBOARD_TRACK_START_HANDLER_HPP_
#define ARGUS_ONBOARD_TRACK_START_HANDLER_HPP_

#include <cstdint>

#include "argus/core/clock.hpp"
#include "argus/mavlink_gateway/follow_command_tracker.hpp"
#include "argus/mavlink_gateway/latest_tracks.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/tracking_status_source.hpp"
#include "argus/perception_api/vision_status_source.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"

namespace argus::mavlink_gateway {

// 모두 TrackStartHandler 보다 오래 유지되어야 한다
struct TrackStartHandlerInputs {
    const core::IClock& clock;
    const SensorSourceMap& sensors;
    const perception_api::ITrackingStatusSource& tracking_status;  // ③ FOLLOWABLE
    const perception_api::IVisionStatusSource& vision_status;      // ③ FOLLOWABLE
    const ILatestTracks& tracks;                                   // ④ 최신 트랙
    RequestHeaderFactory& request_headers;
    vehicle_api::IFollowControlClient& follow_control;
    FollowCommandTracker& tracker;
};

// - 매개변수 범위 밖 → DENIED (parse_track_start)
// - p2 센서가 SensorSourceMap 에 없거나 FOLLOWABLE 이 아님 → TEMPORARILY_REJECTED
// - ④ 불충족 → DENIED (select_track_target)
// - 통과하면 FollowCommandTracker 로 IN_PROGRESS 송신, track_start 요청, 결과 전달
// ② ⑤ ⑥ 은 추종 제어가 검사한다 (icd 4.1)
class TrackStartHandler : public IMavlinkCommandHandler {
public:
    static constexpr CommandRules kRules{true, true, CommandKind::Start};

    explicit TrackStartHandler(const TrackStartHandlerInputs& inputs) : inputs_(inputs) {}

    std::uint16_t command() const override { return MAV_CMD_ARGUS_TRACK_START; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    TrackStartHandlerInputs inputs_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TRACK_START_HANDLER_HPP_
