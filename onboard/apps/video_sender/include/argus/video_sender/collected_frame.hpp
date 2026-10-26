// 같은 (source, frame_id) 의 프레임·탐지 결과·추적 결과를 모은 것. FrameCollector 의 출력이며
// OnvifMetadataSerializer 와 RtspServer 의 입력이다
// 규약: docs/spec/process_interfaces.md 5.3, docs/spec/video.md 3.2
#ifndef ARGUS_VIDEO_SENDER_COLLECTED_FRAME_HPP_
#define ARGUS_VIDEO_SENDER_COLLECTED_FRAME_HPP_

#include <optional>

#include "argus/perception_api/types.hpp"

namespace argus::video_sender {

// 영상 프레임은 항상 있다 (프레임 없는 탐지·추적 결과는 보내지 않는다, process_interfaces.md 5.3).
// 탐지·추적 결과는 대기 시간 안에 오지 않으면 비어 있고, 그때 메타데이터는 객체 없는 tt:Frame 이
// 된다 (video.md 3.2)
struct CollectedFrame {
    perception_api::Frame frame;
    std::optional<perception_api::Observations> observations;
    std::optional<perception_api::Tracks> tracks;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_COLLECTED_FRAME_HPP_
