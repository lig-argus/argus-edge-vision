// 탐지 결과 값 타입. RTSP 메타데이터 트랙(video.md 4장)의 한 프레임을 통제기가 쓰는 형태로 둔다.
// 메타데이터를 이 타입으로 바꾸는 일은 영상 수신 쪽 어댑터가 한다
#ifndef ARGUS_GCS_LINK_DETECTION_HPP_
#define ARGUS_GCS_LINK_DETECTION_HPP_

#include <QString>
#include <cstdint>
#include <vector>

#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

// 박스 하나. 좌표는 영상의 픽셀 좌표 (원점 왼쪽 위)
struct DetectionBox {
    TrackId trackId{0};        // tt:Object@ObjectId
    std::uint32_t classId{0};  // argus:ClassId
    QString label;             // argus:Label
    float confidence{0.0F};    // tt:Type@Likelihood, 0~1
    float left{0.0F};
    float top{0.0F};
    float right{0.0F};
    float bottom{0.0F};
    bool confirmed{false};  // argus:State 가 CONFIRMED
};

// 탐지 결과 한 프레임
struct DetectionFrame {
    std::uint32_t frameId{0};      // argus:FrameId
    std::uint32_t generation{0};   // argus:Generation. 같은 trackId 라도 다르면 다른 표적
    std::uint16_t imageWidth{0};   // argus:ImageWidth
    std::uint16_t imageHeight{0};  // argus:ImageHeight
    QString source;                // tt:Frame@Source (카메라 식별 문자열)
    std::vector<DetectionBox> boxes;
    std::int64_t receivedNs{0};  // 통제기가 받은 시각
    double fps{0.0};             // 그 센서의 최근 1 s 수신 프레임 수 (DetectionStore 가 채운다)
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_DETECTION_HPP_
