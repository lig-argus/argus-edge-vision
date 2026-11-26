// RTSP 메타데이터 트랙의 ONVIF XML 문서 하나(영상 프레임 하나의 추적 결과)를 해석한다.
// 규약: docs/spec/video.md 3.2 (XML 구조, argus: 확장 요소), 3.3 (좌표 변환)
#ifndef ARGUS_GCS_RTSP_ONVIF_METADATA_HPP_
#define ARGUS_GCS_RTSP_ONVIF_METADATA_HPP_

#include <QByteArray>
#include <QMetaType>
#include <QRectF>
#include <QString>
#include <cstdint>
#include <optional>
#include <vector>

namespace argus::gcs::rtsp {

// 추적 결과 하나 (tt:Object)
struct TrackedObject {
    std::uint32_t objectId{0};  // tt:Object@ObjectId = 추적 번호 (1~16,777,215)

    // 영상 안 위치. 영상 폭과 높이를 1 로 둔 좌표(원점 왼쪽 위, y 아래 방향).
    // 표시 크기나 영상 픽셀 크기를 곱해 쓴다
    QRectF box;

    double likelihood{0.0};                   // tt:Type@Likelihood (0~1)
    QString type;                             // tt:Type 글 (ONVIF 분류명, 예: Vehicle)
    std::optional<std::int32_t> classId;      // argus:ClassId
    QString label;                            // argus:Label
    std::optional<std::uint32_t> generation;  // argus:Generation (표적 선택 식별자)
    QString state;                            // argus:State (예: CONFIRMED)
};

// 메타데이터 문서 하나 = 영상 프레임 하나 (tt:Frame)
struct FrameMetadata {
    // tt:Frame@UtcTime (1970년 기준 ns). 영상 프레임과 짝짓는 기준 (video.md 4장)
    std::int64_t utcNs{0};

    QString source;                            // tt:Frame@Source (카메라 식별 문자열)
    std::optional<std::uint64_t> frameId;      // argus:FrameId
    std::optional<std::uint32_t> imageWidth;   // argus:ImageWidth (픽셀)
    std::optional<std::uint32_t> imageHeight;  // argus:ImageHeight (픽셀)
    std::vector<TrackedObject> objects;        // 추적 결과가 없는 프레임이면 비어 있다
};

// XML 문서 하나를 해석한다. tt:Frame 이 없거나, UtcTime 을 읽지 못하거나, XML 이 깨졌으면 없음
std::optional<FrameMetadata> parseOnvifMetadata(const QByteArray& xml);

}  // namespace argus::gcs::rtsp

Q_DECLARE_METATYPE(argus::gcs::rtsp::FrameMetadata)

#endif  // ARGUS_GCS_RTSP_ONVIF_METADATA_HPP_
