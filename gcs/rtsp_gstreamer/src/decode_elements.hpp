// SDP 의 영상 인코딩 이름(a=rtpmap) → GStreamer 해독 요소 (docs/spec/video.md 2장, 2.1, 6장).
// 코덱을 하나 더 받으려면 이 표에 한 줄을 더한다. 수신기는 고치지 않는다
#ifndef ARGUS_GCS_RTSP_GSTREAMER_DECODE_ELEMENTS_HPP_
#define ARGUS_GCS_RTSP_GSTREAMER_DECODE_ELEMENTS_HPP_

#include <QString>
#include <optional>

namespace argus::gcs::rtsp_gstreamer {

// 예: "H264" → "rtph264depay ! avdec_h264". 모르는 인코딩이면 없음
std::optional<QString> videoDecodeElements(const QString& encodingName);

}  // namespace argus::gcs::rtsp_gstreamer

#endif  // ARGUS_GCS_RTSP_GSTREAMER_DECODE_ELEMENTS_HPP_
