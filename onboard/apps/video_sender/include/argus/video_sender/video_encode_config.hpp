// 영상 트랙의 인코딩 설정. 코덱을 바꿀 때 RtspServer 코드가 아니라 이 값만 바뀐다
// 규약: docs/spec/video.md 2장 (RTP/JPEG), 5장. 통제기의 VideoDecodeConfig 와 짝
#ifndef ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_
#define ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace argus::video_sender {

enum class VideoCodec : std::uint8_t {
    Jpeg,  // RTP/JPEG (RFC 2435). 지금 쓰는 코덱
    H264,  // 나중에 쓸 수 있게 자리만 둔다
};

struct VideoEncodeConfig {
    VideoCodec codec = VideoCodec::Jpeg;
    // appsrc 뒤에 붙는 GStreamer 요소 문자열 (video.md 2장)
    std::string gstreamer_elements =
        "videoconvert ! video/x-raw,format=I420 ! jpegenc ! rtpjpegpay";
    std::uint8_t payload_type = 26;  // RTP/JPEG 의 고정 페이로드 타입
    // JPEG 품질 (1~100). 비어 있으면 인코더 기본값. 값은 video.md 9장 12 측정 뒤 설정 파일로 정한다
    std::optional<int> quality;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_
