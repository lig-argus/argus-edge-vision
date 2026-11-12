// 영상 트랙의 인코딩 설정. 코덱을 변경하면 RtspServer 의 코드가 아니라 이 값이 바뀐다
// 규약: docs/spec/video.md 2장 (영상 트랙), 5장 (송신부 동작). 통제기의 VideoDecodeConfig 와 대응
#ifndef ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_
#define ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace argus::video_sender {

enum class VideoCodec : std::uint8_t {
    H264,  // RTP/H.264 (RFC 6184). 기본 코덱. Raspberry Pi 5 에는 하드웨어 인코더가 없어 x264 로
           // 인코딩한다
    Jpeg,  // RTP/JPEG (RFC 2435). 선택 코덱
};

struct VideoEncodeConfig {
    VideoCodec codec = VideoCodec::H264;

    // appsrc 뒤에 연결하는 GStreamer 요소 문자열. 마지막 요소는 RTP 페이로더여야 한다.
    // 각 설정의 이유는 make_video_encode_config() 의 주석
    std::string gstreamer_elements =
        "videoconvert ! video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 ! "
        "rtph264pay aggregate-mode=zero-latency config-interval=-1";

    // RTP 헤더의 payload type. H.264 는 고정 번호가 없어 동적 번호(96~127) 중에서 정하며 메타데이터
    // 트랙의 번호와 달라야 한다. JPEG 은 고정 번호 26 이다 (RFC 3551)
    std::uint8_t payload_type = 96;

    // JPEG 품질 (1~100). 값이 없으면 인코더의 기본값. codec 이 Jpeg 인 경우에만 지정한다.
    // 값은 video.md 9장 12 의 측정 후 설정 파일로 정한다
    std::optional<std::int32_t> quality;

    // 목표 비트레이트 (kbit/s). 값이 없으면 인코더의 기본값. codec 이 H264 인 경우에만 지정한다
    std::optional<std::int32_t> bitrate_kbps;
};

// 코덱별 기본 설정을 반환한다. 실행 파일은 이 함수로 설정을 생성하므로 codec 과 gstreamer_elements
// 가 서로 다른 코덱을 가리키지 않는다
// - H264: tune=zerolatency 는 프레임 재정렬과 미리 보기 버퍼를 사용하지 않아 인코더가 프레임을
//   보관하지 않는다. speed-preset=ultrafast 는 연산량이 가장 작은 설정이다. key-int-max 는 키
//   프레임 간격(프레임 수)이며, 패킷 손실 후의 복구와 중간에 접속한 수신기의 재생 시작이 이 간격
//   안에 이루어진다. 30 은 30 FPS 에서 1초이다. config-interval=-1 은 키 프레임마다 SPS 와 PPS 를
//   함께 송신한다. aggregate-mode=zero-latency 는 한 프레임의 NAL 단위를 다음 프레임을 기다리지
//   않고 송신한다
inline VideoEncodeConfig make_video_encode_config(VideoCodec codec) {
    VideoEncodeConfig config;
    config.codec = codec;
    if (codec == VideoCodec::Jpeg) {
        config.gstreamer_elements = "videoconvert ! video/x-raw,format=I420 ! jpegenc ! rtpjpegpay";
        config.payload_type = 26;
    }
    return config;
}

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_VIDEO_ENCODE_CONFIG_HPP_
