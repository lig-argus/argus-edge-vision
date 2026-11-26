// 영상 송신부가 실제로 송신하는 스트림의 정보 (`video/stream_info`).
// 필드와 단위는 docs/spec/process_interfaces.md 5.1.6 과 같다. 시각은 RPi monotonic clock
// (CLOCK_MONOTONIC) 의 ns 이다. mavlink_gateway 는 이 값으로 VIDEO_STREAM_INFORMATION 을 채운다
// (docs/spec/video.md 7장)
#ifndef ARGUS_VIDEO_API_TYPES_HPP_
#define ARGUS_VIDEO_API_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "argus/vehicle_api/types.hpp"

namespace argus::video_api {

// 영상 트랙의 코덱 (docs/spec/video.md 2장)
enum class VideoCodec : std::uint8_t {
    Unknown,  // 메시지의 UNSPECIFIED 와 이 코드에 정의되지 않은 값
    H264,
    Jpeg,
};

// 스트림 하나. VIDEO_STREAM_INFORMATION 하나에 대응한다
struct VideoStream {
    std::uint32_t stream_id = 0;  // 1부터. VIDEO_STREAM_INFORMATION.stream_id
    vehicle_api::Sensor sensor = vehicle_api::Sensor::Unknown;
    std::string name;  // VIDEO_STREAM_INFORMATION.name

    // RTSP 서버가 실제로 연 포트와 경로. 주소의 호스트는 없다 (통제기가 접속할 주소는
    // mavlink_gateway 가 통제기 쪽 네트워크로 정한다)
    std::uint32_t rtsp_port = 0;
    std::string rtsp_path;  // "/" 로 시작

    std::uint32_t width = 0;   // 송신 해상도 (픽셀)
    std::uint32_t height = 0;  // 송신 해상도 (픽셀)
    double framerate_hz = 0.0;
    VideoCodec codec = VideoCodec::Unknown;
    std::uint32_t bitrate_bps = 0;  // 인코더 설정의 비트 전송률. 모르면 0
    double hfov_deg = 0.0;          // 수평 화각. 모르면 0

    // 최근 1 초 안에 bus 에서 프레임을 받아 RTSP 서버로 내보냈으면 true. 서버만 열려 있고 프레임이
    // 오지 않으면 false
    bool running = false;

    friend bool operator==(const VideoStream& lhs, const VideoStream& rhs) {
        return lhs.stream_id == rhs.stream_id && lhs.sensor == rhs.sensor && lhs.name == rhs.name &&
               lhs.rtsp_port == rhs.rtsp_port && lhs.rtsp_path == rhs.rtsp_path &&
               lhs.width == rhs.width && lhs.height == rhs.height &&
               lhs.framerate_hz == rhs.framerate_hz && lhs.codec == rhs.codec &&
               lhs.bitrate_bps == rhs.bitrate_bps && lhs.hfov_deg == rhs.hfov_deg &&
               lhs.running == rhs.running;
    }
    friend bool operator!=(const VideoStream& lhs, const VideoStream& rhs) { return !(lhs == rhs); }
};

// `video/stream_info` 메시지 하나
struct VideoStreamInfo {
    std::string source;                       // 발행자 식별 문자열
    std::int64_t published_monotonic_ns = 0;  // 발행 시각
    std::vector<VideoStream> streams;         // 송신하는 스트림 전부
};

struct ReceivedVideoStreamInfo {
    VideoStreamInfo info;

    // 구현이 이 값을 받은 시각. info.published_monotonic_ns 는 발행자가 채우는 값이므로, 값이
    // 얼마나 오래됐는지는 이 시각으로 판단한다
    std::int64_t arrived_monotonic_ns = 0;
};

}  // namespace argus::video_api

#endif  // ARGUS_VIDEO_API_TYPES_HPP_
