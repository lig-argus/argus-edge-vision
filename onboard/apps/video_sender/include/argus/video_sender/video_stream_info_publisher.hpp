// 영상 송신부가 실제로 송신하는 스트림의 정보를 video/stream_info 로 발행한다
// 규약: docs/spec/process_interfaces.md 5.1.6, docs/design/uml/class_video.md 1절
//
// 규칙
// - tick() 을 kCheckPeriod 마다 호출한다. 값이 직전에 발행한 값과 다르면 바로 발행하고, 같으면
//   마지막 발행 뒤 kRepublishPeriod 가 지났을 때 다시 발행한다 (늦게 시작한 구독자도 받도록)
// - running: RtspServer 가 push() 로 받은 프레임 가운데 크기가 맞는 프레임의 수가 최근
//   kRunningWindow 안에 늘었으면 true. 클라이언트가 없어 폐기한 프레임도 센다 (프레임을 받아 RTSP
//   서버로 내보낸 것이므로). 늘어난 것을 tick() 에서 확인하므로 마지막 프레임 뒤 최대
//   kRunningWindow + kCheckPeriod 동안 true 로 남는다
// - RTSP 서버가 열려 있지 않으면 (port 0) 발행하지 않는다
// - 발행에 실패해도 발행한 것으로 기록한다. kRepublishPeriod 뒤에 다시 발행하므로 실패할 때마다
//   kCheckPeriod 간격으로 다시 보내지 않는다
//
// RtspServer 객체 대신 포트와 RtspServerStats 값을 받는 이유: 발행 판단을 GStreamer 없이
// 테스트하기 위해서다
// 수명: 생성자에 넘긴 publisher 와 clock 은 이 객체보다 오래 유지되어야 한다
#ifndef ARGUS_VIDEO_SENDER_VIDEO_STREAM_INFO_PUBLISHER_HPP_
#define ARGUS_VIDEO_SENDER_VIDEO_STREAM_INFO_PUBLISHER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/core/clock.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api_bus/bus_video_stream_info_publisher.hpp"
#include "argus/video_sender/rtsp_server.hpp"

namespace argus::video_sender {

// RTSP 서버와 인코딩 설정에서 얻을 수 없는 스트림의 값. 실행 파일이 정한다
struct VideoStreamDescription {
    std::uint32_t stream_id = 0;  // 1부터 (docs/spec/video.md 7장)
    vehicle_api::Sensor sensor = vehicle_api::Sensor::Unknown;
    std::string name;
    std::uint32_t width = 0;   // 송신 해상도 (픽셀)
    std::uint32_t height = 0;  // 송신 해상도 (픽셀)
    double framerate_hz = 0.0;
    double hfov_deg = 0.0;  // 수평 화각. 모르면 0
};

class VideoStreamInfoPublisher {
public:
    // 값이 바뀐 뒤 발행까지의 지연을 kRepublishPeriod 의 1/10 로 제한한다
    static constexpr std::chrono::milliseconds kCheckPeriod{100};
    static constexpr std::chrono::milliseconds kRepublishPeriod{1000};

    // running 의 판단 시간. 통제기의 영상 정지 판단 시간과 같다 (docs/spec/video.md 6장)
    static constexpr std::chrono::milliseconds kRunningWindow{1000};

    // source 는 메시지의 발행자 식별 문자열. rtsp_path 와 encode 는 RtspServerConfig 의 값
    VideoStreamInfoPublisher(video_api_bus::BusVideoStreamInfoPublisher& publisher,
                             const core::IClock& clock, std::string source,
                             VideoStreamDescription description, std::string rtsp_path,
                             const VideoEncodeConfig& encode);

    // port 는 RtspServer::port(), stats 는 RtspServer::stats() 의 값
    void tick(std::uint16_t port, const RtspServerStats& stats);

private:
    video_api::VideoStream make_stream(std::uint16_t port, bool running) const;

    video_api_bus::BusVideoStreamInfoPublisher& publisher_;
    const core::IClock& clock_;
    std::string source_;
    VideoStreamDescription description_;
    std::string rtsp_path_;
    video_api::VideoCodec codec_;
    std::uint32_t bitrate_bps_;

    std::uint64_t accepted_frames_ = 0;  // 직전 tick() 에서 확인한 프레임 수

    // 프레임 수가 늘어난 것을 마지막으로 확인한 시각. 아직 없으면 nullopt
    std::optional<std::int64_t> last_frame_seen_ns_;

    // 마지막으로 발행한 스트림 값과 시각. 아직 발행하지 않았으면 nullopt
    std::optional<video_api::VideoStream> last_published_;
    std::int64_t last_published_ns_ = 0;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_VIDEO_STREAM_INFO_PUBLISHER_HPP_
