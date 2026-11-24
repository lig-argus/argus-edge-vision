// RTSP 세션 하나에 영상 트랙과 메타데이터 트랙을 송신하는 IStreamSink 구현 (gst-rtsp-server)
// 규약: docs/spec/video.md 1장 (스트림 구성), 2장 (영상 트랙), 3.1 (메타데이터 RTP), 4장 (동기화),
//      5장 (송신부 동작)
//
// 동작
// - start() 는 RTSP 서버를 열고 GLib main loop 를 별도 스레드에서 실행한다. 클라이언트의 RTSP
//   요청은 그 스레드와 gst-rtsp-server 의 스레드에서 처리한다
// - 클라이언트가 접속해 PLAY 를 요청하기 전에는 송신할 파이프라인이 없으므로 push() 로 받은
//   프레임을 폐기한다. 클라이언트가 접속하면 그 이후의 프레임부터 송신한다 (video.md 5장).
//   클라이언트가 모두 끊으면 파이프라인이 삭제되고, 다시 접속하면 새로 생성된다
// - 여러 클라이언트가 같은 파이프라인을 공유한다
// - 메타데이터 문서를 RTP 패킷으로 나누어 송신한다. RTP 타임스탬프는 Frame.unix_ns 를 90 kHz
//   클럭으로 변환한 값이다
// - 영상과 메타데이터 버퍼에 같은 PTS 를 부여한다. PTS 는 프레임을 RPi 가 수신한 시각
//   (Frame.received_monotonic_ns)에 해당하는 running time 이다. RTCP Sender Report 가 running time
//   을 wall clock 에 대응시키므로, Frame.unix_ns 와 Frame.received_monotonic_ns 가 같은 시점의
//   값이면 통제기가 계산한 영상 프레임의 UTC 시각이 Frame.unix_ns 와 같아진다 (video.md 4장)
// - 송신하지 못한 프레임은 폐기하고 원인별 횟수를 stats() 로 제공한다
//
// 스레드: start(), stop(), port(), stats(), push() 는 모두 같은 스레드에서 호출한다
#ifndef ARGUS_VIDEO_SENDER_RTSP_SERVER_HPP_
#define ARGUS_VIDEO_SENDER_RTSP_SERVER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "argus/core/clock.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/metadata_rtp_packetizer.hpp"
#include "argus/video_sender/stream_sink.hpp"
#include "argus/video_sender/video_encode_config.hpp"

namespace argus::video_sender {

struct RtspServerConfig {
    // metadata_rtp 는 기본값이 없다 (payload type 과 SSRC 는 실행 파일이 정한다)
    explicit RtspServerConfig(const MetadataRtpConfig& metadata_rtp_value)
        : metadata_rtp(metadata_rtp_value) {}

    // 메타데이터 트랙의 RTP 설정. payload_type 은 패킷의 헤더와 SDP 의 a=rtpmap 에 함께 사용한다
    MetadataRtpConfig metadata_rtp;

    // RTSP 서버의 TCP 포트 (video.md 1장). 0 이면 운영체제가 사용 가능한 포트를 정한다
    std::uint16_t port = 8554;

    // RTSP URL 의 경로 (video.md 1장: /argus/<센서>)
    std::string path = "/argus/ir";

    // 영상 트랙의 인코딩 설정. gstreamer_elements 의 마지막 요소는 RTP 페이로더여야 한다
    // (RtspServer 가 그 요소에 name=pay0 과 pt 를 지정하며, start() 가 이를 확인한다).
    // 코덱별 설정은 make_video_encode_config() 로 생성한다
    VideoEncodeConfig video;
};

// push() 로 받은 프레임의 처리 결과별 횟수
struct RtspServerStats {
    std::uint64_t sent = 0;                 // 영상과 메타데이터를 파이프라인에 넘김
    std::uint64_t dropped_no_client = 0;    // 클라이언트가 없어 파이프라인이 없음
    std::uint64_t dropped_not_playing = 0;  // 파이프라인이 재생 상태가 아니어서 시각을 구할 수 없음
    std::uint64_t dropped_invalid_frame = 0;  // 프레임의 크기 정보가 픽셀 수와 맞지 않음
    std::uint64_t dropped_send_failed = 0;    // 파이프라인이 버퍼를 받지 않음
};

class RtspServer : public IStreamSink {
public:
    RtspServer(const core::IClock& clock, RtspServerConfig config);
    ~RtspServer() override;

    // 서버를 연다. 이미 열려 있으면 true 를 반환한다.
    // 다음 경우 false 를 반환하고 원인을 std::cerr 에 기록한다
    // - 포트를 열지 못함
    // - 영상 트랙의 설정으로 파이프라인을 구성할 수 없음 (인코더 요소가 설치되어 있지 않은 경우
    // 포함)
    // - codec 에 해당하지 않는 값을 지정함 (Jpeg 가 아닌데 quality, H264 가 아닌데 bitrate_kbps)
    // - quality 나 bitrate_kbps 를 지정했는데 gstreamer_elements 에 그 인코더 요소가 없음
    // - 영상 트랙과 메타데이터 트랙의 payload type 이 같음
    bool start();

    // 접속한 클라이언트를 모두 끊고 서버를 닫는다. 소멸자도 호출한다
    void stop();

    // start() 가 성공한 뒤 실제로 열린 포트. 열려 있지 않으면 0
    std::uint16_t port() const;

    RtspServerStats stats() const;

    void push(const perception_api::Frame& frame, std::string_view metadata_document) override;

private:
    // GStreamer 헤더를 이 헤더에 포함하지 않기 위해 구현을 분리한다
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_RTSP_SERVER_HPP_
