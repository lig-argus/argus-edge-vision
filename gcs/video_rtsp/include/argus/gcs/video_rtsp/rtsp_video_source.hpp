// IVideoSource 의 RTSP 구현: 센서마다 RTSP 주소 하나를 IRtspReceiver 로 받아 화면과 녹화가 쓰는
// 모양 (VideoView, heldFrames, 수신 통계)으로 바꾼다. 수신기는 생성 함수로 받으므로 구체
// 수신기(GStreamer)를 모른다. 메타데이터는 영상 프레임과 UTC 로 짝지은 뒤(FrameSynchronizer) 그
// 프레임보다 먼저 metadataReceived 로 낸다. argus:FrameId 를 그 프레임의 frame_id 로 쓰므로 영상
// 칸은 frame_id 로 상자를 찾는다 (docs/spec/video.md 4장, 6장). 주소는 mavlink_gateway 가
// VIDEO_STREAM_INFORMATION 으로 알린 것을 조립하는 쪽이 setStreams 로 넘긴다 (video.md 7장)
#ifndef ARGUS_GCS_VIDEO_RTSP_RTSP_VIDEO_SOURCE_HPP_
#define ARGUS_GCS_VIDEO_RTSP_RTSP_VIDEO_SOURCE_HPP_

#include <QString>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/rtsp/onvif_metadata.hpp"
#include "argus/gcs/rtsp/rtsp_receiver.hpp"
#include "argus/gcs/video/video_source.hpp"

namespace argus::gcs::video_rtsp {

class RtspVideoSource : public video::IVideoSource {
    Q_OBJECT

public:
    struct Stream {
        std::uint8_t sensor{0};  // ARGUS_SENSOR (1 IR, 2 EO)
        QString url;             // rtsp://…

        bool operator==(const Stream& other) const {
            return sensor == other.sensor && url == other.url;
        }
    };

    static constexpr std::int64_t kHoldNs =
        300'000'000;  // 상자와 짝짓기 위해 프레임을 보관하는 시간
    static constexpr std::int64_t kRetryNs =
        3'000'000'000;  // 연결에 실패한 수신기를 다시 여는 간격

    // 이 시간 안에 프레임을 받았으면 receiving (video.md 6장 "영상 정지" 판단 시간)
    static constexpr std::int64_t kReceivingNs = 1'000'000'000;

    static constexpr std::int64_t kRateWindowNs = 1'000'000'000;  // FPS, kB/s 를 세는 구간

    // clock 은 이 객체보다 오래 살아야 한다
    RtspVideoSource(rtsp::RtspReceiverFactory factory, const core::IClock& clock,
                    QObject* parent = nullptr);
    ~RtspVideoSource() override;
    RtspVideoSource(const RtspVideoSource&) = delete;
    RtspVideoSource& operator=(const RtspVideoSource&) = delete;
    RtspVideoSource(RtspVideoSource&&) = delete;
    RtspVideoSource& operator=(RtspVideoSource&&) = delete;

    // 열 스트림 (센서마다 하나). 바뀌면 모두 닫고 다시 연다. 비우면 닫는다
    void setStreams(std::vector<Stream> streams);
    const std::vector<Stream>& streams() const { return streams_; }

    // 연결에 실패한 수신기를 kRetryNs 뒤 다시 연다. 조립하는 쪽의 타이머가 주기적으로 부른다
    void tick();

    // 어느 스트림이든 최근 kReceivingNs 안에 프레임을 받았는가
    bool receiving() const;
    // 어느 스트림이든 연결해 받는 중(Playing)인가
    bool playing() const;

    std::optional<video::VideoView> view(std::uint8_t sensor) const override;
    std::vector<video::VideoFrame> heldFrames(std::uint8_t sensor) const override;
    video::VideoCounters counters() const override;
    std::optional<video::VideoLinkStats> linkStats(std::uint8_t sensor) const override;

signals:
    // 짝지은 메타데이터 (그 프레임의 frameReady 보다 먼저). session: 이 연결을 가리키는 글.
    // setStreams 로 다시 열면 바뀐다 (받는 쪽이 송신 쪽 재시작과 frame_id 순서를 구분할 때 쓴다)
    void metadataReceived(std::uint8_t sensor, const argus::gcs::rtsp::FrameMetadata& metadata,
                          const QString& session);

private:
    struct Channel;

    void openAll();
    void closeAll();
    void onFrame(Channel& channel, const rtsp::DecodedFrame& frame);
    const Channel* channelFor(std::uint8_t sensor) const;

    rtsp::RtspReceiverFactory factory_;
    const core::IClock& clock_;
    std::vector<Stream> streams_;
    std::vector<std::unique_ptr<Channel>> channels_;
    std::uint64_t connection_{0};  // setStreams 로 열 때마다 하나씩 (session 글에 쓴다)
    std::uint64_t displayed_{0};
    std::optional<std::int64_t> lastDisplayNs_;
};

}  // namespace argus::gcs::video_rtsp

#endif  // ARGUS_GCS_VIDEO_RTSP_RTSP_VIDEO_SOURCE_HPP_
