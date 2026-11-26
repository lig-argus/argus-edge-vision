// IRtspReceiver 의 GStreamer 구현 (docs/spec/video.md 6장).
// 파이프라인: rtspsrc ─┬─ 영상 트랙:        <해독 요소> ! videoconvert ! RGBx ! appsink
//                      └─ 메타데이터 트랙:  appsink (RTP 패킷 그대로 → MetadataReassembler)
// 프레임 UTC: 해독 전 영상 RTP 패킷의 (PTS, SSRC, RTP 타임스탬프)를 기록하고(RtpFrameTimes), 수신
// 세션의 RTCP SR 로 RtcpClock 을 갱신한다. 해독한 프레임의 PTS 로 RTP 타임스탬프를 찾아 UTC 로
// 바꾼다 (video.md 4장). GStreamer 콜백은 GStreamer 스레드에서 불리므로 알림은 이 객체의 스레드로
// 넘긴다. GStreamer 버스는 GLib 메인 루프 없이 Qt 이벤트 루프의 타이머로 읽는다. 손실 기록의 시각은
// 생성자로 받은 IClock 으로 GStreamer 스레드에서 읽는다 (IClock 구현은 스레드 안전해야 한다)
#ifndef ARGUS_GCS_RTSP_GSTREAMER_GST_RTSP_RECEIVER_HPP_
#define ARGUS_GCS_RTSP_GSTREAMER_GST_RTSP_RECEIVER_HPP_

#include <QObject>
#include <QString>
#include <cstdint>
#include <memory>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/rtsp/rtsp_receiver.hpp"

namespace argus::gcs::rtsp_gstreamer {

class GstRtspReceiver : public rtsp::IRtspReceiver {
    Q_OBJECT

public:
    static constexpr std::int32_t kDefaultLatencyMs = 100;  // rtspsrc 지터 버퍼 (ms)

    // url 예: rtsp://192.168.4.1:8554/argus/ir. clock 은 이 객체보다 오래 살아야 한다
    GstRtspReceiver(QString url, const core::IClock& clock,
                    std::int32_t latencyMs = kDefaultLatencyMs, QObject* parent = nullptr);
    ~GstRtspReceiver() override;
    GstRtspReceiver(const GstRtspReceiver&) = delete;
    GstRtspReceiver& operator=(const GstRtspReceiver&) = delete;
    GstRtspReceiver(GstRtspReceiver&&) = delete;
    GstRtspReceiver& operator=(GstRtspReceiver&&) = delete;

    void start() override;
    void stop() override;

    QString url() const override { return url_; }
    rtsp::ReceiverState state() const override { return state_; }
    QString lastError() const override { return lastError_; }
    rtsp::ReceiverCounters counters() const override;

private:
    struct Impl;

    void setState(rtsp::ReceiverState state, const QString& error = {});

    QString url_;
    std::int32_t latencyMs_;
    rtsp::ReceiverState state_{rtsp::ReceiverState::Stopped};
    QString lastError_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace argus::gcs::rtsp_gstreamer

#endif  // ARGUS_GCS_RTSP_GSTREAMER_GST_RTSP_RECEIVER_HPP_
