// 테스트용 IRtspReceiver. 테스트가 프레임, 메타데이터, 상태, 누적 값을 직접 넣는다
#ifndef ARGUS_GCS_VIDEO_RTSP_TEST_FAKE_RTSP_RECEIVER_HPP_
#define ARGUS_GCS_VIDEO_RTSP_TEST_FAKE_RTSP_RECEIVER_HPP_

#include <QString>
#include <cstdint>
#include <utility>

#include "argus/gcs/rtsp/rtsp_receiver.hpp"

namespace argus::gcs::video_rtsp {

class FakeRtspReceiver : public rtsp::IRtspReceiver {
    Q_OBJECT

public:
    explicit FakeRtspReceiver(QString url) : url_(std::move(url)) {}

    void start() override {
        ++starts_;
        setState(rtsp::ReceiverState::Connecting);
    }
    void stop() override { setState(rtsp::ReceiverState::Stopped); }

    QString url() const override { return url_; }
    rtsp::ReceiverState state() const override { return state_; }
    QString lastError() const override {
        return state_ == rtsp::ReceiverState::Failed ? QStringLiteral("연결 실패") : QString();
    }
    rtsp::ReceiverCounters counters() const override { return counters_; }

    // 테스트용
    void setState(rtsp::ReceiverState state) {
        if (state_ != state) {
            state_ = state;
            emit stateChanged(state);
        }
    }
    void deliverFrame(const rtsp::DecodedFrame& frame) { emit frameReceived(frame); }
    void deliverMetadata(const rtsp::FrameMetadata& metadata) { emit metadataReceived(metadata); }
    rtsp::ReceiverCounters& mutableCounters() { return counters_; }
    std::int32_t starts() const { return starts_; }

private:
    QString url_;
    rtsp::ReceiverState state_{rtsp::ReceiverState::Stopped};
    rtsp::ReceiverCounters counters_;
    std::int32_t starts_{0};
};

}  // namespace argus::gcs::video_rtsp

#endif  // ARGUS_GCS_VIDEO_RTSP_TEST_FAKE_RTSP_RECEIVER_HPP_
