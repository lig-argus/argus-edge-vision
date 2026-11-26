// RTSP 스트림 하나(센서 하나)의 수신기 인터페이스 (docs/spec/video.md 1장, 6장).
// 영상 트랙은 해독한 프레임과 그 프레임의 UTC 로, 메타데이터 트랙은 해석한 FrameMetadata 로 알린다.
// 둘의 짝짓기는 받는 쪽이 FrameSynchronizer 로 한다.
// 구현: GstRtspReceiver (gcs/rtsp_gstreamer). 사용: RtspVideoSource (gcs/video_rtsp) 가 생성 함수로
// 만든다
#ifndef ARGUS_GCS_RTSP_RTSP_RECEIVER_HPP_
#define ARGUS_GCS_RTSP_RTSP_RECEIVER_HPP_

#include <QByteArray>
#include <QMetaType>
#include <QObject>
#include <QString>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "argus/gcs/rtsp/onvif_metadata.hpp"
#include "argus/gcs/rtsp/rtp_sequence.hpp"

namespace argus::gcs::rtsp {

// 해독한 영상 프레임 하나. 화소는 RGBX 8 비트 4 바이트 (R, G, B, 쓰지 않는 바이트) 순서이고,
// 한 줄은 bytesPerLine 바이트다 (줄 끝에 빈 바이트가 있을 수 있다)
struct DecodedFrame {
    QByteArray rgbx;
    std::int32_t width{0};         // 픽셀
    std::int32_t height{0};        // 픽셀
    std::int32_t bytesPerLine{0};  // 바이트

    // RTCP Sender Report 로 구한 프레임 시각 (1970년 기준 ns). 첫 SR 전이면 없음 (상자 없이 표시)
    std::optional<std::int64_t> utcNs;
};

enum class ReceiverState : std::uint8_t {
    Stopped,     // start() 전, stop() 뒤
    Connecting,  // RTSP 연결과 트랙 준비 중
    Playing,     // 받는 중
    Failed,      // 연결 실패나 오류 (lastError). 다시 start() 로 연결한다
};

// start() 부터 센 누적 값
struct ReceiverCounters {
    std::uint64_t frames{0};             // 해독해 알린 영상 프레임
    std::uint64_t framesWithUtc{0};      // 그중 UTC 가 있는 프레임
    std::uint64_t metadataDocuments{0};  // 해석한 메타데이터 문서
    std::uint64_t droppedDocuments{0};   // 패킷이 빠져 버린 문서
    std::uint64_t badDocuments{0};       // XML 을 해석하지 못한 문서
    std::uint64_t bytes{0};              // 받은 RTP 바이트 (영상 트랙과 메타데이터 트랙, 헤더 포함)
    RtpStats videoRtp;                   // 영상 트랙 RTP 순번으로 센 패킷 손실
    RtpStats metadataRtp;                // 메타데이터 트랙 RTP 순번으로 센 패킷 손실
};

class IRtspReceiver : public QObject {
    Q_OBJECT

public:
    explicit IRtspReceiver(QObject* parent = nullptr) : QObject(parent) {}
    ~IRtspReceiver() override;

    // 연결을 시작한다. 이미 연결 중이면 끊고 다시 연결한다
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual QString url() const = 0;
    virtual ReceiverState state() const = 0;
    virtual QString lastError() const = 0;  // Failed 가 된 이유. 그 밖의 상태에서는 빈 문자열
    virtual ReceiverCounters counters() const = 0;

signals:
    // 모두 이 객체의 스레드(Qt 이벤트 루프)에서 발생한다
    void frameReceived(const argus::gcs::rtsp::DecodedFrame& frame);
    void metadataReceived(const argus::gcs::rtsp::FrameMetadata& metadata);
    void stateChanged(argus::gcs::rtsp::ReceiverState state);
};

// RTSP 주소 하나의 수신기를 만든다. 구체 수신기는 app 이 정해 넘긴다
using RtspReceiverFactory = std::function<std::unique_ptr<IRtspReceiver>(const QString& url)>;

}  // namespace argus::gcs::rtsp

Q_DECLARE_METATYPE(argus::gcs::rtsp::DecodedFrame)

#endif  // ARGUS_GCS_RTSP_RTSP_RECEIVER_HPP_
