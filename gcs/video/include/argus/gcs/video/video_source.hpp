// 영상 출처 인터페이스. 화면(영상 칸)과 녹화는 영상이 어떻게 오는지 모르고 이것만 본다.
// 해독한 프레임만 내며 코덱과 녹화 형식을 드러내지 않는다 (녹화는 해독 전 압축 스트림을 따로
// 저장한다). 시각은 모두 IClock 의 monotonic ns 이다. 구현: RtspVideoSource (gcs/video_rtsp)
#ifndef ARGUS_GCS_VIDEO_VIDEO_SOURCE_HPP_
#define ARGUS_GCS_VIDEO_VIDEO_SOURCE_HPP_

#include <QImage>
#include <QObject>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "argus/gcs/video/video_counters.hpp"

namespace argus::gcs::video {

// 센서 하나의 최신 영상 프레임과 수신 통계 (화면은 이것만 본다)
struct VideoView {
    QImage image;
    std::uint32_t frameId{0};    // 짝지은 메타데이터의 frame_id. 짝이 없으면 0
    std::int64_t receivedNs{0};  // 받은 시각
    double fps{0.0};             // 최근 1 초에 받은 프레임 수
    double kbPerSec{0.0};        // 최근 1 초에 받은 RTP 바이트 (kB/s, 헤더 포함)
};

// 상자와 짝짓기 위해 잠깐 보관하는 영상 프레임
struct VideoFrame {
    QImage image;
    std::uint32_t frameId{0};
    std::int64_t receivedNs{0};
};

// 영상 RTSP 세션의 트랙
enum class VideoTrack : std::uint8_t {
    Video,     // 영상 RTP
    Metadata,  // 메타데이터 RTP
};

// 영상 손실 기록 하나 (통신 통계 세부 보기)
struct VideoLossRecord {
    std::int64_t detectedNs{0};  // 건너뛴 뒤 첫 패킷을 받은 시각
    VideoTrack track{VideoTrack::Video};
    std::uint16_t lastSeq{0};  // 건너뛰기 전에 받은 RTP 순번
    std::uint16_t nextSeq{0};  // 건너뛴 뒤 받은 RTP 순번
    std::int32_t missing{0};   // 빠진 패킷 수
};

// 트랙 하나의 누적 RTP 패킷 수
struct VideoTrackStats {
    VideoTrack track{VideoTrack::Video};
    std::uint64_t received{0};
    std::uint64_t lost{0};
    std::uint64_t reordered{0};
    std::uint64_t duplicated{0};
};

// 센서 하나의 영상 수신 통계 (통신 통계 표의 영상 줄과 세부 보기). 수는 RTP 패킷 단위
struct VideoLinkStats {
    static constexpr std::size_t kMaxLossRecords = 50;

    double fps{0.0};
    double kbPerSec{0.0};
    std::uint64_t received{0};  // 두 트랙의 합
    std::uint64_t lost{0};
    std::uint64_t reordered{0};
    std::uint64_t duplicated{0};
    std::vector<VideoTrackStats> tracks;
    std::deque<VideoLossRecord>
        losses;  // 두 트랙을 시각 순으로 합친 것, 오래된 것부터 최대 kMaxLossRecords
};

class IVideoSource : public QObject {
    Q_OBJECT

public:
    explicit IVideoSource(QObject* parent = nullptr) : QObject(parent) {}
    ~IVideoSource() override;

    // 그 센서(ARGUS_SENSOR)의 최신 프레임. 받은 적 없으면 없음
    virtual std::optional<VideoView> view(std::uint8_t sensor) const = 0;
    // 상자와 짝지으려고 보관하는 최근 프레임, 최신 순
    virtual std::vector<VideoFrame> heldFrames(std::uint8_t sensor) const = 0;
    // 누적 수신 값 (모든 센서의 합)
    virtual VideoCounters counters() const = 0;
    // 그 센서의 수신 통계. 그 센서를 받은 적 없으면 없음
    virtual std::optional<VideoLinkStats> linkStats(std::uint8_t sensor) const = 0;

signals:
    // 그 센서의 새 프레임 (view, heldFrames 가 바뀜). Qt 이벤트 루프 스레드에서 발생한다
    void frameReady(std::uint8_t sensor);
};

}  // namespace argus::gcs::video

#endif  // ARGUS_GCS_VIDEO_VIDEO_SOURCE_HPP_
