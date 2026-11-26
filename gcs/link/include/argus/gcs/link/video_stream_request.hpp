// 영상 주소 요청 (video.md 7장). mavlink_gateway(compid 195)에 MAV_CMD_REQUEST_MESSAGE(param1 = 269
// VIDEO_STREAM_INFORMATION, param2 = 0 모든 스트림)를 보내고, 스트림마다 오는
// VIDEO_STREAM_INFORMATION 을 모아 센서별 RTSP 주소를 만든다. 통제기는 이 주소만 열고 주소를 직접
// 만들지 않는다 (주소의 호스트는 mavlink_gateway 가 요청한 통제기에 맞춰 채운다).
// - 보내기: mavlink_gateway 가 처음 연결되거나 두절 뒤 돌아오면 begin. icd 4.1 의 간격과 횟수로
//   다시 보내고, 스트림이 다 모이면 멈춘다. 매번 새 요청(confirmation 0)으로 보낸다: ACK 만 오고
//   스트림이 빠졌을 때 같은 요청의 되풀이로 보지 않고 다시 보내게 한다 (여러 번 받아도 결과가 같다)
// - 센서: flags 의 THERMAL 비트가 있으면 IR, 없으면 EO (video.md 7장)
// - type 이 RTSP 가 아니거나 주소가 rtsp:// 가 아닌 스트림은 열지 않는다. 코덱은 encoding 이 아니라
//   RTSP DESCRIBE 응답의 SDP 에서 읽으므로 encoding 은 기록에만 쓴다 (video.md 1장)
// 소켓과 시계를 갖지 않는다. 보낼 메시지는 sendDue 가 돌려주고 시각은 monotonic ns 로 받는다
#ifndef ARGUS_GCS_LINK_VIDEO_STREAM_REQUEST_HPP_
#define ARGUS_GCS_LINK_VIDEO_STREAM_REQUEST_HPP_

#include <QMetaType>
#include <QString>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

struct VideoStreamUri {
    std::uint8_t sensor{ARGUS_SENSOR_UNKNOWN};             // ARGUS_SENSOR (IR, EO)
    QString uri;                                           // rtsp://…
    QString name;                                          // 스트림 이름 (기록용)
    std::uint8_t encoding{VIDEO_STREAM_ENCODING_UNKNOWN};  // VIDEO_STREAM_ENCODING (기록용)
};
using VideoStreams = std::vector<VideoStreamUri>;

// 열 수 있는 스트림이면 센서와 주소
std::optional<VideoStreamUri> decodeVideoStream(const mavlink_video_stream_information_t& info);

class VideoStreamRequest {
public:
    enum class Outcome : std::uint8_t {
        None,
        NoReply,  // 횟수만큼 보냈는데 스트림이 다 모이지 않았다
        Denied,   // mavlink_gateway 가 수락, 진행 중이 아닌 결과로 답했다
    };

    // 이 통제기의 sysid (보내는 명령의 sysid, 받은 COMMAND_ACK 의 대상 확인)
    void setSysid(std::uint8_t sysid) { sysid_ = sysid; }

    // mavlink_gateway 연결, 복구: 처음부터 다시 묻는다
    void begin(std::int64_t nowNs);

    // 지금 보낼 요청. 보낼 때가 아니면 nullopt. 돌려준 요청은 보낸 것으로 센다
    std::optional<mavlink_message_t> sendDue(std::int64_t nowNs);

    // 마지막으로 보낸 뒤 재전송 간격이 지나도록 다 모이지 않았고 더 보낼 수 없으면 NoReply (한
    // 번만)
    Outcome check(std::int64_t nowNs);

    // mavlink_gateway 의 COMMAND_ACK (MAV_CMD_REQUEST_MESSAGE). 다른 메시지는 무시한다.
    // 수락, 진행 중이 아니면 Denied 를 돌려주고 그만 묻는다
    Outcome onCommandAck(const mavlink_message_t& message);

    // mavlink_gateway 의 VIDEO_STREAM_INFORMATION. 다른 메시지는 무시한다. count 개가 다 모이면
    // 열 수 있는 것만 센서 순으로 돌려준다 (같은 센서는 stream_id 가 작은 것 하나)
    std::optional<VideoStreams> onStreamInformation(const mavlink_message_t& message);

    bool waiting() const { return waiting_; }
    std::int32_t sends() const { return sends_; }

private:
    mavlink_message_t requestMessage() const;
    VideoStreams collect() const;

    std::uint8_t sysid_{kGcsSysidDefault};
    bool waiting_{false};
    std::int32_t sends_{0};
    std::int64_t lastSendNs_{0};
    std::map<std::uint8_t, std::optional<VideoStreamUri>> received_;  // stream_id → 스트림
};

}  // namespace argus::gcs::link

Q_DECLARE_METATYPE(argus::gcs::link::VideoStreams)

#endif  // ARGUS_GCS_LINK_VIDEO_STREAM_REQUEST_HPP_
