// 수신 바이트를 MAVLink 메시지로 조립하고 송신자별 수신 통계를 센다. 소켓과 무관하다.
// MAVLink C 라이브러리의 채널 단위 손실 수는 송신자가 섞이면 틀리므로
// 송신자(sysid, compid)마다 센다.
// 사용: UdpMavlinkConnection
#ifndef ARGUS_GCS_NET_MAVLINK_STREAM_PARSER_HPP_
#define ARGUS_GCS_NET_MAVLINK_STREAM_PARSER_HPP_

#include <QByteArray>
#include <array>
#include <cstdint>
#include <functional>
#include <map>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/mavlink_connection/link_statistics.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::net {

class MavlinkStreamParser {
public:
    using MessageHandler = std::function<void(const mavlink_message_t&)>;

    explicit MavlinkStreamParser(const core::IClock& clock) : clock_(clock) {}

    // 수신 데이터 하나를 해석한다. 메시지가 여러 개 들어 있어도 되고, 메시지가 잘려 있으면 다음
    // 데이터와 결합한다. 형식이 맞는 메시지마다 handler 를 호출한다
    void feed(const QByteArray& bytes, const MessageHandler& handler);

    const mavlink_connection::LinkStatistics& statistics() const { return statistics_; }

private:
    // 송신자별 seq 추적 상태. 통계 값에는 넣지 않는다
    struct SeqTracker {
        // 최근 한 바퀴(256) 안에 받은 seq. 뒤로 간 seq 가 중복인지 순서 바뀜인지 구분한다
        std::array<bool, 256> seen{};

        // 뒤로 간 패킷이 연속으로 도착한 수와 그동안 센 값 (송신자 재시작 판단과 되돌리기)
        std::int32_t backwardRun{0};
        std::uint64_t runReordered{0};
        std::uint64_t runDuplicated{0};
        std::uint64_t runLostRestored{0};
    };

    void countSender(const mavlink_message_t& message);
    static void restartSequence(mavlink_connection::SenderStatistics& sender, SeqTracker& tracker,
                                std::uint8_t seq);
    void recordLoss(mavlink_connection::SenderStatistics& sender, std::uint8_t seq,
                    std::uint32_t msgid, std::int32_t missing) const;
    static void recordLate(mavlink_connection::SenderStatistics& sender, std::uint8_t seq);

    const core::IClock& clock_;
    mavlink_message_t partial_{};  // 조립 중인 메시지
    mavlink_status_t status_{};
    mavlink_connection::LinkStatistics statistics_;
    std::map<std::uint16_t, SeqTracker> trackers_;  // 키: senderKey(sysid, compid)
};

}  // namespace argus::gcs::net

#endif  // ARGUS_GCS_NET_MAVLINK_STREAM_PARSER_HPP_
