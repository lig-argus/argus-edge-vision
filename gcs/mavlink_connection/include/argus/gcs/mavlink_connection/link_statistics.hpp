// MAVLink 수신 통계 값 타입. 통신 통계 화면과 연결 상태 표시가 읽는다.
// 시각은 모두 IClock 의 monotonic ns 이다. 화면에 시각으로 보일 때는 화면 쪽이 변환한다.
#ifndef ARGUS_GCS_MAVLINK_CONNECTION_LINK_STATISTICS_HPP_
#define ARGUS_GCS_MAVLINK_CONNECTION_LINK_STATISTICS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>

namespace argus::gcs::mavlink_connection {

// 송신자 하나가 보낸 메시지 번호 하나의 수신 통계
struct MessageStatistics {
    std::uint64_t received{0};
    std::uint64_t bytes{0};          // 프레임 전체 바이트 (헤더, CRC, 서명 포함)
    std::int64_t lastReceivedNs{0};  // 마지막 수신 시각
};

// 손실 기록 하나: seq 가 건너뛴 자리
struct LossRecord {
    std::int64_t detectedNs{0};    // 건너뛴 뒤 첫 메시지를 받은 시각
    std::uint8_t lastSeq{0};       // 건너뛰기 전에 받은 seq
    std::uint8_t nextSeq{0};       // 건너뛴 뒤 받은 seq
    std::int32_t missing{0};       // lastSeq 와 nextSeq 사이에서 빠진 수
    std::int32_t late{0};          // 그 가운데 나중에 늦게 도착해 손실에서 뺀 수
    std::uint32_t beforeMsgid{0};  // 건너뛰기 전 마지막 메시지 번호
    std::uint32_t afterMsgid{0};   // 건너뛴 뒤 첫 메시지 번호
};

// 송신자(sysid, compid) 하나의 수신 통계
struct SenderStatistics {
    static constexpr std::size_t kMaxLossRecords = 50;  // 송신자마다 보관하는 최근 손실 기록 수

    std::uint64_t received{0};  // 받은 메시지 수 (방언에 없는 메시지 포함)

    // 패킷 손실: seq 가 건너뛴 수. 늦게 도착한 패킷(순서 바뀜)은 다시 뺀다.
    // mavlink_gateway 가 다른 통제기에만 보낸 메시지(COMMAND_ACK 등)도 손실로 잡히므로 링크 품질의
    // 판단에는 쓰지 않는다
    std::uint64_t lost{0};

    std::uint64_t reordered{0};   // 아직 받지 않은 앞 seq 가 늦게 도착한 수
    std::uint64_t duplicated{0};  // 이미 받은 seq 가 다시 도착한 수
    std::uint64_t bytes{0};       // 받은 프레임 바이트
    std::uint8_t lastSeq{0};      // 가장 앞선 seq
    std::uint32_t lastMsgid{0};   // 가장 최근에 받은 메시지 번호
    std::map<std::uint32_t, MessageStatistics> messages{};  // 키: 메시지 번호
    std::deque<LossRecord> losses{};                        // 오래된 것부터, 최대 kMaxLossRecords
};

struct LinkStatistics {
    std::map<std::uint16_t, SenderStatistics> senders;  // 키: senderKey(sysid, compid)
    std::uint64_t badCrc{0};  // 방언에 있는 메시지인데 CRC 가 틀린 수 (형식 오류)

    // 방언에 없는 메시지 수. 오류가 아니다 (모르는 메시지는 버린다)
    std::uint64_t unknown{0};
};

constexpr std::uint16_t senderKey(std::uint8_t sysid, std::uint8_t compid) {
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(sysid) << 8U) |
                                      static_cast<std::uint32_t>(compid));
}

}  // namespace argus::gcs::mavlink_connection

#endif  // ARGUS_GCS_MAVLINK_CONNECTION_LINK_STATISTICS_HPP_
