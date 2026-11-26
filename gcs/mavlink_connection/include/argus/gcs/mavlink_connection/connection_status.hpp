// MAVLink 연결의 상태 조회 인터페이스 (화면용). 메시지 송수신(IMavlinkConnection)과 나눠, 명령과
// 상태 해석 코드가 통계와 상대 주소에 의존하지 않게 한다.
// 구현: net::UdpMavlinkConnection
// 사용: 통신 통계 화면, 연결 상태 표시
#ifndef ARGUS_GCS_MAVLINK_CONNECTION_CONNECTION_STATUS_HPP_
#define ARGUS_GCS_MAVLINK_CONNECTION_CONNECTION_STATUS_HPP_

#include <optional>

#include "argus/gcs/mavlink_connection/endpoint.hpp"
#include "argus/gcs/mavlink_connection/link_statistics.hpp"

namespace argus::gcs::mavlink_connection {

class IMavlinkConnectionStatus {
public:
    IMavlinkConnectionStatus() = default;
    virtual ~IMavlinkConnectionStatus() = default;
    IMavlinkConnectionStatus(const IMavlinkConnectionStatus&) = delete;
    IMavlinkConnectionStatus& operator=(const IMavlinkConnectionStatus&) = delete;
    IMavlinkConnectionStatus(IMavlinkConnectionStatus&&) = delete;
    IMavlinkConnectionStatus& operator=(IMavlinkConnectionStatus&&) = delete;

    // 지금 주고받는 상대. 자동 탐색 중이거나 연결 대상이 없으면 없음
    virtual std::optional<Endpoint> peer() const = 0;

    // 연결을 연 뒤 누적한 수신 통계
    virtual const LinkStatistics& statistics() const = 0;
};

}  // namespace argus::gcs::mavlink_connection

#endif  // ARGUS_GCS_MAVLINK_CONNECTION_CONNECTION_STATUS_HPP_
