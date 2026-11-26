// 통제기와 mavlink_gateway(RPi UDP 14550) 사이의 MAVLink 연결 (icd 1.1, 1.2)
// - 송신: 메시지를 만든 쪽의 sysid, compid, seq 와 무관하게 이 통제기의 sysid(250~253 중 설정값),
//   compid 190, 이 연결의 seq 로 다시 완성해 MAVLink 2 로 보낸다. seq 는 이 연결 하나가 관리한다
// - HEARTBEAT 1 Hz: 통제기가 먼저 송신해야 mavlink_gateway 가 통제기 주소를 인지한다 (icd 1.2)
// - 주기 작업(HEARTBEAT, 상대 수신 시간 초과)은 tick() 에서 한다. app 의 타이머가 호출한다
// 연결 대상은 UdpPeerSocket, 메시지 조립과 수신 통계는 MavlinkStreamParser 가 맡는다.
#ifndef ARGUS_GCS_NET_UDP_MAVLINK_CONNECTION_HPP_
#define ARGUS_GCS_NET_UDP_MAVLINK_CONNECTION_HPP_

#include <QByteArray>
#include <QObject>
#include <QString>
#include <cstdint>
#include <optional>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/mavlink_connection/connection_status.hpp"
#include "argus/gcs/mavlink_connection/endpoint.hpp"
#include "argus/gcs/mavlink_connection/link_statistics.hpp"
#include "argus/gcs/mavlink_connection/mavlink_connection.hpp"
#include "argus/gcs/net/mavlink_stream_parser.hpp"
#include "argus/gcs/net/udp_peer_socket.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::net {

class UdpMavlinkConnection : public mavlink_connection::IMavlinkConnection,
                             public mavlink_connection::IMavlinkConnectionStatus {
    Q_OBJECT

public:
    static constexpr std::uint8_t kCompid = MAV_COMP_ID_MISSIONPLANNER;  // 190 (icd 1.1)
    static constexpr std::int64_t kHeartbeatPeriodNs = 1'000'000'000;

    // sysid: 이 통제기의 MAVLink sysid (icd 1.1 의 250~253 중 하나. 범위 확인은 설정하는 쪽이 한다)
    UdpMavlinkConnection(std::uint8_t sysid, const core::IClock& clock, QObject* parent = nullptr);

    bool open(QString& error) { return socket_.open(error); }
    std::uint16_t localPort() const { return socket_.localPort(); }
    void setTarget(const mavlink_connection::Endpoint& target) { socket_.setTarget(target); }
    void setDiscovery(std::uint16_t port = UdpPeerSocket::kDiscoveryPort) {
        socket_.setDiscovery(port);
    }
    std::uint8_t sysid() const { return sysid_; }

    // IMavlinkConnection. 방언에 없는 메시지, 보낼 곳이 없는 경우, 소켓 오류면 false
    bool send(const mavlink_message_t& message) override;

    // IMavlinkConnectionStatus
    std::optional<mavlink_connection::Endpoint> peer() const override { return socket_.peer(); }
    const mavlink_connection::LinkStatistics& statistics() const override {
        return parser_.statistics();
    }

    // 주기 작업. 100 ms 이하 간격으로 호출한다
    void tick();

private:
    void sendHeartbeat();

    const std::uint8_t sysid_;
    const core::IClock& clock_;
    UdpPeerSocket socket_;
    MavlinkStreamParser parser_;
    mavlink_status_t txStatus_{};  // 송신 seq
    std::optional<std::int64_t> lastHeartbeatNs_;
};

}  // namespace argus::gcs::net

#endif  // ARGUS_GCS_NET_UDP_MAVLINK_CONNECTION_HPP_
