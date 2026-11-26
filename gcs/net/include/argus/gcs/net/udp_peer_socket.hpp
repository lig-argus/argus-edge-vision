// 통제기의 UDP 소켓. 연결 대상(상대)을 정하고 데이터그램을 주고받는다. MAVLink 를 모른다.
//
// 연결 대상
// - 지정: 정해 둔 IP:포트로 송신한다
// - 자동 탐색: 상대가 정해지기 전에는 활성 IPv4 네트워크마다 그 네트워크의 브로드캐스트 주소로
//   송신한다. 최초 응답 주소를 상대로 저장하고 그 뒤로는 그 주소로만 송신한다.
//   255.255.255.255 는 쓰지 않는다 (Windows 는 네트워크가 여러 개면 한 곳으로만 송신한다)
// - 연결 대상을 정하기 전에는 송신하지도 수신하지도 않는다 (운용자가 연결을 고르기 전에 기체와
//   연결되지 않게)
// 자동 탐색으로 정한 상대로부터 kPeerTimeoutNs 동안 수신이 없으면 상대를 폐기하고 재탐색한다
// (핫스팟 재접속으로 RPi 주소가 변경되는 경우).
// 상대가 아닌 주소에서 온 데이터그램은 폐기한다.
#ifndef ARGUS_GCS_NET_UDP_PEER_SOCKET_HPP_
#define ARGUS_GCS_NET_UDP_PEER_SOCKET_HPP_

#include <QByteArray>
#include <QHostAddress>
#include <QObject>
#include <QString>
#include <QUdpSocket>
#include <cstdint>
#include <optional>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/mavlink_connection/endpoint.hpp"

namespace argus::gcs::net {

class UdpPeerSocket : public QObject {
    Q_OBJECT

public:
    // mavlink_gateway 가 통제기와 주고받는 포트 (icd 1.2)
    static constexpr std::uint16_t kDiscoveryPort = 14550;

    static constexpr std::int64_t kPeerTimeoutNs = 5'000'000'000;

    explicit UdpPeerSocket(const core::IClock& clock, QObject* parent = nullptr);

    // 임의의 포트에 바인딩한다. 통제기가 먼저 송신하므로 수신 포트를 정할 필요가 없다
    bool open(QString& error);
    std::uint16_t localPort() const { return socket_.localPort(); }

    void setTarget(const mavlink_connection::Endpoint& target);
    void setDiscovery(std::uint16_t port = kDiscoveryPort);

    // 지금 주고받는 상대. 자동 탐색 중이거나 연결 대상을 정하지 않았으면 없음
    std::optional<mavlink_connection::Endpoint> peer() const;

    // 상대에게 송신한다. 자동 탐색 중이면 브로드캐스트한다. 보낼 곳이 없거나 소켓 오류면 false
    bool send(const QByteArray& bytes);

    // 자동 탐색으로 정한 상대의 수신 시간 초과를 확인한다 (주기 작업)
    void checkPeerTimeout();

signals:
    void datagramReceived(const QByteArray& bytes);

private:
    void readPending();
    bool broadcast(const QByteArray& bytes);
    void setPeer(std::optional<QHostAddress> address, std::uint16_t port);

    const core::IClock& clock_;
    QUdpSocket socket_;
    bool discovery_{false};
    std::uint16_t discoveryPort_{kDiscoveryPort};
    std::optional<QHostAddress> peerAddress_;
    std::uint16_t peerPort_{0};
    std::int64_t lastReceivedNs_{0};
};

}  // namespace argus::gcs::net

#endif  // ARGUS_GCS_NET_UDP_PEER_SOCKET_HPP_
