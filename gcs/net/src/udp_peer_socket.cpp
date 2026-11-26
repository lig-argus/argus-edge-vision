#include "argus/gcs/net/udp_peer_socket.hpp"

#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <utility>

namespace argus::gcs::net {

UdpPeerSocket::UdpPeerSocket(const core::IClock& clock, QObject* parent)
    : QObject(parent), clock_(clock) {
    connect(&socket_, &QUdpSocket::readyRead, this, &UdpPeerSocket::readPending);
}

bool UdpPeerSocket::open(QString& error) {
    if (!socket_.bind(QHostAddress::AnyIPv4, 0)) {
        error = socket_.errorString();
        return false;
    }
    return true;
}

void UdpPeerSocket::setTarget(const mavlink_connection::Endpoint& target) {
    discovery_ = false;
    setPeer(QHostAddress(target.address), target.port);
}

void UdpPeerSocket::setDiscovery(std::uint16_t port) {
    discovery_ = true;
    discoveryPort_ = port;
    setPeer(std::nullopt, 0);
}

std::optional<mavlink_connection::Endpoint> UdpPeerSocket::peer() const {
    if (!peerAddress_) {
        return std::nullopt;
    }
    return mavlink_connection::Endpoint{peerAddress_->toString(), peerPort_};
}

bool UdpPeerSocket::send(const QByteArray& bytes) {
    if (peerAddress_) {
        return socket_.writeDatagram(bytes, *peerAddress_, peerPort_) == bytes.size();
    }
    if (discovery_) {
        return broadcast(bytes);
    }
    return false;
}

void UdpPeerSocket::checkPeerTimeout() {
    if (discovery_ && peerAddress_ && clock_.nowMonotonicNs() - lastReceivedNs_ > kPeerTimeoutNs) {
        setPeer(std::nullopt, 0);
    }
}

bool UdpPeerSocket::broadcast(const QByteArray& bytes) {
    bool sent = false;
    const auto interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface& interface : interfaces) {
        const auto flags = interface.flags();
        if (!flags.testFlag(QNetworkInterface::IsUp) ||
            !flags.testFlag(QNetworkInterface::IsRunning) ||
            !flags.testFlag(QNetworkInterface::CanBroadcast) ||
            flags.testFlag(QNetworkInterface::IsLoopBack)) {
            continue;
        }
        const auto entries = interface.addressEntries();
        for (const QNetworkAddressEntry& entry : entries) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol &&
                !entry.broadcast().isNull()) {
                sent = socket_.writeDatagram(bytes, entry.broadcast(), discoveryPort_) ==
                           bytes.size() ||
                       sent;
            }
        }
    }
    return sent;
}

void UdpPeerSocket::readPending() {
    while (socket_.hasPendingDatagrams()) {
        const QNetworkDatagram datagram = socket_.receiveDatagram();
        // IPv4 에 바인딩했으므로 송신 주소를 IPv4 로 맞춰 상대 주소와 비교한다
        const QHostAddress sender(datagram.senderAddress().toIPv4Address());
        const auto senderPort = static_cast<std::uint16_t>(datagram.senderPort());
        if (!peerAddress_) {
            if (!discovery_) {
                continue;
            }
            setPeer(sender, senderPort);
        } else if (sender != *peerAddress_ || senderPort != peerPort_) {
            continue;
        }
        lastReceivedNs_ = clock_.nowMonotonicNs();
        emit datagramReceived(datagram.data());
    }
}

void UdpPeerSocket::setPeer(std::optional<QHostAddress> address, std::uint16_t port) {
    peerAddress_ = std::move(address);
    peerPort_ = port;
    lastReceivedNs_ = clock_.nowMonotonicNs();
}

}  // namespace argus::gcs::net
