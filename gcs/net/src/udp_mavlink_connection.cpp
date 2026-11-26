#include "argus/gcs/net/udp_mavlink_connection.hpp"

#include <algorithm>
#include <array>

namespace argus::gcs::net {

namespace {

QByteArray toBytes(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    QByteArray bytes(static_cast<qsizetype>(size), Qt::Uninitialized);
    std::copy_n(buffer.begin(), size, bytes.begin());
    return bytes;
}

}  // namespace

UdpMavlinkConnection::UdpMavlinkConnection(std::uint8_t sysid, const core::IClock& clock,
                                           QObject* parent)
    : IMavlinkConnection(parent), sysid_(sysid), clock_(clock), socket_(clock), parser_(clock) {
    connect(&socket_, &UdpPeerSocket::datagramReceived, this, [this](const QByteArray& bytes) {
        parser_.feed(bytes,
                     [this](const mavlink_message_t& message) { emit messageReceived(message); });
    });
}

bool UdpMavlinkConnection::send(const mavlink_message_t& message) {
    const mavlink_msg_entry_t* entry = mavlink_get_msg_entry(message.msgid);
    if (entry == nullptr) {
        return false;
    }
    // 메시지를 만든 쪽의 sysid, compid, seq 를 이 연결의 값으로 바꾸고 CRC 를 다시 계산한다.
    // 길이는 최대 길이로 넘기고, 끝의 0 바이트는 finalize 가 다시 잘라 낸다 (MAVLink 2)
    mavlink_message_t out = message;
    mavlink_finalize_message_buffer(&out, sysid_, kCompid, &txStatus_, entry->min_msg_len,
                                    entry->max_msg_len, entry->crc_extra);
    return socket_.send(toBytes(out));
}

void UdpMavlinkConnection::tick() {
    socket_.checkPeerTimeout();
    const std::int64_t now = clock_.nowMonotonicNs();
    if (!lastHeartbeatNs_ || now - *lastHeartbeatNs_ >= kHeartbeatPeriodNs) {
        lastHeartbeatNs_ = now;
        sendHeartbeat();
    }
}

void UdpMavlinkConnection::sendHeartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(sysid_, kCompid, &message, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                               MAV_STATE_ACTIVE);
    send(message);
}

}  // namespace argus::gcs::net
