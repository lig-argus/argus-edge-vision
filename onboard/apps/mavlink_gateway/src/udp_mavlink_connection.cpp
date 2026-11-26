#include "argus/mavlink_gateway/udp_mavlink_connection.hpp"

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "log.hpp"
#include "udp_socket.hpp"

namespace argus::mavlink_gateway {
namespace {

// datagram 하나의 수신 버퍼 크기. MAVLink 2 메시지의 최대 길이(280 바이트)보다 크게 둔다
constexpr std::size_t kReceiveBufferSize = 2048;

}  // namespace

std::unique_ptr<UdpMavlinkConnection> UdpMavlinkConnection::create(core::IFdWatcher& watcher,
                                                                   const UdpMavlinkConfig& config) {
    const std::optional<sockaddr_in> address =
        detail::to_socket_address(config.local_address, config.local_port);
    if (!address.has_value()) {
        return nullptr;
    }
    UniqueFd fd = detail::open_bound_udp_socket(address.value_or(sockaddr_in{}));
    if (!fd.valid()) {
        return nullptr;
    }
    const std::optional<std::uint16_t> port = detail::bound_port(fd.get());
    if (!port.has_value()) {
        return nullptr;
    }
    const std::int32_t raw_fd = fd.get();
    // 생성자가 private 이므로 make_unique 를 사용할 수 없다
    std::unique_ptr<UdpMavlinkConnection> connection(
        new UdpMavlinkConnection(std::move(fd), port.value_or(0)));
    UdpMavlinkConnection* const raw = connection.get();
    // registration_ 은 connection 의 멤버이므로 on_readable 이 호출되는 동안 raw 는 유효하다
    connection->registration_ = watcher.add_fd(raw_fd, [raw] { raw->on_readable(); });
    if (!connection->registration_.active()) {
        detail::log_error("UDP 소켓의 watcher 등록 실패");
        return nullptr;
    }
    return connection;
}

UdpMavlinkConnection::UdpMavlinkConnection(UniqueFd fd, std::uint16_t local_port)
    : fd_(std::move(fd)), local_port_(local_port) {}

bool UdpMavlinkConnection::send(const mavlink_message_t& message) {
    if (!remote_known_) {
        return false;
    }
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    const auto* const remote = reinterpret_cast<const sockaddr*>(&remote_);
    const ssize_t sent = sendto(fd_.get(), buffer.data(), size, 0, remote, sizeof(remote_));
    return sent == static_cast<ssize_t>(size);
}

core::Subscription UdpMavlinkConnection::subscribe(std::uint32_t msgid, Handler handler) {
    return handlers_.add(msgid, std::move(handler));
}

void UdpMavlinkConnection::on_readable() {
    std::array<std::uint8_t, kReceiveBufferSize> buffer{};
    for (std::int32_t count = 0; count < kMaxDatagramsPerEvent; ++count) {
        sockaddr_in sender{};
        socklen_t sender_size = sizeof(sender);
        auto* const sender_address = reinterpret_cast<sockaddr*>(&sender);
        const ssize_t received =
            recvfrom(fd_.get(), buffer.data(), buffer.size(), 0, sender_address, &sender_size);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                detail::log_error("UDP 수신 실패: " + detail::errno_text());
            }
            return;
        }
        if (!detail::same_address(sender, assembling_from_)) {
            parse_buffer_ = {};
            parse_status_ = {};
            assembling_from_ = sender;
        }
        const std::vector<mavlink_message_t> messages =
            assemble(buffer.data(), static_cast<std::size_t>(received));
        if (messages.empty()) {
            continue;
        }
        // handler 가 응답을 송신할 수 있도록 handler 호출 전에 상대 주소를 기록한다
        remote_ = sender;
        remote_known_ = true;
        for (const mavlink_message_t& message : messages) {
            handlers_.dispatch(message);
        }
    }
}

std::vector<mavlink_message_t> UdpMavlinkConnection::assemble(const std::uint8_t* data,
                                                              std::size_t size) {
    std::vector<mavlink_message_t> messages;
    for (std::size_t index = 0; index < size; ++index) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        const std::uint8_t result = mavlink_frame_char_buffer(&parse_buffer_, &parse_status_,
                                                              data[index], &message, &status);
        if (result == MAVLINK_FRAMING_OK) {
            messages.push_back(message);
        } else if (result == MAVLINK_FRAMING_BAD_CRC || result == MAVLINK_FRAMING_BAD_SIGNATURE) {
            ++discarded_count_;
        }
    }
    return messages;
}

}  // namespace argus::mavlink_gateway
