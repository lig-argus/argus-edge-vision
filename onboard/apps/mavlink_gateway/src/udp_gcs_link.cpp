#include "argus/mavlink_gateway/udp_gcs_link.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "log.hpp"
#include "udp_socket.hpp"

namespace argus::mavlink_gateway {
namespace {

// datagram 하나의 수신 버퍼 크기. MAVLink 2 메시지의 최대 길이(280 바이트)보다 크게 둔다
constexpr std::size_t kReceiveBufferSize = 2048;

std::string ip_text(const in_addr& address) {
    std::array<char, INET_ADDRSTRLEN> text{};
    if (inet_ntop(AF_INET, &address, text.data(), text.size()) == nullptr) {
        return {};
    }
    return text.data();
}

}  // namespace

std::unique_ptr<UdpGcsLink> UdpGcsLink::create(core::IFdWatcher& watcher,
                                               const UdpGcsConfig& config) {
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
    std::unique_ptr<UdpGcsLink> link(new UdpGcsLink(std::move(fd), port.value_or(0)));
    UdpGcsLink* const raw = link.get();
    // registration_ 은 link 의 멤버이므로 on_readable 이 호출되는 동안 raw 는 유효하다
    link->registration_ = watcher.add_fd(raw_fd, [raw] { raw->on_readable(); });
    if (!link->registration_.active()) {
        detail::log_error("통제기 UDP 소켓의 watcher 등록 실패");
        return nullptr;
    }
    return link;
}

UdpGcsLink::UdpGcsLink(UniqueFd fd, std::uint16_t local_port)
    : fd_(std::move(fd)), local_port_(local_port) {}

bool UdpGcsLink::broadcast(const mavlink_message_t& message) {
    bool any = false;
    bool all_sent = true;
    for (const auto& [key, remote] : remotes_) {
        if (!remote.received_message) {
            continue;
        }
        any = true;
        all_sent = send_to_address(remote.address, message) && all_sent;
    }
    return any && all_sent;
}

bool UdpGcsLink::send_to(const GcsPeer& peer, const mavlink_message_t& message) {
    const std::optional<sockaddr_in> address = detail::to_socket_address(peer.address, peer.port);
    return address.has_value() && send_to_address(address.value_or(sockaddr_in{}), message);
}

core::Subscription UdpGcsLink::subscribe(std::uint32_t msgid, Handler handler) {
    return handlers_.add(msgid, std::move(handler));
}

std::optional<std::string> UdpGcsLink::local_address(const GcsPeer& peer) const {
    const std::optional<sockaddr_in> remote = detail::to_socket_address(peer.address, peer.port);
    if (!remote.has_value()) {
        return std::nullopt;
    }
    // UDP 소켓의 connect 는 패킷을 보내지 않고 경로만 정한다. 운영체제가 고른 송신 주소를
    // getsockname 으로 읽는다. bind 주소가 0.0.0.0 이면 수신 소켓으로는 이 값을 알 수 없다
    const UniqueFd probe(socket(AF_INET, SOCK_DGRAM, 0));
    const sockaddr_in target = remote.value_or(sockaddr_in{});
    if (!probe.valid() ||
        connect(probe.get(), reinterpret_cast<const sockaddr*>(&target), sizeof(target)) < 0) {
        return std::nullopt;
    }
    sockaddr_in local{};
    socklen_t local_size = sizeof(local);
    if (getsockname(probe.get(), reinterpret_cast<sockaddr*>(&local), &local_size) < 0) {
        return std::nullopt;
    }
    std::string text = ip_text(local.sin_addr);
    if (text.empty()) {
        return std::nullopt;
    }
    return text;
}

void UdpGcsLink::on_readable() {
    std::array<std::uint8_t, kReceiveBufferSize> buffer{};
    for (std::int32_t count = 0; count < kMaxDatagramsPerEvent; ++count) {
        sockaddr_in sender{};
        socklen_t sender_size = sizeof(sender);
        auto* const sender_address = reinterpret_cast<sockaddr*>(&sender);
        const ssize_t received =
            recvfrom(fd_.get(), buffer.data(), buffer.size(), 0, sender_address, &sender_size);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                detail::log_error("통제기 UDP 수신 실패: " + detail::errno_text());
            }
            return;
        }
        const RemoteKey key{sender.sin_addr.s_addr, sender.sin_port};
        auto found = remotes_.find(key);
        if (found == remotes_.end()) {
            if (remotes_.size() >= kMaxPeers) {
                continue;
            }
            found = remotes_.emplace(key, Remote{}).first;
            found->second.address = sender;
        }
        Remote& remote = found->second;
        const std::vector<mavlink_message_t> messages =
            assemble(remote, buffer.data(), static_cast<std::size_t>(received));
        if (messages.empty()) {
            continue;
        }
        remote.received_message = true;
        GcsPeer peer;
        peer.address = ip_text(sender.sin_addr);
        peer.port = ntohs(sender.sin_port);
        for (const mavlink_message_t& message : messages) {
            peer.sysid = message.sysid;
            peer.compid = message.compid;
            handlers_.dispatch(message, peer);
        }
    }
}

std::vector<mavlink_message_t> UdpGcsLink::assemble(Remote& remote, const std::uint8_t* data,
                                                    std::size_t size) {
    std::vector<mavlink_message_t> messages;
    for (std::size_t index = 0; index < size; ++index) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        const std::uint8_t result = mavlink_frame_char_buffer(
            &remote.parse_buffer, &remote.parse_status, data[index], &message, &status);
        if (result == MAVLINK_FRAMING_OK) {
            messages.push_back(message);
        } else if (result == MAVLINK_FRAMING_BAD_CRC || result == MAVLINK_FRAMING_BAD_SIGNATURE) {
            ++discarded_count_;
        }
    }
    return messages;
}

bool UdpGcsLink::send_to_address(const sockaddr_in& address,
                                 const mavlink_message_t& message) const {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    const auto* const target = reinterpret_cast<const sockaddr*>(&address);
    const ssize_t sent = sendto(fd_.get(), buffer.data(), size, 0, target, sizeof(address));
    return sent == static_cast<ssize_t>(size);
}

}  // namespace argus::mavlink_gateway
