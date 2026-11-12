#include "argus/onboard/udp_mavlink_connection.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "log.hpp"

namespace argus::onboard {
namespace {

// datagram 하나의 수신 버퍼 크기. MAVLink 2 메시지의 최대 길이(280 바이트)보다 크게 둔다
constexpr std::size_t kReceiveBufferSize = 2048;

std::string errno_text() { return std::strerror(errno); }

// config 의 주소를 소켓 주소로 변환한다. 주소 문자열이 IPv4 주소가 아닌 경우 nullopt
std::optional<sockaddr_in> to_socket_address(const UdpMavlinkConfig& config) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.local_port);
    if (inet_pton(AF_INET, config.local_address.c_str(), &address.sin_addr) != 1) {
        detail::log_error("UDP 주소 변환 실패: " + config.local_address);
        return std::nullopt;
    }
    return address;
}

// non-blocking UDP 소켓을 연다. 실패한 경우 valid() 가 false
UniqueFd open_nonblocking_udp_socket() {
    UniqueFd fd(socket(AF_INET, SOCK_DGRAM, 0));
    if (!fd.valid()) {
        detail::log_error("UDP 소켓 생성 실패: " + errno_text());
        return UniqueFd();
    }
    const int flags = fcntl(fd.get(), F_GETFL, 0);
    // 비트 연산은 부호 없는 타입으로 한다
    const int nonblocking_flags =
        static_cast<int>(static_cast<unsigned int>(flags) | static_cast<unsigned int>(O_NONBLOCK));
    if (flags < 0 || fcntl(fd.get(), F_SETFL, nonblocking_flags) < 0) {
        detail::log_error("UDP 소켓 non-blocking 설정 실패: " + errno_text());
        return UniqueFd();
    }
    return fd;
}

bool bind_socket(int fd, const sockaddr_in& address) {
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        detail::log_error("UDP bind 실패: " + errno_text());
        return false;
    }
    return true;
}

// fd 가 실제로 bind 된 포트. 확인하지 못한 경우 nullopt
std::optional<std::uint16_t> bound_port(int fd) {
    sockaddr_in bound{};
    socklen_t bound_size = sizeof(bound);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &bound_size) < 0) {
        detail::log_error("UDP 포트 확인 실패: " + errno_text());
        return std::nullopt;
    }
    return ntohs(bound.sin_port);
}

bool same_address(const sockaddr_in& lhs, const sockaddr_in& rhs) {
    return lhs.sin_addr.s_addr == rhs.sin_addr.s_addr && lhs.sin_port == rhs.sin_port;
}

}  // namespace

std::unique_ptr<UdpMavlinkConnection> UdpMavlinkConnection::create(core::IFdWatcher& watcher,
                                                                   const UdpMavlinkConfig& config) {
    const std::optional<sockaddr_in> address = to_socket_address(config);
    if (!address.has_value()) {
        return nullptr;
    }
    UniqueFd fd = open_nonblocking_udp_socket();
    if (!fd.valid() || !bind_socket(fd.get(), address.value_or(sockaddr_in{}))) {
        return nullptr;
    }
    const std::optional<std::uint16_t> port = bound_port(fd.get());
    if (!port.has_value()) {
        return nullptr;
    }
    const int raw_fd = fd.get();
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
    for (int count = 0; count < kMaxDatagramsPerEvent; ++count) {
        sockaddr_in sender{};
        socklen_t sender_size = sizeof(sender);
        auto* const sender_address = reinterpret_cast<sockaddr*>(&sender);
        const ssize_t received =
            recvfrom(fd_.get(), buffer.data(), buffer.size(), 0, sender_address, &sender_size);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                detail::log_error("UDP 수신 실패: " + errno_text());
            }
            return;
        }
        if (!same_address(sender, assembling_from_)) {
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

}  // namespace argus::onboard
