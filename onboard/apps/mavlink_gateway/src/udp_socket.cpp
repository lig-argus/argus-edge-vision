#include "udp_socket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "log.hpp"

namespace argus::mavlink_gateway::detail {

std::string errno_text() { return std::strerror(errno); }

std::optional<sockaddr_in> to_socket_address(const std::string& address, std::uint16_t port) {
    sockaddr_in socket_address{};
    socket_address.sin_family = AF_INET;
    socket_address.sin_port = htons(port);
    if (inet_pton(AF_INET, address.c_str(), &socket_address.sin_addr) != 1) {
        log_error("UDP 주소 변환 실패: " + address);
        return std::nullopt;
    }
    return socket_address;
}

UniqueFd open_bound_udp_socket(const sockaddr_in& address) {
    UniqueFd fd(socket(AF_INET, SOCK_DGRAM, 0));
    if (!fd.valid()) {
        log_error("UDP 소켓 생성 실패: " + errno_text());
        return UniqueFd();
    }
    const std::int32_t flags = fcntl(fd.get(), F_GETFL, 0);
    // 비트 연산은 부호 없는 타입으로 한다
    const auto nonblocking_flags = static_cast<std::int32_t>(
        static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(O_NONBLOCK));
    if (flags < 0 || fcntl(fd.get(), F_SETFL, nonblocking_flags) < 0) {
        log_error("UDP 소켓 non-blocking 설정 실패: " + errno_text());
        return UniqueFd();
    }
    if (bind(fd.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        log_error("UDP bind 실패: " + errno_text());
        return UniqueFd();
    }
    return fd;
}

std::optional<std::uint16_t> bound_port(std::int32_t fd) {
    sockaddr_in bound{};
    socklen_t bound_size = sizeof(bound);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &bound_size) < 0) {
        log_error("UDP 포트 확인 실패: " + errno_text());
        return std::nullopt;
    }
    return ntohs(bound.sin_port);
}

bool same_address(const sockaddr_in& lhs, const sockaddr_in& rhs) {
    return lhs.sin_addr.s_addr == rhs.sin_addr.s_addr && lhs.sin_port == rhs.sin_port;
}

}  // namespace argus::mavlink_gateway::detail
