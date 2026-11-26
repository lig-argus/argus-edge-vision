// UDP 소켓을 여는 함수들. UdpMavlinkConnection 과 UdpGcsLink 가 함께 사용한다.
// 실패하면 원인을 로그에 남긴다. 모듈 밖에서는 include 하지 않는다
#ifndef ARGUS_ONBOARD_SRC_UDP_SOCKET_HPP_
#define ARGUS_ONBOARD_SRC_UDP_SOCKET_HPP_

#include <netinet/in.h>

#include <cstdint>
#include <optional>
#include <string>

#include "argus/mavlink_gateway/unique_fd.hpp"

namespace argus::mavlink_gateway::detail {

// address(IPv4 주소 문자열)와 port 를 소켓 주소로 바꾼다. IPv4 주소가 아니면 nullopt
std::optional<sockaddr_in> to_socket_address(const std::string& address, std::uint16_t port);

// non-blocking UDP 소켓을 열어 address 에 bind 한다. 실패하면 valid() 가 false
UniqueFd open_bound_udp_socket(const sockaddr_in& address);

// fd 가 실제로 bind 된 포트. 확인하지 못하면 nullopt
std::optional<std::uint16_t> bound_port(std::int32_t fd);

// 두 소켓 주소의 IP 주소와 포트가 같은지
bool same_address(const sockaddr_in& lhs, const sockaddr_in& rhs);

std::string errno_text();

}  // namespace argus::mavlink_gateway::detail

#endif  // ARGUS_ONBOARD_SRC_UDP_SOCKET_HPP_
