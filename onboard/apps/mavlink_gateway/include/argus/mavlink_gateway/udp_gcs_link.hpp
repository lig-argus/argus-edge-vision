// UDP 소켓 하나로 통제기 여러 대와 MAVLink 메시지를 주고받는 IGcsLink 구현
// 규약: docs/spec/icd.md 1.2 (UDP 14550), docs/design/uml/class_mavlink_gateway.md 1절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 GcsRegistry, MavlinkCommandReceiver 에 IGcsLink
// 로
//      전달
#ifndef ARGUS_ONBOARD_UDP_GCS_LINK_HPP_
#define ARGUS_ONBOARD_UDP_GCS_LINK_HPP_

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/core/fd_watcher.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_handler_registry.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/mavlink_gateway/unique_fd.hpp"

namespace argus::mavlink_gateway {

struct UdpGcsConfig {
    // mavlink_gateway 가 bind 하는 주소. 통제기는 무선 네트워크로 접속하므로 모든 인터페이스에서
    // 받는다
    std::string local_address = "0.0.0.0";
    std::uint16_t local_port = kGcsUdpPort;  // 0 = 운영체제가 정한 포트 (테스트용)
};

// handler 는 호출 중에 이 객체를 소멸시키면 안 된다 (수신 처리가 이 객체의 멤버를 계속 사용).
// 등록의 추가와 해제는 호출 중에도 할 수 있다
class UdpGcsLink : public IGcsLink {
public:
    // 수신 handler 한 번의 호출에서 읽는 datagram 수의 상한 (UdpMavlinkConnection 과 같은 이유)
    static constexpr std::int32_t kMaxDatagramsPerEvent = 32;

    // 기억하는 송신 주소 수의 상한. 통제기 4대(sysid 250~253)와 QGC, 주소가 바뀐 통제기를 담고도
    // 남는 값이다. 넘으면 새 주소의 datagram 을 버린다 (모르는 송신자가 주소표를 계속 늘리지
    // 못하게)
    static constexpr std::size_t kMaxPeers = 16;

    // 소켓을 열어 config 의 주소에 bind 하고 watcher 에 등록한다. 실패하면 원인을 로그에 남기고
    // nullptr 를 반환한다. watcher 는 반환된 객체보다 먼저 소멸하여도 된다
    static std::unique_ptr<UdpGcsLink> create(core::IFdWatcher& watcher,
                                              const UdpGcsConfig& config);

    bool broadcast(const mavlink_message_t& message) override;
    bool send_to(const GcsPeer& peer, const mavlink_message_t& message) override;
    [[nodiscard]] core::Subscription subscribe(std::uint32_t msgid, Handler handler) override;
    std::optional<std::string> local_address(const GcsPeer& peer) const override;

    // 실제로 bind 된 포트 (local_port 가 0 인 경우 확인용)
    std::uint16_t local_port() const { return local_port_; }

    // CRC 가 맞지 않아 폐기한 메시지 수
    std::uint64_t discarded_count() const { return discarded_count_; }

private:
    // 송신 주소 하나의 메시지 조립 상태. 메시지 하나가 같은 주소의 여러 datagram 에 나뉘어 와도
    // 이어서 조립하며, 주소마다 따로 두므로 통제기 여러 대의 datagram 이 섞여도 서로 끊지 않는다
    struct Remote {
        sockaddr_in address{};
        mavlink_message_t parse_buffer{};
        mavlink_status_t parse_status{};
        bool received_message = false;  // 유효한 메시지를 받은 적이 있는지 (broadcast 대상)
    };

    // (IPv4 주소, 포트). 네트워크 바이트 순서
    using RemoteKey = std::pair<std::uint32_t, std::uint16_t>;

    UdpGcsLink(UniqueFd fd, std::uint16_t local_port);

    void on_readable();
    // remote 의 조립 상태에 data 를 이어서 조립하고, 이 호출에서 완성된 메시지를 반환한다
    std::vector<mavlink_message_t> assemble(Remote& remote, const std::uint8_t* data,
                                            std::size_t size);
    bool send_to_address(const sockaddr_in& address, const mavlink_message_t& message) const;

    UniqueFd fd_;
    std::uint16_t local_port_;
    std::map<RemoteKey, Remote> remotes_;
    std::uint64_t discarded_count_ = 0;
    GcsHandlerRegistry handlers_;

    // fd_ 보다 먼저 소멸하도록 fd_ 뒤에 선언한다 (watcher 등록을 해제한 후 fd 를 닫음)
    core::Subscription registration_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_UDP_GCS_LINK_HPP_
