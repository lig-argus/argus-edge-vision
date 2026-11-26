// mavlink_gateway 가 통제기 여러 대와 MAVLink 메시지를 주고받는 연결 인터페이스
// 규약: docs/spec/icd.md 1.2 (UDP 14550), 5.1, docs/design/uml/class_mavlink_gateway.md 1절
// 구현: UdpGcsLink, 테스트용 FakeGcsLink (mavlink_gateway test/)
// 사용: GcsRegistry (수신, sysid 겹침 경고), MavlinkCommandReceiver (응답 송신)
#ifndef ARGUS_ONBOARD_GCS_LINK_HPP_
#define ARGUS_ONBOARD_GCS_LINK_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"

namespace argus::mavlink_gateway {

// 메시지를 보낸 통제기. sysid, compid 는 메시지의 값, address, port 는 그 메시지를 실은 UDP
// datagram 의 송신 주소다
struct GcsPeer {
    std::uint8_t sysid = 0;
    std::uint8_t compid = 0;
    std::string address;  // IPv4 주소 문자열
    std::uint16_t port = 0;

    // 같은 주소(IP 주소, UDP 포트)에서 보냈는지. sysid, compid 는 비교하지 않는다
    bool same_address(const GcsPeer& other) const {
        return address == other.address && port == other.port;
    }
};

// 통제기 인정(icd 5.1)은 하지 않고 받은 메시지를 모두 넘긴다 (인정은 GcsRegistry)
class IGcsLink {
public:
    using Handler = std::function<void(const mavlink_message_t&, const GcsPeer&)>;

    IGcsLink() = default;
    virtual ~IGcsLink() = default;
    IGcsLink(const IGcsLink&) = delete;
    IGcsLink& operator=(const IGcsLink&) = delete;
    IGcsLink(IGcsLink&&) = delete;
    IGcsLink& operator=(IGcsLink&&) = delete;

    // 유효한 MAVLink 메시지를 받은 적이 있는 모든 주소로 보낸다. 기다리지 않고 바로 반환한다.
    // false = 보낼 주소가 없거나 한 주소라도 소켓 오류로 보내지 못함
    virtual bool broadcast(const mavlink_message_t& message) = 0;

    // peer 의 주소(address, port)로 보낸다. false = 주소 형식 오류 또는 소켓 오류
    virtual bool send_to(const GcsPeer& peer, const mavlink_message_t& message) = 0;

    // msgid 메시지가 오면 보낸 통제기와 함께 handler 를 부른다. 같은 msgid 에 여러 번 등록하면
    // 등록한 순서대로 모두 부른다. handler 는 core::IPoller::process_events() 안에서, 그 함수를
    // 부른 스레드에서 불린다. 반환된 Subscription 이 사라지면 등록이 해제된다
    [[nodiscard]] virtual core::Subscription subscribe(std::uint32_t msgid, Handler handler) = 0;

    // peer 에 보낼 때 사용하는 이 컴퓨터의 IPv4 주소 (통제기가 접속할 RTSP 주소의 호스트,
    // docs/spec/video.md 7장). 정하지 못하면 nullopt
    virtual std::optional<std::string> local_address(const GcsPeer& peer) const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_LINK_HPP_
