// onboard 가 FC, 통제기와 MAVLink 메시지를 주고받는 연결 인터페이스
// 규약: docs/spec/icd.md 1장 (mavlink-router 의 UDP 127.0.0.1:14540),
// docs/design/uml/class_onboard.md 1절 구현: (다음 PR) UdpMavlinkConnection, 테스트용
// FakeMavlinkConnection (onboard test/) 사용: FcTelemetryPublisher, TimeSync,
// MavlinkCommandReceiver, GcsLinkMonitor 등 onboard 의 MAVLink 처리 클래스
#ifndef ARGUS_ONBOARD_MAVLINK_CONNECTION_HPP_
#define ARGUS_ONBOARD_MAVLINK_CONNECTION_HPP_

#include <cstdint>
#include <functional>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"

namespace argus::onboard {

// FC 와 통제기의 메시지가 한 UDP 포트로 섞여 온다. 보낸 쪽은 메시지의 sysid, compid 로 구분한다
// (FC = sysid 1, 통제기 = sysid 250, docs/spec/icd.md 1.1).
class IMavlinkConnection {
public:
    using Handler = std::function<void(const mavlink_message_t&)>;

    IMavlinkConnection() = default;
    virtual ~IMavlinkConnection() = default;
    IMavlinkConnection(const IMavlinkConnection&) = delete;
    IMavlinkConnection& operator=(const IMavlinkConnection&) = delete;
    IMavlinkConnection(IMavlinkConnection&&) = delete;
    IMavlinkConnection& operator=(IMavlinkConnection&&) = delete;

    // message 를 보낸다. sysid, compid 는 message 를 만들 때(mavlink_msg_*_pack) 넣는다.
    // 기다리지 않고 바로 반환한다. false = 소켓 오류로 보내지 못함
    virtual bool send(const mavlink_message_t& message) = 0;

    // msgid 메시지가 오면 handler 를 부른다. 같은 msgid 에 여러 번 등록하면 등록한 순서대로 모두
    // 부른다. handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 반환된 Subscription 이 사라지면 등록이 풀린다.
    [[nodiscard]] virtual core::Subscription subscribe(std::uint32_t msgid, Handler handler) = 0;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_MAVLINK_CONNECTION_HPP_
