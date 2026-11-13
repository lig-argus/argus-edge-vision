// UDP 소켓으로 MAVLink 메시지를 송수신하는 IMavlinkConnection 구현
// 규약: docs/spec/icd.md 1.2 (mavlink-router 의 UDP 엔드포인트 127.0.0.1:14540)
// 사용: onboard 의 main 이 하나를 생성하여 MAVLink 를 처리하는 클래스에 IMavlinkConnection 으로
// 전달
#ifndef ARGUS_ONBOARD_UDP_MAVLINK_CONNECTION_HPP_
#define ARGUS_ONBOARD_UDP_MAVLINK_CONNECTION_HPP_

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "argus/core/fd_watcher.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/mavlink_connection.hpp"
#include "argus/onboard/mavlink_handler_registry.hpp"
#include "argus/onboard/unique_fd.hpp"

namespace argus::onboard {

struct UdpMavlinkConfig {
    // onboard 가 bind 하는 주소. mavlink-router 가 이 주소로 송신한다
    std::string local_address = "127.0.0.1";
    std::uint16_t local_port = 14540;  // 0 = 운영체제가 정한 포트 (테스트용)
};

// handler 는 호출 중에 이 객체를 소멸시키면 안 된다 (수신 처리가 이 객체의 멤버를 계속 사용).
// 등록의 추가와 해제는 호출 중에도 할 수 있다
class UdpMavlinkConnection : public IMavlinkConnection {
public:
    // 수신 handler 한 번의 호출에서 읽는 datagram 수의 상한. 다른 소켓의 handler 와 주기 작업이
    // 지연되지 않도록 제한하며, 남은 datagram 은 watcher 가 handler 를 다시 호출할 때 읽는다
    static constexpr int kMaxDatagramsPerEvent = 32;

    // 소켓을 열어 config 의 주소에 bind 하고 watcher 에 등록한다. 소켓 생성, 주소 변환, bind,
    // watcher 등록 중 하나라도 실패한 경우 원인을 로그에 기록하고 nullptr 를 반환한다.
    // watcher 는 반환된 객체보다 먼저 소멸하여도 된다 (등록 해제는 Subscription 이 처리)
    static std::unique_ptr<UdpMavlinkConnection> create(core::IFdWatcher& watcher,
                                                        const UdpMavlinkConfig& config);

    // 유효한 MAVLink 메시지가 들어 있던 datagram 중 가장 최근 것의 송신 주소(mavlink-router)로
    // 송신한다. false = 그러한 datagram 을 아직 받지 못하여 상대 주소가 없음, 또는 소켓 오류
    bool send(const mavlink_message_t& message) override;

    [[nodiscard]] core::Subscription subscribe(std::uint32_t msgid, Handler handler) override;

    // 실제로 bind 된 포트 (local_port 가 0 인 경우 확인용)
    std::uint16_t local_port() const { return local_port_; }

    // CRC 가 맞지 않아 폐기한 메시지 수. 메시지 정의가 다른 송신자의 메시지도 여기에 포함된다
    std::uint64_t discarded_count() const { return discarded_count_; }

private:
    UdpMavlinkConnection(UniqueFd fd, std::uint16_t local_port);

    void on_readable();
    // data 를 이어서 조립하고, 이 호출에서 완성된 메시지를 반환한다
    std::vector<mavlink_message_t> assemble(const std::uint8_t* data, std::size_t size);

    UniqueFd fd_;
    std::uint16_t local_port_;
    sockaddr_in remote_{};
    bool remote_known_ = false;

    // 메시지 조립 상태. 메시지 하나가 같은 송신 주소의 여러 datagram 에 나뉘어 도착한 경우 이어서
    // 조립한다. 송신 주소가 직전 datagram 과 다른 경우 조립 중이던 바이트를 폐기한다
    mavlink_message_t parse_buffer_{};
    mavlink_status_t parse_status_{};
    std::uint64_t discarded_count_ = 0;
    sockaddr_in assembling_from_{};  // 직전 datagram 의 송신 주소

    MavlinkHandlerRegistry handlers_;

    // fd_ 보다 먼저 소멸하도록 fd_ 뒤에 선언한다 (watcher 등록을 해제한 후 fd 를 닫음)
    core::Subscription registration_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_UDP_MAVLINK_CONNECTION_HPP_
