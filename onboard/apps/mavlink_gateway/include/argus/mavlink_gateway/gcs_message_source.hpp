// 통제기로 인정한 송신자의 MAVLink 메시지만 받는 인터페이스
// 규약: docs/spec/icd.md 5.1 (통제기 인정)
// 구현: GcsRegistry
// 사용: ControlAuthority (HEARTBEAT), MavlinkCommandReceiver (COMMAND_LONG, COMMAND_INT)
#ifndef ARGUS_ONBOARD_GCS_MESSAGE_SOURCE_HPP_
#define ARGUS_ONBOARD_GCS_MESSAGE_SOURCE_HPP_

#include <cstdint>

#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"

namespace argus::mavlink_gateway {

class IGcsMessageSource {
public:
    using Handler = IGcsLink::Handler;

    IGcsMessageSource() = default;
    virtual ~IGcsMessageSource() = default;
    IGcsMessageSource(const IGcsMessageSource&) = delete;
    IGcsMessageSource& operator=(const IGcsMessageSource&) = delete;
    IGcsMessageSource(IGcsMessageSource&&) = delete;
    IGcsMessageSource& operator=(IGcsMessageSource&&) = delete;

    // 인정한 통제기가 보낸 msgid 메시지가 오면 handler 를 부른다. 호출 규칙은
    // IGcsLink::subscribe 와 같다. HEARTBEAT 는 구현이 송신자 기록을 갱신한 뒤에 부른다
    [[nodiscard]] virtual core::Subscription subscribe(std::uint32_t msgid, Handler handler) = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_MESSAGE_SOURCE_HPP_
