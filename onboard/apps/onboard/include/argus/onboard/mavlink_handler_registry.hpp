// MAVLink 메시지 번호(msgid)별 handler 의 등록, 해제, 호출
// 사용: IMavlinkConnection 의 구현 (UdpMavlinkConnection, 테스트용 FakeMavlinkConnection).
//      구현들이 같은 호출 규칙을 가지도록 함께 사용한다
#ifndef ARGUS_ONBOARD_MAVLINK_HANDLER_REGISTRY_HPP_
#define ARGUS_ONBOARD_MAVLINK_HANDLER_REGISTRY_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <utility>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/handler_list.hpp"
#include "argus/onboard/mavlink_connection.hpp"

namespace argus::onboard {

class MavlinkHandlerRegistry {
public:
    using Handler = IMavlinkConnection::Handler;

    MavlinkHandlerRegistry() : lists_(std::make_shared<Lists>()) {}

    // msgid 의 handler 를 등록한다. 반환된 Subscription 이 소멸하면 등록이 해제된다.
    // Subscription 은 이 객체보다 늦게 소멸하여도 된다
    [[nodiscard]] core::Subscription add(std::uint32_t msgid, Handler handler) {
        return (*lists_)[msgid].add(std::move(handler));
    }

    // message 의 msgid 에 등록된 handler 를 등록한 순서대로 호출한다. 호출 규칙은 HandlerList::call
    void dispatch(const mavlink_message_t& message) const {
        // handler 가 이 객체를 소멸시키는 경우에도 순회가 유효하도록 목록을 공유한다
        const std::shared_ptr<Lists> lists = lists_;
        const auto found = lists->find(message.msgid);
        if (found != lists->end()) {
            found->second.call(message);
        }
    }

private:
    using Lists = std::map<std::uint32_t, HandlerList<const mavlink_message_t&>>;

    std::shared_ptr<Lists> lists_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_MAVLINK_HANDLER_REGISTRY_HPP_
