// ARGUS_STATUS.health 비트의 현재 값
// 규약: docs/spec/icd.md 3.1, common/mavlink/message_definitions/argus.xml (ARGUS_HEALTH_FLAGS)
// 사용: 연결 상태를 판단하는 클래스가 비트를 설정하고, ARGUS_STATUS 를 송신하는 클래스가 값을
// 읽는다
#ifndef ARGUS_ONBOARD_HEALTH_STATE_HPP_
#define ARGUS_ONBOARD_HEALTH_STATE_HPP_

#include <cstdint>
#include <functional>
#include <utility>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/handler_list.hpp"

namespace argus::onboard {

// 비트가 1 인 경우 정상이다. 처음 값은 모든 비트 0 (확인된 연결 없음)
class HealthState {
public:
    using ChangeHandler = std::function<void()>;

    // flag 의 비트를 설정한다. 값이 실제로 바뀐 경우에만 등록된 handler 를 호출한다
    void set(ARGUS_HEALTH_FLAGS flag, bool healthy) {
        const auto bit = static_cast<std::uint8_t>(flag);
        const auto updated =
            healthy ? static_cast<std::uint8_t>(value_ | bit)
                    : static_cast<std::uint8_t>(value_ & static_cast<std::uint8_t>(~bit));
        if (updated == value_) {
            return;
        }
        value_ = updated;
        change_handlers_.call();
    }

    bool is_set(ARGUS_HEALTH_FLAGS flag) const {
        return (value_ & static_cast<std::uint8_t>(flag)) != 0;
    }

    // ARGUS_STATUS.health 에 넣는 값
    std::uint8_t value() const { return value_; }

    // 값이 바뀔 때마다 handler 를 호출한다 (health 가 바뀐 경우의 즉시 송신에 사용). handler 는
    // value() 로 현재 값을 읽는다. 값을 인자로 전달하지 않는 이유: handler 가 set() 을 호출하여
    // 값이 다시 바뀐 경우에도 뒤에 호출되는 handler 가 현재 값을 읽도록 함. handler 는 set() 을
    // 호출한 코드 안에서 호출된다. 반환된 Subscription 이 소멸하면 등록이 해제된다
    [[nodiscard]] core::Subscription subscribe_changes(ChangeHandler handler) {
        return change_handlers_.add(std::move(handler));
    }

private:
    std::uint8_t value_ = 0;
    HandlerList<> change_handlers_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_HEALTH_STATE_HPP_
