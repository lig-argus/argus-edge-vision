// 테스트용 IForwardedCommandEvents. 테스트가 전달과 FC 의 응답을 직접 알린다
#ifndef ARGUS_ONBOARD_TEST_FAKE_FORWARDED_COMMAND_EVENTS_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_FORWARDED_COMMAND_EVENTS_HPP_

#include <cstdint>
#include <optional>
#include <utility>

#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/forwarded_command_events.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

class FakeForwardedCommandEvents : public IForwardedCommandEvents {
public:
    core::Subscription subscribe(Handler handler) override {
        return handlers_.subscribe(std::move(handler));
    }

    // 테스트용: command 를 FC 로 전달한 것을 알린다
    void simulate_forwarded(std::uint16_t command,
                            std::optional<vehicle_api::FlightMode> mode = std::nullopt) {
        ForwardedCommandEvent event;
        event.command = command;
        event.stage = ForwardStage::Forwarded;
        event.requested_mode = mode;
        handlers_.notify(event);
    }

    // 테스트용: command 에 FC 가 result 로 답한 것을 알린다
    void simulate_answered(std::uint16_t command, MAV_RESULT result) {
        ForwardedCommandEvent event;
        event.command = command;
        event.stage = ForwardStage::Answered;
        event.result = result;
        handlers_.notify(event);
    }

private:
    core::HandlerList<const ForwardedCommandEvent&> handlers_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FAKE_FORWARDED_COMMAND_EVENTS_HPP_
