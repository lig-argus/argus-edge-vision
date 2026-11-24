// 시나리오 테스트가 FcControl 의 상태를 읽는 창구
#ifndef ARGUS_ONBOARD_TEST_FC_COMMAND_RIG_HPP_
#define ARGUS_ONBOARD_TEST_FC_COMMAND_RIG_HPP_

#include <functional>
#include <string>
#include <utility>
#include <variant>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/fc_command_policy.hpp"
#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/mavlink_gateway/fc_control.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

class FcCommandRig {
public:
    FcCommandRig(IMavlinkConnection& connection, const IFcStateSource& fc_state,
                 const core::IClock& clock)
        : control_(connection, fc_state, clock) {}

    vehicle_api::FlightModeReply request_offboard(const vehicle_api::RequestHeader& header) {
        return control_.request_offboard(header);
    }
    vehicle_api::FlightModeReply request_hold(const vehicle_api::RequestHeader& header) {
        return control_.request_hold(header);
    }
    bool stop_from_onboard() { return control_.stop_from_onboard(); }
    void publish(const vehicle_api::VelocitySetpoint& setpoint) { control_.send(setpoint); }
    void tick() { control_.tick(); }
    bool release_lock() { return control_.release_lock(); }
    core::Subscription subscribe_mode_events(
        std::function<void(const vehicle_api::FlightModeEvent&)> handler) {
        return control_.subscribe_mode_events(std::move(handler));
    }

    // 송신권을 가진 세션. 없는 경우 빈 문자열
    std::string owner() const {
        const std::string* session = authority_session(control_.state());
        return session == nullptr ? std::string() : *session;
    }
    bool locked() const { return std::holds_alternative<Locked>(control_.state()); }
    bool holding_position() const { return control_.offboard_hover(); }
    // mavlink_gateway 가 정지 setpoint 를 보내는 상태인지
    bool stopping() const {
        return std::holds_alternative<CancellingEntry>(control_.state()) ||
               std::holds_alternative<Hovering>(control_.state());
    }

private:
    FcControl control_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FC_COMMAND_RIG_HPP_
