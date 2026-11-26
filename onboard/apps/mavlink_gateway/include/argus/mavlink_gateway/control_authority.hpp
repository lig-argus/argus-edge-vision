// 제어권을 가진 통제기 하나의 보관과 판단의 반영
// 규약: docs/spec/icd.md 4.4, 5.1, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 조회하는 쪽에는 IControlAuthority, 요청을 넘기는
//      RequestOperatorControlHandler 에는 IControlRequests 로 전달
#ifndef ARGUS_ONBOARD_CONTROL_AUTHORITY_HPP_
#define ARGUS_ONBOARD_CONTROL_AUTHORITY_HPP_

#include <cstdint>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority_policy.hpp"
#include "argus/mavlink_gateway/control_requests.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/gcs_message_source.hpp"
#include "argus/mavlink_gateway/gcs_presence.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// 제어권을 가진 통제기를 조회하는 인터페이스. 요청과 해제는 IControlRequests
class IControlAuthority {
public:
    IControlAuthority() = default;
    virtual ~IControlAuthority() = default;
    IControlAuthority(const IControlAuthority&) = delete;
    IControlAuthority& operator=(const IControlAuthority&) = delete;
    IControlAuthority(IControlAuthority&&) = delete;
    IControlAuthority& operator=(IControlAuthority&&) = delete;

    // 제어권을 가진 통제기의 sysid. 없으면 nullopt (ARGUS_STATUS.control_sysid 는 0)
    virtual std::optional<std::uint8_t> controller() const = 0;

    // sysid 통제기가 제어권을 가졌으면 true
    virtual bool is_controller(std::uint8_t sysid) const = 0;
};

// 제어권 판단의 입력. 모두 ControlAuthority 보다 오래 유지되어야 한다
struct ControlAuthorityInputs {
    IGcsMessageSource& messages;   // HEARTBEAT 수신
    const IGcsPresence& presence;  // 제어권을 가진 통제기의 마지막 HEARTBEAT
    const core::IClock& clock;
    const IFcStateSource& fc_state;                         // 비행 중
    const IFcCommandStatus& fc_command_status;              // setpoint 송신 중
    const ICommandActivity& commands;                       // 진행 중인 명령
    const vehicle_api::IFollowStatusSource& follow_status;  // 추종, 스캔 중
    const ILandingLockState& landing_lock;
    const IRcOverrideState& rc_override;
};

// HEARTBEAT 와 요청마다 입력을 모아 decide_control() 을 부르고 결과를 반영한다. 판단은
// control_authority_policy 가 맡는다
class ControlAuthority : public IControlAuthority, public IControlRequests {
public:
    explicit ControlAuthority(const ControlAuthorityInputs& inputs);

    std::optional<std::uint8_t> controller() const override { return controller_; }
    bool is_controller(std::uint8_t sysid) const override { return controller_ == sysid; }
    MAV_RESULT request(const GcsPeer& sender, ControlAction action) override;

private:
    ControlObservation observe() const;
    ControlDecision apply(ControlEventKind kind, std::uint8_t sysid);

    ControlAuthorityInputs inputs_;
    std::optional<std::uint8_t> controller_;
    core::Subscription heartbeat_subscription_;  // 이 객체가 사라지면 해제된다
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_CONTROL_AUTHORITY_HPP_
