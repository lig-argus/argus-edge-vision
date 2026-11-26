// 착륙 잠금의 보관과 판단의 실행
// 규약: docs/spec/icd.md 5.3, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kLandingLockCheckPeriod 주기 작업으로
//      등록하고, 조회하는 쪽(ControlAuthority, MavlinkCommandReceiver, GcsFcForwarder,
//      ArgusStatusReporter)에는 ILandingLockState 로 전달
#ifndef ARGUS_ONBOARD_LANDING_LOCK_HPP_
#define ARGUS_ONBOARD_LANDING_LOCK_HPP_

#include <chrono>
#include <functional>

#include "argus/core/clock.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/fc_landing_release.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/forwarded_command_events.hpp"
#include "argus/mavlink_gateway/landing_lock_policy.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// tick() 의 호출 주기. 해제 조건(시동 꺼짐, 복귀 완료)을 알아차리는 지연에만 영향을 준다
inline constexpr std::chrono::milliseconds kLandingLockCheckPeriod{100};

// 잠금을 시작할 때 추종 제어에 보내는 정지 요청의 유효 시간과 응답 대기 시간
inline constexpr std::chrono::milliseconds kLandingStopTimeout{1000};

// 입력. 모두 LandingLock 보다 오래 유지되어야 한다
struct LandingLockInputs {
    IForwardedCommandEvents& forwarded_commands;  // 착륙 명령의 전달과 FC 의 응답
    const IFcStateSource& fc_state;               // armed
    const IRcOverrideState& rc_override;          // 조종사 개입 뒤 복귀 완료
    const core::IClock& clock;
    IFcLandingRelease& landing_release;
    const vehicle_api::IFollowStatusSource& follow_status;  // 추종, 스캔 중
    vehicle_api::IFollowControlClient& follow_control;
    RequestHeaderFactory& request_headers;
};

// 판단은 landing_lock_policy 가 맡고, 이 클래스는 입력을 모아 상태를 보관하고 잠금을 시작할 때의
// 실행을 한다.
// - MAV_CMD_NAV_LAND 의 Forwarded 로 잠금을 시작하고, 시작할 때 IFcLandingRelease 로 FC 제어를
//   내려놓으며, 추종 또는 스캔 중이면 추종 제어에 정지(StopReason::Onboard)를 요청하고,
//   subscribe_started() 의 handler 를 부른다 (진행 중인 통제기 명령의 CANCELLED)
// - MAV_CMD_NAV_LAND 의 Answered 가 최종 결과이고 ACCEPTED 가 아니면 해제한다
// - 시동 꺼짐과 복귀 완료는 tick() 에서 확인한다
class LandingLock : public ILandingLockState {
public:
    explicit LandingLock(const LandingLockInputs& inputs);

    LandingLock(const LandingLock&) = delete;
    LandingLock& operator=(const LandingLock&) = delete;
    LandingLock(LandingLock&&) = delete;
    LandingLock& operator=(LandingLock&&) = delete;
    ~LandingLock() override = default;

    bool active() const override { return status_.active; }

    // 잠금이 시작될 때마다 handler 를 부른다. 반환된 Subscription 이 사라지면 등록이 해제된다
    [[nodiscard]] core::Subscription subscribe_started(std::function<void()> handler);

    // 해제 조건을 확인한다. kLandingLockCheckPeriod 마다 호출한다
    void tick();

private:
    void on_forwarded_command(const ForwardedCommandEvent& event);
    void start();
    void request_follow_stop();

    LandingLockInputs inputs_;
    LandingLockStatus status_;
    core::HandlerList<> started_handlers_;
    core::Subscription stop_request_;  // 응답을 기다리는 정지 요청

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    core::Subscription forwarded_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_LANDING_LOCK_HPP_
