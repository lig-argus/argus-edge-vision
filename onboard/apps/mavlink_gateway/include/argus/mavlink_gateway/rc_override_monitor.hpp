// 조종사(RC) 개입 상태의 보관과 판정의 실행
// 규약: docs/spec/icd.md 5.4, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kRcOverrideCheckPeriod 주기 작업으로
//      등록하고, 조회하는 쪽에는 IRcOverrideState, EndRcOverrideHandler 에는 IRcOverrideReturn 으로
//      전달
#ifndef ARGUS_ONBOARD_RC_OVERRIDE_MONITOR_HPP_
#define ARGUS_ONBOARD_RC_OVERRIDE_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/fc_rc_status_source.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/forwarded_command_events.hpp"
#include "argus/mavlink_gateway/rc_override_policy.hpp"
#include "argus/mavlink_gateway/rc_override_return.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// tick() 의 호출 주기. RC_CHANNELS(20 Hz)를 받는 주기와 같으며 스틱 중립 유지 시간의 측정 간격이다
inline constexpr std::chrono::milliseconds kRcOverrideCheckPeriod{50};

// 입력. 모두 RcOverrideMonitor 보다 오래 유지되어야 한다
struct RcOverrideMonitorInputs {
    const IFcStateSource& fc_state;               // FC 모드, armed
    const IFcRcStatusSource& rc_status;           // 스틱, RC 수신기, 추정 상태
    IForwardedCommandEvents& forwarded_commands;  // 통제기의 DO_SET_MODE 전달
    const core::IClock& clock;
    const ICommandActivity& commands;                       // 진행 중인 명령
    const vehicle_api::IFollowStatusSource& follow_status;  // 추종, 스캔 중
    const IFcCommandStatus& fc_command_status;              // Offboard 진입 중
};

// 판단은 rc_override_policy 가 맡고, 이 클래스는 입력을 모아 개입 상태와 스틱 중립 시작 시각을
// 보관한다.
// - FC 모드가 바뀔 때 is_pilot_takeover() 이면 개입으로 판정한다
// - 비행 상태의 armed 가 false 이면 개입을 끝낸다 (착륙 잠금과 같은 기준, icd 5.3)
// - 스틱 4축이 중립인 동안 중립이 시작된 시각을 보관한다
class RcOverrideMonitor : public IRcOverrideState, public IRcOverrideReturn {
public:
    explicit RcOverrideMonitor(const RcOverrideMonitorInputs& inputs, RcOverrideConfig config = {});

    RcOverrideMonitor(const RcOverrideMonitor&) = delete;
    RcOverrideMonitor& operator=(const RcOverrideMonitor&) = delete;
    RcOverrideMonitor(RcOverrideMonitor&&) = delete;
    RcOverrideMonitor& operator=(RcOverrideMonitor&&) = delete;
    ~RcOverrideMonitor() override = default;

    bool pilot_in_control() const override { return pilot_in_control_; }

    ReturnCheck check_return() const override;
    bool return_still_allowed() const override;
    void end_override() override { pilot_in_control_ = false; }

    // 개입 판정과 스틱 중립 측정. kRcOverrideCheckPeriod 마다 호출한다
    void tick();

private:
    void on_forwarded_command(const ForwardedCommandEvent& event);
    // 지금 fc_epoch 의 가장 최근 비행 상태
    std::optional<vehicle_api::FlightStatus> current_status() const;
    std::optional<FcSensorHealth> current_sensor_health() const;
    ReturnObservation observe_return() const;

    RcOverrideMonitorInputs inputs_;
    RcOverrideConfig config_;
    bool pilot_in_control_ = false;
    std::optional<std::int64_t> neutral_since_ns_;

    // 마지막으로 본 FC 모드와 그 fc_epoch. 모드 변경의 판단에 쓴다
    std::optional<vehicle_api::FlightMode> last_mode_;
    std::uint64_t last_mode_epoch_ = 0;

    std::optional<GcsModeCommand> last_gcs_mode_command_;

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    core::Subscription forwarded_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_RC_OVERRIDE_MONITOR_HPP_
