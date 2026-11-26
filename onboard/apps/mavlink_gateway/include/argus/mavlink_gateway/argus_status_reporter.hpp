// ARGUS_STATUS 송신과 추종 제어 두절 처리
// 규약: docs/spec/icd.md 3.1, 6.4 ⓐ ⓑ, docs/spec/process_interfaces.md 6장,
//      docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kArgusStatusCheckPeriod 주기 작업으로
//      등록
#ifndef ARGUS_ONBOARD_ARGUS_STATUS_REPORTER_HPP_
#define ARGUS_ONBOARD_ARGUS_STATUS_REPORTER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/argus_status_policy.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/cpu_temperature.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/mavlink_gateway/stop_requester.hpp"
#include "argus/vehicle_api/follow_status_events.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// ARGUS_STATUS 의 송신 주기 (icd 3.1)
inline constexpr std::chrono::milliseconds kArgusStatusPeriod{1000};

// tick() 의 호출 주기. 시간이 지나 바뀌는 값(두절 FAULT)을 확인하는 지연이며, 바뀌면 바로 보낸다
inline constexpr std::chrono::milliseconds kArgusStatusCheckPeriod{100};

// 입력. 모두 ArgusStatusReporter 보다 오래 유지되어야 한다
struct ArgusStatusReporterInputs {
    IGcsLink& link;  // 모든 통제기에 broadcast
    const core::IClock& clock;
    const vehicle_api::IFollowStatusSource& follow_status;
    vehicle_api::IFollowStatusEvents& follow_events;  // 받을 때마다 변경 확인
    const IControlAuthority& authority;               // control_sysid
    HealthState& health;                              // 바뀔 때마다 변경 확인
    const IFcCommandStatus& fc_command_status;        // setpoint 유효 시간 경과 정지
    IStopRequester& stop_requester;                   // follow/status 두절 시 정지
    const ICpuTemperature& cpu_temperature;
    const ILandingLockState& landing_lock;  // control_flags
    const IRcOverrideState& rc_override;    // control_flags
};

// - 값은 compose_argus_status() 가 정한다. 이 클래스는 입력을 모으고 송신과 정지만 한다
// - kArgusStatusPeriod 마다, 그리고 mission_state, target_state, health, active_sensor,
//   control_flags 가 바뀌면
//   바로 송신한다
// - follow/status 두절(StatusLost)이 시작될 때 마지막 값이 추종 또는 스캔 중이었으면
//   IStopRequester 로 정지한다. 추종 제어가 멈췄으므로 follow/command 로 요청하지 않는다
// - setpoint 유효 시간 경과 정지의 FAULT 는 setpoint_timeout_cleared() 가 true 가 되면 그 정지를
//   해제된 것으로 기록하고 다시 보고하지 않는다
class ArgusStatusReporter {
public:
    // 생성 시각을 mavlink_gateway 시작 시각(time_boot_ms 의 기준)으로 한다
    explicit ArgusStatusReporter(const ArgusStatusReporterInputs& inputs);

    ArgusStatusReporter(const ArgusStatusReporter&) = delete;
    ArgusStatusReporter& operator=(const ArgusStatusReporter&) = delete;
    ArgusStatusReporter(ArgusStatusReporter&&) = delete;
    ArgusStatusReporter& operator=(ArgusStatusReporter&&) = delete;
    ~ArgusStatusReporter() = default;

    // 두절 판단, 변경 확인, 주기 송신. kArgusStatusCheckPeriod 마다 호출한다
    void tick();

private:
    StatusObservation observe();
    // 값을 구성해 바뀌었으면 보내고, force 이면 바뀌지 않아도 보낸다
    void evaluate(bool force);

    ArgusStatusReporterInputs inputs_;
    std::int64_t started_ns_;
    std::optional<std::int64_t> cleared_timeout_ns_;  // 해제된 setpoint 유효 시간 경과 정지
    FollowFault last_fault_ = FollowFault::None;
    std::optional<mavlink_argus_status_t> last_sent_;
    std::int64_t last_sent_ns_ = 0;

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    core::Subscription follow_subscription_;
    core::Subscription health_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_ARGUS_STATUS_REPORTER_HPP_
