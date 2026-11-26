// ARGUS_STATUS 의 값 구성과 추종 제어 두절(FAULT) 판단. 입력 구조체만 받고 MAVLink 송수신, 시계,
// bus 를 모른다
// 규약: docs/spec/icd.md 3.1, 6.4 ⓐ ⓑ, 부록 B, docs/spec/process_interfaces.md 6장
// 사용: ArgusStatusReporter 가 입력을 모아 부르고 결과를 송신한다
#ifndef ARGUS_ONBOARD_ARGUS_STATUS_POLICY_HPP_
#define ARGUS_ONBOARD_ARGUS_STATUS_POLICY_HPP_

#include <cstdint>
#include <optional>

#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// FAULT 로 보고하는 원인
enum class FollowFault : std::uint8_t {
    None,
    StatusLost,       // follow/status 를 kFollowStatusTimeoutNs 동안 받지 못함 (icd 6.4 ⓑ)
    SetpointTimeout,  // 추종 중 setpoint 의 유효 시간이 지나 정지함 (icd 6.4 ⓐ)
    UnknownState,     // follow/status 의 mission_state 를 해석하지 못함
};

struct StatusObservation {
    std::int64_t now_ns = 0;
    std::int64_t started_ns = 0;  // mavlink_gateway 시작 시각
    std::optional<vehicle_api::ReceivedFollowStatus> follow;

    // 아직 해제되지 않은 setpoint 유효 시간 경과 정지의 시각. 없으면 nullopt
    std::optional<std::int64_t> setpoint_timeout_ns;

    std::optional<std::uint8_t> controller;
    std::uint8_t health = 0;
    std::optional<double> cpu_celsius;
    bool landing_locked = false;    // 착륙 잠금 (control_flags)
    bool pilot_in_control = false;  // 조종사 개입 중 (control_flags)
};

// - StatusLost: 받은 적이 없으면 시작 후, 받았으면 마지막으로 받은 뒤 kFollowStatusTimeoutNs 가
//   지남. 두절 뒤에 새로 받은 값이 있어야 풀린다 (그 전의 값은 이미 오래됐으므로)
// - SetpointTimeout: setpoint_timeout_ns 가 있고, 그 뒤에 받은 follow/status 가 없거나 그 값이
//   FOLLOW_ACTIVE 또는 SCAN_ACTIVE (추종 제어가 정지를 아직 반영하지 않음)
// - UnknownState: mission_state 가 Unknown
FollowFault follow_fault(const StatusObservation& observation);

// setpoint 유효 시간 경과의 FAULT 를 풀어도 되는지. 그 시각 뒤에 받은 follow/status 가
// FOLLOW_ACTIVE, SCAN_ACTIVE 가 아니면 true (추종 제어가 정지를 반영했거나 새 명령을 시작함)
bool setpoint_timeout_cleared(const StatusObservation& observation);

// ARGUS_STATUS 의 값 (time_boot_ms 포함). FAULT 이면 mission_state 를 FAULT 로 바꾸고,
// StatusLost 이면 표적 값도 비운다
mavlink_argus_status_t compose_argus_status(const StatusObservation& observation);

// 마지막 값이 추종 또는 스캔 중이었는지 (PREPARING, FOLLOW_ACTIVE, HOLD_TRANSITION, SCAN_ACTIVE).
// 받은 시각은 보지 않는다 (두절 판단 뒤 정지 여부에 사용)
bool was_following(const std::optional<vehicle_api::ReceivedFollowStatus>& follow);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_ARGUS_STATUS_POLICY_HPP_
