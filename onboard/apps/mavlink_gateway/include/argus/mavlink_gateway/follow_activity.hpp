// follow/status 로 추종 또는 스캔 중인지 판단
// 규약: docs/spec/icd.md 5.1 (제어권), 6.1 (통제기 두절), 부록 B (임무 상태)
// 사용: ControlAuthority, GcsLinkMonitor
#ifndef ARGUS_ONBOARD_FOLLOW_ACTIVITY_HPP_
#define ARGUS_ONBOARD_FOLLOW_ACTIVITY_HPP_

#include <cstdint>
#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// follow/status 를 이 시간 동안 받지 못하면 추종 제어가 멈춘 것으로 본다 (icd 6.4 ⓑ)
inline constexpr std::int64_t kFollowStatusTimeoutNs = 3'000'000'000;

// latest 를 now_ns 에서 kFollowStatusTimeoutNs 이내에 받았고 임무 상태가 PREPARING, FOLLOW_ACTIVE,
// HOLD_TRANSITION, SCAN_ACTIVE 중 하나이면 true.
// 받지 못한 경우는 false 다. 추종 제어가 멈춘 상태이며, 그 정지와 FAULT 보고는 icd 6.4 ⓑ 의
// 처리가 맡는다. true 로 보면 추종 제어가 시작되기 전에는 제어권을 부여할 수 없다
bool follow_in_progress(const std::optional<vehicle_api::ReceivedFollowStatus>& latest,
                        std::int64_t now_ns);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FOLLOW_ACTIVITY_HPP_
