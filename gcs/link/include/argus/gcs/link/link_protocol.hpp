// 통제기가 쓰는 MAVLink 주소와 명령 절차 값 (icd 1.1, 2장, 4.1, 6장). link 모듈의 부품이 같은 값을
// 쓰도록 한 곳에 둔다
#ifndef ARGUS_GCS_LINK_LINK_PROTOCOL_HPP_
#define ARGUS_GCS_LINK_LINK_PROTOCOL_HPP_

#include <cstdint>

#include "argus/mavlink.h"

namespace argus::gcs::link {

// 통제기 sysid. 통제기마다 250~253 중 다른 값을 쓴다 (icd 1.1)
inline constexpr std::uint8_t kGcsSysidMin = 250;
inline constexpr std::uint8_t kGcsSysidMax = 253;
inline constexpr std::uint8_t kGcsSysidDefault = kGcsSysidMin;
constexpr bool isGcsSysid(std::int32_t sysid) {
    return sysid >= kGcsSysidMin && sysid <= kGcsSysidMax;
}

inline constexpr std::uint8_t kGcsCompid = MAV_COMP_ID_MISSIONPLANNER;  // 190
inline constexpr std::uint8_t kVehicleSysid = 1;
inline constexpr std::uint8_t kFcCompid = MAV_COMP_ID_AUTOPILOT1;        // 1
inline constexpr std::uint8_t kGatewayCompid = MAV_COMP_ID_PATHPLANNER;  // 195, mavlink_gateway

inline constexpr std::int64_t kNsPerMs = 1'000'000;
inline constexpr std::int64_t kNsPerSecond = 1'000'000'000;

// 명령 절차 (icd 4.1)
// 응답이 없으면 1 s 간격으로 다시 보내고, 처음을 포함해 최대 3 번 보낸다
inline constexpr std::int32_t kMaxSends = 3;
inline constexpr std::int64_t kRetryIntervalNs = kNsPerSecond;

// 마지막 IN_PROGRESS 뒤 이 시간 안에 최종 응답이 없으면 응답 없음 (mavlink_gateway 는 1 s 마다
// IN_PROGRESS 를 다시 보낸다)
inline constexpr std::int64_t kInProgressTimeoutNs = 3 * kNsPerSecond;

// 명령 번호(p7)는 24 비트 안에서 1 씩 증가하고 순환한다. float32 로 정확히 보낼 수 있는 범위다
inline constexpr std::uint32_t kCommandNumberModulus = 1U << 24U;

// 링크 판단 (icd 2장, 6장)
// HEARTBEAT 의 기대 주기. 기대 주기의 두 배 이상 메시지가 없으면 지연, 3 s 없으면 두절
inline constexpr std::int64_t kHeartbeatPeriodNs = kNsPerSecond;
inline constexpr std::int64_t kLinkDelayedNs = 2 * kHeartbeatPeriodNs;
inline constexpr std::int64_t kLinkLostNs = 3 * kNsPerSecond;

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_LINK_PROTOCOL_HPP_
