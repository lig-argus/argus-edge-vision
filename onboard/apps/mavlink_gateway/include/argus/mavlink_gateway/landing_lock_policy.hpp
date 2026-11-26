// 착륙 잠금의 시작과 해제 판단. 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를 모른다
// 규약: docs/spec/icd.md 5.3
// 사용: LandingLock 이 사건마다 입력을 모아 부르고 결과를 반영한다
#ifndef ARGUS_ONBOARD_LANDING_LOCK_POLICY_HPP_
#define ARGUS_ONBOARD_LANDING_LOCK_POLICY_HPP_

#include <cstdint>
#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 착륙 잠금의 상태
struct LandingLockStatus {
    bool active = false;
    std::int64_t started_ns = 0;  // 잠금을 시작한 시각 (마지막 착륙 명령 전달)

    // 잠금 중에 조종사 개입을 본 적이 있음. 그 뒤 개입이 풀리면 복귀 절차를 마친 것이다
    bool pilot_seen = false;
};

// FC 가 착륙 명령에 보낸 결과의 분류
enum class LandAnswer : std::uint8_t {
    InProgress,  // 최종 결과가 아님 (판단에 쓰지 않음)
    Accepted,
    Rejected,  // ACCEPTED, IN_PROGRESS 가 아닌 결과
};

struct LandingLockStep {
    LandingLockStatus status;
    bool started = false;  // 이번 사건으로 잠금이 시작됨. 시작할 때의 실행(FC 제어 해제 등)을 한다
};

// 주기마다 보는 값
struct LandingLockObservation {
    std::optional<vehicle_api::FlightStatus> status;  // 지금 fc_epoch 의 가장 최근 비행 상태
    bool pilot_in_control = false;
};

// 통제기의 착륙 명령을 FC 로 전달함. 잠금이 아니었으면 시작한다. 잠금 중이면 시작 시각만 바꾼다
LandingLockStep on_land_forwarded(const LandingLockStatus& status, std::int64_t now_ns);

// FC 가 착륙 명령에 답함. 잠금 중이고 결과가 Rejected 이면 해제한다
LandingLockStatus on_land_answered(const LandingLockStatus& status, LandAnswer answer);

// 주기 판단. 잠금 중에 다음이면 해제한다
// - 잠금 시작 뒤에 받은 비행 상태의 armed 가 false (지상이고 시동 꺼짐. PX4 는 비행 중 강제가 아닌
//   시동 끔을 거부한다)
// - 조종사 개입을 본 뒤 개입이 풀림 (통제기의 복귀 명령이 성공함)
LandingLockStatus on_landing_lock_tick(const LandingLockStatus& status,
                                       const LandingLockObservation& observation);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_LANDING_LOCK_POLICY_HPP_
