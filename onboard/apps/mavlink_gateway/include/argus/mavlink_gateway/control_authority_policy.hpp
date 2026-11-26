// 제어권의 판단 (부여, 요청, 해제). 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를 모른다
// 규약: docs/spec/icd.md 4.4, 5.1, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: ControlAuthority 가 입력을 모아 decide() 를 부르고 결과를 반영한다
#ifndef ARGUS_ONBOARD_CONTROL_AUTHORITY_POLICY_HPP_
#define ARGUS_ONBOARD_CONTROL_AUTHORITY_POLICY_HPP_

#include <cstdint>
#include <optional>

#include "argus/mavlink.h"

namespace argus::mavlink_gateway {

// 제어권을 가진 통제기가 살아 있다고 보는 HEARTBEAT 간격의 상한 (icd 5.1, 6.1)
inline constexpr std::int64_t kGcsHeartbeatTimeoutNs = 3'000'000'000;

struct ControlObservation {
    std::int64_t now_ns = 0;
    std::optional<std::uint8_t> controller;                    // 현재 제어권을 가진 통제기
    std::optional<std::int64_t> controller_last_heartbeat_ns;  // 그 통제기의 마지막 HEARTBEAT
    bool flying = false;                                       // 기체가 비행 중
    bool command_in_progress = false;  // 최종 결과를 보내지 않은 통제기 명령이 있음
    bool follow_in_progress = false;   // 추종 또는 스캔 중
    bool sending_setpoints = false;    // mavlink_gateway 가 FC 로 setpoint 를 보내는 중
    bool landing_locked = false;       // 착륙 잠금 상태
    bool pilot_in_control = false;     // 조종사(RC) 개입 중
};

enum class ControlEventKind : std::uint8_t {
    Heartbeat,  // 인정한 통제기의 HEARTBEAT
    Request,    // 45010 요청
    Release,    // 45010 해제
};

struct ControlEvent {
    ControlEventKind kind = ControlEventKind::Heartbeat;
    std::uint8_t sysid = 0;  // 사건을 일으킨 통제기
};

struct ControlDecision {
    std::optional<std::uint8_t> controller;  // 판단 뒤 제어권을 가진 통제기

    // Request, Release 의 COMMAND_ACK 결과. Heartbeat 에서는 사용하지 않는다
    MAV_RESULT result = MAV_RESULT_ACCEPTED;
};

// - Heartbeat: 제어권을 가진 통제기가 없고, 진행 중인 명령, 추종, 스캔, setpoint 송신이 없으면
//   sysid 에 부여한다
// - Request: sysid 가 이미 가졌으면 ACCEPTED. 거부 조건(제어권을 가진 통제기의 HEARTBEAT 가
//   kGcsHeartbeatTimeoutNs 이내이고 비행 중, 진행 중인 명령, 추종 또는 스캔 중, 착륙 잠금, 조종사
//   개입 중)이면
//   TEMPORARILY_REJECTED. 그 밖에는 sysid 에 부여하고 ACCEPTED
// - Release: sysid 가 가졌으면 제어권을 없애고 ACCEPTED, 아니면 DENIED
ControlDecision decide_control(const ControlObservation& observation, const ControlEvent& event);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_CONTROL_AUTHORITY_POLICY_HPP_
