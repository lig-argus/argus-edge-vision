// mavlink_gateway 가 FC 로 보내는 명령의 판단: FC 의 모드 판단, 상태의 전환, 주기마다 보낼
// setpoint 의 선택. 모든 함수는 입력만으로 결과를 정하며 MAVLink 와 시계를 사용하지 않는다
// 규약: docs/spec/icd.md 4.2, 5장, 6.2, 6.6
// 사용: FcControl
#ifndef ARGUS_ONBOARD_FC_COMMAND_POLICY_HPP_
#define ARGUS_ONBOARD_FC_COMMAND_POLICY_HPP_

#include <cstdint>
#include <optional>

#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

struct FcCommandPolicyConfig {
    // 받은 후 이 시간이 지난 비행 상태(HEARTBEAT)는 쓰지 않는다 (icd.md 6.2). HEARTBEAT 는 1 Hz
    // 이므로 연속 세 번을 받지 못한 경우
    std::int64_t status_max_age_ns = 3'000'000'000;

    // Offboard 진입 명령을 송신한 후 이 시간 안에 FC 의 모드가 Offboard 로 확인되지 않으면 그
    // 요청은 실패이다
    std::int64_t entry_confirm_ns = 3'000'000'000;

    // Offboard 진입 요청을 받는 시점에 setpoint 를 이 시간 이상 끊기지 않고 FC 로 송신하고 있어야
    // 한다. PX4 는 Offboard 로 전환하기 전부터 setpoint 를 받고 있어야 전환을 허용한다
    std::int64_t prestream_ns = 1'000'000'000;
};

// 상태가 바뀌면서 FcControl 이 실행할 일
struct FcCommandEffects {
    bool send_entry_command = false;  // FC 로 Offboard 진입 명령을 송신

    // Offboard 진입의 완료를 이 command_id 로 알림
    std::optional<vehicle_api::CommandId> entered_command_id;

    // FC 가 요청 없이 Offboard 를 벗어난 것을 알림 (벗어난 후의 모드)
    std::optional<vehicle_api::FlightMode> exited_to;

    const char* entry_failure = nullptr;  // 진입 실패의 원인 (로그)

    // 추종 중 setpoint 의 유효 시간이 지나 정지함 (icd 6.4 ⓐ. mission_state FAULT 보고에 사용)
    bool setpoint_timed_out = false;
};

struct FcCommandStep {
    FcCommandState state;
    FcCommandEffects effects;
};

struct FcRequestStep {
    FcCommandState state;
    FcCommandEffects effects;
    FcCommandDenial denial = FcCommandDenial::None;  // None = 요청을 수락
};

// 주기마다 FC 로 보낼 setpoint
enum class SetpointKind : std::uint8_t {
    None,
    Velocity,      // 추종 제어가 보낸 값
    ZeroVelocity,  // 속도 0
    Position,      // 제자리 유지의 목표
};

struct SetpointChoice {
    SetpointKind kind = SetpointKind::None;
    HoverTarget target;  // kind 가 Position 인 경우
};

// mavlink_gateway 가 판단한 FC 의 모드.
// 쓸 수 있는 비행 상태: 지금의 fc_epoch 에서 받았고, 받은 후 status_max_age_ns 이내인 값.
// not_offboard_after_ns 가 있는 경우, Offboard 가 아닌 비행 상태는 그 시각 후에 받은 것만
// NotOffboard 로 판단하고 그 전의 값은 Pending 이다
FcModeBelief fc_mode_belief(const FcObservation& observation, const FcCommandPolicyConfig& config,
                            std::optional<std::int64_t> not_offboard_after_ns = std::nullopt);

// 요청 없이 일어나는 전환: fc_epoch 변경, FC 의 모드, 진입 기한, setpoint 의 유효 시간, 제자리
// 유지 목표의 결정. 더 바뀌지 않을 때까지 적용한 결과를 반환한다.
// 주기 작업과 요청 처리의 시작에서 호출한다 (요청을 지난 주기의 FC 모드로 판단하지 않기 위함)
FcCommandStep advance(const FcCommandState& state, const FcObservation& observation,
                      const FcCommandPolicyConfig& config);

// 추종 제어의 Offboard 진입 요청. state 는 advance() 를 적용한 상태여야 한다
FcRequestStep on_offboard_request(const FcCommandState& state, const FcObservation& observation,
                                  const vehicle_api::RequestHeader& header,
                                  const FcCommandPolicyConfig& config);

// 추종 제어의 정지 요청. 송신권을 가진 세션의 요청은 거부되는 경우(fc_epoch 불일치, 유효 시간
// 초과)에도 정지한다. 정지하지 않으면 거부 원인이 없어진 후 그 세션의 setpoint 가 다시 송신된다
FcRequestStep on_hold_request(const FcCommandState& state, const FcObservation& observation,
                              const vehicle_api::RequestHeader& header);

// mavlink_gateway 가 스스로 요청하는 정지. 세션과 fc_epoch 를 확인하지 않는다.
// denial 이 None 이 아닌 경우는 Locked 뿐이다
FcRequestStep on_onboard_stop(const FcCommandState& state);

// FC 가 진입 명령에 응답함. Entering 이 아닌 상태에서는 아무것도 바꾸지 않는다
FcCommandStep on_entry_command_answered(const FcCommandState& state, bool accepted);

// 진입 명령의 송신에 실패함
FcCommandStep on_entry_command_send_failed(const FcCommandState& state);

// 운용자의 Locked 해제. FC 의 비행 상태가 최신인 경우에만 해제한다
FcRequestStep on_lock_release(const FcCommandState& state, const FcObservation& observation,
                              const FcCommandPolicyConfig& config);

// 이번 주기에 보낼 setpoint. state 는 advance() 를 적용한 상태여야 한다.
// Entering, Following, CancellingEntry, Hovering 은 항상 하나를 보낸다
SetpointChoice select_setpoint(const FcCommandState& state, const FcObservation& observation,
                               const FcCommandPolicyConfig& config);

// state 가 FC 제어 명령 송신권을 가진 세션. 없는 경우 nullptr
const std::string* authority_session(const FcCommandState& state);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_COMMAND_POLICY_HPP_
