// mavlink_gateway 가 FC 로 보내는 명령의 상태와, 상태를 바꾸는 판단에 쓰는 값
// 규약: docs/spec/icd.md 4.2, 5장, 6.2, 6.6, docs/spec/process_interfaces.md 6장
// 사용: fc_command_policy (판단), FcControl (보관과 실행)
#ifndef ARGUS_ONBOARD_FC_COMMAND_STATE_HPP_
#define ARGUS_ONBOARD_FC_COMMAND_STATE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 제자리 유지의 목표 (local NED 위치와 yaw)
struct HoverTarget {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
    double yaw_rad = 0.0;
};

// mavlink_gateway 가 FC 를 제어하지 않음. FC 가 Offboard 가 아니라고 믿거나 FC 의 비행 상태를 받지
// 못함
struct Idle {};

// Offboard 진입 명령을 보내고 FC 의 전환을 기다림
struct Entering {
    std::string authority_session;  // FC 제어 명령 송신권을 가진 세션
    vehicle_api::CommandId command_id;
    std::uint64_t fc_epoch = 0;    // 요청을 받은 때의 FC 연결 epoch
    std::int64_t deadline_ns = 0;  // 이 시각까지 FC 가 Offboard 로 확인되지 않으면 실패
    bool acknowledged = false;     // FC 가 진입 명령을 수락함 (COMMAND_ACK)
};

// FC 가 Offboard 임을 확인했고 송신권을 가진 세션이 제어함
struct Following {
    std::string authority_session;
    std::uint64_t fc_epoch = 0;
    std::int64_t values_after_ns = 0;  // 이 시각보다 늦게 받은 setpoint 만 사용

    // 제자리 유지 중에 수락되어 그 세션의 첫 setpoint 를 기다리는 중. 그동안은 제자리 유지를 이어
    // 간다
    bool awaiting_first_value = false;

    std::optional<HoverTarget> hover_target;  // awaiting_first_value 인 동안 보낼 목표
};

// 진입을 취소했으나 FC 가 Offboard 로 바뀌었을 수 있어, 속도 0 을 보내며 확인을 기다림
struct CancellingEntry {
    // 취소한 진입의 기한. FC 가 진입하지 않았다는 판단에는 이 시각 후에 받은 비행 상태만 쓴다
    // (기한 전의 값은 FC 가 아직 전환하지 않은 때의 값일 수 있음)
    std::int64_t deadline_ns = 0;
};

// FC 가 Offboard 임을 확인했고 mavlink_gateway 가 제자리 유지를 보냄
struct Hovering {
    // 자세와 local 위치가 최신인 첫 주기에 정하고 바꾸지 않는다. 정하기 전에는 속도 0 을 보낸다
    std::optional<HoverTarget> target;
};

// FC 가 mavlink_gateway 의 요청 없이 Offboard 를 벗어남. 운용자가 해제할 때까지 아무것도 보내지
// 않음
struct Locked {};

using FcCommandState = std::variant<Idle, Entering, Following, CancellingEntry, Hovering, Locked>;

// mavlink_gateway 가 판단한 FC 의 모드. FC 의 비행 상태는 1 Hz 로 도착한다
enum class FcModeBelief : std::uint8_t {
    Offboard,
    NotOffboard,
    Pending,      // 아직 모름: 판단에 쓸 수 있는 비행 상태가 아직 없음 (정상)
    NotReceived,  // 받지 못함: FC 연결 두절이거나 비행 상태가 오래됨 (icd.md 6.2)
};

// 판단의 입력. 주기 작업과 요청 처리의 시작에서 한 번 모은다
struct FcObservation {
    std::int64_t now_ns = 0;
    std::uint64_t fc_epoch = 0;
    bool fc_connected = false;
    std::optional<vehicle_api::FlightStatus> status;  // 가장 최근에 받은 비행 상태
    std::optional<HoverTarget> fresh_pose;  // 자세와 local 위치가 모두 최신인 경우의 현재 위치

    // 추종 제어가 보낸 setpoint 중 유효 시간이 지나지 않은 가장 최근 값을 받은 시각
    std::optional<std::int64_t> setpoint_received_ns;

    // 추종 제어의 setpoint 를 FC 로 끊기지 않고 송신해 온 시간. 끊긴 경우 0
    std::int64_t streaming_for_ns = 0;
};

// 요청의 거부 원인. 문자열은 응답의 reason 과 로그에 쓴다
enum class FcCommandDenial : std::uint8_t {
    None,
    EmptySession,        // session_id 가 비어 있음
    NoAuthority,         // 다른 세션이 송신권을 가짐
    ExternalModeChange,  // 외부의 모드 변경 후 해제되지 않음
    FcEpochMismatch,     // 요청의 fc_epoch 가 지금 값과 다름
    FcDisconnected,      // FC 연결 두절
    FcStatusStale,       // FC 의 비행 상태가 없거나 오래됨
    Expired,             // 요청의 유효 시간이 지남
    PrestreamNotReady,   // setpoint 를 충분히 오래 송신하고 있지 않음
    EntryCancelled,      // 취소한 진입의 확인을 기다리는 중
    SendFailed,          // FC 로의 송신 실패
};

std::string_view to_string(FcCommandDenial denial);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_COMMAND_STATE_HPP_
