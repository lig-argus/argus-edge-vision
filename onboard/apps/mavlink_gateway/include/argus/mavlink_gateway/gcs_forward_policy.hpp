// 통제기가 FC 로 보낸 메시지 하나의 처리 판단 (FC 로 전달, 직접 응답, 폐기, 정지)
// 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를 모른다
// 규약: docs/spec/icd.md 4.1, 4.3, 5.2, 5.3, 5.4, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: GcsFcForwarder 가 입력을 모아 decide_forward() 를 부르고 결과를 실행한다
#ifndef ARGUS_ONBOARD_GCS_FORWARD_POLICY_HPP_
#define ARGUS_ONBOARD_GCS_FORWARD_POLICY_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// FC 의 비행 상태(HEARTBEAT)를 최신으로 보는 경과 시간의 상한 (icd 6.2 ⓑ 와 같음)
inline constexpr std::int64_t kFcStatusMaxAgeNs = 3'000'000'000;

// 통제기가 FC 를 대상으로 보낸 메시지의 종류. 대상(target_system, target_component)은
// GcsFcForwarder 가 확인한 뒤 판단을 부른다
enum class GcsMessageKind : std::uint8_t {
    Heartbeat,
    CommandLong,
    CommandInt,
    ManualControl,
    ParamRequestRead,
    ParamSet,
};

struct GcsMessageRequest {
    GcsMessageKind kind = GcsMessageKind::Heartbeat;

    // CommandLong, CommandInt 의 MAV_CMD 와 param1~3. 그 밖의 종류에서는 사용하지 않는다
    std::uint16_t command = 0;
    float param1 = 0.0F;
    float param2 = 0.0F;
    float param3 = 0.0F;
};

struct ForwardObservation {
    bool from_controller = false;  // 보낸 통제기가 제어권을 가짐

    // FC 의 비행 상태를 kFcStatusMaxAgeNs 이내에 받았고 FC 와 연결되어 있음. false 이면 아래
    // fc_mode, armed 는 사용하지 않는다
    bool fc_status_known = false;
    vehicle_api::FlightMode fc_mode = vehicle_api::FlightMode::Unknown;
    bool armed = false;

    // mavlink_gateway 가 FC 로 setpoint 를 보내거나 Offboard 진입을 기다리는 중
    bool sending_setpoints = false;
    bool follow_in_progress = false;   // 추종 또는 스캔 중 (follow/status)
    bool command_in_progress = false;  // 최종 결과를 보내지 않은 통제기 명령이 있음
    bool landing_locked = false;       // 착륙 잠금 (icd 5.3)
    bool pilot_in_control = false;     // 조종사 개입 중 (icd 5.4)
};

enum class ForwardAction : std::uint8_t {
    Forward,  // 받은 메시지를 그대로 FC 로 보낸다
    Reply,    // FC 로 보내지 않고 result 로 직접 답한다 (명령만)
    Drop,     // FC 로 보내지 않고 응답도 하지 않는다
    Stop,     // HOLD: FC 로 보내지 않고 mavlink_gateway 가 기체를 정지시킨 뒤 답한다
};

struct ForwardDecision {
    ForwardAction action = ForwardAction::Drop;

    // Reply 의 COMMAND_ACK 결과와 STATUSTEXT 로 보내는 이유. 이유가 비어 있으면 STATUSTEXT 를
    // 보내지 않는다
    MAV_RESULT result = MAV_RESULT_DENIED;
    std::string reason;
};

// MAV_CMD_DO_SET_MODE 의 param1(base_mode), param2(main_mode), param3(sub_mode)가 요청하는 PX4 모드
// (icd 부록 A). base_mode 에 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 가 없거나 값이 0~255 의 정수가
// 아니면 nullopt. 부록 A 의 표에 없는 모드는 Unknown
std::optional<vehicle_api::FlightMode> set_mode_request(float base_mode, float main_mode,
                                                        float sub_mode);

// icd 5.2 의 표로 request 의 처리를 정한다. 확인 순서:
// 1. COMMAND_INT 는 COMMAND_LONG_ONLY (icd 4.1)
// 2. 제어권이 없는 통제기: 명령은 TEMPORARILY_REJECTED("제어권 없음"), 그 밖은 폐기
// 3. NAV_LAND 는 다른 조건 없이 전달 (icd 5.3)
// 4. 표에 없는 명령, 허용하지 않는 값(강제 시동, 허용하지 않는 모드)은 DENIED
// 5. 착륙 잠금, 조종사 개입 중에는 착륙 외의 명령을 TEMPORARILY_REJECTED (icd 5.3, 5.4).
//    MANUAL_CONTROL, PARAM_SET 은 폐기
// 6. 명령과 메시지별 상태 조건. 명령이 아니면 조건에 맞지 않을 때 폐기
ForwardDecision decide_forward(const ForwardObservation& observation,
                               const GcsMessageRequest& request);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_FORWARD_POLICY_HPP_
