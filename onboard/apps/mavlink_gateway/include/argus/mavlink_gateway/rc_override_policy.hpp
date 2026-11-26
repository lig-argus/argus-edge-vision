// 조종사(RC) 개입의 판정과 통제기로의 복귀 조건. 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를
// 모른다
// 규약: docs/spec/icd.md 5.4
// 사용: RcOverrideMonitor 가 입력을 모아 부른다
#ifndef ARGUS_ONBOARD_RC_OVERRIDE_POLICY_HPP_
#define ARGUS_ONBOARD_RC_OVERRIDE_POLICY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "argus/mavlink_gateway/fc_rc_status_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 스틱 하나의 조종기 채널과 보정값. 값은 기체 FC 의 RC_MAP_*, RCn_MIN, RCn_MAX, RCn_TRIM, RCn_REV
// 와 같게 설정한다 (mavlink_gateway 는 FC 파라미터를 읽지 않는다, icd 2장)
struct StickCalibration {
    std::uint8_t channel = 1;  // 1~18
    std::uint16_t min_us = 1000;
    std::uint16_t max_us = 2000;
    std::uint16_t trim_us = 1500;
    bool reversed = false;

    // 중앙을 (min_us + max_us) / 2 로 한다. 스로틀은 TRIM 이 보통 아래 끝값이지만 Position 모드는
    // 스틱 중앙을 고도 유지로 쓰기 때문이다
    bool center_is_midpoint = false;
};

// 스틱 4축의 순서
enum class StickAxis : std::uint8_t { Roll, Pitch, Yaw, Throttle };
inline constexpr std::size_t kStickAxisCount = 4;

struct RcOverrideConfig {
    // 기본값은 PX4 의 기본 채널 배치(RC_MAP_ROLL 1, PITCH 2, THROTTLE 3, YAW 4)와 일반적인 보정값
    std::array<StickCalibration, kStickAxisCount> sticks{
        StickCalibration{1, 1000, 2000, 1500, false, false},
        StickCalibration{2, 1000, 2000, 1500, false, false},
        StickCalibration{4, 1000, 2000, 1500, false, false},
        StickCalibration{3, 1000, 2000, 1000, false, true}};

    // -1~1 로 정규화한 값이 중앙에서 이 범위 안이면 중립. PX4 Position 모드의 입력 없음 범위
    // (MPC_HOLD_DZ 기본값)와 같다
    double neutral_band = 0.1;

    // 스틱 4축이 중립으로 유지되어야 하는 시간
    std::int64_t neutral_hold_ns = 1'000'000'000;

    // 받은 뒤 이 시간 이내인 RC_CHANNELS 만 쓴다 (PX4 의 RC 입력 두절 기준 COM_RC_LOSS_T 기본값)
    std::int64_t rc_max_age_ns = 500'000'000;

    // 받은 뒤 이 시간 이내인 SYS_STATUS 만 쓴다
    std::int64_t sensor_max_age_ns = 1'000'000'000;

    // 받은 뒤 이 시간 이내인 비행 상태(HEARTBEAT)만 쓴다 (icd 6.2 ⓑ)
    std::int64_t status_max_age_ns = 3'000'000'000;

    // FC 모드가 바뀌기 전 이 시간 안에 같은 모드로 바꾸는 통제기 명령을 전달했으면 통제기의
    // 변경으로 본다
    std::int64_t gcs_mode_command_window_ns = 3'000'000'000;
};

// 조종사가 조종기로 직접 조종하는 모드 (Position, Altitude, Stabilized, Manual)
bool is_pilot_mode(vehicle_api::FlightMode mode);

// 채널 값을 -1~1 로 정규화한다. 중앙(TRIM 또는 MIN, MAX 의 가운데)이 0, 끝값이 ±1. 값이 없거나
// (0, UINT16_MAX) 보정값이 맞지 않으면 nullopt
std::optional<double> normalized_stick(std::uint16_t raw_us, const StickCalibration& calibration);

// 스틱 4축이 모두 중앙에서 neutral_band 안이면 true. 채널 수가 0 이거나 채널이 없으면 false
bool sticks_neutral(const RcChannels& channels, const RcOverrideConfig& config);

// FC 모드의 변경 하나
struct ModeChange {
    vehicle_api::FlightMode to = vehicle_api::FlightMode::Unknown;
    std::int64_t received_ns = 0;  // 바뀐 모드를 받은 시각
    bool armed = false;
};

// 통제기의 모드 명령을 FC 로 전달한 것
struct GcsModeCommand {
    vehicle_api::FlightMode mode = vehicle_api::FlightMode::Unknown;
    std::int64_t forwarded_ns = 0;
};

// 조종사 개입으로 판정하는 모드 변경인지: 시동이 걸려 있고, 바뀐 모드가 수동 모드이며, 그 모드로
// 바꾸는 통제기 명령을 바뀌기 전 gcs_mode_command_window_ns 안에 전달하지 않았음
bool is_pilot_takeover(const ModeChange& change, const std::optional<GcsModeCommand>& gcs_command,
                       const RcOverrideConfig& config);

// 복귀 조건 중 충족하지 않은 첫 항목
enum class ReturnBlock : std::uint8_t {
    None,
    NotPilot,    // 조종사 개입 중이 아님
    FcMode,      // FC 모드가 Hold 또는 Position 이 아니거나 비행 상태가 오래됨
    Sticks,      // 스틱 4축 중립이 neutral_hold_ns 동안 유지되지 않음
    RcReceiver,  // RC 수신기 상태가 정상이 아니거나 오래됨
    Estimate,    // 위치나 자세 추정이 유효하지 않음
    Busy,        // 진행 중인 명령, 착륙, 추종, 스캔, Offboard 진입이 있음
};

struct ReturnObservation {
    std::int64_t now_ns = 0;
    bool pilot_in_control = false;
    std::optional<vehicle_api::FlightStatus> status;  // 지금 fc_epoch 의 가장 최근 비행 상태
    std::optional<std::int64_t> neutral_since_ns;     // 스틱 4축이 중립이 된 시각
    std::optional<FcSensorHealth> sensor_health;      // 지금 fc_epoch 의 가장 최근 SYS_STATUS
    bool command_in_progress = false;  // 최종 결과를 보내지 않은 통제기 명령 (45011 자신 제외)
    bool follow_in_progress = false;   // 추종 또는 스캔 중
    bool fc_entering = false;          // mavlink_gateway 의 Offboard 진입 중
};

// 복귀 조건을 순서대로 확인한다 (icd 5.4). FC 모드가 Land 이면 착륙 중이므로 FcMode 이다
ReturnBlock check_return_conditions(const ReturnObservation& observation,
                                    const RcOverrideConfig& config);

// 복귀 절차 중(Offboard 진입 명령을 보내기 전)에 이어 가도 되는지: 조종사 개입 중이고, 스틱 4축이
// 지금 중립이며(neutral_since_ns 가 있음), RC 수신기와 추정 상태가 정상. FC 모드의 변경은
// FcControl 이 확인하고, 진행 중인 명령과 Offboard 진입은 복귀 절차 자신이므로 보지 않는다
bool return_procedure_allowed(const ReturnObservation& observation, const RcOverrideConfig& config);

// 통제기 화면에 보내는 STATUSTEXT 문구. None 이면 빈 문자열
std::string_view to_text(ReturnBlock block);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_RC_OVERRIDE_POLICY_HPP_
