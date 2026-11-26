#include "argus/gcs/link/command_policy.hpp"

#include <cmath>

#include "argus/gcs/link/command_encoder.hpp"
#include "argus/gcs/link/command_traits.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

namespace {

constexpr std::int16_t kThrottlePerPercent = 10;  // ManualAxes.z 0~1000 → 0~100 %

QString missionActiveRefusal() { return QStringLiteral("추종, 스캔 중: [중단] 후 가능"); }

// SCAN_START 매개변수 (icd 4.2): 범위 0 초과 2π 이하, 속도 0 이 아니고 절댓값 75 deg/s 이하, 제한
// 시간은 범위를 속도로 돈 시간보다 길다. NaN 은 mavlink_gateway 의 기본값을 쓰라는 뜻이라 범위
// 검사에서 빼고, 제한 시간 비교는 셋 다 정했을 때만 한다 (기본값은 mavlink_gateway 설정이라
// 통제기가 모른다)
bool scanArgsValid(const CommandArgs& args) {
    const float range = args.scanRangeRad;
    const float rate = args.scanRateRadS;
    const float timeout = args.scanTimeoutS;
    const bool rangeOk =
        std::isnan(range) || (range > 0.0F && range <= CommandPolicy::kScanRangeMaxRad);
    const bool rateOk =
        std::isnan(rate) || (rate != 0.0F && std::fabs(rate) <= CommandPolicy::kScanRateMaxRadS);
    const bool timeoutOk = std::isnan(timeout) || timeout > 0.0F;
    if (!rangeOk || !rateOk || !timeoutOk) {
        return false;
    }
    if (std::isnan(range) || std::isnan(rate) || std::isnan(timeout)) {
        return true;
    }
    return timeout > range / std::fabs(rate);
}

bool validTrackValue(std::uint32_t value) { return value >= 1 && value <= kMaxTrackId; }

}  // namespace

CommandPolicy::CommandPolicy(const VehicleState& state, const FlightConditions& conditions,
                             const CommandContext& context)
    : state_(state), conditions_(conditions), context_(context) {}

std::optional<QString> CommandPolicy::specificRefusal(Command command,
                                                      const CommandArgs& args) const {
    if (sharedRefusal(command)) {
        return std::nullopt;
    }
    return refusal(command, args);
}

std::optional<QString> CommandPolicy::refusal(Command command, const CommandArgs& args) const {
    if (std::optional<QString> why = sharedRefusal(command)) {
        return why;
    }
    if (std::optional<QString> why = authorityRefusal(command)) {
        return why;
    }
    if (std::optional<QString> why = overrideRefusal(command)) {
        return why;
    }
    return commandRefusal(command, args);
}

std::optional<QString> CommandPolicy::paramWriteRefusal() const {
    if (!conditions_.gatewayLinkUp()) {
        return QStringLiteral("임무 컴퓨터 미연결");
    }
    if (std::optional<QString> why = groundRefusal()) {
        return why;
    }
    if (conditions_.missionActive()) {
        return missionActiveRefusal();
    }
    return std::nullopt;
}

// 통제기의 메시지는 모두 mavlink_gateway 를 거쳐 FC 로 간다 (icd 1.2). FC 대상 명령은 두 링크가
// 모두 살아 있어야 한다. 응답 대기는 같은 대상, 같은 MAV_CMD 만 막는다 (icd 4.1): 진행 중인
// TRACK_START, SCAN_START 를 TRACK_STOP 으로 취소할 수 있어야 한다. 착륙은 기다리지 않는다
// (icd 5.3)
std::optional<QString> CommandPolicy::sharedRefusal(Command command) const {
    const CommandTraits& traits = commandTraits(command);
    if (traits.target == CommandTarget::Fc && !conditions_.fcLinkUp()) {
        return QStringLiteral("FC 미연결");
    }
    if (!conditions_.gatewayLinkUp()) {
        return QStringLiteral("임무 컴퓨터 미연결");
    }
    if (traits.emergency) {
        return std::nullopt;
    }
    const std::uint16_t mavCmd = mavCmdOf(command);
    for (const Command awaiting : context_.awaiting) {
        if (commandTraits(awaiting).target == traits.target && mavCmdOf(awaiting) == mavCmd) {
            return QStringLiteral("앞서 전송한 %1 의 응답 대기 중").arg(commandName(awaiting));
        }
    }
    return std::nullopt;
}

// 제어권이 없는 통제기의 명령은 mavlink_gateway 가 거부한다 (icd 5.1). 제어권 요청만 보낼 수 있다.
// ARGUS_STATUS 를 받기 전에는 제어권을 모르므로 막지 않는다
std::optional<QString> CommandPolicy::authorityRefusal(Command command) const {
    if (command == Command::RequestControl || !state_.gateway.received ||
        conditions_.hasControl()) {
        return std::nullopt;
    }
    if (conditions_.controlHeldByOther()) {
        return QStringLiteral("제어권 없음: 통제기 %1 이 보유 중").arg(state_.gateway.controlSysid);
    }
    return QStringLiteral("제어권 없음: [제어권 요청] 필요");
}

// 조종사 개입 중에는 착륙 외의 명령을 mavlink_gateway 가 거부한다 (icd 5.4). 복귀 명령, 개입으로
// 끝난 임무의 정리(TRACK_STOP, icd 6장), 제어권 명령은 보낼 수 있다
std::optional<QString> CommandPolicy::overrideRefusal(Command command) const {
    if (!conditions_.pilotOverride()) {
        return std::nullopt;
    }
    switch (command) {
        case Command::Land:
        case Command::EmergencyLand:
        case Command::TrackStop:
        case Command::RequestControl:
        case Command::ReleaseControl:
        case Command::EndRcOverride:
            return std::nullopt;
        case Command::Arm:
        case Command::Disarm:
        case Command::Takeoff:
        case Command::Rtl:
        case Command::Hold:
        case Command::Stabilized:
        case Command::Altitude:
        case Command::Position:
        case Command::SetHome:
        case Command::RebootFc:
        case Command::TrackStart:
        case Command::ScanStart:
            break;
    }
    return QStringLiteral("조종사(RC) 개입 중: [통제기 복귀] 후 가능");
}

std::optional<QString> CommandPolicy::commandRefusal(Command command,
                                                     const CommandArgs& args) const {
    switch (command) {
        case Command::Arm:
            return armRefusal();
        case Command::Disarm:
        case Command::Rtl:
            return setpointRefusal();
        case Command::Takeoff:
            return takeoffRefusal(args);
        case Command::Land:
        case Command::Hold:
            return std::nullopt;
        case Command::EmergencyLand:
            // 비상 착륙은 떠 있을 때(ARM)만 뜻이 있다
            if (!state_.fc.armed) {
                return QStringLiteral("지상(DISARM)에서는 사용 불가");
            }
            return std::nullopt;
        case Command::Stabilized:
        case Command::Altitude:
        case Command::Position:
            return manualModeRefusal(command);
        case Command::SetHome:
            return groundRefusal();
        case Command::RebootFc:
            if (context_.paramWriteActive) {
                return QStringLiteral("파라미터 기록 중: 완료 후 재부팅 가능");
            }
            return groundRefusal();
        case Command::TrackStart:
            return trackStartRefusal(args);
        case Command::TrackStop:
            if (!conditions_.missionStoppable()) {
                return QStringLiteral("중단할 임무 없음");
            }
            return std::nullopt;
        case Command::ScanStart:
            return scanRefusal(args);
        case Command::RequestControl:
            if (conditions_.hasControl()) {
                return QStringLiteral("이미 제어권 보유 중");
            }
            return std::nullopt;
        case Command::ReleaseControl:
            return std::nullopt;
        case Command::EndRcOverride:
            return endRcOverrideRefusal();
    }
    return std::nullopt;
}

// mavlink_gateway 가 setpoint 를 보내는 중에는 ARM, DISARM, TAKEOFF, RTL 을 FC 로 전달하지 않는다
// (icd 5.2). 임무가 없는데 OFFBOARD 이면 수동 모드로 바꾸어야 setpoint 송신이 끝난다
std::optional<QString> CommandPolicy::setpointRefusal() const {
    if (conditions_.missionActive()) {
        return missionActiveRefusal();
    }
    if (state_.fc.mode == FlightMode::Offboard) {
        return QStringLiteral("FC 가 OFFBOARD(임무 컴퓨터 제어): [Position] 등 모드 전환 후 가능");
    }
    return std::nullopt;
}

// 지상이고 시동이 꺼져 있을 때만 (icd 5.2 의 DO_SET_HOME, 재부팅, PARAM_SET)
std::optional<QString> CommandPolicy::groundRefusal() const {
    if (state_.fc.armed) {
        return QStringLiteral("ARM 상태: 착륙 후 DISARM 필요");
    }
    if (!conditions_.onGroundDisarmed()) {
        return QStringLiteral("비행 중: 착륙 후 가능");
    }
    return std::nullopt;
}

// 수동 모드 전환과 그 ARM 은 FC 가 조종 입력 정상을 보고해야 보낸다. 가상 조종을 켜도 PX4 가 그
// 입력을 받아들이기까지 잠깐 걸리므로 그동안은 기다리라고 안내한다
std::optional<QString> CommandPolicy::manualInputRefusal() const {
    if (conditions_.manualInputReady()) {
        return std::nullopt;
    }
    if (context_.manualOn) {
        return QStringLiteral("FC 가 가상 조종 입력 확인 중: 잠시 후 재시도");
    }
    return QStringLiteral("조종기 신호 없음: RC 조종기 또는 가상 조종기 활성화 필요");
}

// 수동 모드의 ARM 은 조종 입력이 필요하다. 스로틀 조건은 PX4 도 검사하지만 통제기가 먼저 이유를
// 보인다 (Stabilized 는 0 %, Altitude, Position 은 50 % 이하). Position 에서 수평 위치 추정이
// 없으면 이륙 뒤 위치를 잃고 failsafe 로 하강하므로 보내지 않는다 (icd 4.3)
std::optional<QString> CommandPolicy::armRefusal() const {
    if (std::optional<QString> why = setpointRefusal()) {
        return why;
    }
    if (!conditions_.manualMode()) {
        return std::nullopt;
    }
    if (std::optional<QString> why = manualInputRefusal()) {
        return why;
    }
    if (conditions_.modeIs(FlightMode::Position) && !conditions_.horizontalPositionKnown()) {
        return QStringLiteral(
            "수평 위치 추정 없음(GNSS, optical flow 없음): Position 비행 불가. [Altitude] 전환 후 "
            "ARM 가능");
    }
    const std::int16_t throttle = context_.manualAxes.z;
    if (context_.manualOn && conditions_.modeIs(FlightMode::Stabilized) &&
        throttle > kArmThrottleMax) {
        return QStringLiteral("Throttle 이 0 % 가 아님: 0 % 로 낮춘 후 ARM 가능");
    }
    const bool centered =
        conditions_.modeIs(FlightMode::Altitude) || conditions_.modeIs(FlightMode::Position);
    if (context_.manualOn && centered && throttle > kArmThrottleCenter) {
        return QStringLiteral("Throttle 이 50 % 초과: 50 % 이하로 낮춘 후 ARM 가능");
    }
    return std::nullopt;
}

// 이륙 고도가 범위 밖이거나 NaN 이면 보내지 않는다. TAKEOFF p7 은 해발 고도라 전역 위치가 있어야
// 한다 (icd 5.2). 위치 없이 띄울 때는 수동 모드에서 스로틀을 올린다
std::optional<QString> CommandPolicy::takeoffRefusal(const CommandArgs& args) const {
    if (!(args.takeoffAltM >= CommandArgs::kTakeoffAltMinM &&
          args.takeoffAltM <= CommandArgs::kTakeoffAltMaxM)) {
        return QStringLiteral("이륙 고도 범위 밖: %1~%2 m 로 입력 필요")
            .arg(CommandArgs::kTakeoffAltMinM)
            .arg(CommandArgs::kTakeoffAltMaxM);
    }
    if (std::optional<QString> why = setpointRefusal()) {
        return why;
    }
    if (!conditions_.positionKnown()) {
        return QStringLiteral(
            "전역 위치 없음(GNSS 없음): TAKEOFF 사용 불가. [Altitude] → [ARM] → Throttle 상승으로 "
            "이륙");
    }
    return std::nullopt;
}

// 수동 모드는 조종 입력이 있어야 PX4 가 받는다. ARM 된 채 지상에서 Stabilized 로 바꾸면 Altitude,
// Position 의 고도 유지 스로틀(가상 조종 50 %)이 그대로 추력이 되므로 막는다. PX4 는 이 전환을 막지
// 않는다. 공중에서는 막지 않는다 (호버 추력). Position 은 수평 위치 추정도 필요하다
std::optional<QString> CommandPolicy::manualModeRefusal(Command command) const {
    if (std::optional<QString> why = manualInputRefusal()) {
        return why;
    }
    const std::int16_t throttle = context_.manualAxes.z;
    if (command == Command::Stabilized && state_.fc.armed && !conditions_.inAir() &&
        context_.manualOn && throttle > kArmThrottleMax) {
        return QStringLiteral(
                   "지상에서 Stabilized 전환 시 Throttle %1 % 가 즉시 추력으로 적용됨: 0 % 로 "
                   "낮춘 후 가능")
            .arg(throttle / kThrottlePerPercent);
    }
    if (command == Command::Position && !conditions_.horizontalPositionKnown()) {
        return QStringLiteral(
            "수평 위치 추정 없음(GNSS, optical flow 없음): Position 모드 사용 불가. Altitude "
            "모드는 "
            "사용 가능");
    }
    return std::nullopt;
}

// 추종과 스캔의 공통 시작 조건 (icd 4.2): 비행 중, 임무 상태 IDLE 또는 HOLD_WAIT, 정지 상태
std::optional<QString> CommandPolicy::startRefusal() const {
    if (!state_.fc.armed || !conditions_.inAir()) {
        return QStringLiteral("이륙 후 비행 중에만 가능");
    }
    if (conditions_.missionActive()) {
        return missionActiveRefusal();
    }
    if (!conditions_.missionReady()) {
        return QStringLiteral("임무 상태가 대기가 아님: [중단] 후 가능");
    }
    if (!conditions_.stationary()) {
        return QStringLiteral("정지 상태가 아님: [HOLD] 후 가능");
    }
    return std::nullopt;
}

// 추종 시작 (icd 4.2): 시작 조건, 센서의 FOLLOWABLE, 운용자가 최근 탐지에서 고른 확정 표적. 추종 중
// 표적을 바꾸려면 [중단] 뒤 다시 시작한다
std::optional<QString> CommandPolicy::trackStartRefusal(const CommandArgs& args) const {
    if (std::optional<QString> why = startRefusal()) {
        return why;
    }
    if (args.sensor == ARGUS_SENSOR_UNKNOWN || !conditions_.sensorFollowable(args.sensor)) {
        return QStringLiteral("현재 이 센서로 추종 불가 (영상 또는 탐지 끊김)");
    }
    if (!args.targetSelected) {
        return QStringLiteral("영상의 상자 또는 목록에서 표적 선택 필요");
    }
    if (!validTrackValue(args.trackId) || !validTrackValue(args.generation) ||
        args.classId > kMaxTrackId) {
        return QStringLiteral("최신 영상 메타데이터에서 표적 선택 필요");
    }
    return std::nullopt;
}

std::optional<QString> CommandPolicy::scanRefusal(const CommandArgs& args) const {
    if (std::optional<QString> why = startRefusal()) {
        return why;
    }
    if (!scanArgsValid(args)) {
        return QStringLiteral(
            "스캔 매개변수 범위 밖: 범위 0~360°, 속도 75°/s 이하, 제한 시간 확인");
    }
    return std::nullopt;
}

// 통제기 복귀 (icd 5.4): FC 모드가 Hold 또는 Position 이고 진행 중인 임무가 없을 때. 스틱 중립
// 유지와 추정 상태는 mavlink_gateway 가 확인한다
std::optional<QString> CommandPolicy::endRcOverrideRefusal() const {
    if (conditions_.missionActive()) {
        return missionActiveRefusal();
    }
    if (!conditions_.modeIs(FlightMode::Hold) && !conditions_.modeIs(FlightMode::Position)) {
        return QStringLiteral("FC 모드가 Hold 또는 Position 일 때만 복귀 가능");
    }
    return std::nullopt;
}

}  // namespace argus::gcs::link
