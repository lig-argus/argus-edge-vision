#include "argus/gcs/link/command_encoder.hpp"

#include <limits>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

namespace {

constexpr float kNanParam = std::numeric_limits<float>::quiet_NaN();

// param 의 자리 (param1 이 0)
constexpr std::size_t kP1 = 0;
constexpr std::size_t kP2 = 1;
constexpr std::size_t kP3 = 2;
constexpr std::size_t kP4 = 3;
constexpr std::size_t kP5 = 4;
constexpr std::size_t kP6 = 5;
constexpr std::size_t kP7 = 6;

constexpr float kOn = 1.0F;
constexpr float kOff = 0.0F;

// DO_SET_MODE 의 PX4 main mode, sub mode (icd 부록 A)
constexpr float kMainAltitude = 2.0F;
constexpr float kMainPosition = 3.0F;
constexpr float kMainAuto = 4.0F;
constexpr float kMainStabilized = 7.0F;
constexpr float kAutoLoiter = 3.0F;

// PREFLIGHT_REBOOT_SHUTDOWN p1: autopilot 재부팅
constexpr float kRebootAutopilot = 1.0F;
// DO_SET_HOME p1: 지금 위치를 Home 으로
constexpr float kUseCurrentPosition = 1.0F;

void setMode(EncodedCommand& out, float mainMode, float subMode) {
    out.params.at(kP1) = static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    out.params.at(kP2) = mainMode;
    out.params.at(kP3) = subMode;
}

// PX4 는 TAKEOFF p7 을 해발 고도로 해석한다. 지상에서 이륙하므로 지금 해발 고도가 지면의 해발
// 고도이고, 여기에 입력 고도를 더한다. yaw(p4)와 위치(p5, p6)는 NaN 이면 지금 값을 유지한다
void encodeTakeoff(EncodedCommand& out, const CommandArgs& args, const EncodeInputs& in) {
    out.params.at(kP4) = kNanParam;
    out.params.at(kP5) = kNanParam;
    out.params.at(kP6) = kNanParam;
    out.params.at(kP7) = in.altAmslM + args.takeoffAltM;
    out.notes.push_back(QStringLiteral("TAKEOFF %1 m (해발 %2 m)")
                            .arg(args.takeoffAltM, 0, 'f', 1)
                            .arg(out.params.at(kP7), 0, 'f', 1));
}

// yaw 와 위치가 NaN 이면 지금 방향, 지금 위치에 착륙한다. 수평 위치가 없으면 PX4 는 제자리
// 하강(Descend)으로 바꾸어 착륙하므로, 이어 오는 failsafe 문구를 운용자가 이해하도록 먼저 알린다
void encodeLand(EncodedCommand& out, const EncodeInputs& in) {
    out.params.at(kP4) = kNanParam;
    out.params.at(kP5) = kNanParam;
    out.params.at(kP6) = kNanParam;
    out.params.at(kP7) = kNanParam;
    if (!in.horizontalPositionKnown) {
        out.notes.push_back(
            QStringLiteral("수평 위치 없음: 제자리 하강(Descend)으로 착륙. 바람에 밀릴 수 있음"));
    }
}

// p1 track_id, p2 센서, p3 class_id, p4 generation (icd 4.2)
void encodeTrackStart(EncodedCommand& out, const CommandArgs& args) {
    out.params.at(kP1) = static_cast<float>(args.trackId);
    out.params.at(kP2) = static_cast<float>(args.sensor);
    out.params.at(kP3) = static_cast<float>(args.classId);
    out.params.at(kP4) = static_cast<float>(args.generation);
}

// p1 범위(rad), p2 회전 속도(rad/s), p3 제한 시간(s). NaN 이면 mavlink_gateway 의 기본값 (icd 4.2)
void encodeScanStart(EncodedCommand& out, const CommandArgs& args) {
    out.params.at(kP1) = args.scanRangeRad;
    out.params.at(kP2) = args.scanRateRadS;
    out.params.at(kP3) = args.scanTimeoutS;
}

// p1 1 요청, 0 해제. p4 요청하는 통제기의 sysid. p2, p3 은 쓰지 않는다 (icd 4.4)
void encodeControl(EncodedCommand& out, float action, const EncodeInputs& in) {
    out.params.at(kP1) = action;
    out.params.at(kP4) = static_cast<float>(in.gcsSysid);
}

// FC 로 보내는 명령 (icd 4.3, 5.2). p7 은 TAKEOFF 고도와 LAND 위치 말고는 쓰지 않는다
void encodeFlight(EncodedCommand& out, Command command, const CommandArgs& args,
                  const EncodeInputs& in) {
    switch (command) {
        case Command::Arm:
            out.params.at(kP1) = kOn;
            break;
        case Command::Disarm:
            out.params.at(kP1) = kOff;
            break;
        case Command::Takeoff:
            encodeTakeoff(out, args, in);
            break;
        case Command::Land:
        case Command::EmergencyLand:
            encodeLand(out, in);
            break;
        case Command::Rtl:
            break;
        case Command::Hold:
            setMode(out, kMainAuto, kAutoLoiter);
            break;
        case Command::Stabilized:
            setMode(out, kMainStabilized, 0.0F);
            break;
        case Command::Altitude:
            setMode(out, kMainAltitude, 0.0F);
            break;
        case Command::Position:
            setMode(out, kMainPosition, 0.0F);
            break;
        case Command::SetHome:
            out.params.at(kP1) = kUseCurrentPosition;
            break;
        case Command::RebootFc:
            out.params.at(kP1) = kRebootAutopilot;
            break;
        case Command::TrackStart:
        case Command::TrackStop:
        case Command::ScanStart:
        case Command::RequestControl:
        case Command::ReleaseControl:
        case Command::EndRcOverride:
            break;
    }
}

// mavlink_gateway 로 보내는 명령 (icd 4.2, 4.4). p7 은 encodeCommand 가 명령 번호로 채운다
void encodeGateway(EncodedCommand& out, Command command, const CommandArgs& args,
                   const EncodeInputs& in) {
    switch (command) {
        case Command::TrackStart:
            encodeTrackStart(out, args);
            break;
        case Command::ScanStart:
            encodeScanStart(out, args);
            break;
        case Command::RequestControl:
            encodeControl(out, kOn, in);
            break;
        case Command::ReleaseControl:
            encodeControl(out, kOff, in);
            break;
        case Command::TrackStop:
        case Command::EndRcOverride:
        case Command::Arm:
        case Command::Disarm:
        case Command::Takeoff:
        case Command::Land:
        case Command::EmergencyLand:
        case Command::Rtl:
        case Command::Hold:
        case Command::Stabilized:
        case Command::Altitude:
        case Command::Position:
        case Command::SetHome:
        case Command::RebootFc:
            break;
    }
}

}  // namespace

std::uint16_t mavCmdOf(Command command) {
    switch (command) {
        case Command::Arm:
        case Command::Disarm:
            return MAV_CMD_COMPONENT_ARM_DISARM;
        case Command::Takeoff:
            return MAV_CMD_NAV_TAKEOFF;
        case Command::Land:
        case Command::EmergencyLand:
            return MAV_CMD_NAV_LAND;
        case Command::Rtl:
            return MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case Command::Hold:
        case Command::Stabilized:
        case Command::Altitude:
        case Command::Position:
            return MAV_CMD_DO_SET_MODE;
        case Command::SetHome:
            return MAV_CMD_DO_SET_HOME;
        case Command::RebootFc:
            return MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case Command::TrackStart:
            return MAV_CMD_ARGUS_TRACK_START;
        case Command::TrackStop:
            return MAV_CMD_ARGUS_TRACK_STOP;
        case Command::ScanStart:
            return MAV_CMD_ARGUS_SCAN_START;
        case Command::RequestControl:
        case Command::ReleaseControl:
            return MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL;
        case Command::EndRcOverride:
            return MAV_CMD_ARGUS_END_RC_OVERRIDE;
    }
    return 0;
}

EncodedCommand encodeCommand(Command command, const CommandArgs& args, const EncodeInputs& in) {
    EncodedCommand out;
    out.target = commandTraits(command).target;
    out.mavCmd = mavCmdOf(command);
    if (out.target == CommandTarget::Gateway) {
        out.params.fill(kNanParam);
        out.params.at(kP7) = static_cast<float>(in.commandNumber % kCommandNumberModulus);
        encodeGateway(out, command, args, in);
    } else {
        out.params.fill(0.0F);
        encodeFlight(out, command, args, in);
    }
    return out;
}

}  // namespace argus::gcs::link
