#include "argus/gcs/link/command_traits.hpp"

namespace argus::gcs::link {

namespace {

constexpr CommandTarget kFc = CommandTarget::Fc;
constexpr CommandTarget kGateway = CommandTarget::Gateway;

// 명령, 이름, 스크립트 이름, 대상, 비상, 모드 변경, 스캔 결과 닫기, 스캔 요청 지우기.
// Hold 는 FC 대상(DO_SET_MODE, icd 4.3)으로 보내고 mavlink_gateway 가 제자리 유지로 처리한다 (icd
// 5.2). 응답을 기다리지 않는 것은 착륙뿐이다: icd 5.3 이 착륙만 다른 명령의 처리를 기다리지
// 않는다고 정하고, RTL 과 Hold 는 5.2 의 조건 검사를 거친다
constexpr std::array<CommandTraits, kCommandCount> kTraits{{
    {Command::Arm, "ARM", "arm", kFc, false, false, false, false},
    {Command::Disarm, "DISARM", "disarm", kFc, false, false, false, false},
    {Command::Takeoff, "TAKEOFF", "takeoff", kFc, false, true, false, false},
    {Command::Land, "LAND", "land", kFc, true, true, true, false},
    {Command::EmergencyLand, "비상 착륙", "emergency_land", kFc, true, true, true, false},
    {Command::Rtl, "RTL", "rtl", kFc, false, true, true, false},
    {Command::Hold, "HOLD", "hold", kFc, false, true, true, false},
    {Command::Stabilized, "Stabilized", "stabilized", kFc, false, true, false, false},
    {Command::Altitude, "Altitude", "altitude", kFc, false, true, false, false},
    {Command::Position, "Position", "position", kFc, false, true, false, false},
    {Command::SetHome, "Home 설정", "set_home", kFc, false, false, false, false},
    {Command::RebootFc, "FC 재부팅", "reboot_fc", kFc, false, false, false, false},
    {Command::TrackStart, "추종 시작", "track_start", kGateway, false, false, false, true},
    {Command::TrackStop, "중단", "track_stop", kGateway, false, true, false, true},
    {Command::ScanStart, "스캔 시작", "scan_start", kGateway, false, false, false, false},
    {Command::RequestControl, "제어권 요청", "request_control", kGateway, false, false, false,
     false},
    {Command::ReleaseControl, "제어권 해제", "release_control", kGateway, false, false, false,
     false},
    {Command::EndRcOverride, "통제기 복귀", "end_rc_override", kGateway, false, false, false,
     false},
}};

// 표가 Command 순서와 같은지 컴파일할 때 확인한다. 줄이 모자라면 남은 칸이 Command::Arm 으로 채워져
// 순서 검사에 걸린다
constexpr bool inEnumOrder() {
    for (std::size_t i = 0; i < kTraits.size(); ++i) {
        if (static_cast<std::size_t>(kTraits.at(i).command) != i) {
            return false;
        }
    }
    return true;
}
static_assert(inEnumOrder(), "kTraits 는 Command 순서와 같아야 한다");

}  // namespace

const std::array<CommandTraits, kCommandCount>& allCommandTraits() { return kTraits; }

const CommandTraits& commandTraits(Command command) {
    return kTraits.at(static_cast<std::size_t>(command));
}

std::optional<Command> commandFromScriptName(const QString& name) {
    for (const CommandTraits& traits : kTraits) {
        if (name == QLatin1String(traits.scriptName)) {
            return traits.command;
        }
    }
    return std::nullopt;
}

QString commandName(Command command) { return QString::fromUtf8(commandTraits(command).name); }

}  // namespace argus::gcs::link
