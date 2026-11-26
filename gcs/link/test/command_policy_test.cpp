// 명령 거부 (command_policy) 테스트: 링크와 응답 대기(착륙은 예외), 제어권, 조종사 개입, setpoint
// 송신 중 차단, 지상 전용 명령, 이륙 위치, 수동 모드, 추종과 스캔의 시작 조건, 통제기 복귀, 화면
// 전체에 보이는 이유 빼기
#include "argus/gcs/link/command_policy.hpp"

#include <QtTest>
#include <limits>

#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::int64_t kNow = 10 * kNsPerSecond;
constexpr std::uint8_t kOwnSysid = 251;
constexpr std::uint8_t kOtherSysid = 252;

// 두 링크 연결, 제어권 보유, DISARM, 지상, 임무 대기
VehicleState connected() {
    VehicleState state;
    state.gcsSysid = kOwnSysid;
    state.fcLink.state = LinkState::Ok;
    state.gatewayLink.state = LinkState::Ok;
    state.fc.landedState = MAV_LANDED_STATE_ON_GROUND;
    state.gateway.received = true;
    state.gateway.controlSysid = kOwnSysid;
    return state;
}

// 비행 중 Hold 로 정지, 추종 가능한 IR
VehicleState hovering() {
    VehicleState state = connected();
    state.fc.armed = true;
    state.fc.landedState = MAV_LANDED_STATE_IN_AIR;
    state.fc.mode = FlightMode::Hold;
    state.sensors[ARGUS_SENSOR_IR].flags = ARGUS_SENSOR_FLAGS_FOLLOWABLE;
    state.sensors[ARGUS_SENSOR_IR].receivedNs = kNow;
    return state;
}

CommandArgs selectedTarget() {
    CommandArgs args;
    args.sensor = ARGUS_SENSOR_IR;
    args.targetSelected = true;
    args.trackId = 7;
    args.generation = 2;
    args.classId = 1;
    return args;
}

std::optional<QString> refusal(const VehicleState& state, Command command,
                               const CommandContext& context = {}, const CommandArgs& args = {}) {
    const FlightConditions conditions(state, kNow);
    return CommandPolicy(state, conditions, context).refusal(command, args);
}

bool refusedWith(const std::optional<QString>& why, const char* text) {
    return why.value_or(QString()).contains(QString::fromUtf8(text));
}

}  // namespace

class CommandPolicyTest : public QObject {
    Q_OBJECT

private slots:
    // FC 명령은 FC 와 mavlink_gateway 링크가 모두 있어야 하고, mavlink_gateway 명령은 그 링크만
    // 본다
    void linksDown() {
        VehicleState state = connected();
        state.fcLink.state = LinkState::Lost;
        QVERIFY(refusedWith(refusal(state, Command::Hold), "FC 미연결"));
        QVERIFY(!refusedWith(refusal(state, Command::RequestControl), "미연결"));
        state = connected();
        state.gatewayLink.state = LinkState::Never;
        QVERIFY(refusedWith(refusal(state, Command::Arm), "임무 컴퓨터 미연결"));
    }

    // 같은 대상, 같은 MAV_CMD 의 응답 대기 중에는 막는다. 착륙은 기다리지 않고, 진행 중인
    // TRACK_START 는 TRACK_STOP 으로 취소할 수 있다 (icd 4.1)
    void awaitingSameCommand() {
        VehicleState state = connected();
        CommandContext context;
        context.awaiting = {Command::Arm};
        QVERIFY(refusedWith(refusal(state, Command::Disarm, context), "응답 대기 중"));
        QVERIFY(!refusedWith(refusal(state, Command::Hold, context), "응답 대기 중"));
        QVERIFY(!refusal(state, Command::Land, context));
        context.awaiting = {Command::Land};
        QVERIFY(!refusal(state, Command::Land, context));
        state = hovering();
        state.gateway.missionState = MissionState::Preparing;
        context.awaiting = {Command::TrackStart};
        QVERIFY(!refusal(state, Command::TrackStop, context));
        // 화면 전체에 보이는 이유는 안내 줄에 되풀이하지 않는다
        context.awaiting = {Command::Arm};
        const FlightConditions conditions(state, kNow);
        QVERIFY(!CommandPolicy(state, conditions, context).specificRefusal(Command::Disarm, {}));
    }

    // 제어권이 다른 통제기에 있거나 아무도 없으면 제어권 요청만 보낸다 (icd 5.1)
    void controlAuthority() {
        VehicleState state = connected();
        state.gateway.controlSysid = kOtherSysid;
        QVERIFY(refusedWith(refusal(state, Command::Land), "252"));
        QVERIFY(refusedWith(refusal(state, Command::ReleaseControl), "제어권 없음"));
        QVERIFY(!refusal(state, Command::RequestControl));
        state.gateway.controlSysid = 0;
        QVERIFY(refusedWith(refusal(state, Command::Hold), "제어권 요청"));
        state.gateway.controlSysid = kOwnSysid;
        QVERIFY(refusedWith(refusal(state, Command::RequestControl), "이미"));
        QVERIFY(!refusal(state, Command::ReleaseControl));
    }

    // 조종사 개입 중(MANUAL)에는 착륙, 복귀, 정리, 제어권 명령만 (icd 5.4, 6장)
    void pilotOverride() {
        VehicleState state = hovering();
        state.fc.mode = FlightMode::Position;
        state.gateway.missionState = MissionState::Manual;
        QVERIFY(refusedWith(refusal(state, Command::Hold), "개입 중"));
        QVERIFY(refusedWith(refusal(state, Command::Rtl), "개입 중"));
        QVERIFY(!refusal(state, Command::Land));
        QVERIFY(!refusal(state, Command::TrackStop));
        QVERIFY(!refusal(state, Command::EndRcOverride));
    }

    // mavlink_gateway 가 setpoint 를 보내는 중에는 ARM, DISARM, TAKEOFF, RTL 을 막는다 (icd 5.2)
    void setpointSending() {
        VehicleState state = hovering();
        state.gateway.missionState = MissionState::FollowActive;
        QVERIFY(refusedWith(refusal(state, Command::Disarm), "[중단]"));
        QVERIFY(refusedWith(refusal(state, Command::Rtl), "[중단]"));
        state = connected();
        state.fc.mode = FlightMode::Offboard;
        QVERIFY(refusedWith(refusal(state, Command::Arm), "OFFBOARD"));
    }

    // Home 설정, 재부팅은 지상이고 시동이 꺼져 있을 때만 (icd 5.2)
    void groundOnly() {
        VehicleState state = connected();
        QVERIFY(!refusal(state, Command::SetHome));
        CommandContext writing;
        writing.paramWriteActive = true;
        QVERIFY(refusedWith(refusal(state, Command::RebootFc, writing), "파라미터 기록 중"));
        state.fc.armed = true;
        QVERIFY(refusedWith(refusal(state, Command::RebootFc), "ARM 상태"));
        QVERIFY(refusedWith(refusal(state, Command::SetHome), "ARM 상태"));
        const FlightConditions conditions(state, kNow);
        const CommandContext context;
        QVERIFY(CommandPolicy(state, conditions, context).paramWriteRefusal());
        QVERIFY(refusedWith(refusal(connected(), Command::EmergencyLand), "지상"));
        QVERIFY(!refusal(state, Command::EmergencyLand));
    }

    // TAKEOFF 는 고도 범위와 전역 위치가 필요하다 (icd 5.2)
    void takeoffNeedsGlobalPosition() {
        VehicleState state = connected();
        CommandArgs args;
        QVERIFY(refusedWith(refusal(state, Command::Takeoff, {}, args), "전역 위치 없음"));
        state.fc.altAmslM = 100.0F;
        state.fc.globalPositionNs = kNow;
        QVERIFY(!refusal(state, Command::Takeoff, {}, args));
        args.takeoffAltM = std::numeric_limits<float>::quiet_NaN();
        QVERIFY(refusedWith(refusal(state, Command::Takeoff, {}, args), "범위 밖"));
    }

    // ARM 된 채 지상에서 Stabilized 로 바꾸면 가상 조종 Throttle 이 바로 추력이 된다
    void manualModes() {
        VehicleState state = connected();
        QVERIFY(refusedWith(refusal(state, Command::Altitude), "조종기 신호 없음"));
        state.fc.armed = true;
        state.fc.rcPresent = true;
        state.fc.rcHealthy = true;
        state.fc.rcStatusNs = kNow;
        CommandContext context;
        context.manualOn = true;
        context.manualAxes.z = 500;
        QVERIFY(refusedWith(refusal(state, Command::Stabilized, context), "Throttle 50 %"));
        context.manualAxes.z = 0;
        QVERIFY(!refusal(state, Command::Stabilized, context));
        QVERIFY(refusedWith(refusal(state, Command::Position, context), "수평 위치"));
    }

    // 추종 시작: 비행 중 정지, 임무 대기, 추종 가능한 센서, 고른 표적 (icd 4.2)
    void trackStartChecks() {
        VehicleState state = hovering();
        CommandArgs args = selectedTarget();
        QVERIFY(!refusal(state, Command::TrackStart, {}, args));
        args.targetSelected = false;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "표적 선택 필요"));
        args = selectedTarget();
        args.generation = 0;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "메타데이터"));
        args = selectedTarget();
        args.sensor = ARGUS_SENSOR_EO;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "이 센서"));
        args = selectedTarget();
        state.fc.mode = FlightMode::Position;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "정지 상태"));
        state = hovering();
        state.gateway.missionState = MissionState::FollowActive;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "[중단]"));
        state.gateway.missionState = MissionState::Fault;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "대기가 아님"));
        state = hovering();
        state.fc.landedState = MAV_LANDED_STATE_ON_GROUND;
        QVERIFY(refusedWith(refusal(state, Command::TrackStart, {}, args), "비행 중"));
    }

    void trackStopNeedsMission() {
        VehicleState state = hovering();
        QVERIFY(refusedWith(refusal(state, Command::TrackStop), "중단할 임무 없음"));
        state.gateway.missionState = MissionState::ScanActive;
        QVERIFY(!refusal(state, Command::TrackStop));
        state.gateway.missionState = MissionState::Fault;
        QVERIFY(!refusal(state, Command::TrackStop));
    }

    // 스캔: 시작 조건과 매개변수 범위 (icd 4.2). NaN 은 mavlink_gateway 기본값
    void scanChecks() {
        VehicleState state = hovering();
        CommandArgs args;
        QVERIFY(!refusal(state, Command::ScanStart, {}, args));
        args.scanRateRadS = 0.0F;
        QVERIFY(refusedWith(refusal(state, Command::ScanStart, {}, args), "범위 밖"));
        args.scanRangeRad = 3.0F;
        args.scanRateRadS = 0.5F;
        args.scanTimeoutS = 5.0F;  // 3 / 0.5 = 6 s 보다 짧다
        QVERIFY(refusedWith(refusal(state, Command::ScanStart, {}, args), "범위 밖"));
        args.scanTimeoutS = 7.0F;
        QVERIFY(!refusal(state, Command::ScanStart, {}, args));
        state.gateway.missionState = MissionState::HoldWait;
        state.fc.mode = FlightMode::Offboard;
        QVERIFY(!refusal(state, Command::ScanStart, {}, args));
        state.gateway.missionState = MissionState::ScanActive;
        QVERIFY(refusedWith(refusal(state, Command::ScanStart, {}, args), "[중단]"));
    }

    // 통제기 복귀: FC 모드가 Hold 또는 Position (icd 5.4)
    void endRcOverride() {
        VehicleState state = hovering();
        state.gateway.missionState = MissionState::Manual;
        state.fc.mode = FlightMode::Altitude;
        QVERIFY(refusedWith(refusal(state, Command::EndRcOverride), "Hold 또는 Position"));
        state.fc.mode = FlightMode::Position;
        QVERIFY(!refusal(state, Command::EndRcOverride));
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::CommandPolicyTest)
#include "command_policy_test.moc"
