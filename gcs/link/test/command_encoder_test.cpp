// 명령 값 만들기 (command_encoder) 테스트: 대상, MAV_CMD, param1~7 (icd 4.1~4.4, 부록 A)
#include "argus/gcs/link/command_encoder.hpp"

#include <QtTest>
#include <cmath>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::uint32_t kNumber = 1234;

EncodeInputs numbered() {
    EncodeInputs in;
    in.commandNumber = kNumber;
    return in;
}

}  // namespace

class CommandEncoderTest : public QObject {
    Q_OBJECT

private slots:
    // mavlink_gateway 로 보내는 명령은 모두 p7 이 명령 번호이고 빈 칸은 NaN (icd 4.1)
    void gatewayCommandsCarryNumber() {
        for (const Command command :
             {Command::TrackStart, Command::TrackStop, Command::ScanStart, Command::RequestControl,
              Command::ReleaseControl, Command::EndRcOverride}) {
            const EncodedCommand out = encodeCommand(command, {}, numbered());
            QCOMPARE(out.target, CommandTarget::Gateway);
            QCOMPARE(out.params.at(6), static_cast<float>(kNumber));
            QVERIFY(std::isnan(out.params.at(4)));
            QVERIFY(std::isnan(out.params.at(5)));
        }
        EncodeInputs wrapped;
        wrapped.commandNumber = kCommandNumberModulus + 5;
        QCOMPARE(encodeCommand(Command::TrackStop, {}, wrapped).params.at(6), 5.0F);
    }

    // FC 명령은 명령 번호를 쓰지 않는다. 빈 칸은 0
    void fcCommandsIgnoreNumber() {
        const EncodedCommand arm = encodeCommand(Command::Arm, {}, numbered());
        QCOMPARE(arm.target, CommandTarget::Fc);
        QCOMPARE(arm.mavCmd, std::uint16_t{MAV_CMD_COMPONENT_ARM_DISARM});
        QCOMPARE(arm.params.at(0), 1.0F);
        QCOMPARE(arm.params.at(6), 0.0F);
        QCOMPARE(encodeCommand(Command::Disarm, {}, numbered()).params.at(0), 0.0F);
    }

    // HOLD 는 DO_SET_MODE AUTO(4).LOITER(3), 수동 모드는 main mode 만 (부록 A)
    void modesUseCustomMode() {
        const EncodedCommand hold = encodeCommand(Command::Hold, {}, {});
        QCOMPARE(hold.target, CommandTarget::Fc);
        QCOMPARE(hold.mavCmd, std::uint16_t{MAV_CMD_DO_SET_MODE});
        QCOMPARE(hold.params.at(0), 1.0F);
        QCOMPARE(hold.params.at(1), 4.0F);
        QCOMPARE(hold.params.at(2), 3.0F);
        QCOMPARE(encodeCommand(Command::Stabilized, {}, {}).params.at(1), 7.0F);
        QCOMPARE(encodeCommand(Command::Altitude, {}, {}).params.at(1), 2.0F);
        const EncodedCommand position = encodeCommand(Command::Position, {}, {});
        QCOMPARE(position.params.at(1), 3.0F);
        QCOMPARE(position.params.at(2), 0.0F);
    }

    // TAKEOFF 는 FC 로 NAV_TAKEOFF, p7 은 해발 고도 + 입력 고도, yaw 는 NaN (지금 방향)
    void takeoffUsesAmsl() {
        CommandArgs args;
        args.takeoffAltM = 3.0F;
        EncodeInputs in;
        in.altAmslM = 100.0F;
        const EncodedCommand out = encodeCommand(Command::Takeoff, args, in);
        QCOMPARE(out.target, CommandTarget::Fc);
        QCOMPARE(out.mavCmd, std::uint16_t{MAV_CMD_NAV_TAKEOFF});
        QCOMPARE(out.params.at(6), 103.0F);
        QVERIFY(std::isnan(out.params.at(3)));
        QCOMPARE(out.notes.size(), std::size_t{1});
    }

    // 수평 위치가 없으면 LAND 전에 제자리 하강을 알린다. 비상 착륙도 같은 NAV_LAND
    void landWithoutPositionNotes() {
        EncodeInputs in;
        const EncodedCommand land = encodeCommand(Command::Land, {}, in);
        QCOMPARE(land.mavCmd, std::uint16_t{MAV_CMD_NAV_LAND});
        QCOMPARE(land.notes.size(), std::size_t{1});
        in.horizontalPositionKnown = true;
        QVERIFY(encodeCommand(Command::EmergencyLand, {}, in).notes.empty());
        QCOMPARE(encodeCommand(Command::EmergencyLand, {}, in).mavCmd,
                 std::uint16_t{MAV_CMD_NAV_LAND});
    }

    void groundCommands() {
        const EncodedCommand home = encodeCommand(Command::SetHome, {}, {});
        QCOMPARE(home.mavCmd, std::uint16_t{MAV_CMD_DO_SET_HOME});
        QCOMPARE(home.params.at(0), 1.0F);
        const EncodedCommand reboot = encodeCommand(Command::RebootFc, {}, {});
        QCOMPARE(reboot.mavCmd, std::uint16_t{MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN});
        QCOMPARE(reboot.params.at(0), 1.0F);
        QCOMPARE(reboot.params.at(1), 0.0F);
        QCOMPARE(encodeCommand(Command::Rtl, {}, {}).mavCmd,
                 std::uint16_t{MAV_CMD_NAV_RETURN_TO_LAUNCH});
    }

    // TRACK_START: p1 track_id, p2 센서, p3 class_id, p4 generation (icd 4.2)
    void trackStartCarriesIdentity() {
        CommandArgs args;
        args.trackId = 7;
        args.sensor = ARGUS_SENSOR_IR;
        args.classId = 2;
        args.generation = 3;
        const EncodedCommand out = encodeCommand(Command::TrackStart, args, numbered());
        QCOMPARE(out.mavCmd, std::uint16_t{MAV_CMD_ARGUS_TRACK_START});
        QCOMPARE(out.params.at(0), 7.0F);
        QCOMPARE(out.params.at(1), static_cast<float>(ARGUS_SENSOR_IR));
        QCOMPARE(out.params.at(2), 2.0F);
        QCOMPARE(out.params.at(3), 3.0F);
    }

    // SCAN_START: p1 범위, p2 속도, p3 제한 시간. NaN 은 그대로 (mavlink_gateway 기본값)
    void scanStartCarriesArgs() {
        CommandArgs args;
        args.scanRangeRad = 3.0F;
        args.scanRateRadS = -0.5F;
        const EncodedCommand out = encodeCommand(Command::ScanStart, args, numbered());
        QCOMPARE(out.mavCmd, std::uint16_t{MAV_CMD_ARGUS_SCAN_START});
        QCOMPARE(out.params.at(0), 3.0F);
        QCOMPARE(out.params.at(1), -0.5F);
        QVERIFY(std::isnan(out.params.at(2)));
        QVERIFY(std::isnan(out.params.at(3)));
    }

    // 제어권: p1 1 요청, 0 해제, p4 이 통제기의 sysid, p2 p3 NaN (icd 4.4)
    void controlCommands() {
        EncodeInputs in = numbered();
        in.gcsSysid = 252;
        const EncodedCommand request = encodeCommand(Command::RequestControl, {}, in);
        QCOMPARE(request.mavCmd, std::uint16_t{MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL});
        QCOMPARE(request.params.at(0), 1.0F);
        QVERIFY(std::isnan(request.params.at(1)));
        QVERIFY(std::isnan(request.params.at(2)));
        QCOMPARE(request.params.at(3), 252.0F);
        const EncodedCommand release = encodeCommand(Command::ReleaseControl, {}, in);
        QCOMPARE(release.mavCmd, std::uint16_t{MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL});
        QCOMPARE(release.params.at(0), 0.0F);
        const EncodedCommand back = encodeCommand(Command::EndRcOverride, {}, in);
        QCOMPARE(back.mavCmd, std::uint16_t{MAV_CMD_ARGUS_END_RC_OVERRIDE});
        QVERIFY(std::isnan(back.params.at(0)));
    }

    void mavCmdMatchesEncoder() {
        for (std::size_t i = 0; i < kCommandCount; ++i) {
            const auto command = static_cast<Command>(i);
            QCOMPARE(encodeCommand(command, {}, {}).mavCmd, mavCmdOf(command));
            QVERIFY(mavCmdOf(command) != 0);
        }
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::CommandEncoderTest)
#include "command_encoder_test.moc"
