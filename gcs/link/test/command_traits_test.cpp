// 명령 속성 표 (command_traits) 테스트: 모든 명령에 이름이 있고 스크립트 이름이 겹치지 않으며,
// 대상, 비상, 모드 변경, 스캔 후속 처리 표시가 icd 4.2~4.4, 5.3 과 맞다
#include "argus/gcs/link/command_traits.hpp"

#include <QSet>
#include <QtTest>

namespace argus::gcs::link {

class CommandTraitsTest : public QObject {
    Q_OBJECT

private slots:
    void namesAreCompleteAndUnique() {
        QSet<QString> scriptNames;
        for (const CommandTraits& traits : allCommandTraits()) {
            QVERIFY(!commandName(traits.command).isEmpty());
            const QString script = QString::fromLatin1(traits.scriptName);
            QVERIFY2(!scriptNames.contains(script), traits.scriptName);
            scriptNames.insert(script);
            QCOMPARE(commandFromScriptName(script).value_or(Command::Arm), traits.command);
        }
        QVERIFY(!commandFromScriptName(QStringLiteral("fly")));
        QCOMPARE(commandName(Command::EmergencyLand), QStringLiteral("비상 착륙"));
    }

    void tableFollowsCommandOrder() {
        for (std::size_t i = 0; i < kCommandCount; ++i) {
            QCOMPARE(static_cast<std::size_t>(allCommandTraits().at(i).command), i);
        }
    }

    // 추종, 스캔, 제어권 명령은 mavlink_gateway 로, 비행 명령은 FC 로 (TAKEOFF 포함)
    void targets() {
        for (const CommandTraits& traits : allCommandTraits()) {
            const Command c = traits.command;
            const bool gateway = c == Command::TrackStart || c == Command::TrackStop ||
                                 c == Command::ScanStart || c == Command::RequestControl ||
                                 c == Command::ReleaseControl || c == Command::EndRcOverride;
            QCOMPARE(traits.target, gateway ? CommandTarget::Gateway : CommandTarget::Fc);
        }
    }

    // 응답을 기다리지 않는 것은 착륙뿐이다 (icd 5.3)
    void onlyLandingIsEmergency() {
        for (const CommandTraits& traits : allCommandTraits()) {
            const Command c = traits.command;
            QCOMPARE(traits.emergency, c == Command::Land || c == Command::EmergencyLand);
        }
    }

    void modeChangeAndScanFollowUp() {
        QVERIFY(!commandTraits(Command::Arm).changesMode);
        QVERIFY(commandTraits(Command::Position).changesMode);
        QVERIFY(commandTraits(Command::TrackStop).changesMode);
        QVERIFY(!commandTraits(Command::ScanStart).changesMode);
        for (const CommandTraits& traits : allCommandTraits()) {
            const Command c = traits.command;
            QCOMPARE(traits.dismissesScanResult, c == Command::Hold || c == Command::Land ||
                                                     c == Command::Rtl ||
                                                     c == Command::EmergencyLand);
            QCOMPARE(traits.clearsScanRequest, c == Command::TrackStart || c == Command::TrackStop);
        }
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::CommandTraitsTest)
#include "command_traits_test.moc"
