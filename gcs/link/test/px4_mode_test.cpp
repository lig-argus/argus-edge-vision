#include "argus/gcs/link/px4_mode.hpp"

#include <QtTest>

namespace argus::gcs::link {

namespace {

std::uint32_t customMode(std::uint32_t main, std::uint32_t sub) {
    return (main << 16U) | (sub << 24U);
}

}  // namespace

class Px4ModeTest : public QObject {
    Q_OBJECT

private slots:
    void decodesMainModes() {
        QCOMPARE(px4FlightMode(customMode(1, 0)), FlightMode::Manual);
        QCOMPARE(px4FlightMode(customMode(2, 0)), FlightMode::Altitude);
        QCOMPARE(px4FlightMode(customMode(3, 0)), FlightMode::Position);
        QCOMPARE(px4FlightMode(customMode(5, 0)), FlightMode::Acro);
        QCOMPARE(px4FlightMode(customMode(6, 0)), FlightMode::Offboard);
        QCOMPARE(px4FlightMode(customMode(7, 0)), FlightMode::Stabilized);
    }

    void decodesAutoSubModes() {
        QCOMPARE(px4FlightMode(customMode(4, 1)), FlightMode::Ready);
        QCOMPARE(px4FlightMode(customMode(4, 2)), FlightMode::Takeoff);
        QCOMPARE(px4FlightMode(customMode(4, 3)), FlightMode::Hold);
        QCOMPARE(px4FlightMode(customMode(4, 4)), FlightMode::Mission);
        QCOMPARE(px4FlightMode(customMode(4, 5)), FlightMode::Return);
        QCOMPARE(px4FlightMode(customMode(4, 6)), FlightMode::Land);
        QCOMPARE(px4FlightMode(customMode(4, 8)), FlightMode::FollowTarget);
        QCOMPARE(px4FlightMode(customMode(4, 9)), FlightMode::PrecisionLand);
    }

    // 부록 A 에 없는 값은 Unknown
    void unknownValues() {
        QCOMPARE(px4FlightMode(customMode(4, 7)), FlightMode::Unknown);
        QCOMPARE(px4FlightMode(customMode(42, 0)), FlightMode::Unknown);
        QCOMPARE(px4FlightMode(0), FlightMode::Unknown);
    }

    // AUTO 가 아니면 sub mode 를 보지 않는다
    void ignoresSubModeOutsideAuto() {
        QCOMPARE(px4FlightMode(customMode(3, 5)), FlightMode::Position);
    }

    // 화면 이름은 부록 A 의 통제기 화면 이름
    void namesFollowAppendixA() {
        QCOMPARE(flightModeName(FlightMode::Hold), QStringLiteral("Hold"));
        QCOMPARE(flightModeName(FlightMode::Return), QStringLiteral("Return"));
        QCOMPARE(flightModeName(FlightMode::Position), QStringLiteral("Position"));
        QCOMPARE(flightModeName(FlightMode::Offboard), QStringLiteral("Offboard"));
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::Px4ModeTest)
#include "px4_mode_test.moc"
