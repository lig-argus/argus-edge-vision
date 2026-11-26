#include "argus/gcs/link/manual_control.hpp"

#include <QtTest>
#include <vector>

#include "argus/gcs/mavlink_connection_testing/fake_mavlink_connection.hpp"

namespace argus::gcs::link {

using mavlink_connection_testing::FakeMavlinkConnection;

namespace {

constexpr std::int64_t kMs = kNsPerMs;

mavlink_manual_control_t decode(const mavlink_message_t& message) {
    mavlink_manual_control_t control{};
    mavlink_msg_manual_control_decode(&message, &control);
    return control;
}

}  // namespace

class ManualControlTest : public QObject {
    Q_OBJECT

private slots:
    void refusesWithoutFcOrWithPhysicalRc() {
        QVERIFY(ManualControl::refusal(false, false));
        QVERIFY(ManualControl::refusal(true, true));
        QVERIFY(!ManualControl::refusal(true, false));
    }

    // 켜면 바로 한 번 보내고, 그 뒤 50 ms 마다 보낸다 (20 Hz)
    void sendsAxesAtTwentyHertz() {
        FakeMavlinkConnection connection;
        ManualControl control(connection, [](const LogEntry&) {});
        ManualAxes axes;
        axes.x = 300;
        axes.r = -200;
        control.setAxes(axes);

        QVERIFY(!control.enable(std::nullopt, 0));
        QCOMPARE(connection.sent().size(), 1);
        control.tick(std::nullopt, 49 * kMs);
        QCOMPARE(connection.sent().size(), 1);
        control.tick(std::nullopt, 50 * kMs);
        QCOMPARE(connection.sent().size(), 2);

        const mavlink_manual_control_t sent = decode(connection.sent().back());
        QCOMPARE(sent.x, static_cast<std::int16_t>(300));
        QCOMPARE(sent.r, static_cast<std::int16_t>(-200));
        QCOMPARE(sent.z, static_cast<std::int16_t>(500));
        QCOMPARE(sent.target, kVehicleSysid);
    }

    void doesNotEnableWhenRefused() {
        FakeMavlinkConnection connection;
        ManualControl control(connection, [](const LogEntry&) {});

        QVERIFY(control.enable(QStringLiteral("FC 미연결"), 0));
        QVERIFY(!control.enabled());
        QCOMPARE(connection.sent().size(), 0);
    }

    // 켤 수 없는 조건이 생기면 스스로 끄고 심각 기록을 남긴다
    void turnsOffWhenRefusalAppears() {
        FakeMavlinkConnection connection;
        std::vector<LogEntry> logs;
        ManualControl control(connection,
                              [&logs](const LogEntry& entry) { logs.push_back(entry); });
        control.enable(std::nullopt, 0);

        control.tick(QStringLiteral("FC 미연결"), 50 * kMs);
        QVERIFY(!control.enabled());
        QCOMPARE(connection.sent().size(), 1);
        QCOMPARE(logs.back().level, LogLevel::Serious);
    }

    void logsLateSend() {
        FakeMavlinkConnection connection;
        std::vector<LogEntry> logs;
        ManualControl control(connection,
                              [&logs](const LogEntry& entry) { logs.push_back(entry); });
        control.enable(std::nullopt, 0);
        logs.clear();

        control.tick(std::nullopt, 250 * kMs);
        QCOMPARE(logs.size(), static_cast<std::size_t>(1));
        QCOMPARE(logs[0].level, LogLevel::Serious);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::ManualControlTest)
#include "manual_control_test.moc"
