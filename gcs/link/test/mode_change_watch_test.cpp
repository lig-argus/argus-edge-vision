#include "argus/gcs/link/mode_change_watch.hpp"

#include <QtTest>
#include <vector>

namespace argus::gcs::link {

namespace {

FcState fcState(bool armed, FlightMode mode) {
    FcState state;
    state.armed = armed;
    state.mode = mode;
    return state;
}

}  // namespace

class ModeChangeWatchTest : public QObject {
    Q_OBJECT

private slots:
    void warnsWhenOffboardEndsWithoutOperatorCommand() {
        std::vector<LogEntry> logs;
        ModeChangeWatch watch([&logs](const LogEntry& entry) { logs.push_back(entry); });

        watch.check(fcState(true, FlightMode::Offboard), fcState(true, FlightMode::Position), 0);
        QCOMPARE(logs.size(), static_cast<std::size_t>(1));
        QCOMPARE(logs[0].level, LogLevel::Serious);
        QVERIFY(logs[0].text.contains(QStringLiteral("Position")));
    }

    // 운용자가 모드 명령을 보낸 뒤 5 s 안의 이탈은 경고하지 않는다
    void silentAfterOperatorModeCommand() {
        std::vector<LogEntry> logs;
        ModeChangeWatch watch([&logs](const LogEntry& entry) { logs.push_back(entry); });
        watch.noteOperatorModeCommand(0);

        watch.check(fcState(true, FlightMode::Offboard), fcState(true, FlightMode::Hold),
                    ModeChangeWatch::kOperatorWindowNs);
        QVERIFY(logs.empty());
        watch.check(fcState(true, FlightMode::Offboard), fcState(true, FlightMode::Hold),
                    ModeChangeWatch::kOperatorWindowNs + 1);
        QCOMPARE(logs.size(), static_cast<std::size_t>(1));
    }

    void warnsWhenDisarmedInOffboard() {
        std::vector<LogEntry> logs;
        ModeChangeWatch watch([&logs](const LogEntry& entry) { logs.push_back(entry); });

        watch.check(fcState(true, FlightMode::Offboard), fcState(false, FlightMode::Offboard), 0);
        QCOMPARE(logs.size(), static_cast<std::size_t>(1));
        watch.check(fcState(false, FlightMode::Offboard), fcState(false, FlightMode::Offboard), 0);
        QCOMPARE(logs.size(), static_cast<std::size_t>(1));  // 이미 알린 상태는 다시 알리지 않는다
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::ModeChangeWatchTest)
#include "mode_change_watch_test.moc"
