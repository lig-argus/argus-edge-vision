#include "argus/gcs/link/link_monitor.hpp"

#include <QtTest>
#include <vector>

#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

namespace {

constexpr std::int64_t kMs = kNsPerMs;

mavlink_message_t heartbeat(std::uint8_t sysid, std::uint8_t compid) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(sysid, compid, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

}  // namespace

class LinkMonitorTest : public QObject {
    Q_OBJECT

private slots:
    void neverUntilFirstMessage() {
        LinkMonitor monitor([](const LogEntry&) {});
        VehicleState state;
        const LinkMonitor::Connected connected = monitor.update(state, 5'000 * kMs);

        QCOMPARE(state.fcLink.state, LinkState::Never);
        QVERIFY(!connected.fc);
    }

    // 기대 주기의 두 배(2 s)부터 지연, 3 s 부터 두절 (icd 2장, 6장)
    void delayedAtTwicePeriodAndLostAtThreeSeconds() {
        std::vector<LogEntry> logs;
        LinkMonitor monitor([&logs](const LogEntry& entry) { logs.push_back(entry); });
        VehicleState state;
        monitor.onMessage(heartbeat(kVehicleSysid, kFcCompid), 0);

        QVERIFY(monitor.update(state, 0).fc);
        QCOMPARE(state.fcLink.state, LinkState::Ok);
        monitor.update(state, 1'999 * kMs);
        QCOMPARE(state.fcLink.state, LinkState::Ok);
        monitor.update(state, 2'000 * kMs);
        QCOMPARE(state.fcLink.state, LinkState::Delayed);
        monitor.update(state, 3'000 * kMs);
        QCOMPARE(state.fcLink.state, LinkState::Lost);
        QCOMPARE(logs.back().level, LogLevel::Critical);

        monitor.onMessage(heartbeat(kVehicleSysid, kFcCompid), 4'000 * kMs);
        QVERIFY(monitor.update(state, 4'000 * kMs).fc);  // 복구도 연결됨으로 알린다
        QCOMPARE(state.fcLink.state, LinkState::Ok);
    }

    void routerSuspectWhenBothLost() {
        std::vector<LogEntry> logs;
        LinkMonitor monitor([&logs](const LogEntry& entry) { logs.push_back(entry); });
        VehicleState state;
        monitor.onMessage(heartbeat(kVehicleSysid, kFcCompid), 0);
        monitor.onMessage(heartbeat(kVehicleSysid, kGatewayCompid), 0);
        monitor.update(state, 0);

        monitor.update(state, 3'000 * kMs);
        QVERIFY(state.routerSuspect);
        QCOMPARE(logs.back().kind, LogKind::Gcs);
    }

    // 다른 시스템(QGC, 다른 통제기)의 메시지는 링크 판단에 쓰지 않는다
    void ignoresOtherSystems() {
        LinkMonitor monitor([](const LogEntry&) {});
        VehicleState state;
        monitor.onMessage(heartbeat(255, 190), 0);
        monitor.update(state, 0);

        QCOMPARE(state.fcLink.state, LinkState::Never);
        QCOMPARE(state.gatewayLink.state, LinkState::Never);
    }

    // 수신률: 처음 10 s 는 그동안의 기대 수로, 그 뒤는 10 s 구간으로 나눈다
    void qualityCountsHeartbeatsInWindow() {
        LinkMonitor monitor([](const LogEntry&) {});
        VehicleState state;
        for (std::int64_t second = 0; second < 4; ++second) {
            monitor.onMessage(heartbeat(kVehicleSysid, kGatewayCompid), second * kNsPerSecond);
        }
        monitor.update(state, 4 * kNsPerSecond);
        QCOMPARE(state.gatewayLink.qualityPct, 100);

        monitor.update(state, 8 * kNsPerSecond);  // 8 s 동안 4 개
        QCOMPARE(state.gatewayLink.qualityPct, 50);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::LinkMonitorTest)
#include "link_monitor_test.moc"
