#include "argus/gcs/link/vehicle_link.hpp"

#include <QSignalSpy>
#include <QtTest>
#include <cmath>
#include <vector>

#include "argus/gcs/core_testing/fake_clock.hpp"
#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/mavlink_connection_testing/fake_mavlink_connection.hpp"

namespace argus::gcs::link {

using core_testing::FakeClock;
using mavlink_connection_testing::FakeMavlinkConnection;

namespace {

constexpr std::uint8_t kSysid = 251;
constexpr std::uint32_t kFirstNumber = 1000;
constexpr std::uint32_t kOffboardMode = 6U << 16U;
constexpr std::uint32_t kPositionMode = 3U << 16U;

mavlink_message_t fcHeartbeat(bool armed, std::uint32_t customMode) {
    mavlink_message_t message{};
    const auto baseMode = static_cast<std::uint8_t>(
        static_cast<std::uint32_t>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) |
        (armed ? static_cast<std::uint32_t>(MAV_MODE_FLAG_SAFETY_ARMED) : 0U));
    mavlink_msg_heartbeat_pack(kVehicleSysid, kFcCompid, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_PX4, baseMode, customMode, MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t gatewayHeartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(kVehicleSysid, kGatewayCompid, &message, MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t argusStatus(std::uint8_t controlSysid, std::uint8_t missionState) {
    mavlink_message_t message{};
    mavlink_msg_argus_status_pack(kVehicleSysid, kGatewayCompid, &message, 0, 45.0F, 0, 0,
                                  ARGUS_HEALTH_FC_CONNECTED, missionState, ARGUS_TARGET_STATE_NONE,
                                  ARGUS_SENSOR_IR, controlSysid, NAN, NAN, 0);
    return message;
}

mavlink_message_t ack(std::uint8_t compid, std::uint16_t command, std::uint8_t result) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(kVehicleSysid, compid, &message, command, result, 255, 0, kSysid,
                                 kGcsCompid);
    return message;
}

mavlink_message_t statusText(std::uint8_t compid, const char* text) {
    mavlink_message_t message{};
    mavlink_msg_statustext_pack(kVehicleSysid, compid, &message, MAV_SEVERITY_WARNING, text, 0, 0);
    return message;
}

std::vector<mavlink_command_long_t> commandsTo(const FakeMavlinkConnection& connection,
                                               std::uint8_t compid) {
    std::vector<mavlink_command_long_t> commands;
    for (const mavlink_message_t& message : connection.sent()) {
        if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
            continue;
        }
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&message, &command);
        if (command.target_component == compid) {
            commands.push_back(command);
        }
    }
    return commands;
}

struct Fixture {
    FakeMavlinkConnection connection;
    FakeClock clock;
    VehicleLink link{connection, clock, kFirstNumber};
    std::vector<LogEntry> logs;

    Fixture() {
        link.setGcsSysid(kSysid);
        QObject::connect(&link, &VehicleLink::logged,
                         [this](const LogEntry& entry) { logs.push_back(entry); });
    }

    // FC 와 mavlink_gateway 가 연결되고, 이 통제기가 제어권을 가진 상태
    void connectWithControl() {
        connection.simulateMessageReceived(fcHeartbeat(false, kPositionMode));
        connection.simulateMessageReceived(gatewayHeartbeat());
        connection.simulateMessageReceived(argusStatus(kSysid, ARGUS_MISSION_STATE_IDLE));
    }
};

}  // namespace

class VehicleLinkTest : public QObject {
    Q_OBJECT

private slots:
    void initTestCase() { qRegisterMetaType<mavlink_message_t>(); }

    void rejectsSysidOutsideGcsRange() {
        Fixture fixture;
        QVERIFY(!fixture.link.setGcsSysid(1));
        QVERIFY(!fixture.link.setGcsSysid(254));
        QCOMPARE(fixture.link.gcsSysid(), kSysid);
    }

    // 다른 시스템(QGC, 다른 통제기)의 메시지는 상태와 링크에 쓰지 않는다
    void ignoresOtherSystems() {
        Fixture fixture;
        mavlink_message_t other{};
        mavlink_msg_heartbeat_pack(255, kFcCompid, &other, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0,
                                   kOffboardMode, MAV_STATE_ACTIVE);
        fixture.connection.simulateMessageReceived(other);

        QCOMPARE(fixture.link.state().fcLink.state, LinkState::Never);
        QCOMPARE(fixture.link.state().fc.mode, FlightMode::Unknown);
    }

    void decodesStateAndLinks() {
        Fixture fixture;
        fixture.connectWithControl();

        const VehicleState& state = fixture.link.state();
        QCOMPARE(state.fcLink.state, LinkState::Ok);
        QCOMPARE(state.gatewayLink.state, LinkState::Ok);
        QCOMPARE(state.fc.mode, FlightMode::Position);
        QVERIFY(state.gateway.received);
        QCOMPARE(state.gateway.controlSysid, kSysid);
    }

    // mavlink_gateway 가 연결되면 영상 접속 정보를 요청한다 (video.md)
    void requestsVideoStreamsWhenGatewayConnects() {
        Fixture fixture;
        fixture.connection.simulateMessageReceived(gatewayHeartbeat());
        fixture.link.tick();

        const std::vector<mavlink_command_long_t> requests =
            commandsTo(fixture.connection, kGatewayCompid);
        QCOMPARE(requests.size(), static_cast<std::size_t>(1));
        QCOMPARE(requests[0].command, static_cast<std::uint16_t>(MAV_CMD_REQUEST_MESSAGE));
        QCOMPARE(requests[0].param1, static_cast<float>(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION));
    }

    // argus 명령은 param7 에 명령 번호를 넣어 mavlink_gateway 로 보내고, 응답으로 끝난다 (icd 4.1)
    void sendsGatewayCommandWithNumberAndFinishesOnAck() {
        Fixture fixture;
        fixture.connectWithControl();
        fixture.connection.simulateMessageReceived(
            argusStatus(kSysid, ARGUS_MISSION_STATE_FOLLOW_ACTIVE));

        QVERIFY(!fixture.link.request(Command::TrackStop));
        std::vector<mavlink_command_long_t> commands =
            commandsTo(fixture.connection, kGatewayCompid);
        QCOMPARE(commands.back().command, static_cast<std::uint16_t>(MAV_CMD_ARGUS_TRACK_STOP));
        QCOMPARE(commands.back().param7, static_cast<float>(kFirstNumber));
        QVERIFY(fixture.link.awaitingReply(Command::TrackStop));

        fixture.connection.simulateMessageReceived(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_STOP, MAV_RESULT_ACCEPTED));
        QCOMPARE(fixture.link.commandTrace(Command::TrackStop).stage,
                 CommandTrace::Stage::Accepted);

        QVERIFY(!fixture.link.request(Command::TrackStop));
        commands = commandsTo(fixture.connection, kGatewayCompid);
        QCOMPARE(commands.back().param7, static_cast<float>(kFirstNumber + 1));
    }

    // 다른 통제기가 제어권을 가지면 보내지 않고 이유를 기록한다 (icd 5.1)
    void refusesWithoutControl() {
        Fixture fixture;
        fixture.connectWithControl();
        fixture.connection.simulateMessageReceived(argusStatus(252, ARGUS_MISSION_STATE_IDLE));
        const auto sentBefore = fixture.connection.sent().size();

        const std::optional<QString> why = fixture.link.request(Command::TrackStop);
        QVERIFY(why);
        QCOMPARE(fixture.connection.sent().size(), sentBefore);
        QCOMPARE(fixture.logs.back().level, LogLevel::Serious);
        QVERIFY(!fixture.link.request(Command::RequestControl));  // 제어권 요청은 보낸다
    }

    // 기체가 보낸 문구는 출처를 붙여 기록하고 알림 대상이 아니다
    void logsStatusTextWithOrigin() {
        Fixture fixture;
        fixture.connection.simulateMessageReceived(statusText(kGatewayCompid, "control taken"));

        const auto found =
            std::find_if(fixture.logs.begin(), fixture.logs.end(),
                         [](const LogEntry& entry) { return entry.origin == LogOrigin::Gateway; });
        QVERIFY(found != fixture.logs.end());
        QCOMPARE(found->kind, LogKind::Gateway);
        QVERIFY(!needsNotification(*found));
    }

    // 운용자 명령 없이 Offboard 를 벗어나면 경고한다
    void warnsOnOffboardExitWithoutCommand() {
        Fixture fixture;
        fixture.connection.simulateMessageReceived(fcHeartbeat(true, kOffboardMode));
        fixture.logs.clear();

        fixture.connection.simulateMessageReceived(fcHeartbeat(true, kPositionMode));
        const auto found =
            std::find_if(fixture.logs.begin(), fixture.logs.end(), [](const LogEntry& entry) {
                return entry.kind == LogKind::Fc && entry.level == LogLevel::Serious;
            });
        QVERIFY(found != fixture.logs.end());
    }

    void storesDetectionsAndAnswersFrameQueries() {
        Fixture fixture;
        QSignalSpy added(&fixture.link, &VehicleLink::detectionAdded);
        DetectionFrame frame;
        frame.frameId = 100;
        frame.generation = 1;
        frame.source = QStringLiteral("camera-ir");

        QVERIFY(fixture.link.addDetection(ARGUS_SENSOR_IR, frame));
        QCOMPARE(added.count(), 1);
        QVERIFY(fixture.link.detectionForFrame(ARGUS_SENSOR_IR, 100));
        QVERIFY(!fixture.link.detectionForFrame(ARGUS_SENSOR_IR, 101));
    }

    // 링크가 두절되면 tick 이 상태를 바꾼다 (3 s)
    void tickDetectsLinkLoss() {
        Fixture fixture;
        fixture.connectWithControl();
        fixture.clock.advanceNs(kLinkLostNs);
        fixture.link.tick();

        QCOMPARE(fixture.link.state().fcLink.state, LinkState::Lost);
        QVERIFY(fixture.link.state().routerSuspect);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::VehicleLinkTest)
#include "vehicle_link_test.moc"
