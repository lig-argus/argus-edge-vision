#include "argus/gcs/net/udp_mavlink_connection.hpp"

#include <QNetworkDatagram>
#include <QSignalSpy>
#include <QtTest>
#include <algorithm>
#include <array>
#include <vector>

#include "argus/gcs/core_testing/fake_clock.hpp"

namespace argus::gcs::net {

using core_testing::FakeClock;
using mavlink_connection::Endpoint;
using mavlink_connection::IMavlinkConnection;
using mavlink_connection::senderKey;

namespace {

constexpr std::uint8_t kGcsSysid = 251;
constexpr std::uint8_t kGatewaySysid = 1;
constexpr std::uint8_t kGatewayCompid = 195;
constexpr std::int32_t kWaitMs = 1000;
constexpr std::int32_t kQuietMs = 100;

// mavlink_gateway 역할: 받은 데이터그램을 MAVLink 메시지로 해석한다
std::vector<mavlink_message_t> receiveMessages(QUdpSocket& gateway, QNetworkDatagram* last) {
    std::vector<mavlink_message_t> messages;
    mavlink_message_t partial{};
    mavlink_status_t status{};
    while (gateway.hasPendingDatagrams()) {
        const QNetworkDatagram datagram = gateway.receiveDatagram();
        if (last != nullptr) {
            *last = datagram;
        }
        for (const char byte : datagram.data()) {
            mavlink_message_t message{};
            mavlink_status_t frameStatus{};
            if (mavlink_frame_char_buffer(&partial, &status, static_cast<std::uint8_t>(byte),
                                          &message, &frameStatus) == MAVLINK_FRAMING_OK) {
                messages.push_back(message);
            }
        }
    }
    return messages;
}

QByteArray frame(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    QByteArray bytes(static_cast<qsizetype>(size), Qt::Uninitialized);
    std::copy_n(buffer.begin(), size, bytes.begin());
    return bytes;
}

}  // namespace

class UdpMavlinkConnectionTest : public QObject {
    Q_OBJECT

private slots:
    void initTestCase() { qRegisterMetaType<mavlink_message_t>(); }

    // HEARTBEAT 를 1 Hz 로 보낸다 (통제기 sysid, compid 190, MAV_TYPE_GCS)
    void sendsHeartbeatEverySecond() {
        FakeClock clock;
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));
        UdpMavlinkConnection connection(kGcsSysid, clock);
        QString error;
        QVERIFY(connection.open(error));
        connection.setTarget(Endpoint{QStringLiteral("127.0.0.1"), gateway.localPort()});

        connection.tick();
        QVERIFY(gateway.waitForReadyRead(kWaitMs));
        std::vector<mavlink_message_t> messages = receiveMessages(gateway, nullptr);
        QCOMPARE(messages.size(), static_cast<std::size_t>(1));
        QCOMPARE(messages[0].msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
        QCOMPARE(messages[0].sysid, kGcsSysid);
        QCOMPARE(messages[0].compid, UdpMavlinkConnection::kCompid);
        QCOMPARE(mavlink_msg_heartbeat_get_type(&messages[0]),
                 static_cast<std::uint8_t>(MAV_TYPE_GCS));

        clock.advanceNs(UdpMavlinkConnection::kHeartbeatPeriodNs - 1);
        connection.tick();
        QVERIFY(!gateway.waitForReadyRead(kQuietMs));

        clock.advanceNs(1);
        connection.tick();
        QVERIFY(gateway.waitForReadyRead(kWaitMs));
        QCOMPARE(receiveMessages(gateway, nullptr).size(), static_cast<std::size_t>(1));
    }

    // 메시지를 만든 쪽의 sysid, compid, seq 와 무관하게 이 연결의 값으로 보내고, seq 는 1 씩
    // 증가한다
    void sendsWithOwnIdentityAndSequence() {
        FakeClock clock;
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));
        UdpMavlinkConnection connection(kGcsSysid, clock);
        QString error;
        QVERIFY(connection.open(error));
        connection.setTarget(Endpoint{QStringLiteral("127.0.0.1"), gateway.localPort()});

        mavlink_message_t command{};
        mavlink_msg_command_long_pack(7, 7, &command, kGatewaySysid, kGatewayCompid,
                                      MAV_CMD_ARGUS_TRACK_STOP, 0, NAN, NAN, NAN, NAN, NAN, NAN,
                                      12.0F);
        QVERIFY(connection.send(command));
        QVERIFY(connection.send(command));
        QVERIFY(gateway.waitForReadyRead(kWaitMs));
        QTest::qWait(kQuietMs);
        const std::vector<mavlink_message_t> messages = receiveMessages(gateway, nullptr);

        QCOMPARE(messages.size(), static_cast<std::size_t>(2));
        for (const mavlink_message_t& message : messages) {
            QCOMPARE(message.sysid, kGcsSysid);
            QCOMPARE(message.compid, UdpMavlinkConnection::kCompid);
            QCOMPARE(message.magic, static_cast<std::uint8_t>(MAVLINK_STX));  // MAVLink 2
            QCOMPARE(mavlink_msg_command_long_get_command(&message),
                     static_cast<std::uint16_t>(MAV_CMD_ARGUS_TRACK_STOP));
            QCOMPARE(mavlink_msg_command_long_get_param7(&message), 12.0F);
        }
        QCOMPARE(static_cast<std::uint8_t>(messages[1].seq - messages[0].seq),
                 static_cast<std::uint8_t>(1));
    }

    void rejectsMessageOutsideDialect() {
        FakeClock clock;
        UdpMavlinkConnection connection(kGcsSysid, clock);
        QString error;
        QVERIFY(connection.open(error));
        connection.setTarget(Endpoint{QStringLiteral("127.0.0.1"), 14550});
        mavlink_message_t message{};
        message.msgid = 441;  // 방언에 없는 번호

        QVERIFY(!connection.send(message));
    }

    // 상대가 보낸 메시지를 signal 로 전달하고 송신자별 통계에 반영한다
    void receivesMessagesAndCountsStatistics() {
        FakeClock clock;
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));
        UdpMavlinkConnection connection(kGcsSysid, clock);
        QString error;
        QVERIFY(connection.open(error));
        connection.setDiscovery(gateway.localPort());
        QSignalSpy received(&connection, &IMavlinkConnection::messageReceived);

        mavlink_message_t heartbeat{};
        mavlink_msg_heartbeat_pack(kGatewaySysid, kGatewayCompid, &heartbeat,
                                   MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0,
                                   MAV_STATE_ACTIVE);
        gateway.writeDatagram(frame(heartbeat), QHostAddress::LocalHost, connection.localPort());
        QVERIFY(received.wait(kWaitMs));

        const auto message = received.at(0).at(0).value<mavlink_message_t>();
        QCOMPARE(message.compid, kGatewayCompid);
        QCOMPARE(connection.peer(), std::optional<Endpoint>(Endpoint{QStringLiteral("127.0.0.1"),
                                                                     gateway.localPort()}));
        QCOMPARE(
            connection.statistics().senders.at(senderKey(kGatewaySysid, kGatewayCompid)).received,
            1U);
    }
};

}  // namespace argus::gcs::net

QTEST_GUILESS_MAIN(argus::gcs::net::UdpMavlinkConnectionTest)
#include "udp_mavlink_connection_test.moc"
