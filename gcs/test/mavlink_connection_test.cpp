#include <QSignalSpy>
#include <QtTest>

#include "argus/mavlink.h"
#include "fake_mavlink_connection.hpp"

namespace argus::gcs {
namespace {

constexpr quint8 kFcSysid = 1;
constexpr quint8 kOnboardCompid = 195;

mavlink_message_t heartbeatFrom(quint8 sysid, quint8 compid) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(sysid, compid, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

}  // namespace

class MavlinkConnectionTest : public QObject {
    Q_OBJECT

private slots:
    void initTestCase() { qRegisterMetaType<mavlink_message_t>(); }

    void pushedMessageIsEmittedToConnectedSlot() {
        FakeMavlinkConnection connection;
        QVector<quint8> sysids;
        connect(&connection, &IMavlinkConnection::messageReceived, this,
                [&sysids](const mavlink_message_t& m) { sysids.push_back(m.sysid); });

        connection.simulateMessageReceived(heartbeatFrom(kFcSysid, 1));
        connection.simulateMessageReceived(heartbeatFrom(kFcSysid, kOnboardCompid));

        QCOMPARE(sysids, (QVector<quint8>{kFcSysid, kFcSysid}));
    }

    void signalCarriesTheMessage() {
        FakeMavlinkConnection connection;
        QSignalSpy spy(&connection, &IMavlinkConnection::messageReceived);

        connection.simulateMessageReceived(heartbeatFrom(kFcSysid, kOnboardCompid));

        QCOMPARE(spy.count(), 1);
        const auto message = spy.takeFirst().at(0).value<mavlink_message_t>();
        QCOMPARE(message.msgid, static_cast<quint32>(MAVLINK_MSG_ID_HEARTBEAT));
        QCOMPARE(message.compid, kOnboardCompid);
    }

    void destroyedReceiverIsNoLongerCalled() {
        FakeMavlinkConnection connection;
        int received = 0;
        {
            QObject receiver;  // 연결한 객체가 사라지면 Qt 가 연결을 끊는다 (Subscription 역할)
            connect(&connection, &IMavlinkConnection::messageReceived, &receiver,
                    [&received](const mavlink_message_t&) { ++received; });
            connection.simulateMessageReceived(heartbeatFrom(kFcSysid, 1));
        }
        connection.simulateMessageReceived(heartbeatFrom(kFcSysid, 1));

        QCOMPARE(received, 1);
    }

    void recordsSentMessagesAndResult() {
        FakeMavlinkConnection connection;

        QVERIFY(connection.send(heartbeatFrom(250, 190)));
        connection.setSendResult(false);
        QVERIFY(!connection.send(heartbeatFrom(250, 190)));

        QCOMPARE(connection.sent().size(), 2);
        QCOMPARE(connection.sent()[0].sysid, static_cast<quint8>(250));
    }
};

}  // namespace argus::gcs

QTEST_MAIN(argus::gcs::MavlinkConnectionTest)
#include "mavlink_connection_test.moc"
