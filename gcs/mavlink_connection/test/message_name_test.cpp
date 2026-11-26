#include "argus/gcs/mavlink_connection/message_name.hpp"

#include <QtTest>

#include "argus/mavlink.h"

namespace argus::gcs::mavlink_connection {

class MessageNameTest : public QObject {
    Q_OBJECT

private slots:
    void namesStandardAndArgusMessages() {
        QCOMPARE(messageName(MAVLINK_MSG_ID_HEARTBEAT), QStringLiteral("HEARTBEAT"));
        QCOMPARE(messageName(MAVLINK_MSG_ID_ARGUS_STATUS), QStringLiteral("ARGUS_STATUS"));
    }

    void unknownMessageShowsNumber() { QCOMPARE(messageName(441), QStringLiteral("MSG 441")); }
};

}  // namespace argus::gcs::mavlink_connection

QTEST_GUILESS_MAIN(argus::gcs::mavlink_connection::MessageNameTest)
#include "message_name_test.moc"
