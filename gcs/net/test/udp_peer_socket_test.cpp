#include "argus/gcs/net/udp_peer_socket.hpp"

#include <QNetworkDatagram>
#include <QSignalSpy>
#include <QtTest>

#include "argus/gcs/core_testing/fake_clock.hpp"

namespace argus::gcs::net {

using core_testing::FakeClock;
using mavlink_connection::Endpoint;

namespace {

constexpr std::int32_t kWaitMs = 1000;
constexpr std::int32_t kQuietMs = 100;

Endpoint loopback(const QUdpSocket& socket) {
    return Endpoint{QStringLiteral("127.0.0.1"), socket.localPort()};
}

}  // namespace

class UdpPeerSocketTest : public QObject {
    Q_OBJECT

private slots:
    // 연결 대상을 정하기 전에는 받은 데이터그램을 폐기하고 송신하지 않는다
    void silentUntilTargetChosen() {
        FakeClock clock;
        UdpPeerSocket socket(clock);
        QString error;
        QVERIFY2(socket.open(error), qPrintable(error));
        QSignalSpy received(&socket, &UdpPeerSocket::datagramReceived);
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));

        gateway.writeDatagram(QByteArrayLiteral("hb"), QHostAddress::LocalHost, socket.localPort());
        QTest::qWait(kQuietMs);
        QCOMPARE(received.count(), 0);
        QVERIFY(!socket.peer());
        QVERIFY(!socket.send(QByteArrayLiteral("hb")));
        QVERIFY(!gateway.waitForReadyRead(kQuietMs));
    }

    // 지정한 상대와 주고받고, 상대가 아닌 주소의 데이터그램은 폐기한다
    void exchangesWithTargetOnly() {
        FakeClock clock;
        UdpPeerSocket socket(clock);
        QString error;
        QVERIFY(socket.open(error));
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));
        socket.setTarget(loopback(gateway));
        QCOMPARE(socket.peer(), std::optional<Endpoint>(loopback(gateway)));

        QVERIFY(socket.send(QByteArrayLiteral("hb")));
        QVERIFY(gateway.waitForReadyRead(kWaitMs));
        const QNetworkDatagram request = gateway.receiveDatagram();
        QCOMPARE(request.data(), QByteArrayLiteral("hb"));

        QSignalSpy received(&socket, &UdpPeerSocket::datagramReceived);
        QUdpSocket stranger;
        stranger.writeDatagram(QByteArrayLiteral("x"), request.senderAddress(),
                               request.senderPort());
        gateway.writeDatagram(QByteArrayLiteral("reply"), request.senderAddress(),
                              request.senderPort());
        QVERIFY(received.wait(kWaitMs));
        QTest::qWait(kQuietMs);
        QCOMPARE(received.count(), 1);
        QCOMPARE(received.at(0).at(0).toByteArray(), QByteArrayLiteral("reply"));
    }

    // 자동 탐색: 최초 응답 주소를 상대로 저장하고, 수신이 없으면 시간 초과 뒤 폐기한다
    void discoveryAdoptsFirstResponderAndTimesOut() {
        FakeClock clock;
        UdpPeerSocket socket(clock);
        QString error;
        QVERIFY(socket.open(error));
        QUdpSocket gateway;
        QVERIFY(gateway.bind(QHostAddress::LocalHost, 0));
        socket.setDiscovery(gateway.localPort());
        QSignalSpy received(&socket, &UdpPeerSocket::datagramReceived);

        gateway.writeDatagram(QByteArrayLiteral("hb"), QHostAddress::LocalHost, socket.localPort());
        QVERIFY(received.wait(kWaitMs));
        QCOMPARE(socket.peer(), std::optional<Endpoint>(loopback(gateway)));

        clock.advanceNs(UdpPeerSocket::kPeerTimeoutNs);
        socket.checkPeerTimeout();
        QVERIFY(socket.peer());  // 시간 초과와 같은 시간은 유지
        clock.advanceNs(1);
        socket.checkPeerTimeout();
        QVERIFY(!socket.peer());
    }

    // 지정한 상대는 수신이 없어도 폐기하지 않는다 (운용자가 고른 주소)
    void targetIsKeptWithoutTraffic() {
        FakeClock clock;
        UdpPeerSocket socket(clock);
        QString error;
        QVERIFY(socket.open(error));
        const Endpoint target{QStringLiteral("127.0.0.1"), 14550};
        socket.setTarget(target);

        clock.advanceNs(UdpPeerSocket::kPeerTimeoutNs * 2);
        socket.checkPeerTimeout();
        QCOMPARE(socket.peer(), std::optional<Endpoint>(target));
    }
};

}  // namespace argus::gcs::net

QTEST_GUILESS_MAIN(argus::gcs::net::UdpPeerSocketTest)
#include "udp_peer_socket_test.moc"
