#include "argus/gcs/net/endpoint_parser.hpp"

#include <QtTest>

namespace argus::gcs::net {

using mavlink_connection::Endpoint;

class EndpointParserTest : public QObject {
    Q_OBJECT

private slots:
    void parsesIpv4AndIpv6WithPort() {
        const Endpoint rpi =
            parseEndpoint(QStringLiteral(" 192.168.3.99:14550 ")).value_or(Endpoint{});
        QCOMPARE(rpi.address, QStringLiteral("192.168.3.99"));
        QCOMPARE(rpi.port, static_cast<std::uint16_t>(14550));

        const Endpoint loopback6 = parseEndpoint(QStringLiteral("::1:14560")).value_or(Endpoint{});
        QCOMPARE(loopback6.address, QStringLiteral("::1"));
        QCOMPARE(loopback6.port, static_cast<std::uint16_t>(14560));
    }

    void rejectsMalformedText() {
        for (const char* text : {"", "14550", ":14550", "192.168.3.99", "192.168.3.99:",
                                 "192.168.3.99:0", "192.168.3.99:70000", "rpi.local:14550"}) {
            QVERIFY2(!parseEndpoint(QString::fromLatin1(text)), text);
        }
    }
};

}  // namespace argus::gcs::net

QTEST_GUILESS_MAIN(argus::gcs::net::EndpointParserTest)
#include "endpoint_parser_test.moc"
