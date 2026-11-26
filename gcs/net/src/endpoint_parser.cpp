#include "argus/gcs/net/endpoint_parser.hpp"

#include <QHostAddress>
#include <cstdint>

namespace argus::gcs::net {

std::optional<mavlink_connection::Endpoint> parseEndpoint(const QString& text) {
    const QString trimmed = text.trimmed();
    // IPv6 주소에도 ':' 가 있으므로 마지막 ':' 뒤를 포트로 본다
    const auto colon = trimmed.lastIndexOf(QLatin1Char(':'));
    if (colon <= 0) {
        return std::nullopt;
    }
    const QHostAddress address(trimmed.left(colon));
    bool ok = false;
    // toUShort 는 65535 를 넘으면 실패한다 (더 큰 형으로 읽어 자르면 70000 이 4464 가 된다)
    const std::uint16_t port = trimmed.mid(colon + 1).toUShort(&ok);
    if (address.isNull() || !ok || port == 0) {
        return std::nullopt;
    }
    return mavlink_connection::Endpoint{address.toString(), port};
}

}  // namespace argus::gcs::net
