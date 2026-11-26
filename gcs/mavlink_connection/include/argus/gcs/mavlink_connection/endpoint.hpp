// 연결 대상 주소 (연결 설정 칸, 실행 인자 --connect). 해석과 주소 형식 확인은 net 모듈의
// parseEndpoint() 가 한다 (주소 형식 확인에 QtNetwork 가 필요하다)
#ifndef ARGUS_GCS_MAVLINK_CONNECTION_ENDPOINT_HPP_
#define ARGUS_GCS_MAVLINK_CONNECTION_ENDPOINT_HPP_

#include <QString>
#include <cstdint>

namespace argus::gcs::mavlink_connection {

struct Endpoint {
    QString address;        // IPv4 또는 IPv6 주소 문자열
    std::uint16_t port{0};  // 1~65535

    bool operator==(const Endpoint& other) const {
        return address == other.address && port == other.port;
    }
    bool operator!=(const Endpoint& other) const { return !(*this == other); }
};

}  // namespace argus::gcs::mavlink_connection

#endif  // ARGUS_GCS_MAVLINK_CONNECTION_ENDPOINT_HPP_
