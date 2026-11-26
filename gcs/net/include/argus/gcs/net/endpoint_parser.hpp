// "IP:포트" 문자열을 연결 대상으로 해석한다 (연결 설정 칸, 실행 인자 --connect)
#ifndef ARGUS_GCS_NET_ENDPOINT_PARSER_HPP_
#define ARGUS_GCS_NET_ENDPOINT_PARSER_HPP_

#include <QString>
#include <optional>

#include "argus/gcs/mavlink_connection/endpoint.hpp"

namespace argus::gcs::net {

// "192.168.3.99:14550" 처럼 IPv4 또는 IPv6 주소와 1~65535 포트. 앞뒤 공백은 무시한다.
// 호스트 이름은 받지 않는다. 형식이 틀리면 없음. 주소는 정규화한 문자열로 돌려준다
std::optional<mavlink_connection::Endpoint> parseEndpoint(const QString& text);

}  // namespace argus::gcs::net

#endif  // ARGUS_GCS_NET_ENDPOINT_PARSER_HPP_
