// MAVLink 메시지 번호를 이름으로 변환한다 (통신 통계 화면, 운용 로그)
#ifndef ARGUS_GCS_MAVLINK_CONNECTION_MESSAGE_NAME_HPP_
#define ARGUS_GCS_MAVLINK_CONNECTION_MESSAGE_NAME_HPP_

#include <QString>
#include <cstdint>

namespace argus::gcs::mavlink_connection {

// 예: 0 → "HEARTBEAT", 45000 → "ARGUS_STATUS". 방언에 없으면 "MSG <번호>"
QString messageName(std::uint32_t msgid);

}  // namespace argus::gcs::mavlink_connection

#endif  // ARGUS_GCS_MAVLINK_CONNECTION_MESSAGE_NAME_HPP_
