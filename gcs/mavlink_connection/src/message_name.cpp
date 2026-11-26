// 생성 헤더의 메시지 정보 표(MAVLINK_MESSAGE_INFO)는 MAVLINK_USE_MESSAGE_INFO 를 정의한
// 번역 단위에서만 쓸 수 있다. 표가 커서 이 파일에만 두며, 이 정의는 MAVLink 헤더보다 앞에 둔다.
#include <cstddef>  // 메시지 정보 표가 offsetof 를 쓴다

#define MAVLINK_USE_MESSAGE_INFO
#include "argus/gcs/mavlink_connection/message_name.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::mavlink_connection {

QString messageName(std::uint32_t msgid) {
    const mavlink_message_info_t* info = mavlink_get_message_info_by_id(msgid);
    if (info == nullptr) {
        return QStringLiteral("MSG %1").arg(msgid);
    }
    return QString::fromLatin1(info->name);
}

}  // namespace argus::gcs::mavlink_connection
