#include "argus/mavlink_gateway/heartbeat_sender.hpp"

#include "argus/mavlink.h"
#include "gcs_message_encoding.hpp"

namespace argus::mavlink_gateway {

void HeartbeatSender::tick() {
    const MAV_STATE state =
        health_.is_set(ARGUS_HEALTH_FC_CONNECTED) ? MAV_STATE_ACTIVE : MAV_STATE_CRITICAL;
    const mavlink_message_t message = detail::encode_heartbeat(state);
    // 보낼 통제기가 없거나 송신에 실패해도 다음 주기에 다시 보내므로 따로 처리하지 않는다
    static_cast<void>(link_.broadcast(message));
    static_cast<void>(connection_.send(message));
}

}  // namespace argus::mavlink_gateway
