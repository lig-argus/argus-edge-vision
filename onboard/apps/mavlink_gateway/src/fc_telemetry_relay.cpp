#include "argus/mavlink_gateway/fc_telemetry_relay.hpp"

#include <cstdint>

#include "argus/mavlink_gateway/mavlink_ids.hpp"

namespace argus::mavlink_gateway {

FcTelemetryRelay::FcTelemetryRelay(IMavlinkConnection& connection, IGcsLink& link) {
    subscriptions_.reserve(kRelayedFcMessages.size());
    IGcsLink* const target = &link;
    for (const std::uint32_t msgid : kRelayedFcMessages) {
        subscriptions_.push_back(
            connection.subscribe(msgid, [target](const mavlink_message_t& message) {
                if (is_from_fc(message)) {
                    // 보낼 통제기가 없으면 false 이며 따로 처리하지 않는다
                    static_cast<void>(target->broadcast(message));
                }
            }));
    }
}

}  // namespace argus::mavlink_gateway
