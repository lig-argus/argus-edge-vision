#include "argus/mavlink_gateway/gcs_registry.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "gcs_message_encoding.hpp"

namespace argus::mavlink_gateway {

GcsRegistry::GcsRegistry(IGcsLink& link, const core::IClock& clock) : link_(link), clock_(clock) {
    heartbeat_subscription_ = link_.subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [this](const mavlink_message_t& message, const GcsPeer& sender) {
            on_heartbeat(message, sender);
        });
}

core::Subscription GcsRegistry::subscribe(std::uint32_t msgid, Handler handler) {
    // HEARTBEAT 는 on_heartbeat 가 기록을 갱신한 뒤 넘긴다
    if (msgid != MAVLINK_MSG_ID_HEARTBEAT && link_subscriptions_.count(msgid) == 0) {
        link_subscriptions_.emplace(
            msgid,
            link_.subscribe(msgid, [this](const mavlink_message_t& message, const GcsPeer& sender) {
                if (recognized(sender)) {
                    handlers_.dispatch(message, sender);
                }
            }));
    }
    return handlers_.add(msgid, std::move(handler));
}

std::optional<std::int64_t> GcsRegistry::last_heartbeat_ns(std::uint8_t sysid) const {
    const auto found = records_.find(sysid);
    if (found == records_.end()) {
        return std::nullopt;
    }
    return found->second.last_heartbeat_ns;
}

std::optional<GcsPeer> GcsRegistry::peer(std::uint8_t sysid) const {
    const auto found = records_.find(sysid);
    if (found == records_.end()) {
        return std::nullopt;
    }
    return found->second.peer;
}

void GcsRegistry::on_heartbeat(const mavlink_message_t& message, const GcsPeer& sender) {
    if (!is_gcs_sysid(sender.sysid) || sender.compid != kGcsCompid ||
        mavlink_msg_heartbeat_get_type(&message) != MAV_TYPE_GCS) {
        return;
    }
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    const auto found = records_.find(sender.sysid);
    if (found == records_.end()) {
        records_.emplace(sender.sysid, Record{sender, now_ns});
    } else {
        if (!found->second.peer.same_address(sender)) {
            warn_duplicate(found->second.peer, sender);
        }
        found->second = Record{sender, now_ns};
    }
    handlers_.dispatch(message, sender);
}

bool GcsRegistry::recognized(const GcsPeer& sender) const {
    return sender.compid == kGcsCompid && records_.count(sender.sysid) != 0;
}

void GcsRegistry::warn_duplicate(const GcsPeer& recorded, const GcsPeer& sender) {
    const mavlink_message_t warning = detail::encode_statustext(
        MAV_SEVERITY_WARNING, "통제기 sysid " + std::to_string(sender.sysid) + " 중복");
    link_.send_to(recorded, warning);
    link_.send_to(sender, warning);
}

}  // namespace argus::mavlink_gateway
