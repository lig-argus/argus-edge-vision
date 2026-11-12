#include "argus/onboard/time_sync.hpp"

#include <cstdint>
#include <optional>

#include "argus/mavlink.h"
#include "argus/onboard/mavlink_ids.hpp"

namespace argus::onboard {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;

}  // namespace

TimeSync::TimeSync(IMavlinkConnection& connection, const core::IClock& clock,
                   IFcEpochSource& epoch_source, TimeSyncConfig config)
    : connection_(connection), clock_(clock), config_(config) {
    timesync_subscription_ =
        connection_.subscribe(MAVLINK_MSG_ID_TIMESYNC,
                              [this](const mavlink_message_t& message) { on_timesync(message); });
    epoch_subscription_ =
        epoch_source.subscribe_epoch_changes([this](std::uint64_t) { invalidate(); });
}

bool TimeSync::send_request() {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    mavlink_message_t message{};
    // tc1 = 0 은 요청을 뜻한다. FC 는 ts1 을 응답에 그대로 넣고 tc1 에 FC 시각(ns)을 넣는다
    mavlink_msg_timesync_pack(kVehicleSysid, kFcObserverCompid, &message, 0, now_ns, kVehicleSysid,
                              kFcCompid);
    if (!connection_.send(message)) {
        return false;
    }
    pending_request_ns_ = now_ns;
    return true;
}

std::optional<std::int64_t> TimeSync::to_monotonic_ns(std::uint32_t fc_time_boot_ms) const {
    if (!valid()) {
        return std::nullopt;
    }
    return static_cast<std::int64_t>(fc_time_boot_ms) * kNsPerMs + offset_ns_.value_or(0);
}

void TimeSync::invalidate() {
    offset_ns_.reset();
    pending_request_ns_.reset();
}

bool TimeSync::valid() const {
    return offset_ns_.has_value() &&
           clock_.now_monotonic_ns() - offset_updated_ns_ <= config_.max_age_ns;
}

void TimeSync::on_timesync(const mavlink_message_t& message) {
    if (!is_from_fc(message)) {
        return;
    }
    mavlink_timesync_t timesync{};
    mavlink_msg_timesync_decode(&message, &timesync);
    if (timesync.tc1 == 0) {
        return;  // FC 가 송신한 요청. 응답이 아님
    }
    // 응답의 target_system, target_component 는 확인하지 않는다. PX4 는 응답에 이 필드를 채우지
    // 않으므로, 이 객체의 요청에 대한 응답인지는 ts1 이 요청의 송신 시각과 같은지로 판단한다
    if (!pending_request_ns_.has_value() || timesync.ts1 != pending_request_ns_.value_or(0)) {
        ++rejected_count_;
        return;
    }
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    const std::int64_t request_ns = pending_request_ns_.value_or(0);
    pending_request_ns_.reset();
    const std::int64_t round_trip_ns = now_ns - request_ns;
    if (round_trip_ns > config_.max_round_trip_ns) {
        ++rejected_count_;
        return;
    }
    // FC 가 응답한 시점을 요청 송신과 응답 수신의 중간 시각으로 추정한다
    offset_ns_ = request_ns + round_trip_ns / 2 - timesync.tc1;
    offset_updated_ns_ = now_ns;
}

}  // namespace argus::onboard
