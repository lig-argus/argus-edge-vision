#include "argus/onboard/fc_link_tracker.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "argus/mavlink.h"

namespace argus::onboard {

FcLinkTracker::FcLinkTracker(const core::IClock& clock, HealthState& health)
    : clock_(clock), health_(health) {}

std::int64_t FcLinkTracker::on_fc_message() {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    // check_timeout() 이 호출되기 전에 메시지가 도착한 경우에도 두절 후의 메시지로 처리한다
    const bool reconnected = !connected_ || timed_out(now_ns);
    last_received_ns_ = now_ns;
    if (reconnected) {
        connected_ = true;
        advance_epoch();
        health_.set(ARGUS_HEALTH_FC_CONNECTED, true);
        epoch_handlers_.call(fc_epoch_);
    }
    return now_ns;
}

std::int64_t FcLinkTracker::on_fc_message(FcTimedMessage kind, std::uint32_t time_boot_ms) {
    const std::int64_t now_ns = on_fc_message();
    const auto index = static_cast<std::size_t>(kind);
    const std::optional<std::uint32_t> previous = last_time_boot_ms_.at(index);
    if (previous.has_value() && time_boot_ms < previous.value_or(0)) {
        advance_epoch();
        epoch_handlers_.call(fc_epoch_);
    }
    last_time_boot_ms_.at(index) = time_boot_ms;
    return now_ns;
}

bool FcLinkTracker::connected() const {
    return connected_ && !timed_out(clock_.now_monotonic_ns());
}

void FcLinkTracker::check_timeout() {
    if (connected_ && timed_out(clock_.now_monotonic_ns())) {
        connected_ = false;
        health_.set(ARGUS_HEALTH_FC_CONNECTED, false);
    }
}

core::Subscription FcLinkTracker::subscribe_epoch_changes(EpochHandler handler) {
    return epoch_handlers_.add(std::move(handler));
}

bool FcLinkTracker::timed_out(std::int64_t now_ns) const {
    return now_ns - last_received_ns_ > kFcLinkTimeoutNs;
}

void FcLinkTracker::advance_epoch() {
    ++fc_epoch_;
    // 이전 fc_epoch 의 time_boot_ms 와 비교하지 않도록 비운다. 재부팅 후 다른 종류의 첫 메시지가
    // fc_epoch 를 다시 증가시키지 않는다
    last_time_boot_ms_ = {};
}

}  // namespace argus::onboard
