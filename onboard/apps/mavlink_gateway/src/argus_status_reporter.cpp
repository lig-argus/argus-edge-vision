#include "argus/mavlink_gateway/argus_status_reporter.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/vehicle_api/types.hpp"
#include "gcs_message_encoding.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {
namespace {

// 바로 송신하는 기준 필드 (icd 3.1)
bool key_fields_differ(const mavlink_argus_status_t& lhs, const mavlink_argus_status_t& rhs) {
    return lhs.mission_state != rhs.mission_state || lhs.target_state != rhs.target_state ||
           lhs.health != rhs.health || lhs.active_sensor != rhs.active_sensor ||
           lhs.control_flags != rhs.control_flags;
}

}  // namespace

ArgusStatusReporter::ArgusStatusReporter(const ArgusStatusReporterInputs& inputs)
    : inputs_(inputs), started_ns_(inputs.clock.now_monotonic_ns()) {
    follow_subscription_ = inputs_.follow_events.subscribe(
        [this](const vehicle_api::ReceivedFollowStatus& /*received*/) { evaluate(false); });
    health_subscription_ = inputs_.health.subscribe_changes([this] { evaluate(false); });
}

void ArgusStatusReporter::tick() {
    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();
    const std::int64_t period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kArgusStatusPeriod).count();
    evaluate(!last_sent_.has_value() || now_ns - last_sent_ns_ >= period_ns);
}

StatusObservation ArgusStatusReporter::observe() {
    StatusObservation observation;
    observation.now_ns = inputs_.clock.now_monotonic_ns();
    observation.started_ns = started_ns_;
    observation.follow = inputs_.follow_status.latest();
    const std::optional<std::int64_t> timeout_ns =
        inputs_.fc_command_status.last_setpoint_timeout_ns();
    if (timeout_ns != cleared_timeout_ns_) {
        observation.setpoint_timeout_ns = timeout_ns;
    }
    observation.controller = inputs_.authority.controller();
    observation.health = inputs_.health.value();
    observation.cpu_celsius = inputs_.cpu_temperature.celsius();
    observation.landing_locked = inputs_.landing_lock.active();
    observation.pilot_in_control = inputs_.rc_override.pilot_in_control();
    return observation;
}

void ArgusStatusReporter::evaluate(bool force) {
    StatusObservation observation = observe();
    if (observation.setpoint_timeout_ns.has_value() && setpoint_timeout_cleared(observation)) {
        cleared_timeout_ns_ = observation.setpoint_timeout_ns;
        observation.setpoint_timeout_ns.reset();
    }

    const FollowFault fault = follow_fault(observation);
    if (fault == FollowFault::StatusLost && last_fault_ != FollowFault::StatusLost &&
        was_following(observation.follow)) {
        if (!inputs_.stop_requester.stop_from_onboard()) {
            detail::log_error("추종 제어 두절 정지 요청 실패");
        }
    }
    last_fault_ = fault;

    const mavlink_argus_status_t status = compose_argus_status(observation);
    if (!force && last_sent_.has_value() && !key_fields_differ(status, *last_sent_)) {
        return;
    }
    inputs_.link.broadcast(detail::encode_argus_status(status));
    last_sent_ = status;
    last_sent_ns_ = observation.now_ns;
}

}  // namespace argus::mavlink_gateway
