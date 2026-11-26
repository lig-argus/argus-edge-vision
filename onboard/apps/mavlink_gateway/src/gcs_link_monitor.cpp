#include "argus/mavlink_gateway/gcs_link_monitor.hpp"

#include <cstdint>
#include <optional>
#include <string>

#include "argus/mavlink_gateway/control_authority_policy.hpp"
#include "argus/mavlink_gateway/follow_activity.hpp"
#include "argus/vehicle_api/types.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {

void GcsLinkMonitor::tick() {
    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();
    const std::optional<std::uint8_t> controller = inputs_.authority.controller();
    std::optional<std::int64_t> last_heartbeat_ns;
    if (controller.has_value()) {
        last_heartbeat_ns = inputs_.presence.last_heartbeat_ns(*controller);
    }
    const bool link_ok =
        last_heartbeat_ns.has_value() && now_ns - *last_heartbeat_ns <= kGcsHeartbeatTimeoutNs;
    const bool lost = link_ok_ && !link_ok && controller.has_value();
    link_ok_ = link_ok;
    inputs_.health.set(ARGUS_HEALTH_GCS_LINK, link_ok);
    if (lost && follow_in_progress(inputs_.follow_status.latest(), now_ns)) {
        request_stop();
    }
}

void GcsLinkMonitor::request_stop() {
    const vehicle_api::RequestHeader header = inputs_.request_headers.next(kLinkLossStopTimeout);
    stop_request_ = inputs_.follow_control.stop(
        header, vehicle_api::StopReason::LinkLoss, kLinkLossStopTimeout,
        [](std::optional<vehicle_api::FollowResult> result) {
            if (!result.has_value()) {
                detail::log_error("통제기 두절 정지 요청: 추종 제어의 응답 없음");
            } else if (result->outcome == vehicle_api::FollowOutcome::Rejected ||
                       result->outcome == vehicle_api::FollowOutcome::Failed) {
                detail::log_error("통제기 두절 정지 요청 거부: " + result->reason);
            }
        });
}

}  // namespace argus::mavlink_gateway
