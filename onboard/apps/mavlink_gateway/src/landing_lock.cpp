#include "argus/mavlink_gateway/landing_lock.hpp"

#include <optional>
#include <utility>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/follow_activity.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {
namespace {

LandAnswer land_answer_of(MAV_RESULT result) {
    switch (result) {
        case MAV_RESULT_IN_PROGRESS:
            return LandAnswer::InProgress;
        case MAV_RESULT_ACCEPTED:
            return LandAnswer::Accepted;
        default:
            return LandAnswer::Rejected;
    }
}

}  // namespace

LandingLock::LandingLock(const LandingLockInputs& inputs) : inputs_(inputs) {
    forwarded_subscription_ = inputs_.forwarded_commands.subscribe(
        [this](const ForwardedCommandEvent& event) { on_forwarded_command(event); });
}

core::Subscription LandingLock::subscribe_started(std::function<void()> handler) {
    return started_handlers_.subscribe(std::move(handler));
}

void LandingLock::tick() {
    LandingLockObservation observation;
    const vehicle_api::VehicleState vehicle = inputs_.fc_state.latest();
    if (vehicle.status.has_value() && vehicle.status->fc_epoch == inputs_.fc_state.fc_epoch()) {
        observation.status = vehicle.status;
    }
    observation.pilot_in_control = inputs_.rc_override.pilot_in_control();
    status_ = on_landing_lock_tick(status_, observation);
}

void LandingLock::on_forwarded_command(const ForwardedCommandEvent& event) {
    if (event.command != MAV_CMD_NAV_LAND) {
        return;
    }
    if (event.stage == ForwardStage::Answered) {
        if (event.result.has_value()) {
            status_ = on_land_answered(status_, land_answer_of(*event.result));
        }
        return;
    }
    const LandingLockStep step = on_land_forwarded(status_, inputs_.clock.now_monotonic_ns());
    status_ = step.status;
    if (step.started) {
        start();
    }
}

void LandingLock::start() {
    inputs_.landing_release.release_for_landing();
    if (follow_in_progress(inputs_.follow_status.latest(), inputs_.clock.now_monotonic_ns())) {
        request_follow_stop();
    }
    started_handlers_.notify();
}

void LandingLock::request_follow_stop() {
    const vehicle_api::RequestHeader header = inputs_.request_headers.next(kLandingStopTimeout);
    stop_request_ = inputs_.follow_control.stop(
        header, vehicle_api::StopReason::Onboard, kLandingStopTimeout,
        [](std::optional<vehicle_api::FollowResult> result) {
            if (!result.has_value()) {
                detail::log_error("착륙 잠금 정지 요청: 추종 제어의 응답 없음");
            } else if (result->outcome == vehicle_api::FollowOutcome::Rejected ||
                       result->outcome == vehicle_api::FollowOutcome::Failed) {
                detail::log_error("착륙 잠금 정지 요청 거부: " + result->reason);
            }
        });
}

}  // namespace argus::mavlink_gateway
