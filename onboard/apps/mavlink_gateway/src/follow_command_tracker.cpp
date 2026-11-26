#include "argus/mavlink_gateway/follow_command_tracker.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "argus/mavlink_gateway/follow_activity.hpp"
#include "log.hpp"

namespace argus::mavlink_gateway {
namespace {

// Succeeded, Failed, Rejected 의 COMMAND_ACK. InProgress 이면 nullopt
std::optional<CommandAck> final_ack(vehicle_api::FollowOutcome outcome, const std::string& reason) {
    switch (outcome) {
        case vehicle_api::FollowOutcome::Succeeded:
            return CommandAck{MAV_RESULT_ACCEPTED, kProgressUnknown, ""};
        case vehicle_api::FollowOutcome::Failed:
            return CommandAck{MAV_RESULT_FAILED, kProgressUnknown,
                              reason.empty() ? "추종 제어 실행 실패" : reason};
        case vehicle_api::FollowOutcome::Rejected:
            detail::log_error("추종 제어 거부: " + reason);
            return CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown, ""};
        case vehicle_api::FollowOutcome::InProgress:
            break;
    }
    return std::nullopt;
}

}  // namespace

void send_rejection(CommandResponder& responder, const CommandRejection& rejection) {
    if (rejection.result == MAV_RESULT_DENIED) {
        responder.send(CommandAck{rejection.result, kProgressUnknown, rejection.reason});
        return;
    }
    detail::log_error("통제기 명령 거부: " + rejection.reason);
    responder.send(CommandAck{rejection.result, kProgressUnknown, ""});
}

FollowCommandTracker::FollowCommandTracker(const core::IClock& clock,
                                           const vehicle_api::IFollowStatusSource& status_source,
                                           vehicle_api::IFollowStatusEvents& status_events,
                                           std::string onboard_session)
    : clock_(clock),
      status_source_(status_source),
      onboard_session_(std::move(onboard_session)),
      status_subscription_(status_events.subscribe(
          [this](const vehicle_api::ReceivedFollowStatus& received) { on_status(received); })) {}

void FollowCommandTracker::track(vehicle_api::CommandId command_id,
                                 std::optional<double> scan_yaw_range_rad,
                                 CommandResponder responder, const SendRequest& send) {
    erase_completed();
    Entry& entry = entries_[command_id.value];
    entry = Entry{};
    entry.responder = std::move(responder);
    entry.scan_yaw_range_rad = scan_yaw_range_rad;
    entry.progress = scan_yaw_range_rad.has_value() ? 0 : kProgressUnknown;
    entry.responder.send(CommandAck{MAV_RESULT_IN_PROGRESS, entry.progress, ""});
    const std::uint64_t id = command_id.value;
    entry.reply = send(
        [this, id](const std::optional<vehicle_api::FollowResult>& reply) { on_reply(id, reply); });
}

void FollowCommandTracker::tick() {
    erase_completed();
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    for (auto& [command_id, entry] : entries_) {
        if (!entry.accepted_ns.has_value()) {
            continue;  // 응답 대기 시간은 IFollowControlClient 가 확인한다
        }
        const std::int64_t last_ns = std::max(entry.accepted_ns.value(),
                                              last_status_ns_.value_or(entry.accepted_ns.value()));
        if (now_ns - last_ns > kFollowStatusTimeoutNs) {
            entry.responder.send(
                CommandAck{MAV_RESULT_FAILED, kProgressUnknown, "추종 제어 상태 두절"});
        }
    }
    erase_completed();
}

void FollowCommandTracker::on_reply(std::uint64_t command_id,
                                    const std::optional<vehicle_api::FollowResult>& reply) {
    const auto found = entries_.find(command_id);
    if (found == entries_.end()) {
        return;
    }
    Entry& entry = found->second;
    if (!reply.has_value()) {
        entry.responder.send(
            CommandAck{MAV_RESULT_FAILED, kProgressUnknown, "추종 제어 응답 없음"});
        return;
    }
    const std::optional<CommandAck> ack = final_ack(reply->outcome, reply->reason);
    if (ack.has_value()) {
        entry.responder.send(ack.value());
        return;
    }
    entry.accepted_ns = clock_.now_monotonic_ns();
    // 응답보다 먼저 도착한 follow/status 에 결과가 있을 수 있다
    const std::optional<vehicle_api::ReceivedFollowStatus> latest = status_source_.latest();
    if (latest.has_value()) {
        apply_status(command_id, entry, latest->status);
    }
}

void FollowCommandTracker::on_status(const vehicle_api::ReceivedFollowStatus& received) {
    last_status_ns_ = received.arrived_monotonic_ns;
    for (auto& [command_id, entry] : entries_) {
        if (entry.accepted_ns.has_value()) {
            apply_status(command_id, entry, received.status);
        }
    }
    erase_completed();
}

void FollowCommandTracker::apply_status(std::uint64_t command_id, Entry& entry,
                                        const vehicle_api::FollowStatus& status) {
    if (status.onboard_session != onboard_session_) {
        return;
    }
    const auto found = std::find_if(status.command_results.begin(), status.command_results.end(),
                                    [command_id](const vehicle_api::FollowCommandResult& result) {
                                        return result.command_id.value == command_id;
                                    });
    if (found == status.command_results.end()) {
        return;
    }
    const std::optional<CommandAck> ack = final_ack(found->outcome, found->reason);
    if (ack.has_value()) {
        entry.responder.send(ack.value());
        return;
    }
    if (!entry.scan_yaw_range_rad.has_value() || !status.scan.has_value()) {
        return;
    }
    const std::uint8_t progress =
        scan_progress_percent(status.scan->progress_deg, entry.scan_yaw_range_rad.value());
    if (progress != entry.progress) {
        entry.progress = progress;
        entry.responder.send(CommandAck{MAV_RESULT_IN_PROGRESS, progress, ""});
    }
}

void FollowCommandTracker::erase_completed() {
    for (auto it = entries_.begin(); it != entries_.end();) {
        if (it->second.responder.completed()) {
            it = entries_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace argus::mavlink_gateway
