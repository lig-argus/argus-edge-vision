#include "argus/video_sender/frame_collector.hpp"

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

namespace argus::video_sender {
namespace {

// 비어 있는 session_id 는 값이 없는 것으로 처리한다
std::optional<std::string_view> session_of(const std::string& session_id) {
    if (session_id.empty()) {
        return std::nullopt;
    }
    return std::string_view{session_id};
}

std::optional<std::string_view> session_of(const std::optional<std::string>& session_id) {
    if (!session_id.has_value()) {
        return std::nullopt;
    }
    return session_of(session_id.value());
}

// 양쪽 모두 값이 있을 때만 비교한다
bool sessions_differ(const std::optional<std::string>& stored,
                     std::optional<std::string_view> incoming) {
    return stored.has_value() && incoming.has_value() && stored.value() != incoming.value();
}

}  // namespace

FrameCollector::FrameCollector(const core::IClock& clock, FrameCollectorConfig config)
    : clock_(clock),
      max_wait_ns_(std::chrono::nanoseconds{config.max_wait}.count()),
      max_pending_(std::max<std::size_t>(config.max_pending, 1)) {}

void FrameCollector::on_frame(perception_api::Frame frame) {
    discard_other_sessions(frame);
    PendingFrame& pending =
        find_or_add_pending(frame.source, session_of(frame.session_id), frame.frame_id);
    pending.frame = std::move(frame);
    release_ready();
}

void FrameCollector::on_observations(perception_api::Observations observations) {
    PendingFrame& pending =
        find_or_add_pending(observations.source, std::nullopt, observations.frame_id);
    pending.observations = std::move(observations);
    release_ready();
}

void FrameCollector::on_tracks(perception_api::Tracks tracks) {
    PendingFrame& pending =
        find_or_add_pending(tracks.source, session_of(tracks.session_id), tracks.frame_id);
    pending.tracks = std::move(tracks);
    release_ready();
}

void FrameCollector::on_session_change() {
    pending_.clear();
    frame_session_by_source_.clear();
}

void FrameCollector::process_timeouts() {
    // pending_ 은 생성된 순서이므로 맨 앞이 가장 오래된 PendingFrame 이다
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    while (!pending_.empty() &&
           now_ns - pending_.front().first_arrival_monotonic_ns >= max_wait_ns_) {
        release_through(pending_.begin());
    }
}

core::Subscription FrameCollector::subscribe(Handler handler) {
    return handlers_.subscribe(std::move(handler));
}

FrameCollector::PendingFrame& FrameCollector::find_or_add_pending(
    std::string_view source, std::optional<std::string_view> session_id, std::uint64_t frame_id) {
    const auto found = std::find_if(pending_.begin(), pending_.end(), [&](const PendingFrame& p) {
        return p.frame_id == frame_id && p.source == source &&
               !sessions_differ(p.session_id, session_id);
    });
    PendingFrame& pending = found != pending_.end() ? *found : pending_.emplace_back();
    if (found == pending_.end()) {
        pending.source = std::string{source};
        pending.frame_id = frame_id;
        pending.first_arrival_monotonic_ns = clock_.now_monotonic_ns();
    }
    if (!pending.session_id.has_value() && session_id.has_value()) {
        pending.session_id = std::string{session_id.value()};
    }
    return pending;
}

void FrameCollector::discard_other_sessions(const perception_api::Frame& frame) {
    if (frame.session_id.empty()) {
        return;
    }
    const auto [known, inserted] =
        frame_session_by_source_.try_emplace(frame.source, frame.session_id);
    if (inserted || known->second == frame.session_id) {
        return;
    }
    known->second = frame.session_id;
    pending_.remove_if([&](const PendingFrame& p) {
        return p.source == frame.source && p.session_id != frame.session_id;
    });
}

void FrameCollector::release_ready() {
    release_complete();
    process_timeouts();
    release_over_limit();
}

void FrameCollector::release_complete() {
    const auto is_complete = [](const PendingFrame& p) {
        return p.frame.has_value() && p.observations.has_value() && p.tracks.has_value();
    };
    auto complete = std::find_if(pending_.begin(), pending_.end(), is_complete);
    while (complete != pending_.end()) {
        release_through(complete);
        complete = std::find_if(pending_.begin(), pending_.end(), is_complete);
    }
}

void FrameCollector::release_over_limit() {
    while (pending_.size() > max_pending_) {
        release_through(pending_.begin());
    }
}

// last 와, 같은 source 의 PendingFrame 중 last 보다 먼저 생성된 것을 생성된 순서대로 처리한다.
// handler 가 on_*() 를 다시 호출해도 안전하도록 pending_ 에서 먼저 제거한 뒤 출력한다
void FrameCollector::release_through(PendingList::iterator last) {
    const std::string source = last->source;
    const auto end = std::next(last);
    std::vector<PendingFrame> released;
    for (auto it = pending_.begin(); it != end;) {
        if (it->source == source) {
            released.push_back(std::move(*it));
            it = pending_.erase(it);
        } else {
            ++it;
        }
    }
    for (PendingFrame& pending : released) {
        publish(std::move(pending));
    }
}

// 프레임이 도착하지 않은 PendingFrame 은 출력하지 않는다 (process_interfaces.md 5.3)
void FrameCollector::publish(PendingFrame pending) {
    if (!pending.frame.has_value()) {
        return;
    }
    const CollectedFrame collected{std::move(pending.frame).value(),
                                   std::move(pending.observations), std::move(pending.tracks)};
    handlers_.notify(collected);
}

}  // namespace argus::video_sender
