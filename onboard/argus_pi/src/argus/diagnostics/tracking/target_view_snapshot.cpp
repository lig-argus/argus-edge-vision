#include "argus/diagnostics/tracking/include/target_view_snapshot.hpp"

#include <stdexcept>
#include <utility>
namespace follow {
TargetViewSnapshot build_target_view_snapshot(TargetSelectionSnapshot selection,
                                              Clock::time_point now,
                                              std::chrono::milliseconds max_age) {
    if (max_age.count() <= 0) {
        throw std::invalid_argument("target view maximum age must be positive");
    }
    TargetViewSnapshot view;
    view.selection = std::move(selection);
    view.published_at = now;
    const auto& state = view.selection;
    if (!state.latest_frame) {
        return view;
    }
    const auto& frame = *state.latest_frame;
    const auto sampled = Clock::time_point{std::chrono::nanoseconds(frame.sample_monotonic_ns)};
    if (frame.clock_domain != "host_monotonic" || now < sampled) {
        return view;
    }
    view.source_age_s = std::chrono::duration<double>(now - sampled).count();
    view.frame_available = now - sampled <= max_age;
    if (!view.frame_available) {
        return view;  // 기억된 대상과 과거 박스를 새 현재 객체로 합치지 않는다.
    }
    view.objects.reserve(frame.tracks.size());
    for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
        const auto& track = frame.tracks[i];
        const bool selected = state.target && state.target->generation == frame.generation &&
                              state.target->track_id == track.track_id &&
                              state.phase != TargetSelectionSnapshot::Phase::Stopped;
        const bool usable_binding =
            selected && state.mission_target_id > 0 && state.target_phase != TargetPhase::Lost;
        const bool confirmed =
            usable_binding && state.phase == TargetSelectionSnapshot::Phase::Following &&
            state.target_phase == TargetPhase::Locked && !state.recovery_pause &&
            state.fresh_real_detection && track.state == "CONFIRMED" && track.real_detection &&
            track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns;
        view.objects.push_back(
            {i,
             usable_binding ? std::optional<std::uint64_t>(state.mission_target_id) : std::nullopt,
             selected, confirmed});
    }
    return view;
}
}  // namespace follow
