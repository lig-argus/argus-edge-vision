#include <cassert>

#include "argus/control/contracts/include/follow_b_status.hpp"
#include "argus/runtime/follow/include/follow_status_snapshot.hpp"
using namespace follow;
using namespace std::chrono_literals;
TracksV1 frame(Clock::time_point at, unsigned id, bool detected) {
    TracksV1 f;
    f.source = "camera";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(at.time_since_epoch()).count();
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "host_receive";
    f.image_width = 640;
    f.image_height = 480;
    if (detected) {
        f.tracks.push_back(
            {1, 0, "tank", .9, 100, 100, 160, 180, "CONFIRMED", true, f.sample_monotonic_ns});
    }
    return f;
}
std::string state(const TargetSelectionSnapshot& s, Clock::time_point now) {
    FollowBStatusInput i;
    i.target_selected = s.target.has_value();
    i.target_phase = s.target_phase;
    i.target_confirmed = s.fresh_real_detection;
    i.target_prediction_valid = prediction_report_valid(s, now);
    return project_follow_b_status(i).target;
}
int main() {
    const auto t = Clock::time_point{100s};
    TargetSelectorConfig c;
    c.recovery.enabled = true;
    c.recovery.max_gap = 2s;
    TargetSelector s(c);
    assert(s.update(frame(t, 1, true), t));
    assert(s.execute({"pick", TargetCommandAction::Select, 1, 1}, t).accepted);
    assert(s.execute({"start", TargetCommandAction::Start}, t).accepted);
    const auto real = s.snapshot(t);
    assert(state(real, t) == "CONFIRMED");
    assert(real.observation_valid_until == t + 250ms && !real.prediction_valid_until);
    auto gap = s.snapshot(t + 300ms);
    assert(gap.recovery_pause && !gap.fresh_real_detection);
    assert(!gap.observation_valid_until && gap.prediction_valid_until == t + 2s);
    assert(state(gap, t + 300ms) == "PREDICTED");
    assert(s.update(frame(t + 500ms, 2, false), t + 500ms));
    assert(s.update(frame(t + 1000ms, 3, false), t + 1000ms));
    auto again = s.snapshot(t + 1500ms);
    assert(again.prediction_valid_until == t + 2s);
    assert(!again.observation_valid_until && !again.fresh_real_detection);
    assert(state(again, t + 1500ms) == "PREDICTED");
    assert(prediction_report_valid(again, t + 1999ms));
    assert(!prediction_report_valid(again, t + 2s));
    assert(state(s.snapshot(t + 2s), t + 2s) == "LOST");
    assert(s.execute({"stop", TargetCommandAction::Stop}, t + 2s).accepted);
    assert(state(s.snapshot(t + 2s), t + 2s) == "NONE");

    TargetSelectorConfig legacy_config;
    TargetSelector legacy(legacy_config);
    assert(legacy.update(frame(t, 1, true), t));
    assert(legacy.execute({"pick", TargetCommandAction::Select, 1, 1}, t).accepted);
    assert(legacy.execute({"start", TargetCommandAction::Start}, t).accepted);
    auto prediction = frame(t + 100ms, 2, true);
    prediction.tracks[0].state = "PREDICTED";
    prediction.tracks[0].real_detection = false;
    prediction.tracks[0].last_real_detection_monotonic_ns = frame(t, 1, true).sample_monotonic_ns;
    assert(legacy.update(prediction, t + 100ms));
    const auto legacy_view = legacy.snapshot(t + 100ms);
    assert(prediction_report_valid(legacy_view, t + 100ms));
    assert(!legacy_view.observation_valid_until && legacy_view.prediction_valid_until == t + 350ms);
    assert(!prediction_report_valid(legacy_view, t + 350ms));

    TargetSelector invalidated(c);
    assert(invalidated.update(frame(t, 1, true), t));
    assert(invalidated.execute({"pick", TargetCommandAction::Select, 1, 1}, t).accepted);
    assert(invalidated.execute({"start", TargetCommandAction::Start}, t).accepted);
    invalidated.invalidate_recovery("unsafe_context");
    auto lost = invalidated.snapshot(t + 300ms);
    assert(state(lost, t + 300ms) == "LOST" && !lost.prediction_valid_until);

    TargetSelector restored(c);
    assert(restored.update(frame(t, 1, true), t));
    assert(restored.execute({"pick", TargetCommandAction::Select, 1, 1}, t).accepted);
    assert(restored.execute({"start", TargetCommandAction::Start}, t).accepted);
    restored.advance(t + 300ms);
    for (unsigned j = 0; j < 3; ++j) {
        const auto now = t + 500ms + j * 50ms;
        assert(restored.update(frame(now, j + 2, true), now));
        auto view = restored.snapshot(now);
        assert(state(view, now) == (j < 2 ? "PREDICTED" : "CONFIRMED"));
        if (j < 2) {
            assert(view.prediction_valid_until == t + 2s && !view.observation_valid_until);
        } else {
            assert(!view.prediction_valid_until && view.observation_valid_until == now + 250ms);
        }
    }
}
