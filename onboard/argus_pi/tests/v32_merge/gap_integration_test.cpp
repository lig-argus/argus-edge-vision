#include <cassert>
#include <cmath>
#include <cstdlib>

#include "argus/control/target/include/target_selector.hpp"
using namespace follow;
using namespace std::chrono_literals;
void env(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}
Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{1000000000LL + std::llround(seconds * 1e9)}};
}
TracksV1 input(unsigned id, double seconds, bool detected) {
    TracksV1 f;
    f.schema = kTracksV4Schema;
    f.source = "v32-gap-contract";
    f.generation = 1;
    f.frame_id = id;
    f.sample_monotonic_ns = 1000000000LL + std::llround(seconds * 1e9);
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original";
    f.image_width = 640;
    f.image_height = 480;
    f.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    if (detected) {
        TrackRecordV1 t{
            1, 0, "rc_car", .9, 290, 100, 350, 160, "CONFIRMED", true, f.sample_monotonic_ns};
        t.ground = std::array<double, 6>{3. + .6 * seconds, 0, .01, 0, .01, 0};
        t.ground_anchor = std::array<double, 2>{1, 0};
        f.tracks = {t};
    }
    return f;
}
int main() {
    env("ARGUS_V21_ENABLED", "1");
    env("ARGUS_V21_CV_GAP", "1");
    env("ARGUS_V23_CV_RAW_MOTION", "1");
    env("ARGUS_V23_CV_MOVING_ONLY", "1");
    for (const bool history : {false, true}) {
        TargetSelectorConfig c;
        c.recovery.enabled = true;
        c.recovery.max_gap = 4000ms;
        c.recovery.trust_selected_id = true;
        c.recovery.v3_box_motion_guard = true;
        c.recovery.v3_fresh_contact_velocity = true;
        c.recovery.v32_cv_ten_frame = history;
        c.recovery.short_motion.enabled = true;
        c.recovery.short_motion.window_s = .25;
        c.recovery.short_motion.minimum_span_s = .15;
        c.recovery.short_motion.minimum_samples = 4;
        TargetSelector selector(c);
        for (unsigned i = 0; i <= 40; ++i) {
            const auto f = input(i + 1, i * .05, true);
            assert(selector.update(f, at(i * .05)));
            if (i == 0) {
                assert(selector.execute({"select", TargetCommandAction::Select, 1, 1}, at(0))
                           .accepted);
                assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
            }
        }
        auto view = selector.snapshot(at(2));
        assert(view.fresh_real_detection);
        assert(selector.update(input(42, 2.05, false), at(2.05)));
        view = selector.snapshot(at(2.05));
        assert(view.cv_gap_active && !view.fresh_real_detection && !view.observation_valid_until);
        assert(view.cv_origin_sample_ns == input(41, 2, true).sample_monotonic_ns);
        assert(view.cv_gap_deadline == at(5));
        assert(std::abs(view.cv_velocity_north_m_s - .6) < .01);
        assert(bool(view.cv_history_frame_id) == history);
        if (history) {
            assert(view.cv_history_frame_id == 31);
            assert(view.cv_history_source_ns == input(31, 1.5, true).sample_monotonic_ns);
        }
        assert(selector.update(input(43, 2.25, false), at(2.25)));
        view = selector.snapshot(at(2.3));
        assert(view.cv_gap_active && view.cv_gap_deadline == at(5));
        assert(!selector.snapshot(at(5)).cv_gap_active);
        // A new same-ID real observation returns immediately, without extra identity confirmation.
        assert(selector.update(input(44, 5.01, true), at(5.01)));
        view = selector.snapshot(at(5.01));
        assert(view.fresh_real_detection && !view.recovery_pause);
        assert(view.recovery_reason == "selected_native_id_trusted");
    }
}
