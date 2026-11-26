// 원격 대상 관리 개선을 기존 tracks v2로 검사한다. FC·카메라·GT 입력은 없다.
#include <cassert>
#include <cmath>
#include <iostream>

#include "argus/control/target/include/target_manager.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
constexpr auto max_age = 250ms;
constexpr std::int64_t origin_ns = 1000000000LL;

Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{origin_ns + std::llround(seconds * 1e9)}};
}

TrackRecordV1 car(std::int64_t id, double north, double east, double seconds, double left = 200,
                  double width = 60) {
    TrackRecordV1 track;
    track.track_id = id;
    track.class_id = 0;
    track.label = "rc_car";
    track.confidence = .9;
    track.x1 = left;
    track.x2 = left + width;
    track.y1 = 100;
    track.y2 = 160;
    track.state = "CONFIRMED";
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    track.ground = std::array<double, 6>{north, east, .05, 0, .05, 0};
    return track;
}

TracksV1 frame(unsigned number, double seconds, std::vector<TrackRecordV1> tracks) {
    TracksV1 message;
    message.schema = kTracksV2Schema;
    message.source = "verified-ground-contract-test";
    message.frame_id = number;
    message.generation = 1;
    message.sample_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    message.clock_domain = "host_monotonic";
    message.timestamp_source = "original-ground-sample";
    message.image_width = 640;
    message.image_height = 480;
    message.tracks = std::move(tracks);
    assert(valid_tracks_envelope(message));
    for (const auto& track : message.tracks) {
        assert(valid_track_record(track) && track_fits_frame(track, message));
    }
    return message;
}

TargetRecoveryConfig config() {
    TargetRecoveryConfig value;
    value.enabled = true;
    value.max_gap = 4000ms;
    value.allow_competitor_departure = true;
    value.short_motion.enabled = true;
    value.short_motion.window_s = .25;
    value.short_motion.minimum_span_s = .15;
    value.short_motion.minimum_samples = 4;
    value.short_motion.confirmation_s = .05;
    return value;
}

void observe(TargetManager& manager, const TracksV1& message, double seconds) {
    manager.observe(message, at(seconds), max_age);
}

TargetManager warm(double separation = .8) {
    auto options = config();
    options.prefer_selected_continuity = true;
    TargetManager manager(options);
    for (int i = 0; i <= 16; ++i) {
        double t = i * .05;
        auto f =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, separation, t, 400)});
        if (i == 0) {
            manager.select(f, f.tracks.front(), at(t));
        } else {
            observe(manager, f, t);
        }
        assert(manager.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    return manager;
}
void keeps_continuous_selected_target() {
    auto m = warm();
    for (int i = 17; i <= 56; ++i) {
        double t = i * .05;
        auto f = frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)});
        observe(m, f, t);
        auto v = m.snapshot(at(t), max_age);
        assert(v.phase == TargetPhase::Locked && !v.pause_commands);
        assert(v.memory->binding.track_id == 1 &&
               v.memory->last_real_sample_ns == f.sample_monotonic_ns);
    }
    std::cout << "continuous selected target, missing other beyond 500ms: passed\n";
}
void accepts_separated_swap_but_holds_overlap() {
    {
        auto m = warm();
        double t = .85;
        auto f = frame(18, t, {car(1, 8 + .6 * t, .8, t, 400), car(2, 8 + .6 * t, 0, t, 200)});
        const auto mission = m.snapshot(at(.8), max_age).memory->mission_target_id;
        observe(m, f, t);
        auto v = m.snapshot(at(t), max_age);
        assert(v.phase == TargetPhase::Locked && !v.pause_commands &&
               v.memory->binding.track_id == 2 && v.memory->mission_target_id == mission);
    }
    {
        auto m = warm();
        double t = .85;
        observe(m, frame(18, t, {car(1, 8 + .6 * t, .4, t), car(2, 8 + .6 * t, .4, t, 400)}), t);
        auto v = m.snapshot(at(t), max_age);
        assert(v.pause_commands && v.memory->binding.track_id == 1);
    }
    std::cout << "separated reciprocal ID swap continues; genuine overlap HOLD: passed\n";
}
void breaks_continuity_on_jump_missing_id_and_gap() {
    for (int kind = 0; kind < 5; ++kind) {
        auto m = warm();
        for (int i = 17; i <= 30; ++i) {
            double t = i * .05;
            observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
        }
        double t = 1.55;
        std::vector<TrackRecordV1> tracks;
        if (kind == 0) {
            tracks = {car(1, 8 + .6 * t, .3, t)};
        }
        if (kind == 1) {
            tracks = {car(2, 8 + .6 * t, 0, t)};
        }
        if (kind == 2) {
            tracks = {car(3, 8 + .6 * t, 0, t)};
        }
        if (kind == 3) {
            tracks = {};
        }
        auto f = frame(32, t, tracks);
        if (kind == 4) {
            f = frame(32, t, {car(1, 8 + .6 * t, 0, t)});
            m.observe(f, at(t + .3), max_age);
        } else {
            observe(m, f, t);
        }
        auto v = m.snapshot(at(t + (kind == 4 ? .3 : 0)), max_age);
        assert(v.pause_commands);
        assert(v.memory->binding.track_id == 1);
    }
    std::cout
        << "jump, competitor-only, unknown ID, selected occlusion, stale input HOLD: passed\n";
}
void rejects_predicted_competitor_approach() {
    auto options = config();
    options.prefer_selected_continuity = true;
    TargetManager m(options);
    for (int i = 0; i <= 16; ++i) {
        double t = i * .05;
        auto f =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, .8 - .4 * t, t, 400)});
        if (i == 0) {
            m.select(f, f.tracks.front(), at(t));
        } else {
            observe(m, f, t);
        }
        assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    for (int i = 17; i <= 21; ++i) {
        double t = i * .05;
        observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
    }
    assert(m.snapshot(at(1.05), max_age).pause_commands);
    std::cout << "fresh competing trajectory approaching selected path HOLD: passed\n";
}
void partial_competitor_is_not_absent() {
    auto m = warm();
    for (int i = 17; i <= 30; ++i) {
        double t = i * .05;
        observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
    }
    auto other = car(2, 9, .8, 1.55, 400);
    other.ground.reset();
    auto f = frame(32, 1.55, {car(1, 8 + .6 * 1.55, 0, 1.55), other});
    observe(m, f, 1.55);
    assert(m.snapshot(at(1.55), max_age).pause_commands);
    std::cout << "visible competitor with invalid geometry cannot inherit absent-object "
                 "continuity: passed\n";
}
void short_distinct_competitor_does_not_block_selected() {
    auto options = config();
    options.prefer_selected_continuity = true;
    TargetManager m(options);
    for (int i = 0; i <= 23; ++i) {
        double t = i * .05;
        std::vector<TrackRecordV1> tracks{car(1, 8 + .6 * t, 0, t)};
        if (i >= 21) {
            tracks.push_back(car(2, 8 + .6 * t, .8, t, 400));
        }
        auto f = frame(i + 1, t, tracks);
        if (i == 0) {
            m.select(f, f.tracks.front(), at(t));
        } else {
            observe(m, f, t);
        }
        assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    for (int i = 24; i <= 50; ++i) {
        double t = i * .05;
        observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
        assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    std::cout << "three distinct real competitor samples without mature motion do not block "
                 "continuous target: passed\n";
}
void recovers_short_empty_input_with_original_deadline() {
    for (int kind = 0; kind < 4; ++kind) {
        auto m = warm();
        for (int i = 17; i <= 30; ++i) {
            const double t = i * .05;
            observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
        }
        const auto original = m.snapshot(at(1.5), max_age).memory->last_real_sample_ns;
        observe(m, frame(32, 1.55, {}), 1.55);
        auto gap = m.snapshot(at(1.55), max_age);
        assert(gap.pause_commands && gap.memory->last_real_sample_ns == original);
        if (kind == 3) {
            bool resumed = false;
            int first_resume = 0;
            for (int i = 34; i <= 40; ++i) {
                const double t = i * .05;
                observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
                const auto view = m.snapshot(at(t), max_age);
                if (view.phase == TargetPhase::Locked) {
                    if (!resumed) {
                        first_resume = i;
                    }
                    resumed = true;
                } else {
                    assert(!resumed && view.pause_commands);
                    assert(view.memory->last_real_sample_ns == original);
                }
            }
            assert(resumed && first_resume >= 37);
        } else if (kind == 1) {
            m.advance(at(1.751), max_age);
            observe(m, frame(33, 1.8, {car(1, 9.08, 0, 1.8)}), 1.8);
            assert(m.snapshot(at(1.8), max_age).pause_commands);
        } else if (kind == 2) {
            observe(m, frame(33, 1.6, {car(2, 8.96, 0, 1.6)}), 1.6);
            assert(m.snapshot(at(1.6), max_age).pause_commands);
        } else {
            for (int i = 32; i <= 34; ++i) {
                const double t = i * .05;
                observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
                const auto view = m.snapshot(at(t), max_age);
                assert(i == 34 ? view.phase == TargetPhase::Locked : view.pause_commands);
                if (i < 34) {
                    assert(view.memory->last_real_sample_ns == original);
                }
            }
            observe(m, frame(36, 1.75, {car(1, 9.05, 0, 1.75)}), 1.75);
            assert(m.snapshot(at(1.75), max_age).phase == TargetPhase::Locked);
        }
    }
    std::cout << "empty input stays neutral; fresh evidence within 250ms, >=3-frame confirmation, "
                 "next-tick continuity: passed\n";
}

void recent_separated_competitor_survives_neutral_gap() {
    auto m = warm();
    for (int i = 17; i <= 20; ++i) {
        const double t = i * .05;
        observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
    }
    observe(m, frame(22, 1.05, {}), 1.05);
    assert(m.snapshot(at(1.05), max_age).pause_commands);
    for (int i = 22; i <= 26; ++i) {
        const double t = i * .05;
        observe(m, frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)}), t);
        const auto view = m.snapshot(at(t), max_age);
        if (i < 24) {
            assert(view.pause_commands);
        } else {
            assert(view.phase == TargetPhase::Locked);
        }
    }
    std::cout << "recent missing competitor retains fresh geometric separation through neutral "
                 "confirmation: passed\n";
}

void approaching_competitor_still_blocks_after_empty_gap() {
    auto options = config();
    options.prefer_selected_continuity = true;
    TargetManager m(options);
    for (int i = 0; i <= 16; ++i) {
        const double t = i * .05;
        auto f =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, .8 - .4 * t, t, 400)});
        if (i == 0) {
            m.select(f, f.tracks.front(), at(t));
        } else {
            observe(m, f, t);
        }
    }
    observe(m, frame(18, .85, {car(1, 8.51, 0, .85)}), .85);
    assert(m.snapshot(at(.85), max_age).phase == TargetPhase::Locked);
    observe(m, frame(19, .9, {}), .9);
    assert(m.snapshot(at(.9), max_age).pause_commands);
    observe(m, frame(20, 1.05, {car(1, 8.63, 0, 1.05)}), 1.05);
    assert(m.snapshot(at(1.05), max_age).pause_commands);
    assert(m.snapshot(at(1.05), max_age).reason == "known_competitor_identity_unresolved");
    std::cout << "fresh approaching competitor defeats stored identity evidence after neutral gap: "
                 "passed\n";
}

void recovery_does_not_deadlock_on_one_short_fit_window() {
    for (int kind = 0; kind < 3; ++kind) {
        auto m = warm();
        unsigned id = 18;
        for (int i = 17; i <= 34; ++i) {
            const double t = i * .05;
            observe(m, frame(id++, t, {car(1, 8 + .6 * t, 0, t)}), t);
        }
        observe(m, frame(id++, 1.732, {car(1, 9.0392, 0, 1.732)}), 1.732);
        observe(m, frame(id++, 1.768, {}), 1.768);
        assert(m.snapshot(at(1.768), max_age).pause_commands);
        for (double t : {1.868, 1.900, 1.968}) {
            observe(m, frame(id++, t, {car(1, 8 + .6 * t, 0, t)}), t);
        }
        assert(m.snapshot(at(1.968), max_age).phase == TargetPhase::Locked);
        observe(m, frame(id++, 2.0, {car(1, 9.2, 0, 2.0)}), 2.0);
        assert(!m.snapshot(at(2.0), max_age).memory->short_motion.valid);
        auto target = car(1, 8 + .6 * 2.032, kind == 1 ? .3 : 0, 2.032);
        std::vector<TrackRecordV1> tracks{target};
        if (kind == 2) {
            auto unknown = target;
            unknown.track_id = 3;
            tracks.push_back(unknown);
        }
        observe(m, frame(id++, 2.032, tracks), 2.032);
        const auto view = m.snapshot(at(2.032), max_age);
        if (kind == 0) {
            assert(view.phase == TargetPhase::Locked);
            assert(view.memory->short_motion.valid);
            m.advance(at(2.3), max_age);
            assert(m.snapshot(at(2.3), max_age).pause_commands);
        } else {
            assert(view.pause_commands);
        }
    }
    std::cout << "132ms fit remains invalid; bounded identity evidence survives one window; "
                 "jump, ambiguity and source expiry still stop: passed\n";
}

void no_resume_after_lost() {
    auto m = warm();
    m.advance(at(5.), max_age);
    assert(m.snapshot(at(5.), max_age).phase == TargetPhase::Lost);
    observe(m, frame(18, 5.05, {car(1, 11.03, 0, 5.05)}), 5.05);
    assert(m.snapshot(at(5.05), max_age).phase == TargetPhase::Lost);
    std::cout << "Lost requires explicit restart: passed\n";
}
}  // namespace
int main() {
    recovery_does_not_deadlock_on_one_short_fit_window();
    approaching_competitor_still_blocks_after_empty_gap();
    recent_separated_competitor_survives_neutral_gap();
    recovers_short_empty_input_with_original_deadline();
    keeps_continuous_selected_target();
    accepts_separated_swap_but_holds_overlap();
    breaks_continuity_on_jump_missing_id_and_gap();
    no_resume_after_lost();
    rejects_predicted_competitor_approach();
    partial_competitor_is_not_absent();
    short_distinct_competitor_does_not_block_selected();
}
