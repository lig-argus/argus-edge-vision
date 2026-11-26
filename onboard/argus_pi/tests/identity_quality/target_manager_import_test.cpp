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

void short_occlusion_preserves_observed_target() {
    TargetManager manager(config());
    for (int i = 0; i <= 15; ++i) {
        const double t = i * .05;
        const auto message =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, 3, t, 400)});
        if (i == 0) {
            manager.select(message, message.tracks.front(), at(t));
        } else {
            observe(manager, message, t);
        }
        assert(manager.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    // 다른 차량만 잠깐 가려져도 원표적의 새 실관측을 갱신한다.
    const auto visible = frame(17, .8, {car(1, 8 + .6 * .8, 0, .8)});
    observe(manager, visible, .8);
    auto view = manager.snapshot(at(.8), max_age);
    assert(view.phase == TargetPhase::Locked && !view.pause_commands);
    assert(view.memory->binding.track_id == 1 &&
           view.memory->last_real_sample_ns == visible.sample_monotonic_ns);
    // 경쟁차의 관측 근거가 오래되면 같은 ID라는 이유만으로 계속하지 않는다.
    for (int i = 18; i <= 28; ++i) {
        const double t = (i - 1) * .05;
        observe(manager, frame(i, t, {car(1, 8 + .6 * t, 0, t)}), t);
    }
    view = manager.snapshot(at(1.35), max_age);
    assert(view.pause_commands && view.memory->binding.track_id == 1);
    std::cout << "short competitor occlusion and identity expiry passed\n";
}

void crossing_uses_two_real_motion_paths() {
    TargetManager manager(config());
    for (int i = 0; i <= 24; ++i) {
        const double t = i * .05;
        const auto message =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, 1.5 - t, t, 400)});
        if (i == 0) {
            manager.select(message, message.tracks.front(), at(t));
        } else {
            observe(manager, message, t);
        }
        const auto view = manager.snapshot(at(t), max_age);
        assert(view.phase == TargetPhase::Locked && view.memory->binding.track_id == 1);
    }
    // 실제 두 후보가 같은 위치에 겹치면 운동 기억으로 신원을 꾸며내지 않는다.
    const auto coincident = frame(26, 1.25, {car(1, 8.75, 0, 1.25), car(2, 8.75, 0, 1.25, 400)});
    observe(manager, coincident, 1.25);
    const auto view = manager.snapshot(at(1.25), max_age);
    assert(view.pause_commands && view.memory->binding.track_id == 1);
    std::cout << "separated crossing and genuine ambiguity passed\n";
}

void observed_image_departure_preserves_original_sample() {
    TargetManager manager(config());
    for (int i = 0; i <= 18; ++i) {
        const double t = i * .05;
        const auto message =
            frame(i + 1, t, {car(1, 8 + .6 * t, 0, t), car(2, 8 + .6 * t, 3, t, 580 + 40 * t, 20)});
        if (i == 0) {
            manager.select(message, message.tracks.front(), at(t));
        } else {
            observe(manager, message, t);
        }
        assert(manager.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    const auto visible = frame(20, .95, {car(1, 8 + .6 * .95, 0, .95)});
    observe(manager, visible, .95);
    assert(manager.snapshot(at(.95), max_age).phase == TargetPhase::Locked);
    observe(manager, frame(21, 1., {}), 1.);
    const auto view = manager.snapshot(at(1.), max_age);
    assert(view.phase == TargetPhase::Locked);
    assert(view.memory->last_real_sample_ns == visible.sample_monotonic_ns);
    assert(view.memory->last_real_received_at == at(.95));
    manager.advance(at(1.21), max_age);
    assert(manager.snapshot(at(1.21), max_age).pause_commands);
    manager.advance(at(5.), max_age);
    assert(manager.snapshot(at(5.), max_age).phase == TargetPhase::Lost);
    observe(manager, frame(22, 5.05, {car(1, 8, 0, 5.05)}), 5.05);
    assert(manager.snapshot(at(5.05), max_age).phase == TargetPhase::Lost);
    std::cout << "observed departure, original TTL and no automatic Lost resume passed\n";
}

#ifndef ARGUS_BASELINE_20261009
void brief_gap_option_retains_only_observation_history() {
    for (bool retain : {false, true}) {
        auto options = config();
        options.retain_motion_on_brief_gap = retain;
        options.motion_max_sample_gap = 1000ms;
        TargetManager manager(options);
        for (int i = 0; i <= 15; ++i) {
            const double t = i * .05;
            const auto message = frame(i + 1, t, {car(1, 8 + .6 * t, 0, t)});
            if (i == 0) {
                manager.select(message, message.tracks.front(), at(t));
            } else {
                observe(manager, message, t);
            }
        }
        const auto original = manager.snapshot(at(.75), max_age).memory->last_real_sample_ns;
        manager.advance(at(1.01), max_age);
        auto view = manager.snapshot(at(1.01), max_age);
        assert(view.pause_commands && !view.memory->ground_motion_valid &&
               view.memory->last_real_sample_ns == original);
        for (int i = 0; i < 3; ++i) {
            const double t = 1.05 + i * .05;
            observe(manager, frame(17 + i, t, {car(1, 8 + .6 * t, 0, t)}), t);
        }
        view = manager.snapshot(at(1.15), max_age);
        assert(view.phase == TargetPhase::Locked && view.memory->ground_motion_valid == retain);
        manager.pause_ambiguous("explicit_identity_ambiguity");
        assert(manager.snapshot(at(1.15), max_age).pause_commands);
    }
    std::cout << "brief-gap opt-in, neutral gap and explicit ambiguity passed\n";
}
#endif
}  // namespace

int main() {
    short_occlusion_preserves_observed_target();
    crossing_uses_two_real_motion_paths();
    observed_image_departure_preserves_original_sample();
#ifndef ARGUS_BASELINE_20261009
    brief_gap_option_retains_only_observation_history();
#endif
}
