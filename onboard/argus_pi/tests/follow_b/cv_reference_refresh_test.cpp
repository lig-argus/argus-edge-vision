// 2026-11-10 GAP_CV 출발점(cv_visibility_reference) 갱신 시험.
// 10:08 비행: 박스 크기가 크게 출렁여 박스 의심이 3.5초 이어졌고, 그동안 출발점이 갱신되지 않아
// 공백 시작 즉시 GAP_CV가 거부됐다. 켜면 출발점이 1초보다 오래되지 않아야 한다.
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

TrackRecordV1 car(double seconds, double height_px) {
    TrackRecordV1 track;
    track.track_id = 3;
    track.class_id = 0;
    track.label = "rc_car";
    track.confidence = .9;
    track.x1 = 560;
    track.x2 = 660;
    track.y1 = 300;
    track.y2 = 300 + height_px;
    track.state = "CONFIRMED";
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    track.ground = std::array<double, 6>{8 + .3 * seconds, 0, .05, 0, .05, 0};
    return track;
}

TracksV1 frame(unsigned number, double seconds, TrackRecordV1 track) {
    TracksV1 message;
    message.schema = kTracksV4Schema;
    message.source = "verified-ground-contract-test";
    message.frame_id = number;
    message.generation = 1;
    message.sample_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    message.motion_sample_ns = message.sample_monotonic_ns;
    message.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    message.clock_domain = "host_monotonic";
    message.timestamp_source = "original-ground-sample";
    message.image_width = 1204;
    message.image_height = 718;
    message.tracks = {track};
    assert(valid_tracks_envelope(message));
    return message;
}

TargetRecoveryConfig config(bool refresh) {
    TargetRecoveryConfig value;
    value.enabled = true;
    value.max_gap = 4000ms;
    value.allow_competitor_departure = true;
    value.prefer_selected_continuity = true;
    value.trust_selected_id = true;
    value.v3_box_motion_guard = true;
    value.v35_cv_reference_refresh = refresh;
    value.short_motion.enabled = true;
    value.short_motion.window_s = .25;
    value.short_motion.minimum_span_s = .15;
    value.short_motion.minimum_samples = 4;
    value.short_motion.confirmation_s = .05;
    value.motion_history_s = .75;
    value.motion_history_min_s = .3;
    value.motion_history_min_samples = 5;
    return value;
}

// 0~0.8초 높이 70px로 잠근 뒤 0.85~3.0초 높이 45px(80% 미만)로 계속 받는다 → 의심이 이어진다.
struct Result {
    double reference_s;
    bool suspect;
};
Result run(bool refresh) {
    TargetManager m(config(refresh));
    unsigned id = 1;
    for (int i = 0; i <= 16; ++i) {
        const double t = i * .05;
        auto f = frame(id++, t, car(t, 70));
        if (i == 0) {
            m.select(f, f.tracks[0], at(t));
        } else {
            m.observe(f, at(t), max_age);
        }
    }
    double t = .85;
    for (; t <= 3.0 + 1e-9; t += .05) {
        m.observe(frame(id++, t, car(t, 45)), at(t), max_age);
    }
    const auto q = m.snapshot(at(t), max_age);
    assert(q.memory && q.memory->cv_visibility_reference);
    return {(q.memory->cv_visibility_reference->sample_ns - origin_ns) / 1e9,
            q.memory->box_motion_suspect};
}

}  // namespace

int main() {
    const auto legacy = run(false);
    const auto refreshed = run(true);
    std::cout << "legacy reference " << legacy.reference_s << " s, refreshed "
              << refreshed.reference_s << " s (last frame 3.0 s)\n";
    assert(legacy.suspect && refreshed.suspect);  // 의심은 그대로 이어진다
    assert(legacy.reference_s < .9);              // 기존: 잠근 직후 값에 멈춤
    assert(refreshed.reference_s > 3.0 - 1.05);   // 켜면 1초 이내로 따라온다
    std::cout << "PASS CV visibility reference refresh during long box suspicion\n";
    return 0;
}
