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

TracksV1 scene(unsigned i, bool shrink = false, bool crowded = false) {
    const double t = i * .05;
    auto selected = car(38, 8 + .6 * t, 0, t, shrink ? 208 : 200, shrink ? 44 : 60);
    selected.ground->at(2) = selected.ground->at(4) = .2;
    if (shrink) {
        selected.y1 = 108;
        selected.y2 = 152;
    }
    std::vector<TrackRecordV1> tracks{selected};
    if (i >= 9) {
        auto other = car(23, 8 + .6 * t, crowded ? 0.01 : .8, t, crowded ? 210 : 400);
        other.ground->at(2) = other.ground->at(4) = .2;
        tracks.push_back(other);
    }
    auto f = frame(i + 1, t, std::move(tracks));
    f.ground_pose = std::array<double, 6>{0, 0, -1.7, 0, 0, 0};
    return f;
}
TargetManager warmed(bool shrink) {
    auto c = config();
    c.prefer_selected_continuity = true;
    c.v3_box_motion_guard = true;
    TargetManager m(c);
    for (unsigned i = 0; i <= 32; ++i) {
        auto f = scene(i, shrink && i >= 25);
        if (i == 0) {
            m.select(f, f.tracks[0], at(0));
        } else {
            observe(m, f, i * .05);
        }
        auto v = m.snapshot(at(i * .05), max_age);
        assert(v.phase == TargetPhase::Locked && !v.pause_commands);
        assert(v.memory->binding.track_id == 38);
        if (shrink && i >= 25) {
            assert(v.memory->box_motion_suspect);
            assert(!v.memory->short_motion.valid);
            assert(!v.memory->ground_motion_valid);
            assert(v.memory->identity_short_motion.valid);
        }
    }
    return m;
}
void preserves_identity_without_trusting_guarded_speed() {
    auto m = warmed(true);
    auto v = m.snapshot(at(1.6), max_age);
    assert(v.memory->identity_short_motion.velocity_north_m_s > .5);
    assert(!v.memory->short_motion.valid && !v.memory->ground_motion_valid);
    std::cout
        << "box guard blocks control speed, continuous selected38 stays locked with competitor23\n";
}
void real_ambiguity_still_blocks() {
    auto m = warmed(true);
    auto f = scene(33, true, true);
    observe(m, f, 1.65);
    assert(m.snapshot(at(1.65), max_age).pause_commands);
    std::cout << "actual overlapping/crossing evidence still blocks\n";
}
void jump_still_blocks() {
    auto m = warmed(true);
    auto f = scene(33, true);
    f.tracks[0].ground->at(0) += 4;
    observe(m, f, 1.65);
    assert(m.snapshot(at(1.65), max_age).pause_commands);
    std::cout << "same nativeID cannot bypass a ground jump\n";
}
void invalid_or_replayed_observation_cannot_extend_lease() {
    auto m = warmed(true);
    auto f = scene(33, true);
    f.frame_id = 33;
    observe(m, f, 1.65);
    auto v = m.snapshot(at(1.65), max_age);
    assert(v.memory->last_real_sample_ns == origin_ns + 1600000000LL);
    m.advance(at(1.9), max_age);
    assert(m.snapshot(at(1.9), max_age).pause_commands);
    m.advance(at(6), max_age);
    assert(m.snapshot(at(6), max_age).phase == TargetPhase::Lost);
    std::cout << "duplicate frame and missing target never renew original identity lease\n";
}
void context_change_clears_identity() {
    auto m = warmed(true);
    auto f = scene(33, true);
    f.source = "different-camera";
    observe(m, f, 1.65);
    auto v = m.snapshot(at(1.65), max_age);
    assert(v.phase == TargetPhase::Lost);
    assert(!v.memory->identity_short_motion.valid);
    std::cout << "source context change clears identity evidence\n";
}
void predicted_candidate_does_not_become_real() {
    auto m = warmed(true);
    auto f = scene(33, true);
    f.tracks[0].real_detection = false;
    f.tracks[0].state = "PREDICTED";
    observe(m, f, 1.65);
    auto v = m.snapshot(at(1.65), max_age);
    assert(v.pause_commands);
    assert(v.memory->last_real_sample_ns == origin_ns + 1600000000LL);
    std::cout << "prediction cannot refresh real selected identity\n";
}
}  // namespace
int main() {
    preserves_identity_without_trusting_guarded_speed();
    real_ambiguity_still_blocks();
    jump_still_blocks();
    invalid_or_replayed_observation_cannot_extend_lease();
    context_change_clears_identity();
    predicted_candidate_does_not_become_real();
}
