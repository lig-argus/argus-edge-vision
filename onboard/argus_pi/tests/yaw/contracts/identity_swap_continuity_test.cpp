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

TargetManager ready(bool enabled = true, bool mature = true) {
    auto c = config();
    c.prefer_selected_continuity = enabled;
    TargetManager m(c);
    for (int i = 0; i <= 23; ++i) {
        double t = .05 * i;
        std::vector<TrackRecordV1> tracks{car(1, 8 + .6 * t, 0, t, 200)};
        if (mature || i >= 21) {
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
    return m;
}
void follows_physical_target_across_both_swaps(bool mature) {
    auto m = ready(true, mature);
    auto before = m.snapshot(at(1.15), max_age);
    auto mission = before.memory->mission_target_id;
    for (int i = 24; i <= 40; ++i) {
        double t = i * .05;
        bool swapped = i < 34;
        auto f = frame(i + 1, t,
                       {car(swapped ? 2 : 1, 8 + .6 * t, 0, t, 200),
                        car(swapped ? 1 : 2, 8 + .6 * t, .8, t, 400)});
        observe(m, f, t);
        auto v = m.snapshot(at(t), max_age);
        assert(v.phase == TargetPhase::Locked && !v.pause_commands);
        assert(v.memory->binding.track_id == (swapped ? 2 : 1));
        assert(v.memory->mission_target_id == mission);
        assert(v.memory->last_real_sample_ns == f.sample_monotonic_ns);
        assert(v.memory->short_motion.valid);
        assert(std::abs(v.memory->short_motion.velocity_north_m_s - .6) < .01);
    }
    std::cout
        << "forward and reverse ID remap, physical mission/motion/freshness preserved: passed\n";
}
void rejects_unproved_remaps() {
    for (int kind = 0; kind < 7; ++kind) {
        auto m = ready(kind != 0);
        double t = 1.2;
        auto a = car(1, 8 + .6 * t, .8, t, 400), b = car(2, 8 + .6 * t, 0, t, 200);
        std::vector<TrackRecordV1> tracks{a, b};
        if (kind == 1) {
            tracks = {b};
        }
        if (kind == 2) {
            tracks[1].ground = std::array<double, 6>{8 + .6 * t + .35, 0, .05, 0, .05, 0};
        }
        if (kind == 3) {
            tracks.push_back(car(3, 8 + .6 * t, 0, t, 300));
        }
        if (kind == 4) {
            tracks[0].x1 = 200;
            tracks[0].x2 = 260;
        }
        if (kind == 5) {
            tracks[1].track_id = 3;
        }
        auto f = frame(25, t, tracks);
        m.observe(f, at(t + (kind == 6 ? .3 : 0)), max_age);
        auto v = m.snapshot(at(t + (kind == 6 ? .3 : 0)), max_age);
        assert(v.pause_commands);
        assert(v.memory->binding.track_id == 1);
    }
    std::cout << "disabled, missing reciprocal, displaced, third candidate, image overlap, unknown "
                 "ID, stale frame HOLD: passed\n";
}
void keeps_close_native_identity_only_with_image_evidence() {
    for (int kind = 0; kind < 7; ++kind) {
        auto c = config();
        c.prefer_selected_continuity = kind != 6;
        TargetManager m(c);
        for (int i = 0; i <= 23; ++i) {
            const double t = i * .05;
            auto a = car(1, 8 + .6 * t, 0, t, 200, 180);
            auto b = car(2, 8 + .6 * t, .8, t, 180, 180);
            a.y1 = 115;
            a.y2 = 215;
            b.y1 = 65;
            b.y2 = 155;
            auto f = frame(i + 1, t, {a, b});
            if (i == 0) {
                m.select(f, f.tracks.front(), at(t));
            } else {
                observe(m, f, t);
            }
            assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
        }
        // Both physical tracks approach continuously while their image boxes stay distinguishable.
        for (int i = 24; i <= 86; ++i) {
            const double t = i * .05;
            const double separation = std::max(.18, .8 - (i - 23) * .01);
            auto a = car(1, 8 + .6 * t, 0, t, 200, 180);
            auto b = car(2, 8 + .6 * t, separation, t, 180, 180);
            a.y1 = 115;
            a.y2 = 215;
            b.y1 = 65;
            b.y2 = 155;
            if (i == 86 && kind == 1) {
                b.x1 = a.x1;
                b.x2 = a.x2;
                b.y1 = a.y1;
                b.y2 = a.y2;
            }
            if (i == 86 && kind == 2) {
                std::swap(a.track_id, b.track_id);
            }
            if (i == 86 && kind == 3) {
                a.real_detection = false;
                a.state = "PREDICTED";
                a.last_real_detection_monotonic_ns -= 50000000;
            }
            auto f = frame(i + 1, t, {a, b});
            if (i == 86 && kind == 4) {
                auto unknown = a;
                unknown.track_id = 3;
                f.tracks.push_back(unknown);
            }
            observe(m, f, t + (i == 86 && kind == 5 ? .3 : 0));
            const auto v = m.snapshot(at(t + (i == 86 && kind == 5 ? .3 : 0)), max_age);
            if (kind == 6) {
                if (v.phase != TargetPhase::Locked) {
                    break;
                }
                assert(i != 86);
            } else if (i < 86 || kind == 0) {
                assert(v.phase == TargetPhase::Locked && !v.pause_commands);
                assert(v.memory->binding.track_id == 1);
            } else {
                if (!v.pause_commands) {
                    std::cerr << "unguarded kind=" << kind << " reason=" << v.reason << '\n';
                }
                assert(v.pause_commands);
            }
        }
    }
    std::cout << "close IDs persist; overlap, swap, prediction-only, third candidate, stale and "
                 "disabled guarded: passed\n";
}

}  // namespace
int main() {
    keeps_close_native_identity_only_with_image_evidence();
    follows_physical_target_across_both_swaps(true);
    follows_physical_target_across_both_swaps(false);
    rejects_unproved_remaps();
}
