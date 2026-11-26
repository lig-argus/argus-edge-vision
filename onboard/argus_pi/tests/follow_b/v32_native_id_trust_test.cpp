// 원격 대상 관리 개선을 기존 tracks v2로 검사한다. FC·카메라·GT 입력은 없다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "argus/control/target/include/target_manager.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

#ifdef _WIN32
inline int setenv(const char* name, const char* value, int) { return _putenv_s(name, value); }
#endif

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
    message.image_width = 1204;
    message.image_height = 718;
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
    value.motion_history_s = .75;
    value.motion_history_min_s = .3;
    value.motion_history_min_samples = 5;
    return value;
}

void observe(TargetManager& manager, const TracksV1& message, double seconds) {
    manager.observe(message, at(seconds), max_age);
}

TrackRecordV1 red(double t, double variance) {
    auto b = car(38, 8 + .6 * t, 0, t);
    b.confidence = .93;
    b.x1 = 596;
    b.x2 = 834;
    b.y1 = 155;
    b.y2 = 286;
    b.ground->at(2) = b.ground->at(4) = variance;
    return b;
}
TrackRecordV1 blue(double t, double variance) {
    auto b = car(23, 8 + .6 * t, .8, t);
    b.confidence = .93;
    b.x1 = 381;
    b.x2 = 710;
    b.y1 = 360;
    b.y2 = 524;
    b.ground->at(2) = b.ground->at(4) = variance;
    return b;
}

TrackRecordV1 large_red(double t) {
    auto b = red(t, .2);
    b.x1 = 555;
    b.x2 = 875;
    b.y1 = 135.5;
    b.y2 = 305.5;
    return b;
}
TrackRecordV1 jumped_red(double t) {
    auto b = red(t, .2);
    b.x1 = 20;
    b.x2 = 80;
    b.y1 = 20;
    b.y2 = 70;
    return b;
}
TracksV1 observed(unsigned i, double t, std::vector<TrackRecordV1> tracks) {
    auto f = frame(i, t, std::move(tracks));
    f.schema = kTracksV4Schema;
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    return f;
}

TargetRecoveryConfig trusted_config() {
    auto c = config();
    c.prefer_selected_continuity = true;
    c.trust_selected_id = true;
    c.v3_box_motion_guard = true;
    return c;
}
TargetManager trusted_ready(bool with_other = true, bool jump_guard = false) {
    setenv("ARGUS_COMPARISON_ISOLATED", "1", 1);
    setenv("ARGUS_SITL_BOX_MOTION_GUARD", "1", 1);
    auto c = trusted_config();
    c.selected_box_jump_guard = jump_guard;
    TargetManager m(c);
    for (int i = 0; i <= 16; ++i) {
        double t = i * .05;
        std::vector<TrackRecordV1> tracks{large_red(t)};
        if (with_other) {
            tracks.push_back(blue(t, .2));
        }
        auto f = observed(i + 1, t, tracks);
        if (!i) {
            m.select(f, f.tracks[0], at(t));
        } else {
            observe(m, f, t);
        }
        assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    observe(m, observed(18, .85, {red(.85, .2), blue(.85, .2)}), .85);
    return m;
}
}  // namespace
int main(int argc, char** argv) {
    assert(argc == 2);
    auto configured = load_bus_following_config(argv[1]);
    assert(configured.selector.recovery.trust_selected_id);
    // 2026-11-10: 박스 튐 검사(IoU)는 사용자가 설정에서 뺐다(11-09). 설정값은 강제하지 않고,
    // 아래 튐 시나리오는 시험이 직접 켠 설정으로 검사한다.
    {
        auto m = trusted_ready();
        auto q = m.snapshot(at(.85), max_age);
        assert(q.memory->box_motion_suspect && !q.memory->short_motion.valid);
        observe(m, observed(19, .9, {red(.9, .2)}), .9);
        q = m.snapshot(at(.9), max_age);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted" &&
               !q.pause_commands);
        assert(q.memory->binding.track_id == 38);
        assert(!q.memory->short_motion.valid);
        auto t = red(.95, .2);
        t.ground->at(1) = .8;
        t.x1 = 20;
        t.x2 = 80;
        t.y1 = 20;
        t.y2 = 70;
        observe(m, observed(20, .95, {t, blue(.95, .2)}), .95);
        q = m.snapshot(at(.95), max_age);
        assert(q.phase == TargetPhase::Locked && q.memory->binding.track_id == 38);
    }
    {
        auto m = trusted_ready();
        observe(m, observed(19, .9, {blue(.9, .2)}), .9);
        auto q = m.snapshot(at(.9), max_age);
        assert(q.pause_commands && q.reason == "candidate_is_known_other_vehicle");
        assert(q.memory->binding.track_id == 38);
        observe(m, observed(20, .95, {red(.95, .2), blue(.95, .2)}), .95);
        q = m.snapshot(at(.95), max_age);
        assert(q.phase == TargetPhase::Locked && !q.pause_commands &&
               q.reason == "selected_native_id_trusted");
    }
    {
        auto m = trusted_ready();
        for (int i = 18; i <= 20; ++i) {
            double t = i * .05;
            auto z = red(t, .01);
            z.track_id = 99;
            auto other = blue(t, .01);
            observe(m, observed(i + 1, t, {z}), t);
            auto q = m.snapshot(at(t), max_age);
            assert(i < 20 ? q.pause_commands : q.phase == TargetPhase::Locked);
            assert(q.memory->binding.track_id == (i < 20 ? 38 : 99));
        }
    }
    for (int kind = 0; kind < 5; ++kind) {
        auto m = trusted_ready();
        auto f = observed(19, .9, {red(.9, .2)});
        double recv = .9;
        if (kind == 0) {
            f.tracks[0].real_detection = false;
            f.tracks[0].state = "PREDICTED";
        }
        if (kind == 1) {
            f.generation = 2;
        }
        if (kind == 2) {
            f.source = "different-camera";
        }
        if (kind == 3) {
            recv = 1.2;
        }
        if (kind == 4) {
            f.tracks[0].class_id = 9;
        }
        observe(m, f, recv);
        auto q = m.snapshot(at(recv), max_age);
        assert(q.pause_commands || q.phase == TargetPhase::Lost);
        assert(q.memory->last_real_sample_ns < f.sample_monotonic_ns);
    }
    {
        auto m = trusted_ready();
        auto f = observed(19, .9, {red(.9, .2)});
        observe(m, f, .9);
        auto old = m.snapshot(at(.9), max_age).memory->last_real_sample_ns;
        observe(m, f, 1.0);
        assert(m.snapshot(at(1.0), max_age).memory->last_real_sample_ns == old);
        m.advance(at(1.151), max_age);
        assert(m.snapshot(at(1.151), max_age).pause_commands);
    }
    {
        auto m = trusted_ready();
        m.advance(at(5), max_age);
        observe(m, observed(19, 5.01, {red(5.01, .2)}), 5.01);
        assert(m.snapshot(at(5.01), max_age).phase == TargetPhase::Lost);
    }
    {
        TargetSelectorConfig c;
        c.recovery = trusted_config();
        TargetSelector sel(c);
        auto f = observed(1, 0, {red(0, .2)});
        assert(sel.update(f, at(0)));
        auto selected = sel.execute({"select", TargetCommandAction::Select, 1, 38}, at(0));
        assert(selected.accepted);
        auto started = sel.execute({"start", TargetCommandAction::Start, 1, 38}, at(0));
        assert(started.accepted);
        assert(sel.update(observed(2, .05, {red(.05, .2), blue(.05, .2)}), at(.05)));
        auto q = sel.snapshot(at(.05));
        assert(q.state == TargetState::Confirmed && !q.recovery_pause);
        auto dupe = observed(3, .1, {red(.1, .2), red(.1, .2)});
        assert(!sel.update(dupe, at(.1)));
    }
    // selected_box_jump_guard: 같은 ID가 겹침 없이 멀리 튄 1프레임은 버리고
    // 원래 박스로 바로 복귀한다.
    {
        auto m = trusted_ready(true, true);
        const auto before = m.snapshot(at(.85), max_age).memory->last_real_sample_ns;
        observe(m, observed(19, .9, {jumped_red(.9), blue(.9, .2)}), .9);
        auto q = m.snapshot(at(.9), max_age);
        assert(q.phase == TargetPhase::GapHold && q.reason == "selected_id_box_jump");
        assert(q.memory->binding.track_id == 38 && q.memory->last_real_sample_ns == before);
        observe(m, observed(20, .95, {red(.95, .2), blue(.95, .2)}), .95);
        q = m.snapshot(at(.95), max_age);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted");
    }
    // 튄 박스만 0.5초를 넘게 이어지면 다른 대상을 따르지 않고 잃은 것으로 본다.
    {
        auto m = trusted_ready(true, true);
        unsigned id = 19;
        TargetRecoverySnapshot q;
        for (double t = .9; t < 1.42; t += .05) {
            observe(m, observed(id++, t, {jumped_red(t), blue(t, .2)}), t);
            q = m.snapshot(at(t), max_age);
            if (q.phase == TargetPhase::Lost) {
                break;
            }
            assert(q.reason == "selected_id_box_jump");
        }
        assert(q.phase == TargetPhase::Lost && q.reason == "selected_id_box_jump_persisted");
    }
    // 튐 판정 없이 0.5초 넘게 비었던 뒤의 같은 ID는 기존처럼 받아들인다.
    {
        auto m = trusted_ready(true, true);
        observe(m, observed(19, 1.5, {jumped_red(1.5), blue(1.5, .2)}), 1.5);
        auto q = m.snapshot(at(1.5), max_age);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted");
    }
    // 정상 이동(겹침 유지)은 검사를 켜도 그대로 수용한다.
    {
        auto m = trusted_ready(true, true);
        auto moved = red(.9, .2);
        moved.x1 += 60;
        moved.x2 += 60;
        observe(m, observed(19, .9, {moved, blue(.9, .2)}), .9);
        assert(m.snapshot(at(.9), max_age).phase == TargetPhase::Locked);
    }
    std::cout << "PASS selected-ID box jump guard" << std::endl;
    std::cout << "PASS explicit native-ID priority, peer-loss/jump/shape trust, ID-absent "
                 "candidate comparison, new-ID3frame confirmation, immediate same-ID return, "
                 "fresh/context/gen/duplicate/Lost/selector/config boundaries\n";
}
