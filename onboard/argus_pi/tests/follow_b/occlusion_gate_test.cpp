// 2026-11-10 가림 대응 시험: 같은 native ID 복귀의 지면 확인과 공백 비례 재획득 게이트.
// FC·카메라 입력은 없다. 대상은 북쪽으로 0.6 m/s 등속 이동하고, 0.85초부터 가려진다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "argus/control/target/include/target_manager.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
constexpr auto max_age = 250ms;
constexpr std::int64_t origin_ns = 1000000000LL;
constexpr double kSpeed = .6;  // 북쪽 m/s

Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{origin_ns + std::llround(seconds * 1e9)}};
}

double north_at(double t) { return 8 + kSpeed * t; }

TrackRecordV1 car(std::int64_t id, double north, double east, double seconds) {
    TrackRecordV1 track;
    track.track_id = id;
    track.class_id = 0;
    track.label = "rc_car";
    track.confidence = .9;
    track.x1 = 560;
    track.x2 = 660;
    track.y1 = 300;
    track.y2 = 370;
    track.state = "CONFIRMED";
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    track.ground = std::array<double, 6>{north, east, .05, 0, .05, 0};
    return track;
}

TracksV1 frame(unsigned number, double seconds, std::vector<TrackRecordV1> tracks) {
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
    message.tracks = std::move(tracks);
    assert(valid_tracks_envelope(message));
    for (const auto& track : message.tracks) {
        assert(valid_track_record(track) && track_fits_frame(track, message));
    }
    return message;
}

TargetRecoveryConfig config(double growth, int trusted_check_ms) {
    TargetRecoveryConfig value;
    value.enabled = true;
    value.max_gap = 4000ms;
    value.allow_competitor_departure = true;
    value.prefer_selected_continuity = true;
    value.trust_selected_id = true;
    value.short_motion.enabled = true;
    value.short_motion.window_s = .25;
    value.short_motion.minimum_span_s = .15;
    value.short_motion.minimum_samples = 4;
    value.short_motion.confirmation_s = .05;
    value.motion_history_s = .75;
    value.motion_history_min_s = .3;
    value.motion_history_min_samples = 5;
    value.ground_gate_m = 2.0;
    value.ground_gate_growth_m_s = growth;
    value.ground_gate_max_m = growth > 0 ? 4.0 : 0;
    value.trusted_return_check = std::chrono::milliseconds{trusted_check_ms};
    return value;
}

// 0~0.8초 대상 38을 따라 잠근다. 0.85초부터 관측 없음(가림).
TargetManager locked(double growth, int trusted_check_ms) {
    TargetManager m(config(growth, trusted_check_ms));
    for (int i = 0; i <= 16; ++i) {
        const double t = i * .05;
        auto f = frame(i + 1, t, {car(38, north_at(t), 0, t)});
        if (i == 0) {
            m.select(f, f.tracks[0], at(t));
        } else {
            m.observe(f, at(t), max_age);
        }
        assert(m.snapshot(at(t), max_age).phase == TargetPhase::Locked);
    }
    const auto q = m.snapshot(at(.8), max_age);
    assert(std::abs(q.memory->velocity_north_m_s - kSpeed) < .1);
    return m;
}

bool rejects(const TargetRecoveryConfig& c) {
    try {
        TargetManager m(c);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

}  // namespace

int main() {
    // 0. 설정 검사
    {
        auto c = config(.5, 500);
        c.ground_gate_max_m = 1.5;  // 기본 게이트보다 작은 최대값
        assert(rejects(c));
        c = config(-.1, 0);
        assert(rejects(c));
        c = config(0, 500);
        c.trust_selected_id = false;  // 같은 ID 확인은 trust_selected_id에서만
        assert(rejects(c));
        assert(!rejects(config(.5, 500)));
        assert(!rejects(config(0, 0)));
    }

    const double t = 2.0;  // 마지막 관측 0.8초 → 공백 1.2초
    const double predicted = north_at(t);

    // 1. 같은 ID가 예측 위치 근처로 돌아오면 바로 믿는다.
    {
        auto m = locked(.5, 500);
        m.observe(frame(30, t, {car(38, predicted + .5, 0, t)}), at(t), max_age);
        const auto q = m.snapshot(at(t), max_age);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted");
    }
    // 2. 같은 ID가 예측에서 3.5 m(게이트 2+0.5×1.2=2.6 m 밖) 떨어져 돌아오면 바로 믿지 않는다.
    {
        auto m = locked(.5, 500);
        const auto before = m.snapshot(at(.8), max_age).memory->last_real_sample_ns;
        m.observe(frame(30, t, {car(38, predicted + 3.5, 0, t)}), at(t), max_age);
        const auto q = m.snapshot(at(t), max_age);
        assert(q.phase != TargetPhase::Locked);
        assert(q.memory->last_real_sample_ns == before);  // 기억을 갱신하지 않았다
        std::cout << "far same-ID return -> " << q.reason << '\n';
    }
    // 3. 기존 동작(확인 끔): 같은 ID면 위치와 상관없이 믿는다.
    {
        auto m = locked(0, 0);
        m.observe(frame(30, t, {car(38, predicted + 3.5, 0, t)}), at(t), max_age);
        const auto q = m.snapshot(at(t), max_age);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted");
    }
    // 4. 짧은 공백(0.5초 이하)에는 같은 ID 확인을 하지 않는다(기존 동작 유지).
    {
        auto m = locked(.5, 500);
        const double s = 1.2;  // 공백 0.4초
        m.observe(frame(30, s, {car(38, north_at(s) + 3.5, 0, s)}), at(s), max_age);
        assert(m.snapshot(at(s), max_age).phase == TargetPhase::Locked);
    }
    // 5. 새 ID가 예측에서 2.4 m 옆에 나타남: 고정 2 m 게이트는 못 잡고, 커지는 게이트(2.6 m)는
    //    확인 프레임 뒤 다시 잡는다.
    const auto reacquire = [&](double growth, double offset_east) {
        auto m = locked(growth, growth > 0 ? 500 : 0);
        TargetRecoverySnapshot q;
        for (int i = 0; i < 6; ++i) {
            const double s = t + i * .05;
            m.observe(frame(30 + i, s, {car(50, north_at(s), offset_east, s)}), at(s), max_age);
            q = m.snapshot(at(s), max_age);
            if (q.phase == TargetPhase::Locked) {
                break;
            }
        }
        return q;
    };
    {
        const auto fixed = reacquire(0, 2.4);
        assert(fixed.phase != TargetPhase::Locked);
        const auto grown = reacquire(.5, 2.4);
        assert(grown.phase == TargetPhase::Locked && grown.reason == "target_reacquired");
        assert(grown.memory->binding.track_id == 50);
    }
    // 6. 최대 4 m: 공백 3초여도 4.5 m 옆 후보는 잡지 않는다.
    {
        auto m = locked(.5, 500);
        TargetRecoverySnapshot q;
        for (int i = 0; i < 6; ++i) {
            const double s = 3.8 + i * .05;
            m.observe(frame(30 + i, s, {car(50, north_at(s), 4.5, s)}), at(s), max_age);
            q = m.snapshot(at(s), max_age);
        }
        assert(q.phase != TargetPhase::Locked);
    }
    // 7. 게이트 안 후보가 둘이면(점수 차 작음) 잡지 않는다(기존 유일성 검사 유지).
    {
        auto m = locked(.5, 500);
        TargetRecoverySnapshot q;
        for (int i = 0; i < 6; ++i) {
            const double s = t + i * .05;
            m.observe(
                frame(30 + i, s, {car(50, north_at(s), 1.0, s), car(51, north_at(s), -1.0, s)}),
                at(s), max_age);
            q = m.snapshot(at(s), max_age);
        }
        assert(q.phase != TargetPhase::Locked);
    }
    std::cout << "PASS occlusion gate: trusted return ground check, growing reacquire gate, cap, "
                 "ambiguity\n";
    return 0;
}
