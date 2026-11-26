// 2026-11-10 열 외형 원래 기준 고정·갱신 제한(v35_thermal_anchor_gate) 시험.
// 실내 15:28 비행: 실제 차가 화면에서 빠진 뒤 다른 물체가 차로 잘못 검출돼 같은 ID를 받았다.
// 비교 후보가 없어 불일치가 나지 않았고, 기억이 매 프레임 그 물체 쪽으로 갱신돼 약 6초 추종했다.
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "argus/control/target/include/target_manager.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
constexpr auto max_age = 250ms;
constexpr std::int64_t origin_ns = 1000000000LL;

Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{origin_ns + std::llround(seconds * 1e9)}};
}

// 히스토그램 16칸 중 start부터 width칸에 고르게 나눈 열 외형(대비 200 RAW).
ThermalSignature signature(double start, int width = 4) {
    ThermalSignature s{};
    s.at(0) = 1000;
    s.at(1) = 200;
    s.at(2) = 150;
    // 소수 start는 이웃 칸 사이를 선형으로 나눠 점진 변화를 만든다.
    const int base = static_cast<int>(std::floor(start));
    const double frac = start - base;
    for (int k = 0; k < width; ++k) {
        s.at(3 + base + k) += (1 - frac) / width;
        s.at(3 + base + k + 1) += frac / width;
    }
    return s;
}

TracksV1 frame(unsigned id, double seconds, const ThermalSignature& thermal) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "thermal-anchor-contract";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.ground_pose = std::array<double, 6>{0, 0, -1.8, 0, 0, 0};
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original-ground-sample";
    f.image_width = 640;
    f.image_height = 480;
    TrackRecordV1 t;
    t.track_id = 17;
    t.class_id = 0;
    t.label = "rc_car";
    t.confidence = .9;
    t.x1 = 260;
    t.x2 = 380;
    t.y1 = 200;
    t.y2 = 280;
    t.state = "CONFIRMED";
    t.real_detection = true;
    t.last_real_detection_monotonic_ns = f.sample_monotonic_ns;
    t.ground = std::array<double, 6>{1.8, 0, .01, 0, .01, 0};
    t.ground_anchor = std::array<double, 2>{1, 0};
    t.thermal = thermal;
    f.tracks = {t};
    assert(valid_thermal_signature(thermal));
    return f;
}

TargetRecoveryConfig config(bool gate) {
    TargetRecoveryConfig c;
    c.enabled = true;
    c.max_gap = 3000ms;
    c.trust_selected_id = true;
    c.thermal_identity = ThermalIdentityMode::On;
    c.v35_thermal_anchor_gate = gate;
    return c;
}

struct Result {
    TargetPhase phase;
    std::string reason;
    bool ever_low{false};
};

// 0~1초 원래 외형(A), 이후 1~4초는 change(t)가 주는 외형을 같은 ID로 받는다.
template <class F>
Result run(bool gate, F change) {
    TargetManager m(config(gate));
    unsigned id = 1;
    const auto a = signature(9);
    m.select(frame(id, 0, a), frame(id, 0, a).tracks[0], at(0));
    ++id;
    for (int i = 1; i <= 20; ++i) {
        m.observe(frame(id++, i * .05, a), at(i * .05), max_age);
    }
    Result r{};
    for (int i = 21; i <= 80; ++i) {
        const double t = i * .05;
        m.observe(frame(id++, t, change(t)), at(t), max_age);
        const auto q = m.snapshot(at(t), max_age);
        if (q.reason.find("thermal_low_similarity") != std::string::npos) {
            r.ever_low = true;
        }
    }
    const auto q = m.snapshot(at(4), max_age);
    r.phase = q.phase;
    r.reason = q.reason;
    return r;
}

}  // namespace

int main() {
    // 다른 물체: 원래 외형과 교집합 약 0.25(칸 3개 이동). 비교 후보는 없다.
    const auto other = [](double) { return signature(6); };
    const double sim = compare_thermal(signature(9), signature(6)).histogram;
    std::cout << "other object similarity to selection " << sim << std::endl;
    assert(sim < .5);

    const auto legacy = run(false, other);
    std::cout << "legacy: phase " << static_cast<int>(legacy.phase) << " reason " << legacy.reason
              << std::endl;
    assert(legacy.phase == TargetPhase::Locked);  // 기존: 계속 같은 대상으로 본다
    assert(!legacy.ever_low);

    const auto gated = run(true, other);
    std::cout << "gated: phase " << static_cast<int>(gated.phase) << " reason " << gated.reason
              << std::endl;
    assert(gated.ever_low);
    assert(gated.phase != TargetPhase::Locked);  // 켜면 같은 대상으로 보지 않는다

    // 정상 변화: 3초 동안 외형이 1칸씩 천천히 바뀐다(각도·거리 변화). 계속 Locked여야 한다.
    const auto drift = [](double t) { return signature(9 - (t - 1) * (1. / 3.)); };
    const auto slow = run(true, drift);
    std::cout << "gated slow drift: phase " << static_cast<int>(slow.phase) << " reason "
              << slow.reason << std::endl;
    assert(slow.phase == TargetPhase::Locked && !slow.ever_low);
    std::cout << "PASS thermal anchor gate: frozen anchor, gated update, absolute floor\n";
    return 0;
}
