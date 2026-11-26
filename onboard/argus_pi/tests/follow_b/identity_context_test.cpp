// 2026-11-13 신원 판단 문맥(v36_identity_context) 시험. 합성 트랙·합성 열 외형만, 카메라·FC 없음.
// 2026-11-13 12:06 실내 비행: 같은 종류 경쟁차(48)가 떨어진 곳에 나타나 선택(42)보다 기억과
// 0.1~0.17 더 닮게 나오자 0.5초 만에 대상을 버렸다. 사용자 확정 규칙:
//  1) 떨어져 있으면 외형 상대 비교로 버리지 않고 기억을 계속 갱신한다 (급변만 절대 하한).
//  2) 가까워지거나 겹치면 직전 기억을 고정하고 ID로 따라가며, 떨어진 뒤 1초 비교한다.
//  3) 사라졌다 같은 ID로 돌아오면 5프레임 검증한다.
//  4) 공백 뒤 다른 ID는 따라가지 않는다 (완화 정책은 조건부).
// 2026-11-16 사용자 확정 변경 "ID를 최대한 믿는다, ID가 안 바뀌면 바꾸지 말고 새것으로 갱신":
// 2)·3)·급변 하한은 더 이상 대상을 보류·해제하지 않는다. 같은 ID는 계속 Locked이고 기억은 새
// 외형으로 갱신된다(14:58 실내: 돌아선 대상이 같은 ID로 돌아왔는데 복귀 검증으로 상실).
// 4)는 그대로다.
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "argus/control/target/include/target_manager.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
constexpr auto max_age = 250ms;
constexpr std::int64_t origin_ns = 1000000000LL;
constexpr double kDt = 1. / 30.;

Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{origin_ns + std::llround(seconds * 1e9)}};
}

// 히스토그램 16칸 중 start부터 width칸에 고르게 나눈 열 외형(대비 200 RAW). 소수 start는 선형 보간.
ThermalSignature signature(double start, int width = 4) {
    ThermalSignature s{};
    s.at(0) = 1000;
    s.at(1) = 200;
    s.at(2) = 150;
    const int base = static_cast<int>(std::floor(start));
    const double frac = start - base;
    for (int k = 0; k < width; ++k) {
        s.at(3 + base + k) += (1 - frac) / width;
        s.at(3 + base + k + 1) += frac / width;
    }
    return s;
}

struct Car {
    std::int64_t id;
    double x1;  // 박스 왼쪽 (폭 120, 높이 80)
    ThermalSignature thermal;
    double east{0};  // 지면 동쪽 위치 (북쪽은 1.8 고정)
};

TracksV1 frame(unsigned id, double seconds, const std::vector<Car>& cars) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "identity-context-contract";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns = origin_ns + std::llround(seconds * 1e9);
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.ground_pose = std::array<double, 6>{0, 0, -1.8, 0, 0, 0};
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original-ground-sample";
    f.image_width = 640;
    f.image_height = 480;
    for (const auto& car : cars) {
        TrackRecordV1 t;
        t.track_id = car.id;
        t.class_id = 0;
        t.label = "rc_car";
        t.confidence = .9;
        t.x1 = car.x1;
        t.x2 = car.x1 + 120;
        t.y1 = 200;
        t.y2 = 280;
        t.state = "CONFIRMED";
        t.real_detection = true;
        t.last_real_detection_monotonic_ns = f.sample_monotonic_ns;
        t.ground = std::array<double, 6>{1.8, car.east, .01, 0, .01, 0};
        t.ground_anchor = std::array<double, 2>{1, 0};
        t.thermal = car.thermal;
        f.tracks.push_back(t);
    }
    return f;
}

TargetRecoveryConfig config(bool v36, bool relaxed = false) {
    TargetRecoveryConfig c;
    c.enabled = true;
    c.max_gap = 3000ms;
    c.trust_selected_id = true;
    c.selected_box_jump_guard = true;
    c.thermal_identity = ThermalIdentityMode::On;
    c.v35_thermal_anchor_gate = true;
    c.v36_identity_context = v36;
    c.v36_new_id_relaxed = relaxed;
    return c;
}

// 시나리오 진행기: t초까지 cars(t)를 30 fps로 넣는다.
struct Sim {
    TargetManager m;
    unsigned id{1};
    double t{0};
    explicit Sim(TargetRecoveryConfig c, const Car& selected) : m(c) {
        const auto f = frame(id++, 0, {selected});
        m.select(f, f.tracks[0], at(0));
    }
    template <class F>
    void until(double end, F cars) {
        while (t + kDt <= end + 1e-9) {
            t += kDt;
            m.observe(frame(id++, t, cars(t)), at(t), max_age);
        }
    }
    TargetRecoverySnapshot state() const { return m.snapshot(at(t), max_age); }
};

void print(const char* name, const TargetRecoverySnapshot& s) {
    std::cout << name << ": phase " << target_phase_name(s.phase) << " reason " << s.reason
              << (s.memory ? " binding " + std::to_string(s.memory->binding.track_id) : "")
              << std::endl;
}

// 1) 떨어진 곳에 더 닮은 경쟁차 (12:06 사례): v36은 계속 따라가고, 기존은 버린다.
void distant_competitor() {
    const auto run = [](bool v36) {
        Sim s(config(v36), {17, 260, signature(9)});
        s.until(3, [](double) { return std::vector<Car>{{17, 260, signature(9)}}; });
        // 12:06 사례처럼 선택 외형이 기억과 0.8 정도로 낮아진(보는 각도 변화) 때, 처음 외형과 거의
        // 같은 경쟁차(1.0)가 떨어진 곳(간격 120px > 대각선 144px×0.5)에 나타난다.
        s.until(5, [](double) {
            return std::vector<Car>{{17, 260, signature(8.2)}, {48, 20, signature(9), 1.5}};
        });
        return s.state();
    };
    const auto legacy = run(false);
    const auto v36 = run(true);
    print("distant competitor legacy", legacy);
    print("distant competitor v36", v36);
    assert(legacy.phase != TargetPhase::Locked);
    assert(v36.phase == TargetPhase::Locked);
}

// 2) 겹친 뒤 외형이 경쟁차처럼 바뀌어도 ID가 그대로면 계속 따라간다 (2026-11-16 변경).
void overlap_swap() {
    Sim s(config(true), {17, 260, signature(9)});
    s.until(1, [](double) {
        return std::vector<Car>{{17, 260, signature(9)}, {21, 20, signature(3), 1.5}};
    });
    // 경쟁차가 바로 옆까지 온다 (간격 10px).
    s.until(2, [](double) {
        return std::vector<Car>{{17, 260, signature(9)}, {21, 130, signature(3), .5}};
    });
    // 떨어진 뒤: ID 17이 경쟁차 외형, 21이 원래 외형 (추적기가 ID를 바꿔 붙였다).
    s.until(3.5, [](double) {
        return std::vector<Car>{{17, 260, signature(3)}, {21, 20, signature(9), 1.5}};
    });
    const auto q = s.state();
    print("overlap swap", q);
    assert(q.phase == TargetPhase::Locked && q.memory && q.memory->binding.track_id == 17);
}

// 3) 겹친 뒤 외형으로 구분이 안 됨: 움직임 예측과 달라도 ID가 그대로면 계속 (2026-11-16 변경).
void overlap_undecidable() {
    const auto run = [](double selected_east_after) {
        Sim s(config(true), {17, 260, signature(9)});
        s.until(1, [](double) {
            return std::vector<Car>{{17, 260, signature(9)}, {21, 20, signature(9.2), 1.0}};
        });
        s.until(2, [](double) {
            return std::vector<Car>{{17, 260, signature(9)}, {21, 140, signature(9.2), 1.0}};
        });
        s.until(3.5, [=](double) {
            return std::vector<Car>{{17, 260, signature(9.1), selected_east_after},
                                    {21, 20, signature(9.1), 1.0}};
        });
        return s.state();
    };
    const auto consistent = run(0);
    const auto jumped = run(3.0);
    print("overlap undecidable, motion consistent", consistent);
    print("overlap undecidable, motion inconsistent", jumped);
    assert(consistent.phase == TargetPhase::Locked);
    assert(jumped.phase == TargetPhase::Locked && jumped.memory &&
           jumped.memory->binding.track_id == 17);
}

// 4) 사라졌다 같은 ID로 돌아옴: 외형이 달라져도 계속 따라간다 (2026-11-16 변경).
void returned_same_id() {
    const auto run = [](const ThermalSignature& back) {
        Sim s(config(true), {17, 260, signature(9)});
        s.until(1, [](double) { return std::vector<Car>{{17, 260, signature(9)}}; });
        s.until(1.5, [](double) { return std::vector<Car>{}; });
        s.until(2.2, [=](double) { return std::vector<Car>{{17, 260, back}}; });
        return s.state();
    };
    const auto other = run(signature(4));
    const auto same = run(signature(9.1));
    print("returned id, different look", other);
    print("returned id, same look", same);
    assert(other.phase == TargetPhase::Locked && other.memory &&
           other.memory->binding.track_id == 17);
    assert(same.phase == TargetPhase::Locked);
}

// 5) 공백 뒤 다른 ID: 기본은 따라가지 않고, 완화 정책은 후보 하나·강한 유사도일 때만 이어받는다.
void different_id() {
    const auto run = [](bool relaxed, bool two_candidates) {
        Sim s(config(true, relaxed), {17, 260, signature(9)});
        s.until(1, [](double) { return std::vector<Car>{{17, 260, signature(9)}}; });
        s.until(1.3, [](double) { return std::vector<Car>{}; });
        s.until(2.5, [=](double) {
            std::vector<Car> cars{{30, 262, signature(9)}};
            if (two_candidates) {
                cars.push_back({31, 20, signature(9), 1.5});
            }
            return cars;
        });
        return s.state();
    };
    const auto strict = run(false, false);
    const auto relaxed = run(true, false);
    const auto relaxed_two = run(true, true);
    print("different id, default", strict);
    print("different id, relaxed single", relaxed);
    print("different id, relaxed two candidates", relaxed_two);
    assert(strict.phase != TargetPhase::Locked && strict.memory &&
           strict.memory->binding.track_id == 17);
    assert(relaxed.phase == TargetPhase::Locked && relaxed.memory &&
           relaxed.memory->binding.track_id == 30);
    assert(relaxed_two.phase != TargetPhase::Locked);
}

// 6) 기억이 계속 갱신된다: 큰 완만한 변화도, 같은 ID의 급변도 계속 따라간다 (2026-11-16 변경).
void continuous_update() {
    const auto run = [](bool v36, auto look) {
        Sim s(config(v36), {17, 260, signature(9)});
        s.until(4, [&](double t) { return std::vector<Car>{{17, 260, look(t)}}; });
        return s.state();
    };
    const auto drift = [](double t) { return signature(9 - t * .5); };  // 4초에 2칸 (원래와 0.5)
    const auto abrupt = [](double t) { return t < 1 ? signature(9) : signature(6); };
    const auto legacy_drift = run(false, drift);
    const auto v36_drift = run(true, drift);
    const auto v36_abrupt = run(true, abrupt);
    print("large slow drift legacy", legacy_drift);
    print("large slow drift v36", v36_drift);
    print("abrupt change v36", v36_abrupt);
    assert(v36_drift.phase == TargetPhase::Locked);
    assert(v36_abrupt.phase == TargetPhase::Locked && v36_abrupt.memory &&
           v36_abrupt.memory->binding.track_id == 17);
}

}  // namespace

int main() {
    distant_competitor();
    overlap_swap();
    overlap_undecidable();
    returned_same_id();
    different_id();
    continuous_update();
    std::cout << "PASS identity context: distant competitor kept, same id kept after overlap, "
                 "same id kept on return, different id policy, continuous update\n";
    return 0;
}
