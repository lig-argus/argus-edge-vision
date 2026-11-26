// 2026-11-16 사용자 지시 "3초 가라니깐 왜 1초만 감" 시험. 합성 트랙만, 카메라·FC 없음.
// 공백 중 같은 선택 ID가 돌아왔지만 이번 프레임을 받아들이지 않은 판정 보류(지면 예측 복귀
// 밖, 박스 튐)에서도 GAP_CV가 마지막 실관측 + 3초 기한까지 이어져야 한다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "argus/control/target/include/target_selector.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
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

// north는 지면 북쪽 위치, x1은 박스 왼쪽(폭 60, 높이 60). detected=false면 트랙 없음.
TracksV1 input(unsigned id, double seconds, bool detected, double north = 0, double x1 = 290,
               std::int64_t track_id = 1) {
    TracksV1 f;
    f.schema = kTracksV4Schema;
    f.source = "gap-cv-same-id-hold-contract";
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
        TrackRecordV1 t{track_id,
                        0,
                        "rc_car",
                        .9,
                        x1,
                        100,
                        x1 + 60,
                        160,
                        "CONFIRMED",
                        true,
                        f.sample_monotonic_ns};
        t.ground = std::array<double, 6>{north, 0, .01, 0, .01, 0};
        t.ground_anchor = std::array<double, 2>{1, 0};
        f.tracks = {t};
    }
    return f;
}

TargetSelectorConfig config() {
    TargetSelectorConfig c;
    c.recovery.enabled = true;
    c.recovery.max_gap = 4000ms;
    c.recovery.trust_selected_id = true;
    c.recovery.trusted_return_check = 500ms;
    c.recovery.selected_box_jump_guard = true;
    c.recovery.v3_box_motion_guard = true;
    c.recovery.v3_fresh_contact_velocity = true;
    c.recovery.short_motion.enabled = true;
    c.recovery.short_motion.window_s = .25;
    c.recovery.short_motion.minimum_span_s = .15;
    c.recovery.short_motion.minimum_samples = 4;
    return c;
}

// 0~2초 0.6 m/s로 북쪽 이동하며 보이다가 2초 뒤 사라진다. 마지막 실관측은 frame 41(2.0초).
void follow_then_vanish(TargetSelector& selector) {
    for (unsigned i = 0; i <= 40; ++i) {
        const double t = i * .05;
        assert(selector.update(input(i + 1, t, true, 3. + .6 * t), at(t)));
        if (i == 0) {
            assert(selector.execute({"select", TargetCommandAction::Select, 1, 1}, at(0)).accepted);
            assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
        }
    }
    assert(selector.update(input(42, 2.05, false), at(2.05)));
    const auto view = selector.snapshot(at(2.05));
    assert(view.cv_gap_active && view.cv_gap_deadline == at(5));
}

void print(const char* name, const TargetSelectionSnapshot& view) {
    std::cout << name << ": reason " << view.recovery_reason << " cv_gap_active "
              << view.cv_gap_active << std::endl;
}

// 1초 공백 뒤 같은 ID가 지면 예측(약 3+0.6×3=4.8 m)과 먼 곳(9 m)에서 돌아온다: 받아들이지 않지만
// GAP_CV는 이어진다. 기한은 그대로 2.0초 + 3초. 이후 프레임도 매번 다시 판정해(실패 여러 번 뒤에도)
// 예측과 맞는 위치로 돌아오면 바로 받아들인다.
void return_outside_ground_gate() {
    TargetSelector selector(config());
    follow_then_vanish(selector);
    assert(selector.update(input(43, 2.5, false), at(2.5)));
    unsigned id = 44;
    for (double t = 3.0; t <= 3.5 + 1e-9; t += .05) {
        assert(selector.update(input(id++, t, true, 9.), at(t)));
        const auto view = selector.snapshot(at(t));
        assert(view.recovery_pause && view.cv_gap_active && view.cv_gap_deadline == at(5));
        assert(view.cv_origin_sample_ns == input(41, 2, true).sample_monotonic_ns);
    }
    print("return outside ground gate (11 frames)", selector.snapshot(at(3.5)));
    assert(selector.update(input(id++, 3.55, true, 3. + .6 * 3.55), at(3.55)));
    const auto back = selector.snapshot(at(3.55));
    print("return inside ground gate after failures", back);
    assert(!back.recovery_pause && back.recovery_reason == "selected_native_id_trusted");
}

// 짧은 공백(0.1초) 뒤 같은 ID 박스가 화면 반대편으로 튄다: 받아들이지 않지만 GAP_CV는 이어진다.
// 튐이 0.5초를 넘게 이어져도 버리지 않고(예전: selected_id_box_jump_persisted 상실) 매 프레임 다시
// 판정하다가, 원래 자리로 돌아오면 받아들인다.
void box_jump() {
    TargetSelector selector(config());
    follow_then_vanish(selector);
    unsigned id = 43;
    for (double t = 2.1; t <= 2.9 + 1e-9; t += .05) {
        assert(selector.update(input(id++, t, true, 3. + .6 * 2., 10), at(t)));
        const auto view = selector.snapshot(at(t));
        assert(view.recovery_reason == "selected_id_box_jump");
        assert(view.recovery_pause && view.cv_gap_active && view.cv_gap_deadline == at(5));
    }
    print("box jump persisted 0.8 s", selector.snapshot(at(2.9)));
    assert(selector.update(input(id++, 2.95, true, 3. + .6 * 2.), at(2.95)));
    const auto back = selector.snapshot(at(2.95));
    print("box back after jump", back);
    assert(!back.recovery_pause && back.recovery_reason == "selected_native_id_trusted");
}

// 1초 공백 뒤 예측 위치에 다른 ID(7)가 나타나 후보 확인(Confirm)에 들어가도 확정 전까지 GAP_CV는
// 이어진다 (14:58: 확인 단계로 넘어가며 GAP_CV가 꺼져 제자리 정지).
void confirm_keeps_gap_cv() {
    TargetSelector selector(config());
    follow_then_vanish(selector);
    assert(selector.update(input(43, 2.5, false), at(2.5)));
    bool saw_confirm = false;
    unsigned id = 44;
    for (double t = 3.0; t <= 3.3 + 1e-9; t += .05) {
        assert(selector.update(input(id++, t, true, 3. + .6 * t, 290, 7), at(t)));
        const auto view = selector.snapshot(at(t));
        if (!view.recovery_pause) {
            break;  // 확정됐다.
        }
        print("  candidate frame", view);
        saw_confirm = saw_confirm || view.target_phase == TargetPhase::Confirm;
        assert(view.cv_gap_active && view.cv_gap_deadline == at(5));
    }
    assert(saw_confirm);
}

// 모호한 공백(다른 차량과 구분 못함)은 바꾸지 않았다: 여기서는 사라짐만 확인한 기존 동작 유지.
void plain_missing_unchanged() {
    TargetSelector selector(config());
    follow_then_vanish(selector);
    assert(selector.update(input(43, 3.0, false), at(3.0)));
    const auto view = selector.snapshot(at(3.0));
    print("plain missing", view);
    assert(view.cv_gap_active && view.cv_gap_deadline == at(5));
}
}  // namespace

int main() {
    env("ARGUS_V21_ENABLED", "1");
    env("ARGUS_V21_CV_GAP", "1");
    env("ARGUS_V23_CV_RAW_MOTION", "1");
    env("ARGUS_V23_CV_MOVING_ONLY", "1");
    return_outside_ground_gate();
    box_jump();
    confirm_keeps_gap_cv();
    plain_missing_unchanged();
    std::cout << "PASS gap cv same-id hold: ground gate return and box jump keep GAP_CV to 3 s\n";
    return 0;
}
