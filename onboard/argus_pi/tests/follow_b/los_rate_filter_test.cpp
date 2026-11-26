// 2026-11-16 요 시드 시선각 변화율(los_rate) 저역 필터 시험.
// ① 0(꺼짐)은 기본과 출력이 완전히 같다 ② 속도 잡음에서 요 명령 흔들림이 줄어든다
// ③ 일정 속도에서는 꺼짐과 같은 값으로 수렴한다 ④ 잘못된 값은 거부한다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

namespace {
PredictiveFollowConfig make_config() {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;  // 실제 실내 설정 v3_pampc_point_profile=1
    c.image_velocity_weight = .25;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 1.7;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .3;
    c.max_acceleration_m_s2 = 1;
    c.braking_acceleration_m_s2 = 1.8;
    c.max_yaw_rate_deg_s = 75;
    c.max_yaw_accel_deg_s2 = 300;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.333333333;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, 0};
    c.camera.box_anchor_y = 1;
    return c;
}

// 같은 기체 속도 열(실내 플로우 추정 잡음 흉내)을 넣고 요 명령(deg/s) 열을 돌려준다.
// 실제 설정(point 모드)은 표적 속도를 0으로 보므로 시선각 항은 기체 속도에서 나온다.
std::vector<double> run(PredictiveFollower& f, const std::vector<std::array<double, 2>>& vehicle) {
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.6;
    TrackRecordV1 box{
        1,   2,   "car",       .9,   340,         250,
        370, 280, "CONFIRMED", true, 1000000000LL};  // 작은 박스: 시야 여유 (일반 경로)
    box.ground = std::array<double, 6>{2.0, 0.3, .01, 0, .01, 0};
    box.ground_anchor = std::array<double, 2>{1, 0};
    std::vector<double> yaw;
    for (const auto& v : vehicle) {
        p.velocity_north_m_s = v[0];
        p.velocity_east_m_s = v[1];
        const auto r = f.update(p, p, box, {0., 0.}, .01, -1.6, 1. / 30, 1, false,
                                box.last_real_detection_monotonic_ns);
        if (!r) {
            throw std::runtime_error("predictive update rejected: " + f.last_reject_reason());
        }
        yaw.push_back(r->command.yaw_rate_deg_s);
        f.complete_update(true);
    }
    return yaw;
}

double diff_rms(const std::vector<double>& y, std::size_t from) {
    double sum = 0;
    std::size_t n = 0;
    for (std::size_t i = from + 1; i < y.size(); ++i) {
        sum += (y[i] - y[i - 1]) * (y[i] - y[i - 1]);
        ++n;
    }
    return std::sqrt(sum / static_cast<double>(n));
}
}  // namespace

int main() {
    setenv("ARGUS_COMPARISON_ISOLATED", "1", 1);
    setenv("ARGUS_SITL_PAMPC_POINT_ONLY", "1", 1);
    setenv("ARGUS_SITL_QUATERNION_STAGE", "0", 1);
    setenv("ARGUS_SITL_OPTIMIZER_MODE", "0", 1);
    const auto config = make_config();

    std::mt19937 rng(1116);
    std::normal_distribution<double> noise(0., .15);
    std::vector<std::array<double, 2>> noisy;
    for (int i = 0; i < 120; ++i) {
        noisy.push_back({0.0, 0.1 + noise(rng)});  // 기체 옆 속도 0.1m/s + 추정 잡음 0.15
    }
    // ① 꺼짐(0)은 기본과 같다
    PredictiveFollower base(config), zero(config);
    zero.set_los_rate_filter(0);
    const auto y_base = run(base, noisy);
    const auto y_zero = run(zero, noisy);
    for (std::size_t i = 0; i < y_base.size(); ++i) {
        assert(y_base[i] == y_zero[i]);
    }
    // ② 잡음에서 흔들림 감소
    PredictiveFollower filtered(config);
    filtered.set_los_rate_filter(.1);
    const auto y_filtered = run(filtered, noisy);
    const double off = diff_rms(y_base, 10), on = diff_rms(y_filtered, 10);
    std::cerr << "요 명령 연속 변화 RMS: 꺼짐 " << off << " deg/s, τ0.1s " << on << " deg/s\n";
    for (int i = 20; i < 26; ++i) {
        std::cerr << "  " << i << ": " << y_base[i] << " / " << y_filtered[i] << '\n';
    }
    assert(on < .7 * off);
    // ③ 일정 속도면 같은 값으로 수렴
    std::vector<std::array<double, 2>> steady(90, std::array<double, 2>{0.0, 0.1});
    PredictiveFollower a(config), b(config);
    b.set_los_rate_filter(.1);
    const auto ya = run(a, steady);
    const auto yb = run(b, steady);
    std::cout << "일정 속도 마지막 요: 꺼짐 " << ya.back() << ", τ0.1s " << yb.back() << '\n';
    assert(std::abs(ya.back() - yb.back()) < .05);
    // ④ 잘못된 값 거부
    bool rejected = false;
    try {
        a.set_los_rate_filter(-0.1);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
    rejected = false;
    try {
        a.set_los_rate_filter(NAN);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
    std::cout << "LosRateFilterTest PASS\n";
    return 0;
}
