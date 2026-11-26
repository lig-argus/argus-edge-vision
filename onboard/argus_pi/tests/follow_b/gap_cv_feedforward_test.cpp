// 2026-11-10 GAP_CV 속도 이어받기 시험 (PAMPC point 모드).
// 표적이 북쪽 0.5 m/s로 가다 가려진다. 공백 동안 CV 이동량(gap_displacement)만 주던 기존 방식은
// 예측 구간에서 표적이 멈춘다고 보고 늦게 따라갔다. 켜면 CV 속도로 계속 따라가는지 본다.
#include <cassert>
#include <cmath>
#include <iostream>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

namespace {

PredictiveFollowConfig base(bool feedforward) {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 2.0;
    c.max_speed_m_s = .75;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = .6;
    c.braking_acceleration_m_s2 = 1.8;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.333333333;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, 0};
    c.camera.box_anchor_y = 1;
    c.gap_velocity_feedforward = feedforward;
    return c;
}

TrackRecordV1 box() {
    TrackRecordV1 b{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    b.ground = std::array<double, 6>{2., 0, .01, 0, .01, 0};
    b.ground_anchor = std::array<double, 2>{1, 0};
    return b;
}

struct Result {
    double travelled{0};  // 기체 북쪽 이동 (m)
    double last_vx{0};
    bool all_ok{true};
};

// 1.5초 공백. 기체는 명령 속도를 그대로 따른다고 본다.
Result gap_run(bool feedforward, std::array<double, 2> passed_velocity) {
    PredictiveFollower f(base(feedforward));
    Pose observation;
    observation.have_position = observation.have_attitude = observation.have_velocity = true;
    observation.down_m = -1.6;
    observation.velocity_north_m_s = .5;
    Pose p = observation;
    const auto b = box();
    Result r;
    for (int i = 1; i <= 30; ++i) {
        const double age = .05 * i;
        const std::array<double, 2> shift{.5 * age, 0};
        const auto out = f.update(p, observation, b, passed_velocity, age, -1.6, .05, 1, false,
                                  std::nullopt, shift);
        if (!out) {
            r.all_ok = false;
            break;
        }
        f.complete_update(true);
        // yaw 0이라 body x = 북쪽
        p.north_m += out->command.vx_m_s * .05;
        p.east_m += out->command.vy_m_s * .05;
        p.velocity_north_m_s = out->command.vx_m_s;
        p.velocity_east_m_s = out->command.vy_m_s;
        r.last_vx = out->command.vx_m_s;
    }
    r.travelled = p.north_m;
    return r;
}

}  // namespace

int main() {
    const Result legacy = gap_run(false, {0, 0});
    const Result legacy_passed = gap_run(false, {.5, 0});  // 꺼져 있으면 넘겨도 무시
    const Result fed = gap_run(true, {.5, 0});
    const Result fed_zero = gap_run(true, {0, 0});  // 속도 0이면 기존과 같다
    std::cout << "legacy travelled " << legacy.travelled << " m vx " << legacy.last_vx
              << " | feedforward travelled " << fed.travelled << " m vx " << fed.last_vx
              << " (target moved 0.75 m)\n";
    assert(legacy.all_ok && legacy_passed.all_ok && fed.all_ok && fed_zero.all_ok);
    assert(std::abs(legacy.travelled - legacy_passed.travelled) < 1e-9);
    assert(std::abs(legacy.travelled - fed_zero.travelled) < 1e-9);
    // 켜면 표적 이동(0.75 m)에 가깝게 따라가고, 마지막 속도가 표적 속도 근처다.
    assert(fed.travelled > legacy.travelled + .15);
    assert(fed.last_vx > .35 && fed.last_vx <= .75 + 1e-9);

    // 공백이 아니면(이동량 없음) 켜져 있어도 point 모드는 속도를 쓰지 않는다.
    {
        PredictiveFollower on(base(true));
        PredictiveFollower off(base(false));
        Pose p;
        p.have_position = p.have_attitude = p.have_velocity = true;
        p.down_m = -1.6;
        const auto b = box();
        const auto a = on.update(p, p, b, {.5, 0}, .01, -1.6, .05);
        const auto z = off.update(p, p, b, {.5, 0}, .01, -1.6, .05);
        assert(a && z);
        assert(std::abs(a->command.vx_m_s - z->command.vx_m_s) < 1e-12);
    }
    // 너무 큰 CV 속도는 최대 속도로 제한한다.
    {
        const Result fast = gap_run(true, {5., 0});
        assert(fast.all_ok && fast.last_vx <= .75 + 1e-9);
    }
    std::cout << "PASS GAP_CV velocity feedforward\n";
    return 0;
}
