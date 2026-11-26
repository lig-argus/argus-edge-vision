// 2026-11-10 point 모드 거리 무시 구간 설정 시험.
// 표적이 목표 거리(2 m)보다 0.35 m 멀 때: 기본 0.3 m면 전진하고, 0.4 m면 거리 오차로는 움직이지
// 않는다.
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

namespace {

PredictiveFollowConfig config(double deadband) {
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
    c.point_range_deadband_m = deadband;
    return c;
}

double forward(double deadband, double target_north) {
    PredictiveFollower f(config(deadband));
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.6;
    TrackRecordV1 b{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    b.ground = std::array<double, 6>{target_north, 0, .01, 0, .01, 0};
    b.ground_anchor = std::array<double, 2>{1, 0};
    double vx = 0;
    for (int i = 0; i < 10; ++i) {
        const auto out = f.update(p, p, b, {0, 0}, .01, -1.6, .05);
        assert(out);
        f.complete_update(true);
        vx = out->command.vx_m_s;
        p.velocity_north_m_s = vx;
    }
    return vx;
}

bool rejects(double deadband, int profile = 1) {
    auto c = config(deadband);
    c.pampc_point_profile = profile;
    try {
        PredictiveFollower f(c);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

}  // namespace

int main() {
    assert(rejects(.05) && rejects(1.5) && rejects(-.1) && rejects(.4, 0));
    assert(!rejects(0) && !rejects(.4));
    const double legacy = forward(0, 2.35), wide = forward(.4, 2.35);
    std::cout << "0.35 m too far: default(0.3) vx " << legacy << " | 0.4 vx " << wide << '\n';
    assert(legacy > wide + .05);
    // 무시 구간 밖(0.6 m 멀리)이면 0.4여도 전진한다
    const double far = forward(.4, 2.6);
    std::cout << "0.6 m too far with 0.4: vx " << far << '\n';
    assert(far > .05);
    std::cout << "PASS point range deadband setting\n";
    return 0;
}
