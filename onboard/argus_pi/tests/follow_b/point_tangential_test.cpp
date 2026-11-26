// 2026-11-10 PAMPC point 둘레 방향 상대속도 벌점 시험.
// 실외 11:55·12:03 비행: 정지 대상 주위를 옆속도 약 1m/s로 원을 그리며 돌았다.
// 끄면(0) 이전 계획의 옆속도를 이어 쓰고, 켜면 같은 상태에서 옆 명령을 줄여야 한다.
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

namespace {

PredictiveFollowConfig config(double weight) {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;
    // 실제 설정처럼 송신 slew를 켠다. slew는 steady_clock 간격을 쓰므로 호출 사이를 실제로
    // 기다린다.
    c.sent_command_slew = true;
    c.point_tangential_weight = weight;
    c.outdoor_profile = true;
    c.range_m = 2;
    c.min_height_m = 3.5;
    c.max_height_m = 4.5;
    c.target_height_m = 4;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = 1;
    c.braking_acceleration_m_s2 = 1.8;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = 946.65;
    c.camera.fy = 853.01;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, .01};
    return c;
}

// 대상은 북쪽 2.3m(거리 무시 구간 안)에 정지. 드론은 대상을 보며 동쪽(오른쪽)으로 0.9m/s 이동 중.
Pose pose() {
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -4;
    p.velocity_east_m_s = .9;
    return p;
}

TrackRecordV1 box(std::int64_t ns) {
    TrackRecordV1 b{1, 1, "car", .9, 260, 150, 380, 330, "CONFIRMED", true, ns};
    b.ground = std::array<double, 6>{2.3, 0, .03, 0, .03, 20};
    b.ground_anchor = std::array<double, 2>{1, 0};
    return b;
}

// 같은 상태를 여러 번 넣어 warm start가 쌓인 뒤의 옆 명령(body right)을 본다.
double lateral(double weight) {
    PredictiveFollower f(config(weight));
    double vy = 0;
    for (int i = 0; i < 8; ++i) {
        const std::int64_t ns = 1000000000LL + i * 50000000LL;
        const auto r = f.update(pose(), pose(), box(ns), {0, 0}, .02, -4, .05, 1, false, ns);
        assert(r);
        f.complete_update(true);
        vy = r->command.vy_m_s;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return vy;
}

}  // namespace

int main() {
    const double legacy = lateral(0), damped = lateral(10);
    std::cout << "lateral command legacy " << legacy << " m/s, tangential weight 10 " << damped
              << " m/s (current 0.9)\n";
    assert(legacy > .6);  // 기존: 옆속도를 거의 유지
    // 켜면 둘레 방향 옆 명령이 줄어든다. 고정 상태 반복 입력 측정: 0.65→0.51m/s.
    // 실제 기록 Replay(자세가 변하는 입력)에서는 0.9→0.5m/s로 더 크게 줄었다.
    assert(damped < legacy - .1);
    assert(damped > -.2);  // 반대 방향으로 튀지 않는다
    bool threw = false;
    try {
        PredictiveFollower bad(config(60));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "PASS point tangential weight damps orbit around stationary target\n";
    return 0;
}
