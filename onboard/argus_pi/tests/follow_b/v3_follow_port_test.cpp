// V3 (userpc SITL 검증본) 추종 기능 이식 검사.
// 설정 범위(실내·실외 기존 방식만), 잘못된 조합 거부, point 모드 명령, I항 송신 확정을 본다.
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;

namespace {
template <class F>
bool rejects(F&& action) {
    try {
        action();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

PredictiveFollowConfig base_config(bool outdoor) {
    PredictiveFollowConfig config;
    config.camera.image_width = 640;
    config.camera.image_height = 480;
    config.camera.fx = config.camera.fy = 500;
    config.camera.cx = 320;
    config.camera.cy = 240;
    config.camera.camera_to_body = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    config.camera.box_anchor_x = config.camera.box_anchor_y = 0.5;
    config.max_speed_m_s = 1.5;
    config.max_acceleration_m_s2 = 1.0;
    config.braking_acceleration_m_s2 = 1.8;
    if (outdoor) {
        config.outdoor_profile = true;
        config.range_m = 4.0;
        config.min_height_m = 3.5;
        config.max_height_m = 4.5;
        config.target_height_m = 4.0;
    } else {
        config.range_m = 2.0;
        config.min_height_m = 1.1;
        config.max_height_m = 1.9;
    }
    config.pampc_point_profile = 1;
    config.image_velocity_weight = .25;
    config.integral_gain = .02;
    return config;
}

TrackRecordV1 box_at(double north) {
    TrackRecordV1 box{4, 5, "rc_car", 0.9, 290, 210, 350, 270, "CONFIRMED", true, 1000000000};
    box.ground = std::array<double, 6>{north, 0.0, .1, 0, .1, 0};
    box.ground_anchor = std::array<double, 2>{0.5, 0.0};
    return box;
}

Pose pose_at(double down) {
    Pose pose;
    pose.down_m = down;
    pose.have_position = pose.have_attitude = pose.have_velocity = true;
    return pose;
}
}  // namespace

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string dir = argv[1];

    // 1. 설정 범위: 실내 이동·실외 기존 방식만 V3, 실외 PID와 yaw는 끔.
    const auto indoor = load_bus_following_config(dir + "/moving.config");
    const auto outdoor = load_bus_following_config(dir + "/outdoor-predictive-moving.config");
    for (const auto* config : {&indoor, &outdoor}) {
        assert(config->guidance.v3_pampc_point_profile == 1);
        assert(config->guidance.v32_sent_command_slew);
        assert(config->selector.recovery.trust_selected_id);
        assert(config->selector.recovery.v32_cv_ten_frame);
        assert(config->guidance.v3_image_velocity_weight == .25);
        assert(config->guidance.v3_predictive_integral);
        assert(config->guidance.v3_discard_recheck);
        assert(config->selector.recovery.v3_box_motion_guard);
        assert(config->selector.recovery.v3_fresh_contact_velocity);
        assert(config->guidance.max_forward_m_s == 1.5);
        assert(config->guidance.max_horizontal_accel_m_s2 == 1.0);
        (void)FlightGuidanceController(config->guidance);
    }
    // 실외는 4m 기준 고도·거리 설정을 유지한다.
    assert(outdoor.guidance.desired_range_m == 4.0);
    assert(outdoor.guidance.predictive_target_height_m == 4.0);
    assert(outdoor.guidance.predictive_min_height_m == 3.5);
    assert(outdoor.guidance.predictive_max_height_m == 4.5);
    assert(indoor.guidance.desired_range_m == 2.0);
    for (const char* name : {"outdoor-pid-moving.config", "yaw.config", "outdoor-yaw.config"}) {
        const auto other = load_bus_following_config(dir + "/" + name);
        assert(other.guidance.v3_pampc_point_profile == 0);
        assert(!other.guidance.v32_sent_command_slew);
        assert(other.selector.recovery.trust_selected_id);
        assert(other.guidance.v3_image_velocity_weight == 0);
        assert(!other.guidance.v3_predictive_integral);
        assert(!other.guidance.v3_discard_recheck);
        assert(!other.selector.recovery.v3_box_motion_guard);
        assert(!other.selector.recovery.v3_fresh_contact_velocity);
    }

    // 2. PID·yaw 설정에 V3를 켜면 거부한다.
    auto pid = load_bus_following_config(dir + "/outdoor-pid-moving.config").guidance;
    pid.v3_pampc_point_profile = 1;
    assert(rejects([&] { (void)FlightGuidanceController(pid); }));
    auto yaw = load_bus_following_config(dir + "/yaw.config").guidance;
    yaw.v3_discard_recheck = true;
    assert(rejects([&] { (void)FlightGuidanceController(yaw); }));
    auto bad_profile = base_config(false);
    bad_profile.pampc_point_profile = 5;
    assert(rejects([&] { (void)PredictiveFollower(bad_profile); }));
    auto bad_weight = base_config(false);
    bad_weight.pampc_point_profile = 0;
    bad_weight.image_velocity_weight = .3;
    assert(rejects([&] { (void)PredictiveFollower(bad_weight); }));

    // 3. 실내 point 모드: 유한한 명령, 표적 속도 입력은 쓰지 않음, I항은 송신 성공 때만 확정.
    {
        const auto config = base_config(false);
        PredictiveFollower a(config), b(config);
        const auto pose = pose_at(-1.5);
        const auto box = box_at(2.8);  // 2.0±0.3m 범위 밖 → 거리 오차가 있다.
        const auto out_a = a.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
        const auto out_b = b.update(pose, pose, box, {0.6, 0.2}, 0.0, -1.5, 0.05);
        assert(out_a && out_b);
        assert(std::isfinite(out_a->command.vx_m_s + out_a->command.vy_m_s + out_a->command.vz_m_s +
                             out_a->command.yaw_rate_deg_s));
        assert(std::abs(out_a->command.vx_m_s - out_b->command.vx_m_s) < 1e-9);
        assert(std::abs(out_a->command.vy_m_s - out_b->command.vy_m_s) < 1e-9);
        assert(std::hypot(out_a->command.vx_m_s, out_a->command.vy_m_s) <= 1.5 + 1e-9);
        const auto& pending = a.integral_diagnostics();
        assert(pending.enabled && pending.pending);
        const auto proposed = pending.proposed;
        assert(proposed[0] != 0 || proposed[1] != 0);
        a.complete_update(true);
        assert(a.integral_diagnostics().committed == proposed);
        b.complete_update(false);
        assert(b.integral_diagnostics().committed[0] == 0 &&
               b.integral_diagnostics().committed[1] == 0);
        a.reset_motion();
        assert(a.integral_diagnostics().committed[0] == 0);
    }

    // 4. 실외 point 모드: 4m 목표 높이를 유지한다.
    {
        const auto config = base_config(true);
        PredictiveFollower follower(config);
        const auto pose = pose_at(-4.0);
        const auto output = follower.update(pose, pose, box_at(5.0), {0.0, 0.0}, 0.0, -4.0, 0.05);
        assert(output);
        assert(std::abs(output->altitude_target_down_m + 4.0) < 1e-9);
        assert(std::abs(output->vertical_diagnostics.height_reference_m - 4.0) < 1e-9);
        assert(std::isfinite(output->command.vx_m_s + output->command.vz_m_s));
    }

    // 5. V3를 끈 기존 설정은 I항을 준비하지 않는다.
    {
        auto config = base_config(false);
        config.pampc_point_profile = 0;
        config.image_velocity_weight = 0;
        config.integral_gain = 0;
        PredictiveFollower follower(config);
        const auto pose = pose_at(-1.5);
        assert(follower.update(pose, pose, box_at(2.8), {0.0, 0.0}, 0.0, -1.5, 0.05));
        assert(!follower.integral_diagnostics().enabled);
        assert(!follower.integral_diagnostics().pending);
    }
    std::cout
        << "V3 follow port: profile scope, rejection, point mode and integral commit passed\n";
}
