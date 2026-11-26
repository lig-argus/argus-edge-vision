#include <cassert>
#include <cmath>
#include <iostream>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

int main() {
    PredictiveFollowConfig config;
    config.camera.image_width = 640;
    config.camera.image_height = 480;
    config.camera.fx = config.camera.fy = 500;
    config.camera.cx = 320;
    config.camera.cy = 240;
    config.camera.camera_to_body = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    config.camera.box_anchor_x = config.camera.box_anchor_y = 0.5;
    config.min_height_m = 1.2;
    config.max_height_m = 1.8;
    config.max_acceleration_m_s2 = 0.6;

    PredictiveFollower predictor(config);
    Pose pose;
    pose.down_m = -1.5;
    pose.have_position = pose.have_attitude = pose.have_velocity = true;
    pose.velocity_down_m_s = 0.4;
    TrackRecordV1 box{4, 5, "rc_car", 0.9, 270, 200, 370, 260, "CONFIRMED", true, 1000000000};
    box.ground = std::array<double, 6>{2.0, 0.0, .1, 0, .1, 0};
    box.ground_anchor = std::array<double, 2>{0.5, 0.0};

    auto output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    const auto& diagnostics = output->vertical_diagnostics;
    assert(std::isfinite(diagnostics.best_score));
    assert(std::abs(diagnostics.best_score - diagnostics.height_cost - diagnostics.visibility_cost -
                    diagnostics.boundary_cost - diagnostics.other_cost) < 1e-8);
    assert(diagnostics.up_candidate_score && diagnostics.zero_candidate_score &&
           diagnostics.down_candidate_score);
    // At acquisition, use the measured FC vertical speed as the continuity reference.
    assert(std::abs(output->vertical_diagnostics.previous_down_m_s - .4) < 1e-9);
    assert(std::abs(output->command.vz_m_s - .4) <= .030001);
    assert(output->vertical_diagnostics.safety_flags == 0);
    double last = output->command.vz_m_s;
    for (int i = 0; i < 80; ++i) {
        // Alternate visual detections to exercise replanning, not a synthetic direct clamp.
        box.y1 = i % 2 ? 120 : 280;
        box.y2 = i % 2 ? 190 : 350;
        box.last_real_detection_monotonic_ns += 50000000;
        output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
        assert(output);
        assert(output->vertical_diagnostics.safety_flags == 0);
        assert(std::abs(output->command.vz_m_s - last) <= .030001);
        last = output->command.vz_m_s;
    }

    // Height safety must immediately suppress descent even if slew would otherwise retain it.
    predictor.reset_motion();
    pose.down_m = -1.19;
    pose.velocity_down_m_s = .4;
    box.y1 = 200;
    box.y2 = 260;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    assert(output->vertical_diagnostics.limited_down_m_s > 0);
    assert((output->vertical_diagnostics.safety_flags & 4) != 0);
    assert(output->command.vz_m_s <= 0);
    pose.down_m = -1.5;
    pose.velocity_down_m_s = 0;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    assert(std::abs(output->vertical_diagnostics.previous_down_m_s) < 1e-9);
    assert(std::abs(output->command.vz_m_s) <= .030001);

    // Upper-height braking likewise takes precedence over the ordinary slew bound.
    predictor.reset_motion();
    pose.down_m = -1.81;
    pose.velocity_down_m_s = -.4;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    assert(output->vertical_diagnostics.limited_down_m_s < 0);
    assert((output->vertical_diagnostics.safety_flags & 8) != 0);
    assert(output->command.vz_m_s >= 0);

    // Existing visual edge gates retain priority in both directions.
    predictor.reset_motion();
    pose.down_m = -1.5;
    pose.velocity_down_m_s = -.4;
    box.y1 = 340;
    box.y2 = 420;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    assert((output->vertical_diagnostics.safety_flags & 1) != 0);
    assert(output->command.vz_m_s >= 0);
    predictor.reset_motion();
    pose.velocity_down_m_s = .4;
    box.y1 = 60;
    box.y2 = 140;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05);
    assert(output);
    assert((output->vertical_diagnostics.safety_flags & 2) != 0);
    assert(output->command.vz_m_s <= 0);

    // After reset or reacquisition, stale prior commands cannot become the new reference.
    predictor.reset_motion();
    pose.down_m = -1.5;
    pose.velocity_down_m_s = -.2;
    box.y1 = 200;
    box.y2 = 260;
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.05, 1.0, true);
    assert(output);
    assert(std::abs(output->vertical_diagnostics.previous_down_m_s + .2) < 1e-9);
    assert(std::abs(output->command.vz_m_s + .2) <= .030001);
    // Motion-plan reset should not discard the slew reference, unlike a control-session reset.
    last = output->command.vz_m_s;
    predictor.reset_plan();
    output = predictor.update(pose, pose, box, {0.0, 0.0}, 0.0, -1.5, 0.1, 2.0);
    assert(output);
    assert(std::abs(output->vertical_diagnostics.previous_down_m_s - last) < 1e-9);
    assert(std::abs(output->command.vz_m_s - last) <= .030001);
    std::cout << "Predictive vertical rate limit, safety priority and recovery passed\n";
}
