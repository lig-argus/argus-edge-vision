#include <cassert>
#include <cmath>
#include <exception>

#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/control/guidance/include/vertical_command_slew.hpp"
using namespace follow;
int main() try {
    const auto near = [](double a, double b) { assert(std::abs(a - b) < 1e-9); };
    near(limit_vertical_command_change(.3, 0, .05, 1, 1.8), .05);
    near(limit_vertical_command_change(-.3, 0, .05, 1, 1.8), -.05);
    near(limit_vertical_command_change(0, .3, .05, 1, 1.8), .21);
    near(limit_vertical_command_change(0, -.3, .05, 1, 1.8), -.21);
    near(limit_vertical_command_change(-.3, .3, .05, 1, 1.8), .21);
    near(limit_vertical_command_change(.3, -.3, .05, 1, 1.8), -.21);
    PredictiveFollowConfig c;
    c.sent_command_slew = true;
    c.pampc_point_profile = 1;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 1.9;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = 1;
    c.braking_acceleration_m_s2 = 1.8;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.333333333;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, .01};
    c.camera.box_anchor_y = 1;
    PredictiveFollower f(c);
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.5;
    p.velocity_down_m_s = .15;
    TrackRecordV1 box{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    box.ground = std::array<double, 6>{3, 0, .01, 0, .01, 0};
    box.ground_anchor = std::array<double, 2>{1, 0};
    const auto step = [&]() { return f.update(p, p, box, {0, 0}, .01, -1.5, .05); };
    auto a = step();
    if (!a) {
        return 2;
    }
    near(a->vertical_diagnostics.previous_down_m_s, .15);
    f.complete_update(false);
    p.velocity_down_m_s = -.12;
    auto b = step();
    if (!b) {
        return 3;
    }
    near(b->vertical_diagnostics.previous_down_m_s, -.12);
    const double accepted = b->command.vz_m_s;
    f.complete_update(true);
    p.velocity_down_m_s = .2;
    auto d = step();
    if (!d) {
        return 4;
    }
    near(d->vertical_diagnostics.previous_down_m_s, accepted);
    f.complete_update(false);
    auto e = step();
    if (!e) {
        return 5;
    }
    near(e->vertical_diagnostics.previous_down_m_s, accepted);
    f.complete_update(false);
    f.reset_motion();
    auto fresh = step();
    if (!fresh) {
        return 6;
    }
    near(fresh->vertical_diagnostics.previous_down_m_s, .2);
    f.complete_update(false);
    f.reset_motion();
    p.down_m = -1.09;
    p.velocity_down_m_s = .3;
    const auto low = step();
    assert(!low || low->command.vz_m_s <= 0);
    f.complete_update(false);
    f.reset_motion();
    p.down_m = -1.91;
    p.velocity_down_m_s = -.3;
    const auto high = step();
    assert(!high || high->command.vz_m_s >= 0);
    // Keep outdoor 4m safety envelope and command caps unchanged.
    c.outdoor_profile = true;
    c.target_height_m = 4.0;
    c.min_height_m = 3.5;
    c.max_height_m = 4.5;
    c.range_m = 4.0;
    PredictiveFollower outdoor(c);
    p.down_m = -4.0;
    p.velocity_down_m_s = .12;
    (*box.ground)[0] = 4.5;
    const auto outside = outdoor.update(p, p, box, {0, 0}, .01, -4.0, .05);
    if (!outside) {
        return 7;
    }
    near(outside->vertical_diagnostics.previous_down_m_s, .12);
    assert(std::abs(outside->command.vz_m_s - .12) <= .0900001);
    near(outside->altitude_target_down_m, -4.0);
} catch (const std::exception&) {
    return 1;
}
