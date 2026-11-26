#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "argus/control/guidance/include/predictive_follower.hpp"
#ifdef _WIN32
inline int setenv(const char* name, const char* value, int) { return _putenv_s(name, value); }
#endif

using namespace follow;
int main() {
    setenv("ARGUS_COMPARISON_ISOLATED", "1", 1);
    setenv("ARGUS_SITL_PAMPC_POINT_ONLY", "1", 1);
    setenv("ARGUS_SITL_QUATERNION_STAGE", "0", 1);
    setenv("ARGUS_SITL_OPTIMIZER_MODE", "0", 1);
    setenv("ARGUS_SITL_SENT_COMMAND_SLEW", "1", 1);
    PredictiveFollowConfig c;
    c.sent_command_slew = true;
    c.pampc_point_profile = 1;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 1.7;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .3;
    c.max_acceleration_m_s2 = 1;
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
    PredictiveFollower f(c);
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.6;
    TrackRecordV1 box{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    box.ground = std::array<double, 6>{3., 0, .01, 0, .01, 0};
    box.ground_anchor = std::array<double, 2>{1, 0};
    auto step = [&]() {
        return f.update(p, p, box, {0, 0}, .01, -1.6, .05, 1, false,
                        box.last_real_detection_monotonic_ns);
    };
    auto a = step();
    assert(a);
    assert(std::abs(a->command.vz_m_s) <= .0500001);
    assert(a->command.vx_m_s > .01 && a->command.vx_m_s <= .0500001);
    f.complete_update(false);
    auto b = step();
    assert(b);
    assert(std::hypot(b->command.vx_m_s, b->command.vy_m_s) <= .0500001);
    f.complete_update(true);
    auto d = step();
    assert(d);
    assert(std::abs(d->command.vx_m_s - b->command.vx_m_s) < .02);
    f.complete_update(false);
    p.velocity_north_m_s = 3;
    auto incompatible = step();
    assert(!incompatible);
    f.complete_update(false);
    f.reset_motion();
    p.velocity_north_m_s = .4;
    auto e = step();
    assert(e);
    assert(e->command.vx_m_s >= .31 && e->command.vx_m_s <= .4500001);
    f.complete_update(true);
    f.reset_motion();
    p.velocity_north_m_s = .6;
    (*box.ground)[0] = 1.4;
    auto brake = step();
    assert(brake);
    assert(brake->command.vx_m_s >= .5099999);
    assert(brake->command.vx_m_s < .6);
    std::cout << "success/discard/reset/currentFCreference anddistinctbrakingbudget PASS\n";
}
