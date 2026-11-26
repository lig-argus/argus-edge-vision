#include <cassert>
#include <iostream>

#include "argus/control/guidance/include/image_center_follower.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/perception/geometry/include/pi_ground_projector.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
using namespace follow;
int main(int argc, char** argv) {
    assert(argc == 4);
    ImageCenterFollower pid(ImageCenterConfig{});
    Pose pose;
    pose.down_m = -4;
    pose.have_position = pose.have_attitude = pose.have_velocity = true;
    TrackRecordV1 box{1, 2, "tank", .9, 290, 210, 350, 270, "CONFIRMED", true, 1000000000};
    auto u = pid.update(pose, box, .05);
    assert(u && u->vx_m_s == 0 && u->vy_m_s == 0 && u->yaw_rate_deg_s == 0 && u->vz_m_s == 0);
    auto check = [&](double x, double y) {
        pid.reset();
        auto b = box;
        b.x1 = x - 30;
        b.x2 = x + 30;
        b.y1 = y - 30;
        b.y2 = y + 30;
        auto r = pid.update(pose, b, .05);
        assert(r && r->vy_m_s == 0);
        return *r;
    };
    assert(check(430, 240).yaw_rate_deg_s > 0);
    assert(check(210, 240).yaw_rate_deg_s < 0);
    assert(check(320, 170).vx_m_s > 0);
    assert(check(320, 310).vx_m_s < 0);
    assert(std::abs(check(335, 251).yaw_rate_deg_s) < 1e-9 &&
           std::abs(check(335, 251).vx_m_s) < 1e-9);
    assert(check(340, 240).yaw_rate_deg_s > 0);
    pose.down_m = -3.4;
    pose.velocity_down_m_s = .4;
    pid.reset();
    u = pid.update(pose, box, .05);
    assert(u && u->vz_m_s <= 0);
    pose.down_m = -4.6;
    pose.velocity_down_m_s = -.4;
    pid.reset();
    u = pid.update(pose, box, .05);
    assert(u && u->vz_m_s >= 0);
    pose.down_m = -4;
    pose.velocity_down_m_s = 0;
    pid.reset();
    box.y1 = 160;
    box.y2 = 220;
    for (int i = 0; i < 60; i++) {
        u = pid.update(pose, box, .05);
        assert(u && std::abs(u->vx_m_s) <= 1 && std::abs(u->vz_m_s) <= .7 &&
               std::abs(u->yaw_rate_deg_s) <= 75);
    }
    assert(pid.terms()[0].i > 0);
    assert(!pid.update(pose, box, .3));
    auto calibration = read_native_json(argv[1]);
    auto g = calibration.at("geometry");
    auto m = read_native_json(argv[2]);
    const std::string uid = calibration.at("expected_uid");
    const std::int64_t stamp = 1000000000;
    NativeJson pixels{{"schema", "argus.perception.observations.v2"},
                      {"clock_domain", "host_monotonic"},
                      {"coordinate_frame", "camera_pixel"},
                      {"captured_monotonic_ns", stamp},
                      {"image_width", 640},
                      {"image_height", 480},
                      {"detections",
                       {{{"x1", 300.},
                         {"x2", 340.},
                         {"y1", 220.},
                         {"y2", 260.},
                         {"confidence", .9},
                         {"class_id", 2},
                         {"label", "tank"}}}}};
    NativeJson history;
    history["ATTITUDE"] = {
        {{"received_monotonic_ns", stamp}, {"data", {{"roll", 0.}, {"pitch", 0.}, {"yaw", 0.}}}}};
    history["LOCAL_POSITION_NED"] = {
        {{"received_monotonic_ns", stamp}, {"data", {{"x", 0.}, {"y", 0.}, {"z", -4.}}}}};
    history["DISTANCE_SENSOR"] = {{{"received_monotonic_ns", stamp},
                                   {"data",
                                    {{"orientation", 25},
                                     {"current_distance", 400},
                                     {"min_distance", 10},
                                     {"max_distance", 1000}}}}};
    NativeJson fc{{"schema", "argus.fc.observations.v1"},
                  {"clock_domain", "host_monotonic"},
                  {"epoch", 1},
                  {"state",
                   {{"uid", uid},
                    {"identity_ok", true},
                    {"fresh", true},
                    {"local_ok", true},
                    {"flow_quality", nullptr},
                    {"flight_environment", "outdoor"}}},
                  {"sensors", {{"flow_valid", false}, {"gps_valid", true}}},
                  {"history", history}};
    PiGroundProjector projector(g, m, uid);
    auto projected = projector.project(pixels, fc, stamp + 1000000);
    assert(projected.detections.size() == 1);
    for (int fault = 0; fault < 5; fault++) {
        auto bad = fc;
        if (fault == 0) {
            bad["sensors"]["gps_valid"] = false;
        }
        if (fault == 1) {
            bad["state"]["flight_environment"] = "indoor";
        }
        if (fault == 2) {
            bad["history"]["DISTANCE_SENSOR"][0]["data"]["max_distance"] = 300;
        }
        if (fault == 3) {
            bad["state"]["local_ok"] = false;
        }
        if (fault == 4) {
            bad["history"]["ATTITUDE"][0]["received_monotonic_ns"] = stamp - 51000000;
        }
        bool rejected = false;
        try {
            projector.project(pixels, bad, stamp + 1000000);
        } catch (...) {
            rejected = true;
        }
        assert(rejected);
    }
    auto indoor = m;
    indoor.erase("navigation_policy");
    bool rejected = false;
    try {
        PiGroundProjector(g, indoor, uid).project(pixels, fc, stamp + 1000000);
    } catch (...) {
        rejected = true;
    }
    assert(rejected);
    PredictiveFollowConfig pc;
    pc.camera.image_width = 640;
    pc.camera.image_height = 480;
    pc.camera.fx = g["fx"];
    pc.camera.fy = g["fy"];
    pc.camera.cx = g["cx"];
    pc.camera.cy = g["cy"];
    for (int i = 0; i < 9; i++) {
        pc.camera.camera_to_body[i] = g["camera_to_body"][i / 3][i % 3];
    }
    pc.camera.camera_offset_body_m = g["camera_offset_body_m"].get<std::array<double, 3>>();
    pc.range_m = 4;
    pc.side_offset_m = 3.8;
    pc.min_height_m = 3.5;
    pc.max_height_m = 4.5;
    pc.target_height_m = 4;
    rejected = false;
    try {
        PredictiveFollower old(pc);
    } catch (...) {
        rejected = true;
    }
    assert(rejected);
    pc.outdoor_profile = true;
    PredictiveFollower prediction(pc);
    box.x1 = 290;
    box.x2 = 350;
    box.y1 = 210;
    box.y2 = 270;
    box.ground = std::array<double, 6>{4., 0., .1, 0., .1, 0.};
    box.ground_anchor = std::array<double, 2>{.5, 0.};
    auto result = prediction.update(pose, pose, box, {0., 0.}, 0, -4, .05);
    assert(result && std::abs(result->altitude_target_down_m + 4) < 1e-9);
    CalibratedGroundGeometry camera(pc.camera);
    auto center = camera.image_projection(pose, {4., 0., 0.});
    auto too_close = camera.image_projection(pose, {2., 0., 0.});
    assert(center && too_close && std::abs((*center)[1]) < 1 && (*too_close)[1] > 1);
    auto cfg = load_bus_following_config(argv[3]);
    auto make_input = [&]() {
        FlightGuidanceInput in;
        in.now = Clock::now();
        in.connected = in.armed = in.in_air = in.offboard = true;
        in.pose = pose;
        in.pose.position_at = in.pose.attitude_at = in.now;
        in.hold_down_m = -4.;
        in.selection.phase = TargetSelectionSnapshot::Phase::Following;
        in.selection.command_epoch = 7;
        in.selection_command_epoch = 7;
        in.selection.target = SelectedTargetKey{1, 1};
        in.selection.target_phase = TargetPhase::Locked;
        in.selection.state = TargetState::Confirmed;
        in.selection.fresh_real_detection = true;
        in.selection.selected_track = box;
        in.selection.observation_valid_until = in.now + 250ms;
        TracksV1 frame;
        frame.generation = 1;
        frame.image_width = 640;
        frame.image_height = 480;
        in.selection.latest_frame = frame;
        in.geometry.bearing_valid = true;
        return in;
    };
    for (int mode = 0; mode < 5; mode++) {
        FlightGuidanceController control(cfg.guidance);
        control.explicit_start_committed(7);
        auto in = make_input();
        if (mode == 0) {
            auto r = control.update(in);
            assert(r.reason == "outdoor_image_center_pid" && r.send_setpoint &&
                   r.command.vy_m_s == 0 && r.altitude_target_down_m &&
                   std::abs(*r.altitude_target_down_m + 4) < 1e-9);
        }
        if (mode == 1) {
            in.rc_override = true;
            auto r = control.update(in);
            assert(r.mission == "MANUAL" && !r.send_setpoint);
        }
        if (mode == 2) {
            in.selection.recovery_pause = true;
            in.selection.cv_gap_active = true;
            in.selection.cv_gap_deadline = in.now + 3s;
            in.selection.target_phase = TargetPhase::GapHold;
            auto r = control.update(in);
            assert(r.command.vx_m_s == 0 && r.command.vy_m_s == 0 &&
                   r.command.yaw_rate_deg_s == 0 && r.mission != "GAP_CV");
        }
        if (mode == 3) {
            in.pose.position_at = in.now - 300ms;
            auto r = control.update(in);
            assert(r.reason == "fc_or_mode_precondition_failed" && r.command.vx_m_s == 0);
        }
        if (mode == 4) {
            control.request_operator_stop();
            auto r = control.update(in);
            assert(r.request_offboard_stop_once && r.command.vx_m_s == 0);
        }
    }
    std::cout << "outdoor PID signs/deadzone/limits, GPS/range safety, 4m predictive target and "
                 "camera geometry passed\n";
}
