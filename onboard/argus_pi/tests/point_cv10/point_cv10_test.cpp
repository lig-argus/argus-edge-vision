#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/safety/include/flight_command_guard.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
using namespace follow;
using namespace std::chrono_literals;
void env(const char* n, const char* v) {
#ifdef _WIN32
    _putenv_s(n, v);
#else
    setenv(n, v, 1);
#endif
}
Clock::time_point at(double s) {
    return Clock::time_point{std::chrono::nanoseconds{1000000000LL + std::llround(s * 1e9)}};
}
std::int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
TracksV1 frame(unsigned id, double s, bool found) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "cv10-real-contract";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns = ns(at(s));
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original";
    f.image_width = 640;
    f.image_height = 480;
    f.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    if (found) {
        TrackRecordV1 tr{
            1, 2, "rc_car", .9, 290, 180, 350, 240, "CONFIRMED", true, f.sample_monotonic_ns};
        const double n = 3. + .3 * std::min(s, 1.5) + .9 * std::max(0., s - 1.5);
        tr.ground = std::array<double, 6>{n, 0, .01, 0, .01, 0};
        tr.ground_anchor = std::array<double, 2>{1, 0};
        f.tracks = {tr};
    }
    return f;
}
class Flight final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus value;
    VehicleFlightStatus read() const override { return value; }
};
class Port final : public IVehiclePort {
public:
    FollowCommand last;
    unsigned count = 0;
    bool send(const FollowCommand& c) override {
        last = c;
        ++count;
        return true;
    }
};
CameraGroundCalibration geometry_config;
FlightGuidanceInput input(const TargetSelectionSnapshot& view, double s) {
    FlightGuidanceInput in;
    in.now = at(s);
    in.selection = view;
    in.selection_command_epoch = view.command_epoch;
    in.connected = in.armed = in.in_air = in.offboard = true;
    in.hold_down_m = -1.5;
    in.pose.have_position = in.pose.have_attitude = in.pose.have_velocity = true;
    in.pose.position_at = in.pose.attitude_at = in.pose.snapshot_at = at(s);
    in.pose.down_m = -1.5;
    in.observation_pose = in.pose;
    if (view.latest_frame && view.selected_track) {
        in.geometry = CalibratedGroundGeometry(geometry_config)
                          .project(*view.latest_frame, *view.selected_track, in.pose, 0, at(s));
    }
    return in;
}
NativeJson wire(const FlightGuidanceResult& r, const TargetSelectionSnapshot& s,
                Clock::time_point now) {
    return {{"command_epoch", s.command_epoch},
            {"generation", s.target->generation},
            {"track_id", s.target->track_id},
            {"source_sample_monotonic_ns",
             r.mission == "GAP_CV" ? s.cv_origin_sample_ns
                                   : s.selected_track->last_real_detection_monotonic_ns},
            {"issued_monotonic_ns", ns(now)},
            {"valid_until_monotonic_ns", ns(r.valid_until)},
            {"vx_m_s", r.command.vx_m_s},
            {"vy_m_s", r.command.vy_m_s},
            {"vz_m_s", r.command.vz_m_s},
            {"yaw_rate_rad_s", r.command.yaw_rate_deg_s * std::acos(-1.) / 180.},
            {"cv_gap_active", r.mission == "GAP_CV"},
            {"cv_origin_sample_ns", s.cv_origin_sample_ns},
            {"cv_gap_deadline_monotonic_ns", s.cv_gap_deadline ? ns(*s.cv_gap_deadline) : 0}};
}
int main(int argc, char** argv) try {
    assert(argc == 3);
    env("ARGUS_V21_ENABLED", "1");
    env("ARGUS_V21_CV_GAP", "1");
    env("ARGUS_V23_CV_RAW_MOTION", "1");
    env("ARGUS_V23_CV_MOVING_ONLY", "1");
    env("ARGUS_V21_CV_VISIBILITY", "1");
    auto loaded = load_bus_following_config(argv[1]);
    geometry_config = loaded.geometry;
    auto cfg = loaded.selector;
    cfg.recovery.v32_cv_ten_frame = true;
    cfg.recovery.trust_selected_id = true;
    TargetSelector selector(cfg);
    for (unsigned i = 0; i <= 90; ++i) {
        const auto f = frame(i + 1, i * (2. / 90.), true);
        assert(selector.update(f, at(i * (2. / 90.))));
        if (!i) {
            assert(selector.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0)).accepted);
            assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
        }
    }
    auto view = selector.snapshot(at(2));
    assert(view.fresh_real_detection);
    auto gc = loaded.guidance;
    assert(gc.v3_pampc_point_profile == 1);
    FlightGuidanceController control(gc);
    control.explicit_start_committed(view.command_epoch);
    auto normal = control.update(input(view, 2));
    if (normal.mission != "FOLLOW_ACTIVE") {
        std::cerr << normal.mission << " " << normal.reason << std::endl;
        return 8;
    }
    Flight flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.offboard =
        true;
    Port port;
    FlightCommandGuard guard(selector, flight, port);
    guard.set_limits({gc.max_forward_m_s, gc.max_vertical_m_s, gc.max_yaw_rate_deg_s});
    assert(guard.send(normal, view, at(2)) == FlightCommandGuard::Result::Sent);
    control.record_command_result(true);
    NativeJson output;
    output["normal"] = wire(normal, view, at(2));
    assert(selector.update(frame(92, 2.3, false), at(2.3)));
    view = selector.snapshot(at(2.3));
    assert(view.cv_ten_frame_policy && view.cv_gap_active && view.cv_history_frame_id == 31);
    assert(std::abs(view.cv_velocity_north_m_s - .3) <
           .03);  // Latest velocity was .9: this must be frame-60.
    assert(view.cv_history_source_ns == ns(at(30 * (2. / 90.))));
    assert(view.cv_origin_sample_ns == ns(at(2)));
    assert(view.cv_gap_deadline == at(5));
    auto cv = control.update(input(view, 2.3));
    assert(cv.mission == "GAP_CV" && cv.command.vx_m_s > 0);
    assert(cv.command.vx_m_s <= .330001 && cv.command.vz_m_s == 0 &&
           cv.command.yaw_rate_deg_s == 0);
    assert(cv.v21_motion && cv.v21_motion->history_frame_id == 31);
    assert(guard.send(cv, view, at(2.3)) == FlightCommandGuard::Result::Sent);
    control.record_command_result(true);
    output["cv"] = wire(cv, view, at(2.3));
    // Skip the exact frame to exercise fallback, then skip the whole tolerance window.
    for (const bool skip_window : {false, true}) {
        TargetSelector no_history(cfg);
        for (unsigned i = 0; i <= 90; ++i) {
            if (i == 30 || (skip_window && i >= 25 && i <= 35)) {
                continue;
            }
            auto f = frame(i + 1, i * (2. / 90.), true);
            assert(no_history.update(f, at(i * (2. / 90.))));
            if (!i) {
                assert(no_history.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0))
                           .accepted);
                assert(no_history.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
            }
        }
        assert(no_history.update(frame(92, 2.3, false), at(2.3)));
        const auto absent = no_history.snapshot(at(2.3));
        assert(absent.cv_ten_frame_policy);
        if (skip_window) {
            assert(!absent.cv_gap_active && !absent.cv_history_frame_id);
        } else {
            assert(absent.cv_gap_active && absent.cv_history_frame_id == 30);
        }
    }
    // Historical velocity cannot outlive the existing bounded CV window.
    {
        TargetSelector old_history(cfg);
        for (unsigned i = 0; i <= 90; ++i) {
            const double t = i <= 30 ? i * (2. / 90.) : i * (2. / 90.) + 3.;
            auto f = frame(i + 1, t, true);
            assert(old_history.update(f, at(t)));
            if (!i) {
                assert(old_history.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0))
                           .accepted);
                assert(old_history.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
            }
        }
        assert(old_history.update(frame(92, 5.3, false), at(5.3)));
        const auto expired_history = old_history.snapshot(at(5.3));
        assert(!expired_history.cv_gap_active && !expired_history.cv_history_frame_id);
    }
    auto yaw_cfg = load_bus_following_config(
                       (std::filesystem::path(argv[1]).parent_path() / "yaw.config").string())
                       .guidance;
    FlightGuidanceController yaw_only(yaw_cfg);
    yaw_only.explicit_start_committed(view.command_epoch);
    assert(yaw_only.update(input(view, 2.3)).mission != "GAP_CV");
    // A frame-60 miss must not silently use the latest cached .9 velocity.
    auto unavailable = view;
    unavailable.cv_gap_active = false;
    unavailable.cv_history_frame_id.reset();
    unavailable.cv_history_source_ns = 0;
    FlightGuidanceController neutral(gc);
    neutral.explicit_start_committed(view.command_epoch);
    auto stopped = neutral.update(input(unavailable, 2.3));
    assert(stopped.mission != "GAP_CV" && stopped.command.vx_m_s == 0 &&
           stopped.command.vy_m_s == 0);
    auto missing_shape = view;
    missing_shape.cv_visibility_reference.reset();
    FlightGuidanceController blind(gc);
    blind.explicit_start_committed(view.command_epoch);
    auto closed = blind.update(input(missing_shape, 2.3));
    assert(closed.mission == "GAP_HOLD" && closed.command.vx_m_s == 0 &&
           closed.command.vy_m_s == 0);
    for (int k = 0; k < 5; ++k) {
        FlightGuidanceController safe(gc);
        safe.explicit_start_committed(view.command_epoch);
        auto bad = input(view, 2.3);
        if (k == 0) {
            bad.armed = false;
        }
        if (k == 1) {
            bad.in_air = false;
        }
        if (k == 2) {
            bad.offboard = false;
        }
        if (k == 3) {
            bad.rc_override = true;
        }
        if (k == 4) {
            bad.pose.attitude_at = at(1);
        }
        assert(safe.update(bad).mission != "GAP_CV");
    }
    flight.value.rc_override = true;
    assert(guard.send(cv, view, at(2.3)) != FlightCommandGuard::Result::Sent);
    flight.value.rc_override = false;
    FlightGuidanceController exp(gc);
    exp.explicit_start_committed(view.command_epoch);
    assert(exp.update(input(view, 4.91)).request_offboard_stop_once);
    assert(guard.send(cv, view, at(5)) != FlightCommandGuard::Result::Sent);
    assert(selector.update(frame(93, 2.6, true), at(2.6)));
    auto real = selector.snapshot(at(2.6));
    assert(real.fresh_real_detection);
    auto resumed = control.update(input(real, 2.6));
    assert(resumed.mission == "FOLLOW_ACTIVE");
    auto queued = cv;
    queued.valid_until = at(2.7);
    assert(guard.send(queued, view, at(2.6)) == FlightCommandGuard::Result::SelectionChanged);
    output["history_frame_id"] = 31;
    output["history_velocity_north_m_s"] = view.cv_velocity_north_m_s;
    write_native_json(argv[2], output);
    std::cout << "PASS Selector frame-60 -> point Guidance GAP_CV -> actual FlightCommandGuard; "
                 "neutral/no-history, visibility, RC/FC/TTL, real reacquisition\n";
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
