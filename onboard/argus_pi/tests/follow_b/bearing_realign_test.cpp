// 2026-11-10 방위 재정렬(v35_bearing_realign_s)·GAP_CV yaw 감쇠(v35_gap_yaw_decay_s) 시험.
// 실외 12:09·12:13 비행: 방위 오차가 yaw_alignment_deg(25°)를 넘는 순간 추종이 바로 끝났다
// (predictive_observation_state_invalid 3건). 켜면 수평 0·yaw만으로 다시 맞추고, 시간 안에 못
// 맞추면 기존처럼 HOLD한다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/control/safety/include/flight_command_guard.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;

namespace {

Clock::time_point at(double s) {
    return Clock::time_point{std::chrono::nanoseconds{1000000000LL + std::llround(s * 1e9)}};
}
std::int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

constexpr double kDown = -4;

// 대상은 북쪽 3m 지면. box_x는 박스 중심 x(px). 시험 기체는 북쪽을 보고 제자리.
TracksV1 frame(unsigned id, double s, double box_x) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "bearing-realign-contract";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns = ns(at(s));
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original";
    f.image_width = 640;
    f.image_height = 480;
    f.ground_pose = std::array<double, 6>{0, 0, kDown, 0, 0, 0};
    TrackRecordV1 tr{1,          1,   "person",    .9,   box_x - 30,           250,
                     box_x + 30, 400, "CONFIRMED", true, f.sample_monotonic_ns};
    // 박스 위치와 대략 맞는 지면 위치(방위 = atan(e/n)).
    const double bearing = std::atan((box_x - 320) / 946. / .575);
    tr.ground = std::array<double, 6>{3, 3 * std::tan(bearing), .01, 0, .01, 0};
    tr.ground_anchor = std::array<double, 2>{1, 0};
    f.tracks = {tr};
    return f;
}

CameraGroundCalibration geometry_config;

FlightGuidanceInput input(const TargetSelectionSnapshot& view, double s) {
    FlightGuidanceInput in;
    in.now = at(s);
    in.selection = view;
    in.selection_command_epoch = view.command_epoch;
    in.connected = in.armed = in.in_air = in.offboard = true;
    in.hold_down_m = kDown;
    in.pose.have_position = in.pose.have_attitude = in.pose.have_velocity = true;
    in.pose.position_at = in.pose.attitude_at = in.pose.snapshot_at = at(s);
    in.pose.down_m = kDown;
    in.observation_pose = in.pose;
    if (view.latest_frame && view.selected_track) {
        in.geometry = CalibratedGroundGeometry(geometry_config)
                          .project(*view.latest_frame, *view.selected_track, in.pose, 0, at(s));
    }
    return in;
}

struct Run {
    std::string first_offaxis_reason;  // 방위 25° 초과 첫 틱의 결과
    double first_offaxis_bearing{0};
    double first_offaxis_vx{0}, first_offaxis_vy{0}, first_offaxis_yaw{0};
    std::string end_reason;  // 끝까지(또는 HOLD까지) 진행한 마지막 결과
    double end_time{0};
};

// 0~2초 중앙, 2~2.5초 오른쪽으로 이동, 이후 back_at 초부터 다시 중앙(음수면 계속 오른쪽).
Run run(const BusFollowingConfig& loaded, double realign_s, double back_at) {
    auto cfg = loaded.selector;
    cfg.recovery.trust_selected_id = true;
    TargetSelector selector(cfg);
    unsigned id = 1;
    for (int i = 0; i <= 40; ++i) {
        const double s = i * .05;
        assert(selector.update(frame(id++, s, 320), at(s)));
        if (!i) {
            assert(selector.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0)).accepted);
            assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
        }
    }
    auto gc = loaded.guidance;
    gc.v35_bearing_realign_s = realign_s;
    FlightGuidanceController control(gc);
    auto view = selector.snapshot(at(2));
    control.explicit_start_committed(view.command_epoch);
    auto first = control.update(input(view, 2));
    if (first.mission != "FOLLOW_ACTIVE") {
        std::cerr << "start " << first.mission << " " << first.reason << std::endl;
        std::exit(8);
    }
    control.record_command_result(true);
    Run out;
    for (int i = 1; i <= 90; ++i) {
        const double s = 2 + i * .05;
        const double x = back_at > 0 && s >= back_at ? 320 : std::min(600., 320 + 560 * (s - 2));
        assert(selector.update(frame(id++, s, x), at(s)));
        view = selector.snapshot(at(s));
        const auto in = input(view, s);
        const auto r = control.update(in);
        control.record_command_result(r.send_setpoint);
        if (out.first_offaxis_reason.empty() && std::abs(in.geometry.bearing_error_deg) > 25) {
            out.first_offaxis_reason = r.reason;
            out.first_offaxis_bearing = in.geometry.bearing_error_deg;
            out.first_offaxis_vx = r.command.vx_m_s;
            out.first_offaxis_vy = r.command.vy_m_s;
            out.first_offaxis_yaw = r.command.yaw_rate_deg_s;
        }
        out.end_reason = r.reason;
        out.end_time = s;
        if (r.mission.rfind("HOLD", 0) == 0 || r.mission == "FAULT") {
            break;
        }
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) try {
    assert(argc == 2);
    const auto loaded = load_bus_following_config(argv[1]);
    geometry_config = loaded.geometry;
    assert(loaded.guidance.v3_pampc_point_profile == 1);
    assert(loaded.guidance.yaw_alignment_deg == 25);

    // 1. 기존(0): 25° 초과 첫 틱에 바로 HOLD
    const auto legacy = run(loaded, 0, -1);
    std::cout << "legacy first off-axis bearing " << legacy.first_offaxis_bearing << " -> "
              << legacy.first_offaxis_reason << std::endl;
    assert(legacy.first_offaxis_reason == "predictive_observation_state_invalid");

    // 2. 켜면(1.5초): 수평 0, 방위 쪽 yaw. 계속 벗어나 있으면 1.5초 뒤 HOLD
    const auto stuck = run(loaded, 1.5, -1);
    std::cout << "realign first off-axis bearing " << stuck.first_offaxis_bearing << " -> "
              << stuck.first_offaxis_reason << " vx " << stuck.first_offaxis_vx << " vy "
              << stuck.first_offaxis_vy << " yaw " << stuck.first_offaxis_yaw << " | end "
              << stuck.end_reason << " at " << stuck.end_time << " s" << std::endl;
    assert(stuck.first_offaxis_reason == "bearing_realign");
    assert(stuck.first_offaxis_vx == 0 && stuck.first_offaxis_vy == 0);
    assert(stuck.first_offaxis_yaw * stuck.first_offaxis_bearing > 0);
    assert(stuck.end_reason == "bearing_realign_timeout");

    // 3. 켜고 대상이 다시 중앙으로 오면 추종을 이어 간다
    const auto back = run(loaded, 1.5, 3.0);
    std::cout << "realign then centred: end " << back.end_reason << " at " << back.end_time << " s"
              << std::endl;
    assert(back.first_offaxis_reason == "bearing_realign");
    assert(back.end_reason == "current_poi_pampc_guidance");

    // 4. yaw 상한(GAP_CV 감쇠가 쓰는 PredictiveFollower 기능)
    {
        PredictiveFollowConfig c;
        c.pampc_point_profile = 1;
        c.outdoor_profile = true;
        c.range_m = 3;
        c.min_height_m = 3.5;
        c.max_height_m = 4.3;
        c.target_height_m = 4;
        c.max_speed_m_s = 1.5;
        c.max_acceleration_m_s2 = 1;
        c.braking_acceleration_m_s2 = 1.8;
        c.camera = loaded.guidance.pid_camera;
        const auto box = [] {
            TrackRecordV1 b{1,   1,   "person",    .9,   500,         250,
                            560, 400, "CONFIRMED", true, 1000000000LL};
            b.ground = std::array<double, 6>{3, 1.5, .01, 0, .01, 0};
            b.ground_anchor = std::array<double, 2>{1, 0};
            return b;
        }();
        Pose p;
        p.have_position = p.have_attitude = p.have_velocity = true;
        p.down_m = kDown;
        PredictiveFollower free(c), capped(c);
        const auto a = free.update(p, p, box, {0, 0}, .02, kDown, .05, 1, false, 1000000000LL);
        capped.set_yaw_rate_cap(10.);
        const auto b = capped.update(p, p, box, {0, 0}, .02, kDown, .05, 1, false, 1000000000LL);
        assert(a && b);
        std::cout << "yaw free " << a->command.yaw_rate_deg_s << " capped "
                  << b->command.yaw_rate_deg_s << std::endl;
        assert(std::abs(a->command.yaw_rate_deg_s) > 10);
        assert(std::abs(b->command.yaw_rate_deg_s) <= 10 + 1e-9);
    }
    std::cout << "PASS bearing realign and gap yaw cap\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
