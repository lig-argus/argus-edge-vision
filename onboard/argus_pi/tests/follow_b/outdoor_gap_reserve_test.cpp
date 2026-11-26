// 2026-11-17 실외 비행(outdoor-low) 분석 회귀 시험. 합성 트랙만, 카메라/FC/게이트웨이 없음.
// 1. 관측 만료 직전 명령(observation_expiry_reserve)은 실외 4종 설정에서도 vx, vy, vz, yaw가 모두 0
//    이어야 한다. FC 게이트웨이는 근거 관측이 250ms를 넘은 명령을 전부 0일 때만 통과시킨다
//    (11:44:10 NUC 공백에서 vz=-0.038이 남아 source_stale로 추종이 끝났다).
// 2. 방위 재정렬(v35_bearing_realign_s) 중 공백이 오면 재정렬 시간을 처음부터 다시 잰다.
//    공백 시간이 1.5초 예산을 깎아 복귀 직후 bearing_realign_timeout으로 끝나면 안 된다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;

namespace {

Clock::time_point at(double s) {
    return Clock::time_point{std::chrono::nanoseconds{1000000000LL + std::llround(s * 1e9)}};
}
std::int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

// 대상은 북쪽 3m 지면. box_x는 박스 중심 x(px). 시험 기체는 북쪽을 보고 제자리.
TracksV1 frame(unsigned id, double s, double box_x, double down) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "outdoor-gap-reserve-contract";
    f.frame_id = id;
    f.generation = 1;
    f.sample_monotonic_ns = ns(at(s));
    f.motion_sample_ns = f.sample_monotonic_ns;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "original";
    f.image_width = 640;
    f.image_height = 480;
    f.ground_pose = std::array<double, 6>{0, 0, down, 0, 0, 0};
    TrackRecordV1 tr{1,          1,   "person",    .9,   box_x - 30,           250,
                     box_x + 30, 400, "CONFIRMED", true, f.sample_monotonic_ns};
    const double bearing = std::atan((box_x - 320) / 946. / .575);
    tr.ground = std::array<double, 6>{3, 3 * std::tan(bearing), .01, 0, .01, 0};
    tr.ground_anchor = std::array<double, 2>{1, 0};
    f.tracks = {tr};
    return f;
}

struct Rig {
    BusFollowingConfig loaded;
    double hold{0};  // 설정의 고도 유지 (NED down)
    double down{0};  // 시험 기체 고도. hold와 다르게 두면 고도 보정 vz가 생긴다.
    FlightGuidanceInput input(const TargetSelectionSnapshot& view, double s) const {
        FlightGuidanceInput in;
        in.now = at(s);
        in.selection = view;
        in.selection_command_epoch = view.command_epoch;
        in.connected = in.armed = in.in_air = in.offboard = true;
        in.hold_down_m = hold;
        in.pose.have_position = in.pose.have_attitude = in.pose.have_velocity = true;
        in.pose.position_at = in.pose.attitude_at = in.pose.snapshot_at = at(s);
        in.pose.down_m = down;
        in.observation_pose = in.pose;
        if (view.latest_frame && view.selected_track) {
            in.geometry = CalibratedGroundGeometry(loaded.geometry)
                              .project(*view.latest_frame, *view.selected_track, in.pose, 0, at(s));
        }
        return in;
    }
};

TargetSelectorConfig selector_config(const Rig& rig) {
    auto cfg = rig.loaded.selector;
    cfg.recovery.trust_selected_id = true;
    return cfg;
}

void start_selector(TargetSelector& selector, const Rig& rig, unsigned& id) {
    for (int i = 0; i <= 40; ++i) {
        const double s = i * .05;
        assert(selector.update(frame(id++, s, 320, rig.down), at(s)));
        if (!i) {
            assert(selector.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0)).accepted);
            assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
        }
    }
}

bool all_zero(const FollowCommand& c) {
    return c.vx_m_s == 0 && c.vy_m_s == 0 && c.vz_m_s == 0 && c.yaw_rate_deg_s == 0;
}

// 1. 2초 추종 뒤 프레임이 끊긴다(NUC). 고도 오차 0.1m를 둔다.
void expiry_reserve_all_zero(const char* path) {
    Rig rig;
    rig.loaded = load_bus_following_config(path);
    rig.hold = rig.loaded.runtime.hold_down_m;
    rig.down = rig.hold + .1;
    unsigned id = 1;
    TargetSelector selector(selector_config(rig));
    start_selector(selector, rig, id);
    FlightGuidanceController control(rig.loaded.guidance);
    auto view = selector.snapshot(at(2));
    control.explicit_start_committed(view.command_epoch);
    const auto first = control.update(rig.input(view, 2));
    if (first.mission != "FOLLOW_ACTIVE") {
        std::cerr << path << " start " << first.mission << " " << first.reason << std::endl;
        std::exit(8);
    }
    control.record_command_result(true);
    bool reserve = false;
    for (double s = 2.05; s <= 2.6 + 1e-9; s += .025) {
        view = selector.snapshot(at(s));
        const auto r = control.update(rig.input(view, s));
        control.record_command_result(r.send_setpoint);
        if (r.reason == "observation_expiry_reserve") {
            std::cout << path << " reserve at " << s << " s: vx " << r.command.vx_m_s << " vy "
                      << r.command.vy_m_s << " vz " << r.command.vz_m_s << " yaw "
                      << r.command.yaw_rate_deg_s << std::endl;
            assert(all_zero(r.command));
            reserve = true;
            break;
        }
        assert(r.mission.rfind("HOLD", 0) != 0 && r.mission != "FAULT");
    }
    assert(reserve);
}

// 2. 0~2초 중앙, 이후 오른쪽으로 이동해 방위 25°를 넘으면 재정렬이 시작된다.
//    gap_from~gap_to 동안 프레임이 없다(NUC). 복귀 뒤에도 back_at까지 오른쪽에 있다가 중앙으로
//    온다.
std::string realign_across_gap(const char* path, double gap_from, double gap_to, double back_at,
                               double& realign_seen_after_gap) {
    Rig rig;
    rig.loaded = load_bus_following_config(path);
    rig.hold = rig.down = rig.loaded.runtime.hold_down_m;
    unsigned id = 1;
    TargetSelector selector(selector_config(rig));
    start_selector(selector, rig, id);
    auto gc = rig.loaded.guidance;
    assert(gc.v35_bearing_realign_s == 1.5);
    FlightGuidanceController control(gc);
    auto view = selector.snapshot(at(2));
    control.explicit_start_committed(view.command_epoch);
    assert(control.update(rig.input(view, 2)).mission == "FOLLOW_ACTIVE");
    control.record_command_result(true);
    std::string end;
    realign_seen_after_gap = 0;
    for (int i = 1; i <= 120; ++i) {
        const double s = 2 + i * .05;
        const bool gap = s >= gap_from && s < gap_to;
        if (!gap) {
            const double x = s >= back_at ? 320 : std::min(600., 320 + 560 * (s - 2));
            assert(selector.update(frame(id++, s, x, rig.down), at(s)));
        }
        view = selector.snapshot(at(s));
        const auto r = control.update(rig.input(view, s));
        control.record_command_result(r.send_setpoint);
        if (s >= gap_to && r.reason == "bearing_realign") {
            realign_seen_after_gap = s;
        }
        end = r.reason;
        if (r.mission.rfind("HOLD", 0) == 0 || r.mission == "FAULT") {
            std::cout << "  hold at " << s << " s: " << r.reason << std::endl;
            break;
        }
    }
    return end;
}

}  // namespace

int main(int argc, char** argv) try {
    assert(argc == 5);
    for (int i = 1; i < argc; ++i) {
        expiry_reserve_all_zero(argv[i]);
    }
    // 재정렬은 2.45초쯤 시작한다. 2.6~3.15초 공백(0.55초). 복귀 뒤 4.3초까지 오른쪽.
    // 예전: 2.45 + 1.5 = 3.95초에 bearing_realign_timeout. 이제: 3.15초부터 다시 재서 4.3초에
    // 중앙으로 돌아오면 추종을 이어 간다.
    double seen = 0;
    const auto end = realign_across_gap(argv[1], 2.6, 3.15, 4.3, seen);
    std::cout << "realign across gap: last realign " << seen << " s, end " << end << std::endl;
    assert(seen >= 4.0);
    assert(end == "current_poi_pampc_guidance");
    std::cout << "PASS outdoor gap reserve all-zero and realign timer reset by gap\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
