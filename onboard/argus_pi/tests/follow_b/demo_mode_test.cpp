// 2026-11-18 실내 데모 모드(demo) 시험. 합성 트랙과 가짜 FC 상태만 쓴다 (카메라, FC, 게이트웨이
// 없음).
// 1. demo-yaw.config 읽기와 설정 거부(높이 범위, 실외 demo, yaw 전용 없는 고도 유지)
// 2. yaw 전용 FOLLOW_ACTIVE 명령의 고도 유지: 1.5 m 아래 상승, 위 하강, 0.05 m 안 0, 상한 제한,
//    수평 vx, vy는 항상 0
// 3. 공백, 만료 직전, 정지 명령의 vz는 정확히 0
// 4. 송신 직전 검사: 범위 안쪽 vz 허용, 범위 밖으로 더 미는 vz와 vx, vy 거부
// 5. vMy 설정 짝(demo 두 개 yaw 전용) 허용과 데모 모드의 이동 추종 전환 거부
// 인자: demo-yaw.config yaw.config moving.config
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/safety/include/flight_command_guard.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
#include "argus/runtime/follow/include/bus_following_application.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {

// 2026-11-24: 실제 demo-yaw.config는 고도 유지를 끈다(이륙 높이 유지). 고도 유지 기능 시험은
// 예전 1.5 m 설정을 되살린 임시 사본(g_demo_path)으로 계속하고, 실제 설정은 g_live_demo_path.
std::string g_demo_path, g_yaw_path, g_moving_path, g_live_demo_path;

Clock::time_point at(double s) {
    return Clock::time_point{std::chrono::nanoseconds{1000000000LL + std::llround(s * 1e9)}};
}
std::int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

std::string read_text(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open " + path);
    }
    std::stringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

// key=value 줄을 바꾸거나(value 빈 문자열이면 지움) 없으면 끝에 붙인다.
std::string with_key(const std::string& text, const std::string& key, const std::string& value) {
    std::istringstream in(text);
    std::string line, out;
    bool found = false;
    while (std::getline(in, line)) {
        if (line.rfind(key + "=", 0) == 0) {
            found = true;
            if (!value.empty()) {
                out += key + "=" + value + "\n";
            }
            continue;
        }
        out += line + "\n";
    }
    if (!found && !value.empty()) {
        out += key + "=" + value + "\n";
    }
    return out;
}

BusFollowingConfig parse_text(const std::string& text) {
    std::istringstream in(text);
    return parse_bus_following_config(in);
}

// 실제 기동처럼 실행용 사본에 tracks 엔드포인트를 넣어 읽는다 (deployment_config.materialize).
std::string materialized_text(const std::string& path) {
    auto text = read_text(path);
    for (const char* key : {"tracks_pub_endpoint", "tracks_sub_endpoint"}) {
        text = with_key(text, key, "tcp://127.0.0.1:6375");
    }
    return text;
}

BusFollowingConfig load_config(const std::string& path) {
    return parse_text(materialized_text(path));
}

void expect_reject(const std::string& text, const std::string& expected, const char* label) {
    try {
        (void)parse_text(text);
    } catch (const std::exception& error) {
        const std::string what = error.what();
        if (what.find(expected) == std::string::npos) {
            std::cerr << "FAIL " << label << ": unexpected error: " << what << '\n';
            std::exit(2);
        }
        std::cout << "  rejected " << label << ": " << what << '\n';
        return;
    }
    std::cerr << "FAIL " << label << ": accepted\n";
    std::exit(2);
}

// 1. 설정 읽기와 거부
void config_test() {
    const auto demo = load_config(g_demo_path);
    assert(demo.follow_controller_profile == "demo");
    assert(valid_follow_controller_profile(demo.follow_controller_profile));
    assert(demo_follow_controller_profile(demo.follow_controller_profile));
    assert(!mount30_follow_controller_profile(demo.follow_controller_profile));
    assert(!demo.guidance.outdoor_profile);
    assert(demo.guidance.yaw_only && demo.guidance.yaw_only_altitude_hold);
    assert(demo.guidance.yaw_only_min_height_m == 1.0);
    assert(demo.guidance.yaw_only_max_height_m == 2.0);
    assert(std::abs(demo.runtime.ground_down_m - demo.runtime.hold_down_m - 1.5) < 1e-9);
    assert(demo.guidance.max_vertical_m_s == 0.3 && demo.guidance.altitude_gain == 0.8);
    assert(demo.guidance.max_forward_m_s > 0);  // 상한은 설정 검사용. yaw 전용이라 송신은 0.
    const auto& r = demo.geometry.camera_to_body;
    const double tilt = std::atan2(r.at(8), r.at(2)) * 180. / 3.14159265358979323846;
    assert(std::abs(tilt - 45.) < 1.);
    const FlightGuidanceController controller(demo.guidance);
    const auto band = controller.yaw_only_altitude_band();
    assert(band.enabled && band.max_vertical_m_s == 0.3 && band.min_height_m == 1.0 &&
           band.max_height_m == 2.0 && band.ground_down_m == demo.runtime.ground_down_m);
    const auto yaw = load_config(g_yaw_path);
    assert(!FlightGuidanceController(yaw.guidance).yaw_only_altitude_band().enabled);

    const auto text = materialized_text(g_demo_path);
    expect_reject(with_key(text, "yaw_only_min_height_m", "1.6"), "min < nominal", "min above 1.5");
    expect_reject(with_key(text, "yaw_only_max_height_m", "1.4"), "min < nominal", "max below 1.5");
    expect_reject(with_key(text, "yaw_only_max_height_m", "2.5"), "0.5..2.0", "indoor max 2.5");
    expect_reject(with_key(text, "yaw_only_min_height_m", "0.4"), "0.5..2.0", "indoor min 0.4");
    expect_reject(with_key(text, "yaw_only_min_height_m", "0"), "0 < min", "min 0");
    expect_reject(with_key(text, "yaw_only_max_height_m", ""), "missing config key", "missing max");
    expect_reject(with_key(text, "outdoor_profile", "true"), "indoor only", "demo outdoor");
    expect_reject(with_key(text, "yaw_only", "false"), "requires yaw_only=true",
                  "demo without yaw_only");
    // 고도 유지 키는 yaw 전용 설정에서만 쓴다 (실내 기본 이름으로 확인)
    expect_reject(
        with_key(with_key(text, "follow_controller_profile", "predictive"), "yaw_only", "false"),
        "altitude hold keys require yaw_only=true", "altitude hold without yaw_only");
    expect_reject(with_key(text, "yaw_only_altitude_hold", "false"),
                  "require yaw_only_altitude_hold=true", "band without hold");
    expect_reject(with_key(text, "follow_controller_profile", "predictive_low"),
                  "predictive, demo or demo2", "indoor predictive_low");
    expect_reject(with_key(text, "follow_controller_profile", "demo3"), "demo or demo2",
                  "unknown name");
    // 실내 기본(predictive) yaw 설정은 고도 유지 없이 그대로 읽힌다.
    const auto plain = parse_text(
        with_key(with_key(with_key(with_key(text, "follow_controller_profile", "predictive"),
                                   "yaw_only_altitude_hold", ""),
                          "yaw_only_min_height_m", ""),
                 "yaw_only_max_height_m", ""));
    assert(!plain.guidance.yaw_only_altitude_hold && plain.guidance.yaw_only);
    std::cout << "PASS demo config load and rejects\n";
}

// 대상은 북쪽 3 m 지면. box_x는 박스 중심 x(px). 시험 기체는 북쪽을 보고 제자리.
TracksV1 frame(unsigned id, double s, double box_x, double down) {
    TracksV1 f;
    f.schema = kTracksV5Schema;
    f.fc_epoch = 1;
    f.source = "demo-mode-contract";
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
    const double bearing = std::atan((box_x - 320) / 1083. / .7);
    tr.ground = std::array<double, 6>{3, 3 * std::tan(bearing), .01, 0, .01, 0};
    tr.ground_anchor = std::array<double, 2>{1, 0};
    f.tracks = {tr};
    return f;
}

struct Rig {
    BusFollowingConfig loaded;
    double down{0};
    FlightGuidanceInput input(const TargetSelectionSnapshot& view, double s) const {
        FlightGuidanceInput in;
        in.now = at(s);
        in.selection = view;
        in.selection_command_epoch = view.command_epoch;
        in.connected = in.armed = in.in_air = in.offboard = true;
        in.hold_down_m = loaded.runtime.hold_down_m;
        in.pose.have_position = in.pose.have_attitude = in.pose.have_velocity = true;
        in.pose.position_at = in.pose.attitude_at = in.pose.snapshot_at = at(s);
        in.pose.down_m = down;
        in.observation_pose = in.pose;
        if (view.latest_frame && view.selected_track) {
            in.geometry = CalibratedGroundGeometry(loaded.geometry)
                              .project(*view.latest_frame, *view.selected_track, in.pose,
                                       loaded.runtime.ground_down_m, at(s), true);
        }
        return in;
    }
};

TargetSelectorConfig selector_config(const Rig& rig) {
    auto cfg = rig.loaded.selector;
    cfg.recovery.trust_selected_id = true;
    return cfg;
}

void start_selector(TargetSelector& selector, const Rig& rig, unsigned& id, double box_x) {
    for (int i = 0; i <= 40; ++i) {
        const double s = i * .05;
        assert(selector.update(frame(id++, s, box_x, rig.down), at(s)));
        if (!i) {
            assert(selector.execute({"pick", TargetCommandAction::Select, 1, 1}, at(0)).accepted);
            assert(selector.execute({"start", TargetCommandAction::Start}, at(0)).accepted);
        }
    }
}

// 높이 height_m에서 2초 추종 뒤 첫 명령
FlightGuidanceResult follow_command(const std::string& path, double height_m, double box_x) {
    Rig rig;
    rig.loaded = load_config(path);
    rig.down = rig.loaded.runtime.ground_down_m - height_m;
    unsigned id = 1;
    TargetSelector selector(selector_config(rig));
    start_selector(selector, rig, id, box_x);
    FlightGuidanceController control(rig.loaded.guidance);
    const auto view = selector.snapshot(at(2));
    control.explicit_start_committed(view.command_epoch);
    const auto r = control.update(rig.input(view, 2));
    std::cout << "  height " << height_m << " box_x " << box_x << ": " << r.mission << " "
              << r.reason << " vx " << r.command.vx_m_s << " vy " << r.command.vy_m_s << " vz "
              << r.command.vz_m_s << " yaw " << r.command.yaw_rate_deg_s << '\n';
    assert(r.mission == "FOLLOW_ACTIVE" && r.reason == "yaw_only_confirmed_target");
    assert(r.command.vx_m_s == 0 && r.command.vy_m_s == 0);
    assert(r.send_setpoint);
    return r;
}

// 2. FOLLOW_ACTIVE 고도 유지 (NED down: vz < 0이 상승)
void altitude_hold_test() {
    const auto low = follow_command(g_demo_path, 1.3, 360);
    assert(std::abs(low.command.vz_m_s - (-0.16)) < 1e-9);  // 0.8 x 0.2 m 상승
    assert(low.command.yaw_rate_deg_s > 0);                 // 대상이 오른쪽: yaw만 돈다
    assert(low.altitude_target_down_m && *low.altitude_target_down_m == -1.5);
    const auto high = follow_command(g_demo_path, 1.7, 320);
    assert(std::abs(high.command.vz_m_s - 0.16) < 1e-9);  // 하강
    assert(follow_command(g_demo_path, 1.5, 320).command.vz_m_s == 0);
    assert(follow_command(g_demo_path, 1.54, 320).command.vz_m_s == 0);  // 무시 구간 0.05 m
    assert(follow_command(g_demo_path, 1.46, 280).command.vz_m_s == 0);
    assert(follow_command(g_demo_path, 0.6, 320).command.vz_m_s == -0.3);  // 상한 0.3 m/s
    assert(follow_command(g_demo_path, 2.4, 320).command.vz_m_s == 0.3);
    // 실내 기본 yaw 설정은 높이 오차가 있어도 vz 0 (기존 동작)
    assert(follow_command(g_yaw_path, 1.3, 360).command.vz_m_s == 0);
    std::cout << "PASS demo altitude hold in yaw-only FOLLOW_ACTIVE only\n";
}

// 2-1. 실제 demo-yaw.config: 고도 유지 꺼짐, 높이 오차가 있어도 vz 0 (게이트웨이가 시작 높이 유지)
void live_demo_takeoff_height_test() {
    const auto live = load_config(g_live_demo_path);
    assert(live.follow_controller_profile == "demo" && live.guidance.yaw_only);
    assert(!live.guidance.yaw_only_altitude_hold);
    assert(!FlightGuidanceController(live.guidance).yaw_only_altitude_band().enabled);
    for (const double height : {1.2, 1.3, 1.7}) {
        const auto r = follow_command(g_live_demo_path, height, 360);
        assert(r.command.vz_m_s == 0 && !r.altitude_target_down_m);
        assert(r.command.yaw_rate_deg_s > 0);
    }
    std::cout << "PASS live demo keeps takeoff height (vz 0)\n";
}

// 3. 공백(입력 끊김), 만료 직전, 정지 명령은 높이 오차가 있어도 vz 0
void gap_paths_zero_test() {
    Rig rig;
    rig.loaded = load_config(g_demo_path);
    rig.down = rig.loaded.runtime.ground_down_m - 1.2;  // 1.2 m: FOLLOW_ACTIVE라면 상승할 높이
    unsigned id = 1;
    TargetSelector selector(selector_config(rig));
    start_selector(selector, rig, id, 340);
    FlightGuidanceController control(rig.loaded.guidance);
    auto view = selector.snapshot(at(2));
    control.explicit_start_committed(view.command_epoch);
    const auto first = control.update(rig.input(view, 2));
    assert(first.mission == "FOLLOW_ACTIVE" && first.command.vz_m_s < 0);
    control.record_command_result(true);
    bool reserve = false;
    int other = 0;
    std::string last;
    for (double s = 2.025; s <= 5.0 + 1e-9; s += .025) {
        view = selector.snapshot(at(s));
        const auto r = control.update(rig.input(view, s));
        control.record_command_result(r.send_setpoint);
        if (r.mission + " " + r.reason != last) {
            last = r.mission + " " + r.reason;
            std::cout << "  gap " << s << " s: " << last << " vz " << r.command.vz_m_s << '\n';
        }
        // 마지막 실관측이 아직 유효한 동안의 FOLLOW_ACTIVE만 고도 유지 vz를 낼 수 있다.
        if (r.mission == "FOLLOW_ACTIVE") {
            assert(r.command.vx_m_s == 0 && r.command.vy_m_s == 0);
            continue;
        }
        if (r.command.vz_m_s != 0 || r.command.vx_m_s != 0 || r.command.vy_m_s != 0) {
            std::cerr << "FAIL gap " << s << " " << r.mission << " " << r.reason << " vz "
                      << r.command.vz_m_s << '\n';
            std::exit(3);
        }
        if (r.reason == "observation_expiry_reserve") {
            reserve = true;
        } else {
            ++other;
        }
        if (r.mission.rfind("HOLD", 0) == 0 || r.mission == "FAULT") {
            std::cout << "  gap ended at " << s << " s: " << r.mission << " " << r.reason << '\n';
            break;
        }
    }
    assert(reserve && other > 0);
    // 정지 요청 명령
    Rig stop_rig = rig;
    unsigned stop_id = 1;
    TargetSelector stop_selector(selector_config(stop_rig));
    start_selector(stop_selector, stop_rig, stop_id, 320);
    FlightGuidanceController stop_control(stop_rig.loaded.guidance);
    view = stop_selector.snapshot(at(2));
    stop_control.explicit_start_committed(view.command_epoch);
    assert(stop_control.update(stop_rig.input(view, 2)).command.vz_m_s < 0);
    stop_control.request_operator_stop();
    const auto stop = stop_control.update(stop_rig.input(view, 2.05));
    assert(stop.mission == "HOLD_TRANSITION" && stop.command.vz_m_s == 0 &&
           stop.command.vx_m_s == 0 && stop.command.vy_m_s == 0);
    std::cout << "PASS demo gap, expiry reserve and stop commands keep vz 0\n";
}

class FakeFlightState final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus value;
    VehicleFlightStatus read() const override { return value; }
};

class FakePose final : public IVehicleStateSource {
public:
    Pose value;
    Pose read() const override { return value; }
};

class FakeVehicle final : public IVehiclePort {
public:
    std::vector<FollowCommand> sent;
    bool send_until(const FollowCommand& command, Clock::time_point) override {
        return send(command);
    }
    bool send(const FollowCommand& command) override {
        sent.push_back(command);
        return true;
    }
};

// 4. 송신 직전 검사
void guard_test() {
    const auto demo = load_config(g_demo_path);
    const FlightGuidanceController controller(demo.guidance);
    TargetSelector selector;
    FakeFlightState flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.offboard =
        true;
    FakeVehicle vehicle;
    FakePose pose;
    const auto now = Clock::now();
    pose.value.have_position = pose.value.have_attitude = true;
    pose.value.position_at = pose.value.attitude_at = now;
    TracksV1 f;
    f.source = "guard-fixture";
    f.frame_id = 1;
    f.sample_monotonic_ns = 1000000000;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "fixture";
    f.image_width = 640;
    f.image_height = 480;
    f.generation = 3;
    f.tracks.push_back({8, 5, "rc_car", 0.9, 300, 120, 380, 240, "CONFIRMED", true, 1000000000});
    assert(selector.update(f, now));
    assert(selector.execute({"guard-select", TargetCommandAction::Select, 3, 8}, now).accepted);
    assert(selector.execute({"guard-start", TargetCommandAction::Start}, now).accepted);
    const auto selection = selector.snapshot(now);

    FlightCommandGuard guard(selector, flight, vehicle, true);
    guard.set_limits(controller.command_limits());
    guard.set_yaw_only_altitude_band(controller.yaw_only_altitude_band(), &pose);
    FlightGuidanceResult candidate;
    candidate.send_setpoint = true;
    candidate.valid_until = now + 20ms;
    candidate.selection_command_epoch = selection.command_epoch;
    candidate.command.yaw_rate_deg_s = 5;
    const auto send = [&](double height_m, double vx, double vy, double vz) {
        pose.value.down_m = demo.runtime.ground_down_m - height_m;
        auto c = candidate;
        c.command.vx_m_s = vx;
        c.command.vy_m_s = vy;
        c.command.vz_m_s = vz;
        return guard.send(c, selection, now + 1ms);
    };
    using R = FlightCommandGuard::Result;
    assert(send(1.3, 0, 0, -0.16) == R::Sent);      // 범위 안 상승
    assert(send(1.7, 0, 0, 0.16) == R::Sent);       // 범위 안 하강
    assert(send(1.3, 0, 0, -0.3) == R::Sent);       // 상한 그대로
    assert(send(1.3, 0, 0, -0.31) == R::Rejected);  // 상한 초과
    assert(send(1.3, 0.1, 0, 0) == R::Rejected);    // 수평 이동 거부
    assert(send(1.3, 0, -0.1, -0.1) == R::Rejected);
    assert(send(2.0, 0, 0, -0.1) == R::Rejected);  // 최대 높이에서 더 상승
    assert(send(2.2, 0, 0, -0.1) == R::Rejected);
    assert(send(2.2, 0, 0, 0.1) == R::Sent);      // 범위 밖에서 안쪽으로
    assert(send(1.0, 0, 0, 0.1) == R::Rejected);  // 최소 높이에서 더 하강
    assert(send(0.8, 0, 0, 0.1) == R::Rejected);
    assert(send(0.8, 0, 0, -0.1) == R::Sent);
    assert(send(1.5, 0, 0, 0) == R::Sent);
    pose.value.position_at = now - 2s;  // 오래된 위치로는 vz를 허용하지 않는다
    assert(send(1.3, 0, 0, -0.1) == R::Rejected);
    assert(send(1.3, 0, 0, 0) == R::Sent);
    pose.value.position_at = now;
    pose.value.have_position = false;
    assert(send(1.3, 0, 0, -0.1) == R::Rejected);
    pose.value.have_position = true;
    guard.set_yaw_only_altitude_band(controller.yaw_only_altitude_band(), nullptr);
    assert(send(1.3, 0, 0, -0.1) == R::Rejected);
    // 고도 유지가 없는 yaw 전용(실내 기본 yaw)은 기존처럼 vz != 0을 거부한다
    const auto yaw = load_config(g_yaw_path);
    const FlightGuidanceController yaw_controller(yaw.guidance);
    guard.set_limits(yaw_controller.command_limits());
    guard.set_yaw_only_altitude_band(yaw_controller.yaw_only_altitude_band(), &pose);
    assert(send(1.3, 0, 0, -0.1) == R::Rejected);
    assert(send(1.3, 0, 0, 0) == R::Sent);
    std::cout << "PASS demo command guard bounded vz\n";
}

// 5. vMy 짝과 모드 전환 거부
void vmy_test() {
    const auto demo = load_config(g_demo_path);
    const auto yaw = load_config(g_yaw_path);
    const auto moving = load_config(g_moving_path);
    const std::string endpoint = "tcp://127.0.0.1:6399";
    {
        BusFollowingApplication app(demo, [] { return true; });
        app.enable_vmy(demo, demo, endpoint, 10, 60000ms);
    }
    {
        BusFollowingApplication app(yaw, [] { return true; });
        app.enable_vmy(moving, yaw, endpoint, 10, 60000ms);  // 실내 기본 짝은 그대로
    }
    const auto rejects = [&](const BusFollowingConfig& m, const BusFollowingConfig& y) {
        BusFollowingApplication app(y, [] { return true; });
        try {
            app.enable_vmy(m, y, endpoint, 10, 60000ms);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    assert(rejects(demo, yaw));     // 이름 불일치
    assert(rejects(yaw, demo));     // 기본 이름에 yaw 전용 moving은 안 된다
    assert(rejects(moving, demo));  // 데모 짝에 이동 설정
    auto demo_altered = demo;
    demo_altered.follow_controller_profile = "predictive";
    assert(rejects(demo_altered, demo_altered));  // yaw 전용 두 개는 demo 이름에서만
    assert(follow_mode_change_rejection("demo", "moving") == "데모 모드는 회전 추종만 가능합니다");
    assert(follow_mode_change_rejection("demo", "yaw").empty());
    assert(follow_mode_change_rejection("predictive", "moving").empty());
    assert(follow_mode_change_rejection("predictive_low", "moving").empty());
    std::cout << "PASS demo vMy pair and moving mode rejection\n";
}

}  // namespace

int main(int argc, char** argv) try {
    if (argc != 4) {
        std::cerr << "usage: DemoModeTest demo-yaw.config yaw.config moving.config\n";
        return 1;
    }
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic) C++17 main 인자
    g_live_demo_path = argv[1];
    g_yaw_path = argv[2];
    g_moving_path = argv[3];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    {
        auto held = with_key(read_text(g_live_demo_path), "yaw_only_altitude_hold", "true");
        held = with_key(held, "yaw_only_min_height_m", "1.0");
        held = with_key(held, "yaw_only_max_height_m", "2.0");
        g_demo_path = "/tmp/demo_mode_test_held_" + std::to_string(::getpid()) + ".config";
        std::ofstream(g_demo_path) << held;
    }
    config_test();
    altitude_hold_test();
    live_demo_takeoff_height_test();
    gap_paths_zero_test();
    guard_test();
    vmy_test();
    std::cout << "PASS demo mode\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
}
