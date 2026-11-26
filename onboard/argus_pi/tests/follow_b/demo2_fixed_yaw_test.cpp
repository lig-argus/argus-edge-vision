// 2026-11-19 실내 데모2(demo2) yaw 고정 이동 추종 시험. 합성 트랙만 쓴다 (카메라, FC, 게이트웨이
// 없음).
// 1. 예측 추종기: yaw 고정이면 yaw 명령이 정확히 0이고 대상 쪽으로 옆이동한다 (좌우 모두).
//    같은 입력에서 기존(끔)은 yaw로 돌린다.
// 2. 재정렬: 방위 오차가 기준각을 넘으면 재정렬 속도 이하로만 돌고, 기준각 절반 아래에서 멈춘다.
// 3. 설정 범위 거부 (point 모드 아님, 각/속도 짝 불일치, 속도 상한 초과, 끈 상태의 값)
// 4. demo2 설정 파일 읽기, demo2 거부 규칙, vMy 짝(demo2 이동 + demo2 회전) 허용
// 인자: demo2-moving.config demo2-yaw.config moving.config yaw.config
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
#include "argus/runtime/follow/include/bus_following_application.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {

constexpr double kPi = 3.14159265358979323846;

PredictiveFollowConfig config(bool fixed, double realign_deg = 15, double realign_rate = 10) {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;
    c.sent_command_slew = true;
    c.range_m = 1.8;
    c.min_height_m = 1.1;
    c.max_height_m = 2.0;
    c.max_speed_m_s = 1.5;
    c.max_backward_m_s = .8;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = 1;
    c.braking_acceleration_m_s2 = 1.8;
    c.max_yaw_rate_deg_s = 75;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.3333333333335;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, -.242};
    c.fixed_yaw_follow = fixed;
    if (fixed) {
        c.fixed_yaw_realign_deg = realign_deg;
        c.fixed_yaw_realign_rate_deg_s = realign_rate;
    }
    return c;
}

Pose pose(double yaw_deg = 0) {
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.5;
    p.yaw_deg = yaw_deg;
    return p;
}

// 지면 위 대상(north, east)을 실제 투영과 맞는 bbox로 만든다 (bias가 0에 가깝게).
TrackRecordV1 box(const PredictiveFollowConfig& c, const Pose& p, double north, double east,
                  std::int64_t ns) {
    const CalibratedGroundGeometry geometry(c.camera);
    const auto uv = geometry.image_projection_with_depth(p, {north, east, 0});
    assert(uv);
    const double u = ((*uv)[0] + 1) * c.camera.image_width / 2;
    const double v = ((*uv)[1] + 1) * c.camera.image_height / 2;
    TrackRecordV1 b{1, 1, "car", .9, u - 40, v - 110, u + 40, v, "CONFIRMED", true, ns};
    b.ground = std::array<double, 6>{north, east, .03, 0, .03, 20};
    b.ground_anchor = std::array<double, 2>{1, 0};
    return b;
}

struct Outcome {
    double vy{0}, yaw_rate{0}, max_abs_yaw{0};
    bool realigning{false};
};

// 정지 상태에서 같은 대상을 여러 번 넣어 warm start가 쌓인 뒤의 명령을 본다.
Outcome run(PredictiveFollower& f, const PredictiveFollowConfig& c, double north, double east,
            int ticks = 6, double yaw_deg = 0) {
    Outcome out;
    static std::int64_t clock = 1000000000LL;
    for (int i = 0; i < ticks; ++i) {
        clock += 50000000LL;
        const Pose p = pose(yaw_deg);
        const auto r =
            f.update(p, p, box(c, p, north, east, clock), {0, 0}, .02, -1.5, .05, 1, false, clock);
        if (!r) {
            std::cerr << "FAIL update rejected: " << f.last_reject_reason() << '\n';
            std::exit(2);
        }
        f.complete_update(true);
        out.vy = r->command.vy_m_s;
        out.yaw_rate = r->command.yaw_rate_deg_s;
        out.max_abs_yaw = std::max(out.max_abs_yaw, std::abs(out.yaw_rate));
        out.realigning = f.fixed_yaw_realigning();
        std::this_thread::sleep_for(50ms);
    }
    return out;
}

double bearing_deg(double north, double east) { return std::atan2(east, north - .125) * 180 / kPi; }

void follower_test() {
    // 대상: 앞 2.0 m, 오른쪽 0.35 m (방위 약 10도, 재정렬 기준 15도 아래)
    const double n = 2.0, e = .35;
    std::cout << "target bearing " << bearing_deg(n, e) << " deg\n";
    {
        const auto c = config(false);
        PredictiveFollower f(c);
        const auto legacy = run(f, c, n, e);
        std::cout << "legacy: vy " << legacy.vy << " m/s, yaw " << legacy.yaw_rate << " deg/s\n";
        assert(legacy.yaw_rate > 5);  // 기존은 yaw로 맞춘다
    }
    for (const double side : {1., -1.}) {
        const auto c = config(true);
        PredictiveFollower f(c);
        const auto fixed = run(f, c, n, side * e);
        std::cout << "fixed side " << side << ": vy " << fixed.vy << " m/s, yaw " << fixed.yaw_rate
                  << " deg/s, max |yaw| " << fixed.max_abs_yaw << '\n';
        assert(fixed.max_abs_yaw == 0);  // 모든 틱 정확히 0
        assert(!fixed.realigning);
        assert(side * fixed.vy > .05);  // 대상 쪽으로 옆이동
    }
    // 대상이 정면이면 옆이동도 yaw도 없다.
    {
        const auto c = config(true);
        PredictiveFollower f(c);
        const auto centred = run(f, c, 2.0, 0);
        std::cout << "fixed centred: vy " << centred.vy << ", yaw " << centred.yaw_rate << '\n';
        assert(centred.max_abs_yaw == 0 && std::abs(centred.vy) < .05);
    }
    // 재정렬 끔(0, 0): 큰 방위에서도 yaw 0
    {
        const auto c = config(true, 0, 0);
        PredictiveFollower f(c);
        const auto wide = run(f, c, 1.8, .9);  // 약 27도... 정렬 게이트는 이 시험 밖
        std::cout << "fixed no realign wide: vy " << wide.vy << ", yaw " << wide.yaw_rate << '\n';
        assert(wide.max_abs_yaw == 0 && wide.vy > .05);
    }
    std::cout << "PASS fixed yaw: zero yaw command, lateral motion toward target\n";
}

void realign_test() {
    const auto c = config(true, 15, 10);
    PredictiveFollower f(c);
    // 약 20도: 재정렬 시작, yaw는 10 deg/s 이하로만
    const auto start = run(f, c, 1.8, .63);
    std::cout << "realign 20deg: yaw " << start.yaw_rate << ", max " << start.max_abs_yaw
              << ", realigning " << start.realigning << '\n';
    assert(start.realigning && start.yaw_rate > 1 && start.max_abs_yaw <= 10 + 1e-9);
    // 약 10도: 기준각 절반(7.5도)보다 크므로 계속 재정렬 (이력)
    const auto middle = run(f, c, 2.0, .35, 2);
    assert(middle.realigning && middle.max_abs_yaw <= 10 + 1e-9);
    // 약 4도: 재정렬 끝, 다시 yaw 0
    const auto end = run(f, c, 2.0, .13, 3);
    std::cout << "realign end: yaw " << end.yaw_rate << ", realigning " << end.realigning << '\n';
    assert(!end.realigning && end.yaw_rate == 0);
    // reset 뒤에는 재정렬 상태도 지운다.
    run(f, c, 1.8, .63, 1);
    assert(f.fixed_yaw_realigning());
    f.reset();
    assert(!f.fixed_yaw_realigning());
    std::cout << "PASS fixed yaw realign: capped at 10 deg/s, hysteresis 15/7.5 deg\n";
}

bool throws(const PredictiveFollowConfig& c) {
    try {
        PredictiveFollower f(c);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

void range_test() {
    auto no_point = config(true);
    no_point.pampc_point_profile = 0;
    assert(throws(no_point));
    assert(throws(config(true, 15, 0)));
    assert(throws(config(true, 0, 10)));
    assert(throws(config(true, 50, 10)));
    assert(throws(config(true, 15, 80)));  // max_yaw_rate 75 초과
    assert(throws(config(true, -1, 10)));
    auto off = config(false);
    off.fixed_yaw_realign_deg = 15;
    off.fixed_yaw_realign_rate_deg_s = 10;
    assert(throws(off));
    assert(!throws(config(true, 0, 0)) && !throws(config(true, 15, 10)));
    // 2026-11-24: 데모2(yaw 고정)만 최소 높이 0.8 m 허용, 0.7은 거부, 고정이 아니면 1 m 미만 거부.
    auto low = config(true, 20, 10);
    low.min_height_m = .8;
    assert(!throws(low));
    low.min_height_m = .7;
    assert(throws(low));
    auto low_off = config(false);
    low_off.min_height_m = .8;
    assert(throws(low_off));
    std::cout << "PASS fixed yaw settings range\n";
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

std::string materialized(const std::string& path) {
    auto text = read_text(path);
    for (const char* key : {"tracks_pub_endpoint", "tracks_sub_endpoint"}) {
        text = with_key(text, key, "tcp://127.0.0.1:6375");
    }
    return text;
}

BusFollowingConfig parse_text(const std::string& text) {
    std::istringstream in(text);
    return parse_bus_following_config(in);
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

void config_test(const std::string& demo2_moving_path, const std::string& demo2_yaw_path,
                 const std::string& moving_path, const std::string& yaw_path) {
    const auto moving_text = materialized(demo2_moving_path);
    const auto demo2 = parse_text(moving_text);
    assert(demo2.follow_controller_profile == "demo2");
    assert(valid_follow_controller_profile("demo2"));
    assert(demo2_follow_controller_profile("demo2") && !demo2_follow_controller_profile("demo"));
    assert(!demo_follow_controller_profile("demo2") && !mount30_follow_controller_profile("demo2"));
    assert(follow_mode_change_rejection("demo2", "moving").empty());
    assert(!demo2.guidance.outdoor_profile && !demo2.guidance.yaw_only);
    assert(demo2.guidance.predictive_follow && demo2.guidance.v3_pampc_point_profile == 1);
    assert(demo2.guidance.pampc_fixed_yaw_follow);
    assert(demo2.guidance.pampc_fixed_yaw_realign_deg == 20);  // 2026-11-23 15 -> 20
    assert(demo2.guidance.predictive_min_height_m == .8);      // 2026-11-24 데모2만 0.8 m
    assert(demo2.guidance.pampc_fixed_yaw_realign_rate_deg_s == 10);
    assert(demo2.guidance.v35_point_tangential_weight == 0);
    (void)FlightGuidanceController(demo2.guidance);  // 추종기 생성 검사 통과
    const auto demo2_yaw = parse_text(materialized(demo2_yaw_path));
    assert(demo2_yaw.follow_controller_profile == "demo2" && demo2_yaw.guidance.yaw_only);
    assert(!demo2_yaw.guidance.pampc_fixed_yaw_follow);

    expect_reject(with_key(moving_text, "outdoor_profile", "true"), "indoor only", "outdoor demo2");
    expect_reject(with_key(moving_text, "pampc_fixed_yaw_follow", "false"),
                  "pampc_fixed_yaw_follow=true", "demo2 without fixed yaw");
    expect_reject(with_key(moving_text, "follow_controller_profile", "predictive"),
                  "requires follow_controller_profile=demo2", "fixed yaw outside demo2");
    expect_reject(with_key(moving_text, "pampc_fixed_yaw_realign_rate_deg_s", "0"),
                  "fixed yaw follow needs", "realign angle without rate");
    // 기존 설정은 그대로 읽힌다 (fixed yaw 꺼짐).
    const auto moving = parse_text(materialized(moving_path));
    const auto yaw = parse_text(materialized(yaw_path));
    assert(!moving.guidance.pampc_fixed_yaw_follow &&
           moving.follow_controller_profile == "predictive");

    const std::string endpoint = "tcp://127.0.0.1:6399";
    {
        BusFollowingApplication app(demo2_yaw, [] { return true; });
        app.enable_vmy(demo2, demo2_yaw, endpoint, 10, 60000ms);
    }
    {
        BusFollowingApplication app(yaw, [] { return true; });
        app.enable_vmy(moving, yaw, endpoint, 10, 60000ms);
    }
    bool rejected = false;
    try {
        BusFollowingApplication app(yaw, [] { return true; });
        app.enable_vmy(demo2, yaw, endpoint, 10, 60000ms);  // 이름 불일치
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
    std::cout << "PASS demo2 config: indoor only, fixed yaw required, vMy pair\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: demo2_fixed_yaw_test demo2-moving.config demo2-yaw.config "
                     "moving.config yaw.config\n";
        return 2;
    }
    follower_test();
    realign_test();
    range_test();
    config_test(argv[1], argv[2], argv[3], argv[4]);
    std::cout << "PASS demo2 fixed yaw follow\n";
    return 0;
}
