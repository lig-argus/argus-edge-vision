// 2026-11-19 PAMPC #3(rollout 기체 응답 이득), #5(producer 지면 기준과 투영 자세) 시험.
// 장비를 열지 않는 합성 입력만 쓴다. 확인 항목:
//   1. 새 설정 키 파싱과 범위 검사 (실내 moving.config 사용, 예측 추종이 아니면 거부)
//   2. 기능 꺼짐(0/false)이면 기존 동작과 명령 double 값까지 같다
//   3. rollout 이득은 예측에만 쓰고 첫 명령의 PX4 가속 예산(1.8)은 그대로다
//   4. 지면 기준 전환, 투영 자세 사용, 기준 실패 시 기존 동작 복귀와 누적 횟수
//   5. 높이 하한/상한 검사는 보수적인 기준을 쓴다
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;

namespace {

PredictiveFollowConfig base_config() {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;
    c.range_m = 1.8;
    c.min_height_m = 1.1;
    c.max_height_m = 2.0;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = .6;
    c.braking_acceleration_m_s2 = 1.8;
    c.velocity_response = 1.8;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.333333333;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, -.242};
    c.camera.box_anchor_y = 1;
    c.height_anchor_m = 1.8;
    c.height_anchor_weight = 10;
    c.height_anchor_deadband_m = .05;
    c.vertical_deadband_m_s = .1;
    return c;
}

bool rejects(const PredictiveFollowConfig& c) {
    try {
        PredictiveFollower follower(c);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

struct Tick {
    std::optional<PredictiveFollowResult> result;
};

struct Scenario {
    double height_ekf{1.8};    // 0 - down (m)
    double target_north{1.8};  // 표적 지면 북쪽 위치 (m)
    double velocity_north{0};  // 기체 현재 속도
    double age{.02};
    std::optional<std::array<double, 2>> gap;
    int ticks{6};
};

TrackRecordV1 make_box() {
    TrackRecordV1 box;
    box.track_id = 1;
    box.class_id = 2;
    box.label = "tank";
    box.confidence = .9;
    box.x1 = 280;
    box.y1 = 300;
    box.x2 = 360;
    box.y2 = 380;
    box.state = "CONFIRMED";
    box.real_detection = true;
    box.last_real_detection_monotonic_ns = 1000000000LL;
    box.ground = std::array<double, 6>{1.8, 0, .01, 0, .01, 0};
    box.ground_anchor = std::array<double, 2>{1, 0};
    return box;
}

// 같은 입력을 여러 tick 넣고 각 결과를 모은다 (기체 운동은 적분하지 않는 개방 루프).
std::vector<std::optional<PredictiveFollowResult>> run(
    const PredictiveFollowConfig& c, const Scenario& s,
    std::optional<ProducerGround> producer = std::nullopt, PredictiveFollower* external = nullptr) {
    std::optional<PredictiveFollower> owned;
    if (!external) {
        owned.emplace(c);
    }
    auto& follower = external ? *external : *owned;
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -s.height_ekf;
    p.velocity_north_m_s = s.velocity_north;
    auto box = make_box();
    (*box.ground)[0] = s.target_north;
    std::vector<std::optional<PredictiveFollowResult>> out;
    for (int i = 0; i < s.ticks; ++i) {
        auto r = follower.update(p, p, box, {0, 0}, s.age, -1.9, .05, 1, false,
                                 box.last_real_detection_monotonic_ns, s.gap, producer);
        follower.complete_update(true);
        out.push_back(r);
        box.last_real_detection_monotonic_ns += 50000000;
    }
    return out;
}

bool same_command(const PredictiveFollowResult& a, const PredictiveFollowResult& b) {
    const auto& x = a.command;
    const auto& y = b.command;
    return std::memcmp(&x.vx_m_s, &y.vx_m_s, sizeof(double)) == 0 &&
           std::memcmp(&x.vy_m_s, &y.vy_m_s, sizeof(double)) == 0 &&
           std::memcmp(&x.vz_m_s, &y.vz_m_s, sizeof(double)) == 0 &&
           std::memcmp(&x.yaw_rate_deg_s, &y.yaw_rate_deg_s, sizeof(double)) == 0 &&
           std::memcmp(&a.altitude_target_down_m, &b.altitude_target_down_m, sizeof(double)) == 0 &&
           std::memcmp(&a.vertical_diagnostics.best_score, &b.vertical_diagnostics.best_score,
                       sizeof(double)) == 0;
}

bool same_runs(const std::vector<std::optional<PredictiveFollowResult>>& a,
               const std::vector<std::optional<PredictiveFollowResult>>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].has_value() != b[i].has_value() || (a[i] && !same_command(*a[i], *b[i]))) {
            return false;
        }
    }
    return true;
}

// 설정 본문에서 key 줄을 지우고 필요한 줄을 덧붙인다.
std::string with_lines(const std::string& text, const std::vector<std::string>& remove,
                       const std::string& append);

// 실제 기동처럼 실행용 사본에 tracks 엔드포인트를 넣어 읽는다 (deployment_config.materialize).
std::string read_file(const char* path) {
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return with_lines(buffer.str(), {"tracks_pub_endpoint", "tracks_sub_endpoint"},
                      "tracks_pub_endpoint=tcp://127.0.0.1:6375\n"
                      "tracks_sub_endpoint=tcp://127.0.0.1:6375\n");
}

std::string with_lines(const std::string& text, const std::vector<std::string>& remove,
                       const std::string& append) {
    std::istringstream in(text);
    std::string line, out;
    while (std::getline(in, line)) {
        bool drop = false;
        for (const auto& key : remove) {
            if (line.rfind(key + "=", 0) == 0) {
                drop = true;
            }
        }
        if (!drop) {
            out += line + "\n";
        }
    }
    return out + append;
}

bool parse_rejects(const std::string& text) {
    std::istringstream in(text);
    try {
        (void)parse_bus_following_config(in);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

int checks = 0;
void check(bool condition, const char* what) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << what << '\n';
        std::abort();
    }
}

}  // namespace

int main(int argc, char** argv) {
    // 1. 설정 키: 실내 예측 moving.config에 적합값이 들어 있고, 범위 밖이나 비예측 설정은 거부한다.
    if (argc >= 2) {
        const auto text = read_file(argv[1]);
        std::istringstream in(text);
        const auto loaded = parse_bus_following_config(in);
        check(std::abs(loaded.guidance.pampc_plant_horizontal_response - .85) < 1e-12,
              "moving.config plant horizontal 0.85");
        check(std::abs(loaded.guidance.pampc_plant_vertical_response - 1.5) < 1e-12,
              "moving.config plant vertical 1.5");
        check(loaded.guidance.pampc_use_producer_ground, "moving.config producer ground on");
        check(loaded.guidance.pampc_model_log, "moving.config model log on");
        check(loaded.guidance.pampc_producer_ground_max_deviation_m == .2 &&
                  loaded.guidance.pampc_producer_ground_hold_s == 5 &&
                  loaded.guidance.pampc_ground_datum_rate_m_s == .05 &&
                  loaded.guidance.pampc_ceiling_margin_m == .1,
              "moving.config safety constants");
        check(parse_rejects(with_lines(text, {"pampc_producer_ground_hold_s"},
                                       "pampc_producer_ground_hold_s=20\n")),
              "hold 20 s rejected");
        check(std::abs(loaded.guidance.fov_velocity_p_acc - 1.8) < 1e-12,
              "PX4 acceleration gain stays 1.8");
        const std::vector<std::string> keys{"pampc_plant_horizontal_response",
                                            "pampc_plant_vertical_response",
                                            "pampc_use_producer_ground",
                                            "pampc_model_log",
                                            "pampc_producer_ground_max_deviation_m",
                                            "pampc_producer_ground_hold_s",
                                            "pampc_ground_datum_rate_m_s",
                                            "pampc_ceiling_margin_m"};
        std::istringstream legacy_in(with_lines(text, keys, ""));
        const auto legacy = parse_bus_following_config(legacy_in);
        check(legacy.guidance.pampc_plant_horizontal_response == 0 &&
                  legacy.guidance.pampc_plant_vertical_response == 0 &&
                  !legacy.guidance.pampc_use_producer_ground && !legacy.guidance.pampc_model_log,
              "keys absent means legacy defaults");
        // 되돌림용 값(0, 0, false, false)을 명시해도 키가 없는 것과 같다.
        std::istringstream rollback_in(
            with_lines(text, keys,
                       "pampc_plant_horizontal_response=0\npampc_plant_vertical_response=0\n"
                       "pampc_use_producer_ground=false\npampc_model_log=false\n"));
        const auto rollback = parse_bus_following_config(rollback_in);
        check(rollback.guidance.pampc_plant_horizontal_response == 0 &&
                  rollback.guidance.pampc_plant_vertical_response == 0 &&
                  !rollback.guidance.pampc_use_producer_ground &&
                  !rollback.guidance.pampc_model_log,
              "explicit rollback values equal legacy defaults");
        check(parse_rejects(with_lines(text, {keys[0]}, keys[0] + "=0.1\n")),
              "plant horizontal 0.1 rejected");
        check(parse_rejects(with_lines(text, {keys[1]}, keys[1] + "=6\n")),
              "plant vertical 6 rejected");
        check(parse_rejects(with_lines(text, {keys[2]}, keys[2] + "=maybe\n")),
              "producer ground must be boolean");
        check(parse_rejects(text + keys[0] + "=0.9\n"), "duplicate key rejected");
        check(!parse_rejects(with_lines(text, {keys[0]}, keys[0] + "=0\n")),
              "plant horizontal 0 (legacy) accepted");
    }
    if (argc >= 3) {
        // yaw 전용(비예측) 설정에 새 키를 넣으면 거부한다.
        const auto yaw = read_file(argv[2]);
        check(!parse_rejects(yaw), "yaw.config parses unchanged");
        check(parse_rejects(yaw + "pampc_plant_horizontal_response=0.85\n"),
              "plant key requires predictive_follow");
        check(parse_rejects(yaw + "pampc_use_producer_ground=true\n"),
              "producer ground key requires predictive_follow");
    }

    // 2. 범위 검사 (PredictiveFollower 생성자).
    {
        auto c = base_config();
        check(!rejects(c), "base config valid");
        for (double bad : {.1, -1., 5.5, std::nan("")}) {
            c = base_config();
            c.plant_horizontal_response = bad;
            check(rejects(c), "invalid plant horizontal rejected");
            c = base_config();
            c.plant_vertical_response = bad;
            check(rejects(c), "invalid plant vertical rejected");
        }
        c = base_config();
        c.plant_horizontal_response = .85;
        c.plant_vertical_response = 1.5;
        check(!rejects(c), "fitted plant gains valid");
    }

    // 3. 꺼짐이면 기존과 같다: 0은 velocity_response/2.0과 double 값까지 같고, 기능이 꺼져 있으면
    //    producer 값을 넘겨도 명령이 같다.
    {
        Scenario moving;
        moving.velocity_north = .3;
        moving.target_north = 3.4;
        moving.height_ekf = 1.6;
        auto legacy = base_config();
        auto explicit_gains = base_config();
        explicit_gains.plant_horizontal_response = 1.8;
        explicit_gains.plant_vertical_response = 2.0;
        const auto a = run(legacy, moving);
        const auto b = run(explicit_gains, moving);
        check(a.front().has_value(), "legacy run produced a command");
        check(same_runs(a, b), "plant 0 equals explicit legacy gains");
        const auto c = run(legacy, moving, ProducerGround{-.12, {1., 2., 0.}});
        check(same_runs(a, c), "producer fields ignored when feature off");
        check(a.front()->vertical_diagnostics.model.datum_source == PampcGroundSource::Legacy &&
                  a.front()->vertical_diagnostics.model.fallback == PampcGroundFallback::None,
              "legacy diagnostics");
        check(c.front()->vertical_diagnostics.model.producer_ground_down_m &&
                  *c.front()->vertical_diagnostics.model.producer_ground_down_m == -.12,
              "received producer value logged even when off");
        // 수축(contracting) 정지 계획도 지나는 긴 시나리오에서도 같다.
        Scenario approach = moving;
        approach.ticks = 20;
        approach.target_north = 2.2;
        check(same_runs(run(legacy, approach), run(explicit_gains, approach)),
              "legacy identity over longer run");
    }

    // 4. rollout 이득은 예측에만 쓴다: 첫 명령의 수평 변화는 PX4 가속 setpoint 이득 1.8 기준
    //    예산(0.6 m/s^2 / 1.8 = 0.333 m/s)을 넘지 않는다 (0.85를 썼다면 0.706 m/s까지 허용).
    {
        auto fitted = base_config();
        fitted.plant_horizontal_response = .85;
        fitted.plant_vertical_response = 1.5;
        Scenario far;
        far.target_north = 4.5;
        far.ticks = 1;
        const auto legacy = run(base_config(), far);
        const auto changed = run(fitted, far);
        check(legacy.front() && changed.front(), "far scenario commands");
        const double limit = .6 / 1.8 + 1e-9;
        const auto& cmd = changed.front()->command;
        check(std::hypot(cmd.vx_m_s, cmd.vy_m_s) <= limit, "first block uses PX4 budget 1.8");
        check(changed.front()->vertical_diagnostics.model.plant_horizontal_response == .85 &&
                  changed.front()->vertical_diagnostics.model.plant_vertical_response == 1.5,
              "plant gains reported");
        check(legacy.front()->vertical_diagnostics.model.plant_horizontal_response == 1.8 &&
                  legacy.front()->vertical_diagnostics.model.plant_vertical_response == 2.0,
              "legacy plant gains reported");
        check(legacy.front()->vertical_diagnostics.best_score !=
                  changed.front()->vertical_diagnostics.best_score,
              "rollout prediction changes plan cost");
        Scenario moving;
        moving.velocity_north = .3;
        moving.target_north = 3.4;
        moving.height_ekf = 1.6;
        check(!same_runs(run(base_config(), moving), run(fitted, moving)),
              "fitted gains change the planned commands");
    }

    // 5. 지면 기준 전환: producer 평면 down -0.12 (EKF가 레이저보다 0.12m 높게 읽음).
    {
        auto c = base_config();
        c.use_producer_ground = true;
        c.ground_datum_rate_m_s = 0;  // 이 절은 변화율 제한 없이 값만 본다
        Scenario s;
        s.ticks = 1;
        const ProducerGround producer{-.12, {0., 0., 0.}};
        const auto r = run(c, s, producer).front();
        check(r.has_value(), "producer datum command");
        const auto& m = r->vertical_diagnostics.model;
        check(m.datum_source == PampcGroundSource::Producer &&
                  m.attitude_source == PampcGroundSource::Producer &&
                  m.fallback == PampcGroundFallback::None,
              "producer datum selected");
        check(std::abs(m.ground_datum_down_m + .12) < 1e-12, "datum value");
        // EKF 상한 여유: 1.8 + 0.05 + 0.12 > 2.0 - 0.1 이므로 기준 높이를 1.73 m(레이저)로 줄인다.
        check(m.height_ref_clamped && std::abs(m.height_ref_limit_m - 1.73) < 1e-12 &&
                  m.height_ref_clamped_ticks == 1,
              "height reference clamped below EKF ceiling margin");
        check(std::abs(r->vertical_diagnostics.height_reference_m - 1.73) < 1e-12,
              "clamped anchor 1.73 m above producer plane");
        check(std::abs(r->altitude_target_down_m - (-.12 - 1.73)) < 1e-12,
              "altitude target EKF down -1.85");
        const auto legacy = run(base_config(), s).front();
        check(std::abs(legacy->altitude_target_down_m + 1.8) < 1e-12, "legacy target -1.8");
        check(m.producer_ticks == 1 && m.fallback_ticks == 0, "producer tick counted");
        check(m.image_forecast_valid, "image forecast logged");
        // EKF 1.8m(레이저 1.68m)에서 새 기준은 상승(vz<0) 쪽, 기존 기준은 높이 비용이 작다.
        check(r->vertical_diagnostics.height_cost > legacy->vertical_diagnostics.height_cost,
              "height cost uses producer plane");
    }

    // 6. 투영 자세: t0 재투영에 producer 자세를 쓰고, 꺼짐이면 관측 pose 자세를 쓴다.
    {
        auto c = base_config();
        c.use_producer_ground = true;
        Scenario s;
        s.ticks = 1;
        const auto r = run(c, s, ProducerGround{0., {1.5, -2., .5}}).front();
        const auto& a = r->vertical_diagnostics.model.projection_attitude_deg;
        check(a[0] == 1.5 && a[1] == -2. && a[2] == .5, "producer attitude used");
        const auto off = run(base_config(), s, ProducerGround{0., {1.5, -2., .5}}).front();
        const auto& b = off->vertical_diagnostics.model.projection_attitude_deg;
        check(b[0] == 0 && b[1] == 0 && b[2] == 0, "legacy attitude is observation pose");
        // 같은 평면(0)이라도 자세가 다르면 재투영 bias가 달라져 계획 비용이 달라진다.
        check(r->vertical_diagnostics.best_score != off->vertical_diagnostics.best_score,
              "attitude changes t0 reprojection");
        // 자세가 관측과 같고 평면도 0이면 기존과 같은 명령이다.
        const auto same = run(c, s, ProducerGround{0., {0., 0., 0.}}).front();
        check(same_command(*same, *off), "identical inputs give identical command");
    }

    // 7. 기준 실패는 그 tick만 기존 동작으로 돌아가고 센다.
    {
        auto c = base_config();
        c.use_producer_ground = true;
        Scenario s;
        s.ticks = 1;
        const auto legacy = run(base_config(), s).front();
        struct Case {
            std::optional<ProducerGround> producer;
            PampcGroundFallback expected;
            double age;
            std::optional<std::array<double, 2>> gap;
        };
        const std::vector<Case> cases{
            {std::nullopt, PampcGroundFallback::Absent, .02, std::nullopt},
            {ProducerGround{std::nan(""), {0, 0, 0}}, PampcGroundFallback::NonFinite, .02,
             std::nullopt},
            {ProducerGround{0., {0, std::nan(""), 0}}, PampcGroundFallback::NonFinite, .02,
             std::nullopt},
            {ProducerGround{.6, {0, 0, 0}}, PampcGroundFallback::Deviation, .02, std::nullopt},
            {ProducerGround{-.51, {0, 0, 0}}, PampcGroundFallback::Deviation, .02, std::nullopt},
            {ProducerGround{-.21, {0, 0, 0}}, PampcGroundFallback::Deviation, .02, std::nullopt},
            {ProducerGround{0., {0, 20, 0}}, PampcGroundFallback::Attitude, .02, std::nullopt},
            {ProducerGround{0., {0, 0, 200}}, PampcGroundFallback::Attitude, .02, std::nullopt},
        };
        for (const auto& item : cases) {
            Scenario t = s;
            t.age = item.age;
            t.gap = item.gap;
            const auto r = run(c, t, item.producer).front();
            check(r.has_value(), "fallback still commands");
            const auto& m = r->vertical_diagnostics.model;
            check(m.fallback == item.expected, "fallback reason");
            check(m.datum_source == PampcGroundSource::Legacy && m.ground_datum_down_m == 0,
                  "fallback uses legacy datum");
            check(m.fallback_ticks == 1 && m.producer_ticks == 0, "fallback counted");
            check(same_command(*r, *legacy), "fallback command equals legacy");
        }
        // 공백(GAP) 경로: 1초 넘은 관측은 오래된 것으로 보고 기존 기준을 쓴다.
        Scenario gap = s;
        gap.gap = std::array<double, 2>{0., 0.};
        gap.age = 1.5;
        const auto stale = run(c, gap, ProducerGround{-.12, {0, 0, 0}}).front();
        check(stale && stale->vertical_diagnostics.model.fallback == PampcGroundFallback::Stale,
              "stale gap reference falls back");
        gap.age = .8;
        const auto fresh_gap = run(c, gap, ProducerGround{-.12, {0, 0, 0}}).front();
        check(fresh_gap &&
                  fresh_gap->vertical_diagnostics.model.datum_source == PampcGroundSource::Producer,
              "gap reference within 1 s keeps producer datum");
        // 누적 횟수: 한 follower에서 성공 2, 실패 1.
        PredictiveFollower follower(c);
        Scenario one = s;
        (void)run(c, one, ProducerGround{-.12, {0, 0, 0}}, &follower);
        (void)run(c, one, std::nullopt, &follower);
        const auto last = run(c, one, ProducerGround{-.12, {0, 0, 0}}, &follower).front();
        check(last->vertical_diagnostics.model.producer_ticks == 2 &&
                  last->vertical_diagnostics.model.fallback_ticks == 1,
              "cumulative counters");
    }

    // 8. 높이 하한/상한: 하한은 낮은 기준(레이저), 상한은 높은 기준(EKF)으로 막는다.
    {
        auto c = base_config();
        c.use_producer_ground = true;
        c.ground_datum_rate_m_s = 0;
        const ProducerGround producer{-.12, {0, 0, 0}};
        Scenario low;
        low.ticks = 1;
        low.height_ekf = 1.2;  // 레이저 1.08 m < 하한 1.1 m
        const auto low_new = run(c, low, producer).front();
        const auto low_old = run(base_config(), low).front();
        check(low_new && (low_new->vertical_diagnostics.safety_flags & 4) != 0 &&
                  low_new->command.vz_m_s <= 0,
              "minimum height guard uses lower (laser) height");
        check(low_old && (low_old->vertical_diagnostics.safety_flags & 4) == 0,
              "legacy minimum guard unchanged");
        Scenario high;
        high.ticks = 1;
        high.height_ekf = 2.0;  // 레이저 1.88 m, EKF 2.0 m = 상한
        const auto high_new = run(c, high, producer).front();
        const auto high_old = run(base_config(), high).front();
        check(high_new && (high_new->vertical_diagnostics.safety_flags & 8) != 0 &&
                  high_new->command.vz_m_s >= 0,
              "maximum height guard keeps EKF ceiling");
        check(high_old && (high_old->vertical_diagnostics.safety_flags & 8) != 0,
              "legacy maximum guard unchanged");
    }
    // 9. 검토 수정: 상한 여유 clamp, 기준 유지(hold), 변화율 제한, 3초 공백, epoch 초기화.
    {
        // 9-1. 설정 검사: 안전 상수 범위와 고정 높이 + 무시 구간 <= 상한 - 여유.
        auto c = base_config();
        c.use_producer_ground = true;
        check(!rejects(c), "default safety constants valid");
        for (auto mutate : std::vector<void (*)(PredictiveFollowConfig&)>{
                 [](PredictiveFollowConfig& x) { x.producer_ground_max_deviation_m = .01; },
                 [](PredictiveFollowConfig& x) { x.producer_ground_max_deviation_m = .6; },
                 [](PredictiveFollowConfig& x) { x.producer_ground_hold_s = 11; },
                 [](PredictiveFollowConfig& x) { x.ground_datum_rate_m_s = -1; },
                 [](PredictiveFollowConfig& x) { x.ceiling_margin_m = .4; },
                 [](PredictiveFollowConfig& x) { x.ceiling_margin_m = .2; }}) {
            auto bad = c;
            mutate(bad);
            check(rejects(bad), "invalid producer safety constant rejected");
        }
        auto off = base_config();
        off.ceiling_margin_m = .2;  // 기능 꺼짐이면 고정 높이 조건을 걸지 않는다
        check(!rejects(off), "margin anchor check only when producer ground on");

        // 9-2. clamp: 기준 차이가 작으면 줄이지 않고, 크면 EKF 고정 높이 + 무시 구간 <= 1.9 m.
        c.ground_datum_rate_m_s = 0;
        Scenario one;
        one.ticks = 1;
        for (double plane : {-.04, -.12, -.19}) {
            const auto r = run(c, one, ProducerGround{plane, {0, 0, 0}}).front();
            const auto& m = r->vertical_diagnostics.model;
            const double ref = r->vertical_diagnostics.height_reference_m;
            check(ref + .05 + (0 - plane) <= 1.9 + 1e-12, "EKF anchor top stays 0.1 m below 2.0");
            check(m.height_ref_clamped == (plane < -.05 + 1e-12), "clamp only when needed");
        }

        // 9-3. 변화율 제한: epoch 첫 tick은 기존 기준(0)에서 0.05 m/s x 0.05 s씩 움직인다.
        auto rate = base_config();
        rate.use_producer_ground = true;
        PredictiveFollower follower(rate);
        const ProducerGround fresh{-.12, {0, 0, 0}};
        std::vector<double> datum;
        for (int i = 0; i < 60; ++i) {
            const auto r = run(rate, one, fresh, &follower).front();
            datum.push_back(r->vertical_diagnostics.model.ground_datum_down_m);
        }
        check(std::abs(datum[0] + .0025) < 1e-12 && std::abs(datum[9] + .025) < 1e-12,
              "datum ramps at 0.05 m/s from legacy");
        check(std::abs(datum.back() + .12) < 1e-12, "datum converges to producer plane");
        bool bounded = true;
        for (std::size_t i = 1; i < datum.size(); ++i) {
            bounded = bounded && std::abs(datum[i] - datum[i - 1]) <= .0025 + 1e-12;
        }
        check(bounded, "datum step bounded every tick");

        // 9-4. 3초 공백: 같은 기준 프레임이 1초 넘게 낡아도(Stale) 유지 값으로 계속 -0.12.
        double worst = 0;
        bool held_seen = false;
        Scenario gap;
        gap.ticks = 1;
        gap.gap = std::array<double, 2>{0., 0.};
        for (int i = 1; i <= 60; ++i) {
            gap.age = .05 * i;
            const auto r = run(rate, gap, fresh, &follower).front();
            check(r.has_value(), "gap tick commands");
            const auto& m = r->vertical_diagnostics.model;
            worst = std::max(worst, std::abs(m.ground_datum_down_m + .12));
            held_seen = held_seen || (m.datum_source == PampcGroundSource::Held &&
                                      m.fallback == PampcGroundFallback::Stale);
        }
        check(held_seen && worst < 1e-12, "3 s stale gap keeps producer datum without a step");
        // 같은 공백을 유지 없이(hold 0) 보내면 기존 기준 쪽으로 천천히(제한 안에서) 돌아간다.
        auto nohold = rate;
        nohold.producer_ground_hold_s = 0;
        PredictiveFollower plain(nohold);
        for (int i = 0; i < 60; ++i) {
            (void)run(nohold, one, fresh, &plain);
        }
        double previous = -.12, max_step = 0, last = -.12;
        for (int i = 1; i <= 60; ++i) {
            gap.age = .05 * i;
            const auto r = run(nohold, gap, fresh, &plain).front();
            last = r->vertical_diagnostics.model.ground_datum_down_m;
            max_step = std::max(max_step, std::abs(last - previous));
            previous = last;
        }
        check(last > -.12 + .05 && max_step <= .0025 + 1e-12,
              "without hold the stale datum returns to legacy at the rate limit");

        // 9-5. 짧은 필드 공백(Absent)도 유지한다. 유지 시간이 지나면 기존 기준으로 향한다.
        const auto absent = run(rate, one, std::nullopt, &follower).front();
        check(absent->vertical_diagnostics.model.datum_source == PampcGroundSource::Held &&
                  std::abs(absent->vertical_diagnostics.model.ground_datum_down_m + .12) < 1e-12,
              "absent fields keep held datum");
        auto short_hold = rate;
        short_hold.producer_ground_hold_s = .05;
        short_hold.ground_datum_rate_m_s = 0;
        PredictiveFollower expiring(short_hold);
        (void)run(short_hold, one, fresh, &expiring);
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        const auto expired = run(short_hold, one, std::nullopt, &expiring).front();
        check(expired->vertical_diagnostics.model.datum_source == PampcGroundSource::Legacy &&
                  expired->vertical_diagnostics.model.ground_datum_down_m == 0,
              "hold expires after hold_s");

        // 9-6. FC epoch이 바뀌면 유지 값과 적용 값을 버리고 기존 기준에서 다시 시작한다.
        ProducerGround next_epoch = fresh;
        next_epoch.fc_epoch = 7;
        const auto reset = run(rate, one, next_epoch, &follower).front();
        check(std::abs(reset->vertical_diagnostics.model.ground_datum_down_m + .0025) < 1e-12,
              "epoch change restarts datum ramp");
        check(reset->vertical_diagnostics.model.held_ticks > 0, "held ticks counted");
    }
    std::cout << "PampcModelFixTest checks=" << checks << " passed\n";
    return 0;
}
