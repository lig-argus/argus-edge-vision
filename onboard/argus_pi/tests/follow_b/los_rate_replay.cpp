// 2026-11-16 시선각 변화율(los_rate) 저역 필터 재생 검증 도구 (장비를 열지 않는다).
// 실제 비행 결정 기록(control.csv.decision.jsonl)의 추종 tick 입력을 PredictiveFollower에 그대로
// 넣고 필터 꺼짐/켬 요 명령을 비교한다. 개방 루프 재생이다: 기체는 기록 당시 실제로 보낸 명령대로
// 움직였으므로, 켬 쪽 명령이 기체 운동을 바꾸는 효과(폐루프)는 재현하지 않는다.
// 사용: argus_los_rate_replay <moving.config> <camera.json> <decision.jsonl> <out.csv> [tau...]
// 2026-11-19 PAMPC #3, #5 lane: p<이득>(수평 rollout 응답), z<이득>(수직 rollout 응답), g(producer
// 지면 기준). g는 기록의 pampc_model(신버전) 또는 ARGUS_REPLAY_PRODUCER_GROUND CSV(frame sample_ns,
// 평면 down m, roll, pitch, yaw deg; 구 기록에서 재구성한 값)에서 같은 프레임의 값을 읽는다.
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

// 예측기는 명령 변화율 간격에 steady_clock::now()(벽시계)를 쓴다. 재생에서 이 값이 실행마다 µs씩
// 달라지면 계획 선택이 갈라지므로, 이 실행 파일 안에서만 CLOCK_MONOTONIC을 기록 시각으로 고정한다.
static std::atomic<std::int64_t> g_replay_monotonic_ns{0};
extern "C" int clock_gettime(clockid_t id, struct timespec* ts) {
    const std::int64_t fake = g_replay_monotonic_ns.load();
    if (fake > 0 && (id == CLOCK_MONOTONIC || id == CLOCK_MONOTONIC_RAW)) {
        ts->tv_sec = fake / 1000000000;
        ts->tv_nsec = fake % 1000000000;
        return 0;
    }
    return static_cast<int>(syscall(SYS_clock_gettime, id, ts));
}

namespace {

PredictiveFollowConfig make_follow_config(const FlightGuidanceConfig& g) {
    // flight_guidance_controller.cpp 생성자의 대응을 그대로 옮겼다 (2026-11-16 판).
    PredictiveFollowConfig c;
    c.camera = g.pid_camera;
    c.range_m = g.desired_range_m;
    c.ground_down_m = g.pid_ground_down_m;
    c.velocity_response = g.fov_velocity_p_acc;
    c.max_speed_m_s = g.max_forward_m_s;
    c.max_backward_m_s = g.max_backward_m_s;
    c.max_vertical_m_s = g.max_vertical_m_s;
    c.max_acceleration_m_s2 = g.max_horizontal_accel_m_s2;
    c.braking_acceleration_m_s2 = std::max(g.max_horizontal_accel_m_s2, g.max_backward_accel_m_s2);
    c.max_yaw_rate_deg_s = g.max_yaw_rate_deg_s;
    c.max_yaw_accel_deg_s2 = g.max_yaw_accel_deg_s2;
    c.min_height_m = g.predictive_min_height_m;
    c.max_height_m = g.predictive_max_height_m;
    c.outdoor_profile = g.outdoor_profile;
    c.target_height_m = g.predictive_target_height_m;
    c.position_gain = g.pid.forward.p;
    c.velocity_gain = g.pid.forward.d;
    if (g.v3_predictive_integral) {
        c.integral_gain = g.pid.forward.i;
        c.integral_anti_windup = g.pid.anti_windup;
    }
    c.sent_command_slew = g.v32_sent_command_slew;
    c.pampc_point_profile = g.v3_pampc_point_profile;
    c.image_velocity_weight = g.v3_image_velocity_weight;
    c.side_offset_m = g.follow_side_offset_m;
    c.height_anchor_m = g.predictive_height_anchor_m;
    c.height_anchor_weight = g.predictive_height_anchor_weight;
    c.height_anchor_deadband_m = g.predictive_height_anchor_deadband_m;
    c.vertical_deadband_m_s = g.predictive_vertical_deadband_m_s;
    c.vertical_deadband_release_m = g.predictive_vertical_deadband_release_m;
    c.vertical_release_max_m_s = g.predictive_vertical_release_max_m_s;
    c.gap_velocity_feedforward = g.v35_gap_cv_feedforward;
    c.sent_command_slew_reconcile = g.v35_slew_reconcile;
    c.point_range_deadband_m = g.v35_point_range_deadband_m;
    c.point_range_near_deadband_m = g.v35_point_range_near_deadband_m;
    c.point_bottom_margin = g.v35_point_bottom_margin;
    c.point_tangential_weight = g.v35_point_tangential_weight;
    c.plant_horizontal_response = g.pampc_plant_horizontal_response;
    c.plant_vertical_response = g.pampc_plant_vertical_response;
    c.use_producer_ground = g.pampc_use_producer_ground;
    c.producer_ground_max_deviation_m = g.pampc_producer_ground_max_deviation_m;
    c.producer_ground_hold_s = g.pampc_producer_ground_hold_s;
    c.ground_datum_rate_m_s = g.pampc_ground_datum_rate_m_s;
    c.ceiling_margin_m = g.pampc_ceiling_margin_m;
    return c;
}

Pose to_pose(const Json& j) {
    Pose p;
    p.north_m = j.at("ned_m").at(0);
    p.east_m = j.at("ned_m").at(1);
    p.down_m = j.at("ned_m").at(2);
    p.roll_deg = j.at("rpy_deg").at(0);
    p.pitch_deg = j.at("rpy_deg").at(1);
    p.yaw_deg = j.at("rpy_deg").at(2);
    p.velocity_north_m_s = j.at("velocity_ned_m_s").at(0);
    p.velocity_east_m_s = j.at("velocity_ned_m_s").at(1);
    p.velocity_down_m_s = j.at("velocity_ned_m_s").at(2);
    p.have_position = j.at("have_position");
    p.have_attitude = j.at("have_attitude");
    p.have_velocity = j.at("have_velocity");
    p.position_monotonic_ns = j.at("position_monotonic_ns");
    p.attitude_monotonic_ns = j.at("attitude_monotonic_ns");
    p.position_at = Clock::time_point{std::chrono::nanoseconds{p.position_monotonic_ns}};
    p.attitude_at = Clock::time_point{std::chrono::nanoseconds{p.attitude_monotonic_ns}};
    return p;
}

TrackRecordV1 to_track(const Json& t) {
    TrackRecordV1 r;
    r.track_id = t.at("track_id");
    r.class_id = t.at("class_id");
    r.confidence = t.at("confidence");
    const auto& b = t.at("bbox_xyxy_px");
    r.x1 = b.at(0);
    r.y1 = b.at(1);
    r.x2 = b.at(2);
    r.y2 = b.at(3);
    r.state = t.at("state");
    r.real_detection = t.at("real_detection");
    r.last_real_detection_monotonic_ns = t.at("last_real_detection_monotonic_ns");
    if (t.contains("ground_anchor_ne_cov_alignment")) {
        r.ground = t.at("ground_anchor_ne_cov_alignment").get<std::array<double, 6>>();
    }
    if (t.contains("ground_anchor_spec")) {
        r.ground_anchor = t.at("ground_anchor_spec").get<std::array<double, 2>>();
    }
    return r;
}

struct Lane {
    std::string name;
    std::unique_ptr<PredictiveFollower> follower;
    std::vector<double> yaw, vx, vy;
    unsigned rejects{0};
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr
            << "usage: argus_los_rate_replay moving.config camera.json decision.jsonl out.csv "
               "[tau...]\n";
        return 2;
    }
    auto config = load_bus_following_config(argv[1]);
    const auto camera = read_native_json(argv[2]);
    const auto& geometry = camera.at("geometry");
    auto& cal = config.geometry;
    cal.fx = geometry.at("fx");
    cal.fy = geometry.at("fy");
    cal.cx = geometry.at("cx");
    cal.cy = geometry.at("cy");
    cal.image_width = geometry.at("image_width");
    cal.image_height = geometry.at("image_height");
    for (unsigned i = 0; i < 9; ++i) {
        cal.camera_to_body[i] = geometry.at("camera_to_body").at(i / 3).at(i % 3);
    }
    cal.camera_offset_body_m = geometry.at("camera_offset_body_m").get<std::array<double, 3>>();
    config.guidance.pid_camera = cal;
    const auto follow_config = make_follow_config(config.guidance);
    std::cerr << "profile range_m=" << config.guidance.desired_range_m
              << " side_offset_m=" << config.guidance.follow_side_offset_m
              << " point_profile=" << follow_config.pampc_point_profile
              << " max_yaw=" << follow_config.max_yaw_rate_deg_s
              << " yaw_acc=" << follow_config.max_yaw_accel_deg_s2 << '\n';

    // lane 지정: '+'로 이은 t<τ>(시선각 필터), h<여유>(①), s(②), f(④), n<m>(가까운 쪽 거리
    // 데드밴드), a<gain>(접근 감쇠), b<정규화>(하단 시야 여유). 첫 lane은 항상 base(모두 꺼짐).
    std::vector<std::string> specs{"base"};
    for (int i = 5; i < argc; ++i) {
        specs.emplace_back(argv[i]);
    }
    std::vector<Lane> lanes;
    for (const auto& spec : specs) {
        auto lane_config = follow_config;
        double tau = 0, hysteresis = 0, approach = 0;
        bool stop_seed = false, follow_seed = false;
        std::size_t start = 0;
        while (spec != "base" && start < spec.size()) {
            const auto end = spec.find('+', start);
            const auto token =
                spec.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (token[0] == 't') {
                tau = std::stod(token.substr(1));
            } else if (token[0] == 'h') {
                hysteresis = std::stod(token.substr(1));
            } else if (token[0] == 'a') {
                approach = std::stod(token.substr(1));
            } else if (token[0] == 'b') {
                lane_config.point_bottom_margin = std::stod(token.substr(1));
            } else if (token[0] == 'v') {
                lane_config.vertical_release_max_m_s = std::stod(token.substr(1));
            } else if (token[0] == 'r') {
                lane_config.vertical_deadband_release_m = std::stod(token.substr(1));
            } else if (token[0] == 'n') {
                lane_config.point_range_near_deadband_m = std::stod(token.substr(1));
            } else if (token[0] == 'p') {
                lane_config.plant_horizontal_response = std::stod(token.substr(1));
            } else if (token[0] == 'z') {
                lane_config.plant_vertical_response = std::stod(token.substr(1));
            } else if (token == "g") {
                lane_config.use_producer_ground = true;
            } else if (token == "s") {
                stop_seed = true;
            } else if (token == "f") {
                follow_seed = true;
            } else {
                throw std::invalid_argument("unknown lane token: " + token);
            }
            start = end == std::string::npos ? spec.size() : end + 1;
        }
        Lane lane{spec, std::make_unique<PredictiveFollower>(lane_config)};
        lane.follower->set_los_rate_filter(tau);
        lane.follower->set_yaw_plan_options(hysteresis, stop_seed, follow_seed);
        lane.follower->set_approach_feedforward(approach, .05);
        lanes.push_back(std::move(lane));
    }

    // 2026-11-19 구 기록용 producer 지면 기준 (frame sample_ns 기준). 없으면 기록 안의 값만 쓴다.
    std::map<std::int64_t, ProducerGround> producer_table;
    if (const char* table = std::getenv("ARGUS_REPLAY_PRODUCER_GROUND"); table && *table) {
        std::ifstream csv(table);
        std::string row;
        while (std::getline(csv, row)) {
            if (row.empty() || row[0] == '#' || row[0] == 's') {
                continue;
            }
            std::replace(row.begin(), row.end(), ',', ' ');
            std::istringstream fields(row);
            std::int64_t sample = 0;
            ProducerGround value;
            if (fields >> sample >> value.ground_plane_down_m >> value.attitude_deg[0] >>
                value.attitude_deg[1] >> value.attitude_deg[2]) {
                producer_table[sample] = value;
            }
        }
        std::cerr << "producer ground rows=" << producer_table.size() << "\n";
    }
    std::ifstream in(argv[3]);
    std::ofstream out(argv[4]);
    out << "tick_ns,segment,logged_yaw,logged_vx,logged_vy,bearing_deg,range_m";
    for (const auto& lane : lanes) {
        out << ",yaw_" << lane.name << ",vx_" << lane.name << ",vy_" << lane.name << ",vz_"
            << lane.name;
    }
    // 첫 lane(τ0)의 요 계획 단계 진단 (deg/s)
    out << ",datum_down_m,datum_source,height_ref_m,height_ref_clamped";
    out << ",t_los,t_p,t_seed,t_prev,t_init,t_coarse,t_search,t_vis,t_point,t_contract,t_final,"
           "t_source,t_seed_score,t_best_score,t_range_rate,t_approach_ff,t_visibility";
    out << '\n' << std::setprecision(9);

    std::string line;
    std::int64_t previous_follow_tick = 0;
    double last_yaw_deg = 0;
    int segment = 0;
    bool in_follow = false;
    std::size_t follow_ticks = 0;
    while (std::getline(in, line)) {
        const auto d = Json::parse(line);
        if (!d.contains("mission")) {
            continue;
        }
        const std::int64_t tick = d.at("tick_monotonic_ns");
        const std::string mission = d.at("mission");
        // 시계를 기록의 tick 시각으로 둔다 (실제 제어기에서 update가 불린 시각).
        g_replay_monotonic_ns = tick;
        const bool follow = mission == "FOLLOW_ACTIVE" && d.contains("frame") &&
                            d.contains("vehicle_pose") && d.contains("observation_pose");
        if (follow) {
            if (!in_follow) {
                ++segment;
            }
            in_follow = true;
            const auto current = to_pose(d.at("vehicle_pose"));
            const auto observation = to_pose(d.at("observation_pose"));
            last_yaw_deg = current.yaw_deg;
            const auto track = to_track(d.at("target"));
            const auto velocity = d.at("follow_goal")
                                      .at("admitted_target_velocity_ne_m_s")
                                      .get<std::array<double, 2>>();
            const std::int64_t frame_ns = d.at("frame").at("sample_monotonic_ns");
            const std::int64_t feature_ns = d.at("frame").value("exposure_monotonic_ns", frame_ns);
            const double age = (tick - frame_ns) / 1e9;
            const double hold_down = d.at("vertical_decision").at("hold_down_m");
            double dt = .05;
            if (previous_follow_tick != 0) {
                const double gap = (tick - previous_follow_tick) / 1e9;
                if (gap > 0 && gap <= .2) {
                    dt = gap;
                }
            }
            previous_follow_tick = tick;
            const bool sent = d.at("command_sent");
            const auto& logged = d.at("sent_command");
            const auto& target = d.at("target");
            out << tick << ',' << segment << ',' << logged.at("yaw_rate_deg_s").get<double>() << ','
                << logged.at("vx_forward_m_s").get<double>() << ','
                << logged.at("vy_right_m_s").get<double>() << ','
                << target.value("bearing_error_deg", NAN) << ','
                << target.value("estimated_range_m", NAN);
            const std::int64_t generated = d.value("command_generated_ns", tick);
            std::optional<ProducerGround> producer;
            if (d.contains("pampc_model") &&
                d.at("pampc_model").contains("producer_attitude_deg") &&
                d.at("pampc_model").contains("producer_ground_plane_down_m") &&
                !d.at("pampc_model").at("producer_attitude_deg").is_null() &&
                !d.at("pampc_model").at("producer_ground_plane_down_m").is_null()) {
                const auto& m = d.at("pampc_model");
                producer =
                    ProducerGround{m.at("producer_ground_plane_down_m").get<double>(),
                                   m.at("producer_attitude_deg").get<std::array<double, 3>>()};
            } else if (const auto row = producer_table.find(frame_ns);
                       row != producer_table.end()) {
                producer = row->second;
            }
            double datum_down = NAN, height_ref = NAN;
            int datum_source = -1, clamped = -1;
            for (auto& lane : lanes) {
                g_replay_monotonic_ns = tick;
                const auto r =
                    lane.follower->update(current, observation, track, velocity, age, hold_down, dt,
                                          1, false, feature_ns, std::nullopt, producer);
                g_replay_monotonic_ns = generated > tick ? generated : tick;  // 송신 확인 시각
                double yaw = NAN, vx = NAN, vy = NAN, vz = NAN;
                if (r) {
                    yaw = r->command.yaw_rate_deg_s;
                    vx = r->command.vx_m_s;
                    vy = r->command.vy_m_s;
                    vz = r->command.vz_m_s;
                    lane.follower->complete_update(sent);
                    if (&lane == &lanes.back()) {
                        datum_down = r->vertical_diagnostics.model.ground_datum_down_m;
                        datum_source = static_cast<int>(r->vertical_diagnostics.model.datum_source);
                        height_ref = r->vertical_diagnostics.height_reference_m;
                        clamped = r->vertical_diagnostics.model.height_ref_clamped ? 1 : 0;
                    }
                } else {
                    ++lane.rejects;
                }
                out << ',' << yaw << ',' << vx << ',' << vy << ',' << vz;
            }
            out << ',' << datum_down << ',' << datum_source << ',' << height_ref << ',' << clamped;
            const auto& t = lanes.front().follower->yaw_trace();
            const double deg = 180 / std::acos(-1.);
            for (double v :
                 {t.los_rate, t.p_term, t.seed, t.previous, t.after_init, t.after_coarse,
                  t.after_search, t.after_visibility, t.after_point, t.after_contract, t.final}) {
                out << ',' << v * deg;
            }
            out << ',' << t.source << ',' << t.seed_score << ',' << t.best_score << ','
                << t.range_rate << ',' << t.approach_ff << ',' << t.visibility;
            out << '\n';
            ++follow_ticks;
        } else {
            in_follow = false;
            previous_follow_tick = 0;  // GAP_CV는 PID 시각을 비운다 (첫 복귀 tick dt=0.05)
            if (d.contains("sent_command") && (mission == "GAP_CV" || mission == "GAP_HOLD")) {
                // 실제로 보낸 공백 명령을 이력에 반영한다 (공백 중 예측기 내부 갱신은 근사).
                const auto& c = d.at("sent_command");
                const double yaw = last_yaw_deg * std::acos(-1.) / 180;
                const double fwd = c.at("vx_forward_m_s"), right = c.at("vy_right_m_s");
                const std::array<double, 2> world{std::cos(yaw) * fwd - std::sin(yaw) * right,
                                                  std::sin(yaw) * fwd + std::cos(yaw) * right};
                for (auto& lane : lanes) {
                    lane.follower->note_external_command(world, c.at("vz_down_m_s"),
                                                         c.at("yaw_rate_deg_s"));
                }
            } else if (mission != "GAP_CV" && mission != "GAP_HOLD") {
                for (auto& lane : lanes) {
                    lane.follower->reset();
                }
            }
        }
    }
    std::cerr << "follow ticks=" << follow_ticks << " segments=" << segment;
    for (const auto& lane : lanes) {
        std::cerr << " rejects(" << lane.name << ")=" << lane.rejects;
    }
    std::cerr << '\n';
    return 0;
}
