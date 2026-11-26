// 짧은 입력 공백(IR NUC 약 0.5초 등) 회귀 시험 (2026-11-09 실기 21:22 재현).
// 실제 실내 moving 설정으로 추종 중 공백 뒤 같은 트랙이 돌아올 때 추종이 풀리지 않는지,
// 공백이 길면 기존처럼 HOLD로 넘기는지, short_gap_hover_ms=0이면 기존 동작인지 본다.
// 2026-11-09 사용자 결정(공백 3초 유지): 기본 3초, 60프레임 기록 없는 이동 표적도 CV로 버틴다.
// 내부에서 실제 시계를 함께 쓰므로 실제 시간으로 돈다. 사용: short_gap_follow_test <moving.config>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "argus/common/time/include/pose_history.hpp"
#include "argus/control/runtime/include/flight_control_runtime.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
int& failures() {
    static int count = 0;
    return count;
}
void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

class PoseSource final : public IVehicleStateSource {
public:
    Pose value;
    PoseHistory history{512};
    Pose read() const override { return value; }
    std::optional<Pose> read_aligned(std::int64_t time_ns, const std::string& domain,
                                     std::chrono::nanoseconds bracket) const override {
        return history.interpolate(time_ns, domain, bracket);
    }
};
class FlightSource final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus value;
    VehicleFlightStatus read() const override { return value; }
};
class VehiclePort final : public IVehiclePort {
public:
    FollowCommand last;
    bool send_until(const FollowCommand& command, Clock::time_point /*until*/) override {
        return send(command);
    }
    void revoke_commands() override {}
    bool send(const FollowCommand& command) override {
        last = command;
        return true;
    }
};
class ModePort final : public IFlightModePort {
public:
    int stops{0};
    explicit ModePort(FlightSource& source) : flight_(&source) {}
    std::optional<bool> start_result() const override { return true; }
    std::optional<bool> hold_result() const override { return true; }
    bool start_offboard() override {
        flight_->value.hold = false;
        flight_->value.offboard = true;
        return true;
    }
    bool stop_offboard() override {
        ++stops;
        flight_->value.offboard = false;
        flight_->value.hold = true;
        return true;
    }

private:
    FlightSource* flight_;
};

std::int64_t to_ns(Clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

struct Outcome {
    int stops{0};
    double first_stop_s{-1};
    std::string stop_trigger;
    bool following_after_gap{false};
    bool hover_seen{false};
    bool cv_seen{false};
    double max_hover_horizontal{0};
};

// 실제 기동처럼 실행용 사본에 엔드포인트를 덧붙여 읽는다 (deployment_config.materialize).
BusFollowingConfig load(const std::string& path, const std::string& extra) {
    const std::ifstream file(path);
    std::stringstream text;
    text << file.rdbuf();
    std::string content = text.str();
    const std::array<std::string, 2> keys{"tracks_pub_endpoint", "tracks_sub_endpoint"};
    for (const auto& key : keys) {
        if (content.find("\n" + key + "=") == std::string::npos) {
            content += "\n" + key + "=tcp://127.0.0.1:6375";
        }
    }
    // 설정 파일에 이미 있는 키를 덮어쓸 때는 기존 줄을 지운다 (중복 키는 파싱 오류).
    const auto equals = extra.find('=');
    if (equals != std::string::npos) {
        const std::string line_start = "\n" + extra.substr(0, equals + 1);
        const auto found = content.find(line_start);
        if (found != std::string::npos) {
            content.erase(found, content.find('\n', found + 1) - found);
        }
    }
    content += "\n" + extra + "\n";
    std::istringstream stream(content);
    return parse_bus_following_config(stream);
}

// move_from: 표적이 움직이기 시작하는 시각(초). 공백 직전에만 움직이면 60프레임 기록이 없다.
Outcome run(const BusFollowingConfig& config, double speed, double gap_s, bool empty_frames,
            double move_from = 0.) {
    auto selector_config = config.selector;
    selector_config.allow_idle_camera_context_switch = true;
    TargetSelector selector{selector_config};
    PoseSource pose;
    FlightSource flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.hold = true;
    flight.value.position = flight.value.ground_known = true;
    VehiclePort vehicle;
    ModePort mode{flight};
    const CalibratedGroundGeometry geometry{config.geometry};
    FlightGuidanceController guidance{config.guidance};
    FlightSafetyEventLatch latch;
    auto runtime_config = config.runtime;
    runtime_config.require_explicit_stop_recovery = true;
    FlightControlRuntime control{selector, pose, flight,         geometry, guidance,
                                 vehicle,  mode, runtime_config, &latch};
    const auto start = Clock::now();
    const double gap_begin = 3.0;
    const double gap_end = gap_begin + gap_s;
    const double stop_at = gap_end + 1.5;
    std::uint64_t frame_id = 1000;
    std::uint64_t seq = 1;
    auto next_frame = start;
    auto next_control = start;
    bool started = false;
    Outcome out;
    for (;;) {
        const auto now = Clock::now();
        const double t = std::chrono::duration<double>(now - start).count();
        if (t > stop_at) {
            break;
        }
        if (now >= next_frame) {
            next_frame += 33333us;
            Pose sample;
            sample.have_position = sample.have_attitude = sample.have_velocity = true;
            sample.position_at = sample.attitude_at = sample.snapshot_at = now;
            sample.position_monotonic_ns = sample.attitude_monotonic_ns = to_ns(now);
            sample.down_m = -1.5;
            pose.value = sample;
            pose.history.offer(sample);
            const bool in_gap = t >= gap_begin && t < gap_end;
            if (!in_gap || empty_frames) {
                TracksV1 frame;
                frame.schema = static_cast<const char*>(kTracksV5Schema);
                frame.source = "ir_sdk:short-gap-test";
                frame.frame_id = ++frame_id;
                frame.seq = seq++;
                frame.fc_epoch = 1;
                frame.sample_monotonic_ns = to_ns(now);
                frame.motion_sample_ns = frame.sample_monotonic_ns;
                frame.clock_domain = "host_monotonic";
                frame.timestamp_source = "sdk_fc_boot_receipt_estimate_unverified";
                frame.image_width = 640;
                frame.image_height = 480;
                frame.generation = 7;
                frame.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
                if (!in_gap) {
                    TrackRecordV1 track{5,
                                        0,
                                        "tank",
                                        .9,
                                        280,
                                        300,
                                        360,
                                        360,
                                        "CONFIRMED",
                                        true,
                                        frame.sample_monotonic_ns};
                    track.ground = std::array<double, 6>{
                        4. + (speed * std::max(0., t - move_from)), 0, .01, 0, .01, 5};
                    track.ground_anchor = std::array<double, 2>{1, .08};
                    frame.tracks = {track};
                }
                selector.update(frame, now);
            }
        }
        if (!started && t >= .3) {
            started = true;
            check(selector.execute({"select", TargetCommandAction::Select, 7, 5}, now).accepted,
                  "선택");
            check(selector.execute({"start", TargetCommandAction::Start, 7, 5}, now).accepted,
                  "시작");
        }
        if (now >= next_control) {
            next_control += 50ms;
            const auto status = control.step(now);
            if (status.reason == "v34_cv60_pampc") {
                out.cv_seen = true;
            }
            if (status.reason == "short_gap_hover") {
                out.hover_seen = true;
                out.max_hover_horizontal =
                    std::max(out.max_hover_horizontal,
                             std::hypot(status.command.vx_m_s, status.command.vy_m_s));
            }
            if (mode.stops > out.stops) {
                out.stops = mode.stops;
                if (out.first_stop_s < 0) {
                    out.first_stop_s = t;
                    out.stop_trigger = status.stop_trigger_reason;
                }
            }
            if (t > gap_end + .5) {
                out.following_after_gap =
                    status.mission == "FOLLOW_ACTIVE" && control.following_active_flag().load();
            }
        }
        std::this_thread::sleep_for(2ms);
    }
    std::cout << "  speed=" << speed << " gap=" << gap_s << " empty=" << empty_frames
              << " stops=" << out.stops << " first_stop=" << out.first_stop_s
              << " trigger=" << out.stop_trigger << " following_after=" << out.following_after_gap
              << " cv=" << out.cv_seen << " hover=" << out.hover_seen
              << " max_hover_h=" << out.max_hover_horizontal << '\n';
    return out;
}
}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, std::next(argv, argc));
    if (args.size() < 2) {
        std::cerr << "usage: short_gap_follow_test <moving.config>\n";
        return 2;
    }
    // 배치 v2_1.json이 켜는 환경 변수 (runtime_settings.sh)
    for (const char* key :
         {"ARGUS_V21_ENABLED", "ARGUS_V21_BOX_MOTION_GUARD", "ARGUS_V21_CV_GAP",
          "ARGUS_V21_CV_VISIBILITY", "ARGUS_V21_VERTICAL_CENTER", "ARGUS_V23_CV_LATERAL",
          "ARGUS_V23_CV_MOVING_ONLY", "ARGUS_V23_CV_RAW_MOTION", "ARGUS_V23_CV_TILT_MODEL"}) {
        setenv(key, "1", 1);
    }
    try {
        const auto config = load(args.at(1), "");
        check(config.guidance.short_gap_hover == 3000ms, "short_gap_hover 기본 3000ms");
        // 1. 0.5초 공백 뒤 같은 트랙 복귀: 정지·이동, 빈 프레임·프레임 없음 모두 추종 유지
        for (const bool empty : {true, false}) {
            for (const double speed : {0., .6}) {
                const auto r = run(config, speed, .5, empty);
                check(r.stops == 0, "0.5초 공백에서 HOLD로 넘김");
                check(r.following_after_gap, "0.5초 공백 뒤 추종 복귀");
                check(r.max_hover_horizontal == 0, "짧은 공백 유지 중 수평 명령");
            }
        }
        // 2. 3초 안 공백(1.2초·2.0초)은 추종 유지, 3초 넘는 공백(3.5초)은 공백 기한에 HOLD
        for (const double gap : {1.2, 2.0}) {
            const auto r = run(config, 0., gap, true);
            check(r.stops == 0, "3초 안 공백에서 HOLD로 넘김");
            check(r.following_after_gap, "3초 안 공백 뒤 추종 복귀");
        }
        {
            const auto r = run(config, 0., 3.5, true);
            check(r.stops == 1, "3초 넘는 공백은 HOLD");
            check(r.stop_trigger == "v34_detection_gap_expired" ||
                      r.stop_trigger == "v34_cv60_history_or_identity_unavailable",
                  "긴 공백 HOLD 이유: " + r.stop_trigger);
            check(r.first_stop_s >= 5.8 && r.first_stop_s <= 6.3,
                  "HOLD 시각이 공백 시작 뒤 약 2.9초");
            check(!r.following_after_gap, "긴 공백 뒤 자동 재추종 없음");
        }
        // 3. short_gap_hover_ms=0은 기존 동작: 예측 근거가 없으면 공백 시작 즉시 HOLD
        {
            const auto legacy = load(args.at(1), "short_gap_hover_ms=0");
            check(legacy.guidance.short_gap_hover == 0ms, "short_gap_hover_ms=0 읽기");
            const auto r = run(legacy, 0., .5, true);
            check(r.stops == 1 && !r.hover_seen, "0이면 기존처럼 즉시 HOLD");
            check(r.first_stop_s >= 3.0 && r.first_stop_s <= 3.2, "기존 HOLD 시각");
        }
        // 4. 범위 밖 값은 거부
        bool rejected = false;
        try {
            load(args.at(1), "short_gap_hover_ms=3500");
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected, "short_gap_hover_ms 범위 검사");
        // 5. 공백 0.8초 전부터 움직인 표적(60프레임 기록 없음)도 마지막 속도로 CV 이동해 버틴다
        {
            const auto r = run(config, .6, 1.0, true, 2.2);
            check(r.cv_seen, "기록 없는 이동 표적 공백 CV");
            check(r.stops == 0 && r.following_after_gap, "기록 없는 이동 표적 공백 뒤 추종 유지");
        }
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() == 0) {
        std::cout << "short gap follow: all checks passed\n";
    }
    return failures() == 0 ? 0 : 1;
}
