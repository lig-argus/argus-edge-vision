// 계약 PUB/SUB 이후 시작·보정 방향·만료/선택 epoch 검사와 단일 vehicle writer를 확인한다.
#include "argus/control/runtime/include/flight_control_runtime.hpp"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "argus/common/time/include/pose_history.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
class FakePoseSource final : public IVehicleStateSource {
public:
    Pose value;
    PoseHistory history{2};
    mutable std::chrono::nanoseconds aligned_bracket{};
    std::vector<std::string>* calls{nullptr};
    Pose read() const override {
        if (calls) {
            calls->push_back("pose.read");
        }
        return value;
    }
    std::optional<Pose> read_aligned(std::int64_t time_ns, const std::string& domain,
                                     std::chrono::nanoseconds bracket) const override {
        aligned_bracket = bracket;
        return history.interpolate(time_ns, domain, bracket);
    }
};

class FakeFlightStateSource final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus value;
    std::vector<std::string>* calls{nullptr};
    VehicleFlightStatus read() const override {
        if (calls) {
            calls->push_back("fc.read");
        }
        return value;
    }
};

class FakeVehiclePort final : public follow::IVehiclePort {
public:
    std::vector<follow::FollowCommand> sent;
    std::vector<std::string>* calls{nullptr};
    bool accept_send{true};
    std::vector<Clock::time_point> deadlines;
    int revocations{0};
    bool send_until(const follow::FollowCommand& command, Clock::time_point until) override {
        deadlines.push_back(until);
        return send(command);
    }
    void revoke_commands() override { ++revocations; }
    bool send(const follow::FollowCommand& command) override {
        if (calls) {
            calls->push_back("vehicle.send");
        }
        sent.push_back(command);
        return accept_send;
    }
};

class FakeModePort final : public IFlightModePort {
public:
    FakeFlightStateSource& state;
    int starts{0};
    int stops{0};
    bool accept_stop{true};
    bool leave_offboard_on_reject{false};
    bool confirm_hold{true};
    bool accept_start{true};
    bool confirm_offboard{true};
    std::optional<bool> start_ack{true}, hold_ack{true};
    std::optional<bool> start_result() const override { return start_ack; }
    std::optional<bool> hold_result() const override { return hold_ack; }
    std::vector<std::string>* calls{nullptr};
    explicit FakeModePort(FakeFlightStateSource& s) : state(s) {}
    bool start_offboard() override {
        if (calls) {
            calls->push_back("mode.start");
        }
        ++starts;
        if (!accept_start) {
            return false;
        }
        if (confirm_offboard) {
            state.value.hold = false;
            state.value.offboard = true;
        }
        return true;
    }
    bool stop_offboard() override {
        if (calls) {
            calls->push_back("mode.stop");
        }
        ++stops;
        if (!accept_stop) {
            if (leave_offboard_on_reject) {
                state.value.offboard = false;
            }
            return false;
        }
        if (confirm_hold) {
            state.value.offboard = false;
            state.value.hold = true;
        }
        return accept_stop;
    }
};

CameraGroundCalibration calibration() {
    CameraGroundCalibration c;
    c.image_width = 640;
    c.image_height = 480;
    c.fx = c.fy = 500;
    c.cx = 320;
    c.cy = 240;
    // optical camera xyz를 body FRD xyz로 매핑한다.
    c.camera_to_body = {0, 0, 1, 1, 0, 0, 0, 1, 0};
    return c;
}

FlightGuidanceConfig guidance() {
    FlightGuidanceConfig c;
    c.desired_range_m = 5;
    c.range_deadband_m = 0.2;
    c.yaw_kp_deg_s_per_deg = 2;
    c.max_yaw_rate_deg_s = 15;
    c.yaw_alignment_deg = 5;
    c.forward_gain_m_s_per_m = 0.2;
    c.max_forward_m_s = 0.3;
    c.altitude_gain = 0.8;
    c.max_vertical_m_s = 0.2;
    c.edge_margin_fraction = 0.05;
    return c;
}
}  // namespace

void hold_timeout_keeps_single_stop() {
    const auto now = Clock::now();
    TargetSelector selector;
    TracksV1 tracks;
    tracks.source = "test-yolox";
    tracks.frame_id = 1;
    tracks.sample_monotonic_ns = 1000000000;
    tracks.clock_domain = "host_monotonic";
    tracks.timestamp_source = "host_receive";
    tracks.exposure_time_valid = false;
    tracks.image_width = 640;
    tracks.image_height = 480;
    tracks.generation = 4;
    tracks.tracks.push_back(
        {11, 5, "rc_car", 0.92, 360, 120, 420, 240, "CONFIRMED", true, 1000000000});
    assert(selector.update(tracks, now));
    assert(selector.execute({"select-1", TargetCommandAction::Select, 4, 11}, now).accepted);
    assert(selector.execute({"start-1", TargetCommandAction::Start, 4, 11}, now).accepted);

    FakePoseSource pose;
    pose.value.have_position = pose.value.have_attitude = true;
    pose.value.position_at = pose.value.attitude_at = now;
    pose.value.position_monotonic_ns = pose.value.attitude_monotonic_ns = 1000000000;
    pose.value.clock_domain = "host_monotonic";
    FakeFlightStateSource flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.hold = true;
    FakeVehiclePort vehicle;
    FakeModePort mode(flight);
    CalibratedGroundGeometry geometry(calibration());
    FlightGuidanceController controller(guidance());
    FlightControlRuntimeConfig runtime_config;
    runtime_config.ground_down_m = 0;
    runtime_config.hold_down_m = -5;
    runtime_config.prestream_setpoint_count = 1;
    runtime_config.prestream_interval = 5ms;
    FlightSafetyEventLatch safety_events;
    FlightControlRuntime runtime(selector, pose, flight, geometry, controller, vehicle, mode,
                                 runtime_config, &safety_events);

    const auto preparing = runtime.step(now);
    assert(preparing.mission == "OFFBOARD_WAIT" && preparing.offboard_start_requested);
    assert(vehicle.sent.size() == 1 && mode.starts == 1);
    assert(!runtime.following_active_flag().load());
    const auto following = runtime.step(now + 1ms);
    assert(following.mission == "FOLLOW_ACTIVE" && following.command_sent);
    assert(runtime.following_active_flag().load());
    assert(vehicle.sent.size() == 2);
    // exposure 시각이 없으므로 미터 전진은 막고, 설정된 픽셀/장착 보정 방향의 yaw는 허용한다.
    assert(vehicle.sent.back().vx_m_s == 0);
    assert(vehicle.sent.back().yaw_rate_deg_s > 0);

    safety_events.publish(FlightSafetyEvent::TargetUnavailable);
    mode.confirm_hold = false;
    const auto stopped = runtime.step(now + 2ms);
    assert(stopped.mission == "HOLD_WAIT" && stopped.offboard_stop_requested);
    assert(mode.stops == 1 && vehicle.sent.size() == 3);
    assert(vehicle.sent.back().vx_m_s == 0 && vehicle.sent.back().yaw_rate_deg_s == 0);
    // 기한 안에 실제 Hold가 확인되지 않으면 FAULT로 고정하고 mode 재요청을 하지 않는다.
    const auto timed_out = runtime.step(now + runtime_config.hold_confirm_timeout + 3ms);
    assert(timed_out.mission == "FAULT" && mode.stops == 1 && vehicle.sent.size() == 3);
    assert(!runtime.following_active_flag().load());
}

namespace {
void require(bool condition, const char* reason) {
    if (!condition) {
        throw std::runtime_error(reason);
    }
}

// 동일한 입력을 수정 전·후 구현에 실행한다. 절대 시각은 출력하지 않고 포트 순서와 값만 비교한다.
struct RuntimeScenario {
    Clock::time_point now{Clock::now()};
    TargetSelector selector;
    FakePoseSource pose;
    FakeFlightStateSource flight;
    FakeVehiclePort vehicle;
    FakeModePort mode{flight};
    CalibratedGroundGeometry geometry{calibration()};
    FlightGuidanceController controller{guidance()};
    FlightControlRuntimeConfig config;
    FlightSafetyEventLatch safety_events;
    std::unique_ptr<FlightControlRuntime> runtime;
    std::vector<std::string> calls;
    std::string name;

    RuntimeScenario(std::string scenario, std::uint32_t prestream_count = 1, bool recovery = false,
                    bool strict_recovery = false)
        : selector([recovery] {
              TargetSelectorConfig settings;
              settings.recovery.enabled = recovery;
              return settings;
          }()),
          name(std::move(scenario)) {
        TracksV1 tracks;
        tracks.source = "contract-yolox";
        tracks.frame_id = 1;
        tracks.sample_monotonic_ns = 1000000000;
        tracks.clock_domain = "host_monotonic";
        tracks.timestamp_source = "host_receive";
        tracks.image_width = 640;
        tracks.image_height = 480;
        tracks.generation = 4;
        tracks.tracks.push_back(
            {11, 5, "rc_car", 0.92, 360, 120, 420, 240, "CONFIRMED", true, 1000000000});
        require(selector.update(tracks, now), "scenario observation rejected");
        require(selector.execute({"select", TargetCommandAction::Select, 4, 11}, now).accepted,
                "scenario selection rejected");
        pose.value.have_position = pose.value.have_attitude = true;
        pose.value.position_at = pose.value.attitude_at = now;
        pose.value.clock_domain = "host_monotonic";
        flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.hold =
            true;
        config.require_explicit_stop_recovery = strict_recovery;
        config.hold_down_m = -5;
        config.prestream_setpoint_count = prestream_count;
        config.prestream_interval = 5ms;
        runtime = std::make_unique<FlightControlRuntime>(
            selector, pose, flight, geometry, controller, vehicle, mode, config, &safety_events);
        pose.calls = flight.calls = vehicle.calls = mode.calls = &calls;
    }

    void command(TargetCommandAction action, std::chrono::milliseconds offset = 0ms) {
        const char* id = action == TargetCommandAction::Start ? "start" : "stop";
        require(selector.execute({id, action, 4, 11}, now + offset).accepted,
                "scenario operator command rejected");
    }

    FlightControlRuntimeStatus tick(std::chrono::milliseconds offset) {
        calls.clear();
        const auto result = runtime->step(now + offset);
        std::cout << name << ' ' << offset.count() << ' ' << result.mission << ' ' << result.reason
                  << ' ' << result.selection_command_epoch << ' ' << result.command_sent << ' '
                  << result.offboard_start_requested << ' ' << result.offboard_stop_requested << ' '
                  << runtime->following_active_flag().load() << " starts=" << mode.starts
                  << " stops=" << mode.stops;
        for (const auto& call : calls) {
            std::cout << ' ' << call;
        }
        for (const auto& sent : vehicle.sent) {
            std::cout << " [" << std::setprecision(17) << sent.vx_m_s << ',' << sent.vy_m_s << ','
                      << sent.vz_m_s << ',' << sent.yaw_rate_deg_s << ',' << sent.mode << ']';
        }
        std::cout << '\n';
        return result;
    }

    void follow() {
        command(TargetCommandAction::Start);
        require(tick(0ms).mission == "OFFBOARD_WAIT",
                "Start must wait for actual Offboard confirmation");
        require(tick(1ms).mission == "FOLLOW_ACTIVE", "confirmation tick must continue following");
        require(vehicle.sent.size() == 2 && mode.starts == 1,
                "confirmation sent wrong number of commands");
    }
};

void transition_contracts() {
    {
        RuntimeScenario scenario("selected_without_start");
        require(scenario.tick(0ms).mission == "SELECTED", "selection must wait for explicit Start");
        require(scenario.vehicle.sent.empty() && scenario.mode.starts == 0,
                "selection sent a command");
    }
    {
        RuntimeScenario scenario("cancel_priming", 2);
        scenario.command(TargetCommandAction::Start);
        require(scenario.tick(0ms).mission == "OFFBOARD_PREPARING", "priming ended too early");
        scenario.tick(1ms);
        require(scenario.vehicle.sent.size() == 1, "priming interval sent twice");
        scenario.command(TargetCommandAction::Stop, 2ms);
        require(scenario.tick(2ms).mission == "STOPPED", "Stop did not cancel preparation");
        scenario.tick(3ms);
        require(scenario.mode.starts == 0 && scenario.mode.stops == 0 &&
                    scenario.vehicle.sent.size() == 1,
                "cancelled preparation called a mode port");
    }
    {
        RuntimeScenario scenario("delayed_confirmation");
        scenario.mode.confirm_offboard = false;
        scenario.command(TargetCommandAction::Start);
        scenario.tick(0ms);
        require(scenario.tick(1ms).mission == "OFFBOARD_WAIT",
                "mode request treated as confirmation");
        scenario.flight.value.hold = false;
        scenario.flight.value.offboard = true;
        require(scenario.tick(2ms).mission == "FOLLOW_ACTIVE",
                "confirmation postponed guidance by a tick");
        require(scenario.vehicle.sent.size() == 3 && scenario.mode.starts == 1,
                "confirmation repeated Start or lost entry zero refresh");
    }
    {
        RuntimeScenario scenario("cancel_mode_wait");
        scenario.mode.confirm_offboard = false;
        scenario.command(TargetCommandAction::Start);
        scenario.tick(0ms);
        scenario.command(TargetCommandAction::Stop, 1ms);
        require(scenario.tick(1ms).mission == "HOLD_CONFIRMED", "Stop did not cancel mode wait");
        scenario.tick(2ms);
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 1,
                "cancellation sent again");
    }
    {
        RuntimeScenario scenario("offboard_timeout");
        scenario.mode.confirm_offboard = false;
        scenario.command(TargetCommandAction::Start);
        scenario.tick(0ms);
        require(scenario.tick(scenario.config.offboard_start_timeout + 1ms).reason ==
                    "offboard_mode_not_confirmed_no_retry",
                "mode timeout reason changed");
        scenario.tick(scenario.config.offboard_start_timeout + 2ms);
        require(scenario.mode.starts == 1 && scenario.vehicle.sent.size() == 1,
                "mode timeout retried");
    }
    {
        RuntimeScenario scenario("prime_failure");
        scenario.vehicle.accept_send = false;
        scenario.command(TargetCommandAction::Start);
        require(scenario.tick(0ms).reason == "offboard_prime_setpoint_failed_no_retry",
                "prime failure changed");
        scenario.tick(1ms);
        require(scenario.mode.starts == 0 && scenario.vehicle.sent.size() == 1,
                "failed priming retried");
    }
    {
        RuntimeScenario scenario("start_failure");
        scenario.mode.accept_start = false;
        scenario.command(TargetCommandAction::Start);
        require(scenario.tick(0ms).reason == "offboard_start_request_failed_no_retry",
                "start failure changed");
        scenario.tick(1ms);
        require(scenario.mode.starts == 1 && scenario.vehicle.sent.size() == 1,
                "failed Start retried");
    }
    {
        RuntimeScenario scenario("manual_priority");
        scenario.follow();
        scenario.flight.value.rc_override = true;
        scenario.pose.value.have_attitude = false;
        require(scenario.tick(2ms).mission == "MANUAL",
                "RC must precede other failed preconditions");
        scenario.flight.value.rc_override = false;
        require(scenario.tick(3ms).mission == "MANUAL", "manual latch resumed automatically");
        require(scenario.mode.stops == 0 && scenario.vehicle.sent.size() == 2,
                "manual mode sent a command");
    }
    {
        RuntimeScenario scenario("stop_in_follow");
        scenario.follow();
        scenario.command(TargetCommandAction::Stop, 2ms);
        require(scenario.tick(2ms).mission == "HOLD_CONFIRMED", "Stop did not confirm actual Hold");
        require(scenario.calls == std::vector<std::string>{"fc.read", "pose.read", "fc.read",
                                                           "vehicle.send", "mode.stop", "fc.read"},
                "operator Stop call order changed");
        scenario.tick(3ms);
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 3,
                "Stop repeated a command");
        const auto& last = scenario.vehicle.sent.back();
        require(
            last.vx_m_s == 0 && last.vy_m_s == 0 && last.vz_m_s == 0 && last.yaw_rate_deg_s == 0,
            "Stop reused a following command");
    }
    {
        RuntimeScenario scenario("stop_failure");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        // 송신권 없는 정지 거부는 FC 연결부의 기존 위치 유지를 송신 없이 짧게 기다린다.
        const auto waiting = scenario.tick(2ms);
        require(waiting.mission == "HOLD_WAIT" && waiting.reason == "awaiting_existing_hold_owner",
                "rejected stop did not wait for an existing hold owner");
        require(!waiting.stop_trigger_reason.empty() &&
                    waiting.safety_event == FlightSafetyEvent::TargetUnavailable,
                "stop failure lost its original trigger or safety event");
        scenario.tick(3ms);
        require(scenario.vehicle.sent.size() == 3, "waiting for an existing hold sent a command");
        const auto failed = scenario.tick(2ms + scenario.config.hold_confirm_timeout);
        require(
            failed.mission == "FAULT" && failed.reason == "offboard_stop_request_failed_no_retry",
            "stop failure without hold owner changed");
        scenario.tick(3ms + scenario.config.hold_confirm_timeout);
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 3,
                "failed Stop retried");
    }
    {
        // 2026-11-04 실기: FC 연결부가 송신 거부 뒤 Offboard 위치 유지를 이어받았는데
        // 추종기가 뒤이은 정지 거부를 FAULT로 기록했다.
        RuntimeScenario scenario("stop_rejected_existing_hover_late");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        require(scenario.tick(2ms).mission == "HOLD_WAIT",
                "rejected stop faulted before hover report");
        scenario.flight.value.offboard_hover = true;
        const auto confirmed = scenario.tick(30ms);
        require(confirmed.mission == "HOLD_CONFIRMED" &&
                    confirmed.reason == "existing_offboard_hover_confirmed",
                "existing Offboard hover was not accepted as the stop result");
        require(!scenario.runtime->following_active_flag().load(), "follow stayed active");
        const auto held = scenario.tick(31ms);
        require(
            held.mission == "HOLD_CONFIRMED" && held.reason == "explicit_stop_recovery_confirmed",
            "existing hover did not leave the explicit-stop recovery state");
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 3,
                "existing hover acceptance sent a command or retried Stop");
        scenario.flight.value.offboard_hover = false;
        require(scenario.tick(32ms).mission != "HOLD_CONFIRMED",
                "lost hover still reported as held");
    }
    {
        RuntimeScenario scenario("stop_rejected_existing_hover_now");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.flight.value.offboard_hover = true;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        const auto confirmed = scenario.tick(2ms);
        require(confirmed.mission == "HOLD_CONFIRMED" &&
                    confirmed.reason == "existing_offboard_hover_confirmed",
                "already reported hover was not accepted");
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 3,
                "hover acceptance retried");
    }
    {
        RuntimeScenario scenario("stop_rejected_existing_actual_hold");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        scenario.tick(2ms);
        scenario.flight.value.offboard = false;
        scenario.flight.value.hold = true;
        const auto confirmed = scenario.tick(3ms);
        require(confirmed.mission == "HOLD_CONFIRMED" &&
                    confirmed.reason == "existing_actual_hold_confirmed",
                "actual Hold by another owner was not accepted");
    }
    {
        RuntimeScenario scenario("stop_rejected_rc_during_wait");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        scenario.tick(2ms);
        scenario.flight.value.rc_override = true;
        scenario.flight.value.offboard_hover = true;  // RC must win over a hover report
        require(scenario.tick(3ms).mission == "MANUAL", "RC override during wait not latched");
        require(scenario.vehicle.sent.size() == 3, "RC wait path sent a command");
    }
    {
        RuntimeScenario scenario("stop_rejected_link_lost");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        scenario.tick(2ms);
        scenario.flight.value.connected = false;
        const auto failed = scenario.tick(3ms);
        require(
            failed.mission == "FAULT" && failed.reason == "offboard_stop_request_failed_no_retry",
            "lost FC link during wait did not fault");
    }
    {
        // 거부 시점에 FC가 Offboard가 아니면(Hold도 아님) 기다릴 위치 유지가 없어 기존처럼 바로
        // FAULT다.
        RuntimeScenario scenario("stop_rejected_not_offboard");
        scenario.follow();
        scenario.mode.accept_stop = false;
        scenario.mode.leave_offboard_on_reject = true;
        scenario.flight.value.hold = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        const auto failed = scenario.tick(2ms);
        require(
            failed.mission == "FAULT" && failed.reason == "offboard_stop_request_failed_no_retry",
            "non-Offboard stop rejection waited for a hover owner");
    }
    {
        RuntimeScenario scenario("delayed_hold");
        scenario.follow();
        scenario.mode.confirm_hold = false;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        require(scenario.tick(2ms).mission == "HOLD_WAIT",
                "stop request treated as Hold confirmation");
        const auto bridge = scenario.tick(3ms);
        require(bridge.command_sent && scenario.vehicle.sent.size() == 4,
                "fresh zero bridge was not refreshed");
        const auto& zero = scenario.vehicle.sent.back();
        require(
            zero.vx_m_s == 0 && zero.vy_m_s == 0 && zero.vz_m_s == 0 && zero.yaw_rate_deg_s == 0,
            "Hold bridge repeated motion");
        scenario.flight.value.hold = true;
        scenario.flight.value.offboard = false;
        require(scenario.tick(4ms).mission == "HOLD_CONFIRMED", "actual Hold not accepted");
        scenario.tick(5ms);
        require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == 4 &&
                    scenario.vehicle.revocations > 0,
                "Hold confirmation failed to revoke the bridge or repeated mode request");
    }
}

void recovery_control_contracts() {
    {
        RuntimeScenario concurrent("arrival_after_tick_start", 1, true);
        concurrent.follow();
        auto newer = *concurrent.selector.snapshot(concurrent.now).latest_frame;
        newer.frame_id = 2;
        newer.sample_monotonic_ns += 100000000;
        newer.tracks[0].last_real_detection_monotonic_ns = newer.sample_monotonic_ns;
        require(concurrent.selector.update(newer, concurrent.now + 101ms),
                "newer arrival rejected");
        concurrent.pose.value.position_at = concurrent.pose.value.attitude_at =
            concurrent.now + 100ms;
        const auto result = concurrent.tick(100ms);
        require(
            result.mission == "FOLLOW_ACTIVE" && result.command_sent && concurrent.mode.stops == 0,
            "concurrent tracks arrival falsely expired recovery");
        FlightControlWatchdog watchdog(concurrent.selector, concurrent.flight, concurrent.pose,
                                       concurrent.safety_events,
                                       concurrent.runtime->following_active_flag(), 20ms, 100ms);
        require(watchdog.inspect_once(concurrent.now + 100ms) == FlightSafetyEvent::None,
                "concurrent tracks arrival falsely triggered watchdog");
    }
    RuntimeScenario scenario("vision_gap_recovery", 1, true);
    auto frame = *scenario.selector.snapshot(scenario.now).latest_frame;
    scenario.follow();
    const auto mission = scenario.selector.snapshot(scenario.now).mission_target_id;
    scenario.pose.value.position_at = scenario.pose.value.attitude_at = scenario.now + 300ms;
    auto result = scenario.tick(300ms);  // 영상 메시지가 오지 않는 경우도 복구 대기한다.
    require(result.mission == "GAP_HOLD" && result.command_sent && !result.offboard_stop_requested,
            "vision gap must keep neutral Offboard stream");
    const auto& pause = scenario.vehicle.sent.back();
    require(pause.vx_m_s == 0 && pause.vy_m_s == 0 && pause.yaw_rate_deg_s == 0 &&
                std::abs(pause.vz_m_s) <= guidance().max_vertical_m_s,
            "vision gap sent horizontal motion or unbounded altitude command");
    FlightControlWatchdog watchdog(scenario.selector, scenario.flight, scenario.pose,
                                   scenario.safety_events,
                                   scenario.runtime->following_active_flag(), 20ms, 100ms);
    require(watchdog.inspect_once(scenario.now + 300ms) == FlightSafetyEvent::None,
            "watchdog stopped eligible recovery");
    scenario.flight.value.rc_override = true;
    require(watchdog.inspect_once(scenario.now + 300ms) == FlightSafetyEvent::ManualOverride,
            "vision recovery masked RC override");
    scenario.flight.value.rc_override = false;
    for (unsigned index = 0; index < 3; ++index) {
        const auto offset = 1000ms + index * 50ms;
        frame.frame_id = 2 + index;
        frame.generation = 5;
        frame.sample_monotonic_ns =
            1000000000 + std::chrono::duration_cast<std::chrono::nanoseconds>(offset).count();
        frame.tracks[0].track_id = 22;
        frame.tracks[0].last_real_detection_monotonic_ns = frame.sample_monotonic_ns;
        require(scenario.selector.update(frame, scenario.now + offset), "recovery frame rejected");
        scenario.pose.value.position_at = scenario.pose.value.attitude_at = scenario.now + offset;
        result = scenario.tick(offset);
        require(result.command_sent && !result.offboard_stop_requested,
                "recovery interrupted Offboard");
        require(result.mission == (index < 2 ? "CONFIRM" : "FOLLOW_ACTIVE"),
                "recovery resumed before three real observations");
        require(scenario.selector.snapshot(scenario.now + offset).mission_target_id == mission,
                "recovery changed mission identity");
    }
    require(scenario.mode.starts == 1 && scenario.mode.stops == 0,
            "vision recovery restarted flight mode");
    scenario.pose.value.position_at = scenario.pose.value.attitude_at = scenario.now + 3100ms;
    result = scenario.tick(3100ms);  // 복구 뒤 마지막 실관측에서 다시 2초가 경과했다.
    require(result.offboard_stop_requested && scenario.mode.stops == 1,
            "recovery deadline failed to transition to Hold");
    scenario.tick(3101ms);
    require(scenario.mode.stops == 1, "deadline stop retried");

    for (const auto fault : {0, 1, 2, 3, 4}) {
        RuntimeScenario failed("recovery_hard_fault", 1, true);
        auto fresh = *failed.selector.snapshot(failed.now).latest_frame;
        failed.follow();
        auto confirming = fresh;
        confirming.frame_id = 2;
        confirming.generation = 5;
        confirming.tracks[0].track_id = 22;
        confirming.sample_monotonic_ns = confirming.tracks[0].last_real_detection_monotonic_ns =
            1250000000;
        require(failed.selector.update(confirming, failed.now + 250ms),
                "confirmation frame rejected");
        require(failed.selector.snapshot(failed.now + 250ms).target_phase == TargetPhase::Confirm,
                "hard fault scenario must begin during confirmation");
        failed.pose.value.position_at = failed.pose.value.attitude_at = failed.now + 300ms;
        if (fault == 0) {
            failed.flight.value.rc_override = true;
        }
        if (fault == 1) {
            failed.pose.value.have_attitude = false;
        }
        if (fault == 2) {
            failed.vehicle.accept_send = false;
        }
        if (fault == 3) {
            failed.flight.value.offboard = false;
        }
        if (fault == 4) {
            failed.command(TargetCommandAction::Stop, 300ms);
        }
        const auto stopped = failed.tick(300ms);
        require(stopped.mission != "GAP_HOLD" && stopped.mission != "CONFIRM",
                "recovery masked hard stop condition");
        failed.flight.value.rc_override = false;
        failed.flight.value.offboard = true;
        failed.pose.value.have_attitude = true;
        failed.vehicle.accept_send = true;
        const auto sends_after_fault = failed.vehicle.sent.size();
        for (unsigned index = 0; index < 3; ++index) {
            const auto offset = 500ms + index * 50ms;
            fresh.frame_id = 3 + index;
            fresh.sample_monotonic_ns =
                1000000000 + std::chrono::duration_cast<std::chrono::nanoseconds>(offset).count();
            fresh.tracks[0].last_real_detection_monotonic_ns = fresh.sample_monotonic_ns;
            require(failed.selector.update(fresh, failed.now + offset),
                    "fresh fault frame rejected");
            failed.pose.value.position_at = failed.pose.value.attitude_at = failed.now + offset;
            failed.tick(offset);
        }
        require(failed.vehicle.sent.size() == sends_after_fault && failed.mode.starts == 1,
                "fresh vision cleared hard fault or operator Stop");
        const auto target = failed.selector.snapshot(failed.now + 600ms);
        require(fault == 4 ? target.phase == TargetSelectionSnapshot::Phase::Stopped
                           : target.target_phase == TargetPhase::Lost,
                "hard fault retained recoverable target memory");
    }
}
void pose_bracket_configuration() {
    for (const auto bracket : {100ms, 20ms, 250ms}) {
        RuntimeScenario scenario("pose_bracket");
        auto camera = calibration();
        if (bracket != 100ms) {
            camera.max_pose_bracket_distance = bracket;
        }
        scenario.geometry = CalibratedGroundGeometry(camera);
        auto frame = *scenario.selector.snapshot(scenario.now).latest_frame;
        constexpr std::int64_t exposure_ns = 1100000000;
        constexpr std::int64_t distance_ns = 75000000;
        frame.frame_id = 2;
        frame.sample_monotonic_ns = exposure_ns;
        frame.exposure_time_valid = true;
        frame.exposure_monotonic_ns = exposure_ns;
        frame.tracks[0].last_real_detection_monotonic_ns = exposure_ns;
        frame.tracks[0].x1 = 280;
        frame.tracks[0].x2 = 360;
        frame.tracks[0].y1 = 280;
        frame.tracks[0].y2 = 380;
        assert(scenario.selector.update(frame, scenario.now));
        scenario.pose.value.down_m = -5;
        scenario.pose.value.position_monotonic_ns = scenario.pose.value.attitude_monotonic_ns =
            exposure_ns + distance_ns;
        for (const auto timestamp : {exposure_ns - distance_ns, exposure_ns + distance_ns}) {
            auto sample = scenario.pose.value;
            sample.position_monotonic_ns = sample.attitude_monotonic_ns = timestamp;
            scenario.pose.history.offer(sample);
        }
        scenario.command(TargetCommandAction::Start);
        assert(scenario.runtime->step(scenario.now).mission == "OFFBOARD_WAIT");
        const auto result = scenario.runtime->step(scenario.now + 1ms);
        assert(result.mission == "FOLLOW_ACTIVE" && result.command_sent);
        assert(scenario.pose.aligned_bracket == bracket);
        // 실제 보간 표본은 양쪽 75ms 거리다. 20ms만 정렬/미터 전진을 거부한다.
        assert((scenario.vehicle.sent.back().vx_m_s > 0) == (bracket >= 75ms));
    }
}

void bounded_recovery_priming() {
    RuntimeScenario scenario("zero_only_recovery_priming", 3, true);
    scenario.config.allow_recovery_during_priming = true;
    scenario.runtime = std::make_unique<FlightControlRuntime>(
        scenario.selector, scenario.pose, scenario.flight, scenario.geometry, scenario.controller,
        scenario.vehicle, scenario.mode, scenario.config);
    auto frame = *scenario.selector.snapshot(scenario.now).latest_frame;
    scenario.command(TargetCommandAction::Start);
    require(scenario.tick(0ms).command_sent, "initial zero stream missing");
    frame.frame_id = 2;
    frame.sample_monotonic_ns += 50000000;
    frame.tracks.clear();
    require(scenario.selector.update(frame, scenario.now + 50ms),
            "missing-observation frame rejected");
    const auto paused = scenario.tick(50ms);
    require(paused.mission == "OFFBOARD_PREPARING" && scenario.mode.starts == 0,
            "priming gap entered Offboard or faulted before its deadline");
    for (const auto& cmd : scenario.vehicle.sent) {
        require(cmd.vx_m_s == 0 && cmd.vy_m_s == 0 && cmd.vz_m_s == 0 && cmd.yaw_rate_deg_s == 0,
                "priming during recovery sent motion");
    }
    const auto expired = scenario.tick(3100ms);
    require(expired.mission == "FAULT" && scenario.mode.starts == 0,
            "unconfirmed priming entered Offboard after deadline");
}

void hold_completion_contracts() {
    {
        RuntimeScenario scenario("already_in_hold");
        scenario.follow();
        scenario.flight.value.offboard = false;
        scenario.flight.value.hold = true;
        scenario.command(TargetCommandAction::Stop, 2ms);
        const auto result = scenario.tick(2ms);
        require(result.mission == "HOLD_CONFIRMED" &&
                    result.reason == "actual_hold_already_confirmed" && !result.command_sent &&
                    !result.offboard_stop_requested &&
                    !scenario.runtime->following_active_flag().load() && scenario.mode.stops == 0 &&
                    scenario.vehicle.sent.size() == 2,
                "already-Hold path requested a stop or changed completion");
    }
    {
        RuntimeScenario scenario("cancel_wait_delayed_hold");
        scenario.mode.confirm_offboard = false;
        scenario.mode.confirm_hold = false;
        scenario.command(TargetCommandAction::Start);
        scenario.tick(0ms);
        scenario.flight.value.hold = false;
        scenario.command(TargetCommandAction::Stop, 1ms);
        require(scenario.tick(1ms).mission == "HOLD_WAIT" && scenario.mode.stops == 1,
                "Start cancellation did not enter Holding");
        scenario.flight.value.hold = true;
        require(scenario.tick(2ms).mission == "HOLD_CONFIRMED" &&
                    scenario.vehicle.sent.size() == 1 &&
                    !scenario.runtime->following_active_flag().load(),
                "cancellation completion changed");
        auto probe = FlightGuidanceInput{};
        probe.now = scenario.now + 2ms;
        probe.selection.phase = TargetSelectionSnapshot::Phase::Following;
        probe.selection.command_epoch = probe.selection_command_epoch = 1;
        probe.offboard = true;
        require(scenario.controller.update(probe).request_offboard_stop_once,
                "cancellation with no pending result recorded a controller stop");
    }
    for (const bool delayed : {false, true}) {
        for (const bool zero_ok : {false, true}) {
            RuntimeScenario scenario(std::string("hold_contract_") +
                                     (delayed ? "delayed_" : "immediate_") +
                                     (zero_ok ? "success" : "zero_failure"));
            scenario.follow();
            scenario.vehicle.accept_send = zero_ok;
            scenario.mode.confirm_hold = !delayed;
            scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
            auto result = scenario.tick(2ms);
            require(scenario.calls == std::vector<std::string>{"fc.read", "fc.read", "pose.read",
                                                               "fc.read", "vehicle.send",
                                                               "mode.stop", "fc.read"},
                    "stop call order changed");
            require(result.command_sent == zero_ok && result.offboard_stop_requested &&
                        !result.offboard_start_requested && result.selection_command_epoch == 1,
                    "stop status flags or epoch changed");
            FlightControlWatchdog watchdog(scenario.selector, scenario.flight, scenario.pose,
                                           scenario.safety_events,
                                           scenario.runtime->following_active_flag(), 20ms, 100ms);
            if (delayed) {
                require(result.mission == "HOLD_WAIT" && result.reason == "operator_stop",
                        "delayed stop must enter Holding before confirmation");
                require(!scenario.runtime->following_active_flag().load() &&
                            watchdog.inspect_once(scenario.now + 2ms) == FlightSafetyEvent::None,
                        "Holding must disable watchdog inspection");
                result = scenario.tick(3ms);
                require(
                    result.mission == "HOLD_WAIT" &&
                        result.reason == "awaiting_hold_mode_confirmation" &&
                        scenario.calls ==
                            (zero_ok ? std::vector<std::string>{"fc.read", "fc.read", "pose.read",
                                                                "vehicle.send"}
                                     : std::vector<std::string>{"fc.read", "fc.read", "pose.read"}),
                    "pending Hold did not refresh/revoke the zero bridge correctly");
                scenario.flight.value.hold = true;
                scenario.flight.value.offboard = false;
                result = scenario.tick(2ms + scenario.config.hold_confirm_timeout);
                require(scenario.calls == std::vector<std::string>{"fc.read", "fc.read"} &&
                            !result.command_sent && !result.offboard_stop_requested,
                        "Hold confirmation repeated external operations");
            }
            require(result.mission == (zero_ok ? "HOLD_CONFIRMED" : "FAULT") &&
                        result.reason == (zero_ok ? "offboard_stopped_and_hold_confirmed"
                                                  : "zero_setpoint_failed"),
                    "Hold completion result changed (Hold must win at deadline)");
            require(!scenario.runtime->following_active_flag().load(),
                    "completed Hold retained a Following watchdog flag");
            require(watchdog.inspect_once(scenario.now + 4ms) == FlightSafetyEvent::None,
                    "completed Hold produced a stale Following watchdog event");
            FlightGuidanceInput probe;
            probe.now = scenario.now + 4ms;
            probe.selection.phase = TargetSelectionSnapshot::Phase::Following;
            probe.selection.command_epoch = probe.selection_command_epoch = 1;
            const auto recorded = scenario.controller.update(probe);
            require(recorded.mission == (zero_ok ? "HOLD_WAIT" : "FAULT") &&
                        !recorded.request_offboard_stop_once && !recorded.send_setpoint,
                    "runtime failed to record stop result");
            result = scenario.tick(3ms + scenario.config.hold_confirm_timeout);
            require(result.mission == (zero_ok ? "IDLE" : "FAULT") &&
                        scenario.calls == std::vector<std::string>{"fc.read"} &&
                        scenario.mode.stops == 1 &&
                        scenario.vehicle.sent.size() == (delayed && zero_ok ? 4U : 3U),
                    "completion phase timing or exactly-once stop changed");
            scenario.vehicle.accept_send = true;
            scenario.mode.confirm_hold = true;
            const auto restart_at = scenario.now + 4ms + scenario.config.hold_confirm_timeout;
            auto frame = *scenario.selector.snapshot(scenario.now).latest_frame;
            ++frame.frame_id;
            frame.sample_monotonic_ns += 3000000000;
            frame.tracks.front().last_real_detection_monotonic_ns = frame.sample_monotonic_ns;
            require(scenario.selector.update(frame, restart_at), "fresh restart frame rejected");
            scenario.pose.value.position_at = scenario.pose.value.attitude_at = restart_at;
            require(scenario.selector
                        .execute({"restart", TargetCommandAction::Start, 4, 11},
                                 scenario.now + 4ms + scenario.config.hold_confirm_timeout)
                        .accepted,
                    "explicit restart rejected");
            require(scenario.tick(4ms + scenario.config.hold_confirm_timeout).mission ==
                        "OFFBOARD_WAIT",
                    "explicit Start must reset completed stop state");
            require(scenario.tick(5ms + scenario.config.hold_confirm_timeout).mission ==
                        "FOLLOW_ACTIVE",
                    "restarted Offboard did not commit");
            scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
            result = scenario.tick(6ms + scenario.config.hold_confirm_timeout);
            require(result.mission == "HOLD_CONFIRMED" && scenario.mode.stops == 2 &&
                        scenario.vehicle.sent.size() == (delayed && zero_ok ? 7U : 6U),
                    "new stop retained pending result or repeated operations");
        }
    }
    for (const bool timeout : {false, true}) {
        RuntimeScenario scenario(timeout ? "hold_timeout_contract" : "hold_stop_failure_contract");
        scenario.follow();
        scenario.mode.confirm_hold = false;
        scenario.mode.accept_stop = timeout;
        scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable);
        auto result = scenario.tick(2ms);
        if (timeout) {
            require(result.mission == "HOLD_WAIT", "successful stop must wait for Hold");
            result = scenario.tick(2ms + scenario.config.hold_confirm_timeout);
            require(scenario.calls == std::vector<std::string>{"fc.read", "fc.read"},
                    "Hold timeout call order changed");
        } else {
            // 정지 거부 직후 기존 위치 유지 보고를 한 번 읽는다(fc.read). 송신은 늘지 않는다.
            require(scenario.calls == std::vector<std::string>{"fc.read", "fc.read", "pose.read",
                                                               "fc.read", "vehicle.send",
                                                               "mode.stop", "fc.read"},
                    "failed stop reordered calls");
            require(
                result.mission == "HOLD_WAIT" && result.reason == "awaiting_existing_hold_owner",
                "rejected stop did not wait for an existing hold owner");
            result = scenario.tick(2ms + scenario.config.hold_confirm_timeout);
        }
        require(result.mission == "FAULT" &&
                    result.reason == (timeout ? "hold_not_confirmed_no_retry"
                                              : "offboard_stop_request_failed_no_retry") &&
                    !scenario.runtime->following_active_flag().load(),
                "stop failure/timeout state changed");
        scenario.flight.value.hold = true;
        scenario.tick(3ms + scenario.config.hold_confirm_timeout);
        require(scenario.calls == std::vector<std::string>{"fc.read"} && scenario.mode.stops == 1 &&
                    scenario.vehicle.sent.size() == 3,
                "failed stop retried or accepted late Hold");
    }
}
}  // namespace

void actual_goal_diagnostic_contract() {
    using namespace follow;
    const auto now = Clock::now();
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    TargetSelector selector;
    TracksV1 frame;
    frame.schema = kTracksV2Schema;
    frame.source = "goal-diagnostic";
    frame.frame_id = 1;
    frame.sample_monotonic_ns = ns;
    frame.clock_domain = "host_monotonic";
    frame.timestamp_source = "host_receive";
    frame.image_width = 640;
    frame.image_height = 480;
    frame.generation = 1;
    TrackRecordV1 car{1, 2, "car", .9, 250, 180, 390, 330, "CONFIRMED", true, ns};
    car.ground = std::array<double, 6>{8, 0, .05, 0, .05, 0};
    frame.tracks = {car};
    assert(selector.update(frame, now));
    assert(selector.execute({"goal-select", TargetCommandAction::Select, 1, 1}, now).accepted);
    assert(selector.execute({"goal-start", TargetCommandAction::Start}, now).accepted);
    FakePoseSource pose;
    pose.value.have_position = pose.value.have_attitude = true;
    pose.value.north_m = 3;
    pose.value.down_m = -5;
    pose.value.position_at = pose.value.attitude_at = now;
    FakeFlightStateSource flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.offboard =
        true;
    FakeVehiclePort vehicle;
    FakeModePort mode(flight);
    CalibratedGroundGeometry geometry(calibration());
    auto config = guidance();
    config.ground_vector_follow = config.motion_ground_follow = config.observed_path_rear_follow =
        true;
    FlightGuidanceController controller(config);
    FlightControlRuntimeConfig runtime_config;
    runtime_config.hold_down_m = -5;
    FlightControlRuntime runtime(selector, pose, flight, geometry, controller, vehicle, mode,
                                 runtime_config);
    const auto result = runtime.step(now);
    assert(result.command_sent && result.rear_goal && result.guidance_pose &&
           result.vehicle_reference_ned);
    assert(result.rear_goal->north_m == 3 && result.rear_goal->east_m == 0);
    assert((*result.vehicle_reference_ned)[0] == 8 && result.guidance_pose->north_m == 3);
    assert(result.command_valid_until > now && result.command_sample_ns == ns);
    selector.execute({"goal-stop", TargetCommandAction::Stop}, now + std::chrono::milliseconds(1));
    const auto stopped = runtime.step(now + std::chrono::milliseconds(1));
    assert(!stopped.rear_goal);
}

void restart_does_not_consume_old_loss_event() {
    RuntimeScenario scenario("restart-old-safety-event");
    scenario.follow();
    const auto old_epoch = scenario.selector.snapshot(scenario.now).command_epoch;
    scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable, old_epoch);
    scenario.tick(2ms);  // Actual Hold after loss.
    scenario.safety_events.publish(FlightSafetyEvent::TargetUnavailable,
                                   old_epoch);  // Inspection completes late.
    assert(scenario.selector
               .execute({"safety-retry-stop", TargetCommandAction::Stop}, scenario.now + 3ms)
               .accepted);
    assert(scenario.selector
               .execute({"safety-retry-select", TargetCommandAction::Select, 4, 11},
                        scenario.now + 3ms)
               .accepted);
    assert(scenario.selector
               .execute({"safety-retry-start", TargetCommandAction::Start}, scenario.now + 3ms)
               .accepted);
    scenario.tick(3ms);
    scenario.tick(4ms);
    const auto resumed = scenario.tick(5ms);
    assert(resumed.mission == "FOLLOW_ACTIVE" && resumed.command_sent);
    scenario.safety_events.publish(FlightSafetyEvent::OffboardLost,
                                   scenario.selector.snapshot(scenario.now + 6ms).command_epoch);
    const auto current = scenario.tick(6ms);
    assert(current.mission == "HOLD_CONFIRMED" || current.mission == "HOLD_WAIT");
}

void start_rejects_missing_pose_before_priming() {
    RuntimeScenario scenario("missing_pose_before_start");
    scenario.pose.value.have_position = false;
    scenario.command(TargetCommandAction::Start);
    const auto result = scenario.tick(0ms);
    require(
        result.mission == "FAULT" && result.reason == "start_requires_fresh_pose_before_offboard",
        "missing pose was not rejected before Offboard preparation");
    require(scenario.vehicle.sent.empty() && scenario.mode.starts == 0,
            "missing pose sent a priming command or mode request");
}
void asynchronous_ack_and_actual_mode_are_independent() {
    RuntimeScenario scenario("async_mode_ack");
    scenario.mode.start_ack.reset();
    scenario.command(TargetCommandAction::Start);
    require(scenario.tick(0ms).mission == "OFFBOARD_WAIT", "async start request was not pending");
    require(scenario.tick(1ms).mission == "OFFBOARD_WAIT" && scenario.vehicle.sent.size() == 2,
            "actual Offboard without ACK enabled guidance");
    scenario.mode.start_ack = true;
    require(scenario.tick(2ms).mission == "FOLLOW_ACTIVE",
            "accepted ACK/actual mode failed to commit");
    scenario.mode.hold_ack.reset();
    scenario.mode.confirm_hold = false;
    scenario.command(TargetCommandAction::Stop, 3ms);
    require(scenario.tick(3ms).mission == "HOLD_WAIT", "async Hold did not preserve transition");
    require(scenario.tick(4ms).command_sent, "async Hold pending ACK stopped the zero bridge");
    scenario.mode.hold_ack = true;
    require(scenario.tick(5ms).mission == "HOLD_WAIT", "ACK alone was accepted as actual Hold");
    scenario.flight.value.hold = true;
    scenario.flight.value.offboard = false;
    require(scenario.tick(6ms).mission == "HOLD_CONFIRMED" && scenario.vehicle.revocations > 0,
            "actual Hold did not close egress");
    const auto count = scenario.vehicle.sent.size();
    scenario.tick(7ms);
    require(scenario.mode.stops == 1 && scenario.vehicle.sent.size() == count,
            "Hold completion sent again");
}

void explicit_position_handoff_preserves_manual_override() {
    RuntimeScenario denied("position_without_handoff");
    denied.flight.value.hold = false;
    denied.flight.value.position = denied.flight.value.rc_override = true;
    denied.command(TargetCommandAction::Start);
    require(denied.tick(0ms).mission == "MANUAL", "Position start bypassed explicit handoff");
    require(denied.vehicle.sent.empty(), "unprepared Position sent commands");

    RuntimeScenario ready("prepared_position_handoff");
    ready.flight.value.hold = false;
    ready.flight.value.position = ready.flight.value.rc_override = true;
    require(ready.runtime->prepare_handoff(ready.now), "fresh inactive Position rejected");
    ready.command(TargetCommandAction::Start);
    require(ready.tick(0ms).mission == "OFFBOARD_WAIT", "prepared Position did not prime");
    ready.flight.value.position = ready.flight.value.rc_override = false;
    require(ready.tick(1ms).mission == "FOLLOW_ACTIVE", "prepared handoff did not start");
    ready.flight.value.offboard = false;
    ready.flight.value.position = ready.flight.value.rc_override = true;
    require(ready.tick(2ms).mission == "MANUAL", "pilot Position takeover bypassed guard");
    require(!ready.runtime->handoff_ready(ready.now + 2ms), "handoff capability reused");

    RuntimeScenario ground("ground_handoff");
    ground.flight.value.hold = false;
    ground.flight.value.position = true;
    ground.flight.value.in_air = ground.flight.value.armed = false;
    require(ground.runtime->prepare_handoff(ground.now), "ground readiness unavailable");
    ground.command(TargetCommandAction::Start);
    require(ground.tick(0ms).mission == "FAULT" && ground.vehicle.sent.empty(),
            "ground handoff bypassed armed airborne guard");

    RuntimeScenario expired("expired_handoff");
    expired.flight.value.hold = false;
    expired.flight.value.position = true;
    require(expired.runtime->prepare_handoff(expired.now), "handoff preparation rejected");
    require(!expired.runtime->handoff_ready(expired.now + 16s), "expired handoff still ready");
}

void mode_wait_uses_pose_read_time_without_accepting_future_or_stale_samples() {
    for (int kind = 0; kind < 3; ++kind) {
        RuntimeScenario scenario("mode_wait_pose_clock");
        scenario.mode.confirm_offboard = false;
        scenario.command(TargetCommandAction::Start);
        require(scenario.tick(0ms).mission == "OFFBOARD_WAIT", "mode wait fixture failed");
        scenario.pose.value.snapshot_at = scenario.now + 2ms;
        scenario.pose.value.position_at = scenario.pose.value.attitude_at = scenario.now + 2ms;
        if (kind == 1) {
            scenario.pose.value.position_at = scenario.now + 3ms;
        } else if (kind == 2) {
            scenario.pose.value.snapshot_at = scenario.now + 400ms;
        }
        const auto result = scenario.tick(1ms);
        require(result.mission == (kind == 0 ? "OFFBOARD_WAIT" : "FAULT"),
                "pose read clock must allow new receipt but reject future/stale samples");
    }
}

void explicit_stop_recovery_contract() {
    RuntimeScenario fault("strict_fault", 1, false, true);
    fault.mode.accept_start = false;
    fault.command(TargetCommandAction::Start);
    require(fault.tick(0ms).mission == "FAULT", "fault fixture");
    fault.mode.accept_start = true;
    require(fault.tick(1ms).mission == "FAULT", "health recovery auto cleared fault");
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 1ms), "recovery without STOP");
    const auto starts = fault.mode.starts;
    require(fault.selector
                .execute({"retry-without-stop", TargetCommandAction::Start, 4, 11}, fault.now + 2ms)
                .accepted,
            "selector test command");
    require(fault.tick(2ms).mission == "FAULT" && fault.mode.starts == starts,
            "START bypassed fault latch");
    fault.command(TargetCommandAction::Stop, 3ms);
    fault.pose.value.attitude_at = fault.now - 1s;
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 3ms), "stale pose recovered");
    require(fault.tick(3ms).mission == "FAULT", "failed recovery changed phase");
    fault.pose.value.attitude_at = fault.now;
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 4ms),
            "old STOP recovered after health change");
    require(
        fault.selector.execute({"fresh-stop", TargetCommandAction::Stop}, fault.now + 4ms).accepted,
        "fresh STOP");
    fault.flight.value.rc_override = true;
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 4ms), "RC recovery");
    fault.flight.value.rc_override = false;
    fault.flight.value.hold = false;
    fault.flight.value.position = true;
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 4ms), "Position recovery");
    fault.flight.value.position = false;
    fault.flight.value.hold = true;
    fault.flight.value.connected = false;
    require(!fault.runtime->recover_after_explicit_stop(fault.now + 4ms), "disconnected recovery");
    fault.flight.value.connected = true;
    const auto sent = fault.vehicle.sent.size();
    const auto stops = fault.mode.stops;
    require(fault.runtime->recover_after_explicit_stop(fault.now + 4ms),
            "valid explicit STOP recovery");
    require(fault.tick(5ms).mission == "HOLD_CONFIRMED", "recovery not reported as actual Hold");
    require(fault.tick(6ms).mission == "HOLD_CONFIRMED", "recovery hold report not retained");
    require(fault.vehicle.sent.size() == sent && fault.mode.stops == stops &&
                fault.mode.starts == starts,
            "recovery sent motion or mode command");
    require(!fault.selector.snapshot(fault.now + 6ms).target, "recovery retained selected target");
    require(
        fault.selector.execute({"new-select", TargetCommandAction::Select, 4, 11}, fault.now + 7ms)
            .accepted,
        "new selection");
    require(
        fault.selector.execute({"new-start", TargetCommandAction::Start, 4, 11}, fault.now + 7ms)
            .accepted,
        "new START");
    require(fault.tick(7ms).mission == "OFFBOARD_WAIT", "fresh START after recovery failed");

    RuntimeScenario manual("strict_manual", 1, false, true);
    manual.follow();
    manual.flight.value.rc_override = true;
    require(manual.tick(2ms).mission == "MANUAL", "manual fixture");
    manual.flight.value.rc_override = false;
    manual.flight.value.offboard = false;
    manual.flight.value.hold = true;
    require(manual.tick(3ms).mission == "MANUAL", "manual auto cleared");
    manual.command(TargetCommandAction::Stop, 4ms);
    require(manual.runtime->recover_after_explicit_stop(manual.now + 4ms), "manual STOP recovery");
    require(manual.tick(5ms).mission == "HOLD_CONFIRMED", "manual recovery missing Hold");
}

// 2026-11-06 실기: 착륙 뒤 FAULT 고정이 풀리지 않아 메인 GUI가 ARM을 막았다.
void ground_reset_clears_fault_latch() {
    RuntimeScenario fault("ground_fault", 1, false, true);
    fault.mode.accept_start = false;
    fault.command(TargetCommandAction::Start);
    require(fault.tick(0ms).mission == "FAULT", "ground fault fixture");
    require(fault.runtime->recovery_latched(), "fault not latched");
    // 공중·ARM이면 초기화하지 않는다.
    require(!fault.runtime->reset_ground_context(fault.now + 1ms), "airborne fault reset");
    fault.flight.value.armed = false;
    fault.flight.value.in_air = false;
    fault.flight.value.ground_known = true;
    fault.flight.value.landed = true;
    fault.flight.value.offboard = true;  // PX4 failsafe 뒤 DISARM 상태의 OFFBOARD 복귀
    fault.flight.value.rc_override = true;
    require(!fault.runtime->reset_ground_context(fault.now + 2ms), "RC override reset");
    fault.flight.value.rc_override = false;
    fault.flight.value.connected = false;
    require(!fault.runtime->reset_ground_context(fault.now + 3ms), "disconnected reset");
    fault.flight.value.connected = true;
    fault.flight.value.ground_known = false;
    require(!fault.runtime->reset_ground_context(fault.now + 4ms), "unknown ground reset");
    fault.flight.value.ground_known = true;
    const auto sent = fault.vehicle.sent.size();
    const auto stops = fault.mode.stops;
    const auto starts = fault.mode.starts;
    require(fault.runtime->reset_ground_context(fault.now + 5ms), "ground DISARM reset");
    require(!fault.runtime->recovery_latched() && fault.runtime->inactive(), "fault kept");
    require(fault.vehicle.sent.size() == sent && fault.mode.stops == stops &&
                fault.mode.starts == starts,
            "ground reset sent motion or mode command");
    require(!fault.selector.snapshot(fault.now + 5ms).target, "ground reset kept target");
    require(fault.tick(6ms).mission != "FAULT", "fault reported after ground reset");
}

int main() {
    ground_reset_clears_fault_latch();
    explicit_stop_recovery_contract();
    {
        RuntimeScenario scenario("actual_hold_before_ack");
        scenario.follow();
        scenario.mode.hold_ack.reset();
        scenario.mode.confirm_hold = false;
        scenario.command(TargetCommandAction::Stop, 3ms);
        require(scenario.tick(3ms).mission == "HOLD_WAIT", "Hold request not pending");
        const auto revoked = scenario.vehicle.revocations;
        const auto sent = scenario.vehicle.sent.size();
        scenario.flight.value.hold = true;
        scenario.flight.value.offboard = false;
        require(scenario.tick(4ms).mission == "HOLD_WAIT", "actual Hold alone completed command");
        require(scenario.vehicle.revocations == revoked && scenario.vehicle.sent.size() == sent,
                "actual Hold before ACK cancelled pending confirmation or sent a command");
        scenario.mode.hold_ack = true;
        require(scenario.tick(5ms).mission == "HOLD_CONFIRMED",
                "late ACK did not finish actual Hold");
    }
    mode_wait_uses_pose_read_time_without_accepting_future_or_stale_samples();
    explicit_position_handoff_preserves_manual_override();
    asynchronous_ack_and_actual_mode_are_independent();
    start_rejects_missing_pose_before_priming();
    restart_does_not_consume_old_loss_event();
    actual_goal_diagnostic_contract();
    bounded_recovery_priming();
    recovery_control_contracts();
    hold_timeout_keeps_single_stop();
    transition_contracts();
    hold_completion_contracts();
    pose_bracket_configuration();
}
