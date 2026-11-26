// Offline pure core test: no sockets, SDK, FC, camera or flight commands.
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "argus/control/contracts/include/follow_b_request.hpp"
#include "argus/control/contracts/include/follow_b_status.hpp"
#include "argus/control/contracts/include/icd_command_progress.hpp"
#include "argus/runtime/follow/include/follow_status_snapshot.hpp"
#include "argus/runtime/follow/include/follow_target_request.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
template <class F>
void rejects(F function) {
    bool rejected = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

void scan_test() {
    using namespace follow;
    const auto defaults = parse_scan_request({});
    assert(defaults.sweep_deg == 360 && defaults.timeout.count() == 60000);
    assert(std::abs(defaults.rate_deg_s - 10) < 1e-12);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto nan_defaults = parse_scan_request({nan, nan, nan});
    assert(nan_defaults.sweep_deg == 360 && nan_defaults.timeout == defaults.timeout);
    const auto float_boundary =
        parse_scan_request({double(float(2 * kPi)), double(float(1.309)), 60.});
    assert(float_boundary.sweep_deg <= 360 && float_boundary.rate_deg_s <= 75);
    assert(std::abs(float_boundary.sweep_deg - 360) < 1e-10 &&
           std::abs(float_boundary.rate_deg_s - 75) < 1e-10);
    const auto xml_boundary =
        parse_scan_request({double(float(6.2832)), -double(float(1.309)), 60.});
    assert(xml_boundary.sweep_deg <= 360 && xml_boundary.rate_deg_s >= -75);
    const auto xml_double = parse_scan_request({6.2832, 1.309, 60.});
    assert(xml_double.sweep_deg <= 360 && xml_double.rate_deg_s <= 75);
    rejects([&] { parse_scan_request({6.2833, {}, {}}); });
    rejects([&] { parse_scan_request({{}, 1.31, {}}); });
    auto custom = defaults;
    custom.sweep_deg = 123.456789;
    custom.timeout = std::chrono::milliseconds{60001};
    const auto preserved = parse_scan_request({nan, nan, nan}, custom);
    assert(preserved.sweep_deg == custom.sweep_deg && preserved.timeout == custom.timeout);
    const auto half = parse_scan_request({kPi, -kPi / 18, 30.0001});
    assert(half.sweep_deg == 180 && half.timeout.count() == 30001);
    assert(std::abs(half.rate_deg_s + 10) < 1e-12);
    for (const auto bad : {inf, -inf, 0.0, -1.0, 2 * kPi + .01}) {
        rejects([&] { parse_scan_request({bad, {}, {}}); });
    }
    for (const auto bad : {inf, -inf, 0.0, 76 * kPi / 180, -76 * kPi / 180}) {
        rejects([&] { parse_scan_request({{}, bad, {}}); });
    }
    for (const auto bad : {inf, -inf, 0.0, -1.0, 36.0, 1e100}) {
        rejects([&] { parse_scan_request({{}, {}, bad}); });
    }
    YawScanConfig old_aggregate{10, std::chrono::milliseconds{60000}};
    assert(old_aggregate.sweep_deg == 360);
    YawScanConfig old_four{10, std::chrono::milliseconds{60000}, std::chrono::milliseconds{200},
                           75};
    assert(old_four.sweep_deg == 360);
    for (const double direction : {1.0, -1.0}) {
        auto config = parse_scan_request({kPi, direction * 10 * kPi / 180, 30});
        YawScanController scan(config);
        const auto begin = Clock::time_point{std::chrono::seconds{1}};
        scan.start(170, begin, begin, 1);
        // Elapsed time on a stationary aircraft is not a measured sweep.
        auto stamp = begin;
        for (int i = 0; i < 10; ++i) {
            stamp += std::chrono::milliseconds{100};
            assert(scan.update(170, stamp, stamp, true, false, i + 2));
        }
        assert(scan.active() && scan.progress_deg() == 0);
        for (int i = 1; i <= 180; ++i) {
            stamp += std::chrono::milliseconds{100};
            const double yaw = std::remainder(170 + direction * i, 360.0);
            assert(scan.update(yaw, stamp, stamp, true, false, i + 20));
            if (i < 180) {
                assert(scan.active());
            }
        }
        assert(scan.state() == YawScanController::State::Completed);
        assert(scan.progress_deg() == 180);
    }
    // A stalled actuator remains scanning and receives bounded lookahead even
    // after the clock reference has advanced beyond a full turn.
    for (const double direction : {1.0, -1.0}) {
        auto config = defaults;
        config.rate_deg_s *= direction;
        YawScanController scan(config);
        auto stamp = Clock::time_point{std::chrono::seconds{1}};
        scan.start(170, stamp, stamp, 1);
        for (int i = 1; i <= 500; ++i) {
            stamp += std::chrono::milliseconds{100};
            assert(scan.update(170, stamp, stamp, true, false, i + 1));
            assert(scan.active() && scan.progress_deg() == 0);
            const double delta = direction * std::remainder(scan.heading_deg() - 170, 360.0);
            assert(delta >= 0 && delta <= 10.000001);
        }
    }
    // Model heading commands sampled every 500 ms and a lagging first-order
    // actuator. Both signed sweeps must complete from measured yaw within 60 s.
    for (const double direction : {1.0, -1.0}) {
        auto config = defaults;
        config.rate_deg_s *= direction;
        YawScanController scan(config);
        auto stamp = Clock::time_point{std::chrono::seconds{1}};
        double yaw = 170;
        double held_heading = yaw;
        scan.start(yaw, stamp, stamp, 1);
        for (int i = 1; i < 600 && scan.active(); ++i) {
            yaw = std::remainder(yaw + .1 * std::remainder(held_heading - yaw, 360.0), 360.0);
            stamp += std::chrono::milliseconds{100};
            assert(scan.update(yaw, stamp, stamp, true, false, i + 1));
            if (i % 5 == 0) {
                held_heading = scan.heading_deg();
            }
        }
        assert(scan.state() == YawScanController::State::Completed);
        assert(scan.progress_deg() >= 360);
    }
    YawScanController candidate;
    auto now = Clock::time_point{std::chrono::seconds{1}};
    candidate.start(0, now, now, 10);
    now += std::chrono::milliseconds{100};
    assert(candidate.update(1, now, now, true, true, 11));
    assert(candidate.state() == YawScanController::State::Candidate);
    assert(candidate.reason() == "candidate_requires_explicit_selection_and_start");
    now += std::chrono::milliseconds{100};
    assert(!candidate.update(2, now, now, true, false, 12));
    assert(candidate.state() == YawScanController::State::Candidate);
}

void icd_test() {
    using namespace follow;
    const auto begin = Clock::time_point{std::chrono::seconds{100}};
    YawScanConfig config;
    config.sweep_deg = 1;
    config.rate_deg_s = 10;
    config.timeout = std::chrono::milliseconds{200};
    YawScanController timeout(config);
    timeout.start(12, begin, begin, 1);
    const auto end = begin + std::chrono::milliseconds{200};
    assert(timeout.update(12, end, end, true, false, 2));
    assert(timeout.state() == YawScanController::State::TimedOut && timeout.heading_deg() == 12);
    assert(timeout.progress_deg() == 0);  // 시간 종료를 한 바퀴 완료로 위장하지 않는다.
    timeout.start(12, begin, begin, 1);
    assert(!timeout.update(12, end, end, false, false, 2));
    assert(timeout.state() == YawScanController::State::Fault);

    TracksV1 frame;
    frame.source = "camera";
    frame.generation = 3;
    TrackRecordV1 track;
    track.track_id = 2;
    track.class_id = 4;
    track.label = "car";
    track.state = "CONFIRMED";
    track.real_detection = true;
    frame.tracks.push_back(track);
    NativeJson request{{"action", "track_start"}, {"command_id", "start"}, {"sensor", "camera"},
                       {"generation", 3},         {"class_id", 4},         {"track_id", 2}};
    assert(resolve_icd_target(request, frame).at("label") == "car");
    frame.tracks.front().class_id = 5;
    rejects([&] { resolve_icd_target(request, frame); });
    frame.tracks.front() = track;
    frame.tracks.push_back(track);
    rejects([&] { resolve_icd_target(request, frame); });

    IcdCommandProgress progress;
    progress.begin(request, true);
    assert(progress.snapshot().back().at("outcome") == "IN_PROGRESS");
    progress.reject({{"command_id", "denied"}, {"action", "search"}}, "invalid_state");
    assert(progress.snapshot().back().at("outcome") == "REJECTED");
    assert(progress.snapshot().front().at("pending") == true);
    assert(progress.snapshot().back().at("request_accepted") == false);
    for (int i = 0; i < 40; ++i) {
        progress.reject({{"command_id", i}, {"action", "search"}}, "invalid_state");
    }
    assert(progress.snapshot().size() == 32 && progress.snapshot().front().at("pending") == true);
    // Use a fresh history for the original execution assertions.
    progress = IcdCommandProgress{};
    progress.begin(request, true);
    progress.update("PREPARING", false, true, false, false, false, "Idle", "", nullptr);
    assert(progress.snapshot().back().at("pending") == true);
    progress.update("FOLLOW_ACTIVE", true, false, false, false, false, "Idle", "",
                    {{"generation", 3}, {"track_id", 2}});
    assert(progress.snapshot().back().at("result") == "accepted");
    progress.begin({{"command_id", "scan"}, {"action", "search"}}, true);
    progress.update("HOLD_TRANSITION", false, true, true, false, true, "TimedOut", "limit",
                    nullptr);
    assert(progress.snapshot().back().at("pending") == true);
    progress.update("HOLD_WAIT", false, true, true, false, false, "TimedOut", "limit", nullptr);
    assert(progress.snapshot().back().at("result") == "accepted");
    progress.begin({{"command_id", "stop"}, {"action", "stop"}}, true);
    progress.update("MANUAL", false, false, true, false, false, "Stopped", "", nullptr);
    assert(progress.snapshot().back().at("result") == "failed");
    assert(progress.snapshot().back().at("outcome") == "FAILED");
    assert(progress.snapshot().back().at("request_accepted") == true);
    progress.begin({{"command_id", "idle-stop"}, {"action", "stop"}}, false);
    assert(progress.snapshot().back().at("outcome") == "SUCCEEDED");
    const auto fields = native_tracking_status(timeout, TargetPhase::Lost).at("scan_config");
    assert(std::abs(fields.at("yaw_rate_rad_s").get<double>() - config.rate_deg_s * kPi / 180) <
           1e-12);
    assert(fields.at("timeout_s").get<double>() == 0.2);
    assert(fields.at("timeout_ms") == 200 && fields.at("rate_deg_s") == 10);
}

void status_test() {
    using namespace follow;
    FollowBStatusInput input;
    assert(project_follow_b_status(input).mission == "IDLE");
    input.mission = "OFFBOARD_PREPARING";
    assert(project_follow_b_status(input).mission == "PREPARING");
    input.mission = "OFFBOARD_WAIT";
    assert(project_follow_b_status(input).mission == "PREPARING");
    input.mission = "FOLLOW_ACTIVE";
    assert(project_follow_b_status(input).mission == "FOLLOW_ACTIVE");
    input.mission = "HOLD_WAIT";
    assert(project_follow_b_status(input).mission == "HOLD_TRANSITION");
    input.actual_hold = true;
    assert(project_follow_b_status(input).mission == "HOLD_WAIT");
    input.hold_transition = true;
    assert(project_follow_b_status(input).mission == "HOLD_TRANSITION");
    input.hold_transition = false;
    // 2026-11-13: [임무 중단] 뒤 Offboard 제자리 유지도 실제 정지(HOLD_WAIT → GUI IDLE)다
    {
        FollowBStatusInput stopped;
        stopped.mission = "HOLD_CONFIRMED";
        stopped.actual_hold = confirmed_stop_hold(false, true, true);
        assert(project_follow_b_status(stopped).mission == "HOLD_WAIT");
        stopped.actual_hold = confirmed_stop_hold(false, true, false);
        assert(project_follow_b_status(stopped).mission == "HOLD_TRANSITION");
        stopped.actual_hold = confirmed_stop_hold(false, false, true);
        assert(project_follow_b_status(stopped).mission == "HOLD_TRANSITION");
        assert(confirmed_stop_hold(true, false, false));
    }
    input.scan_active = true;
    assert(project_follow_b_status(input).mission == "SCAN_ACTIVE");
    input.fault = true;
    assert(project_follow_b_status(input).mission == "FAULT");
    input.rc_override = true;
    assert(project_follow_b_status(input).mission == "MANUAL");
    input.target_phase = TargetPhase::Locked;
    input.target_confirmed = true;
    assert(project_follow_b_status(input).target == "NONE");
    input.target_selected = true;
    assert(project_follow_b_status(input).target == "CONFIRMED");
    input.target_phase = TargetPhase::GapHold;
    input.target_prediction_valid = true;
    input.recovery_reason = "brief_gap";
    input.ambiguous = true;
    const auto predicted = project_follow_b_status(input);
    assert(predicted.target == "PREDICTED" && predicted.target_phase == "GAP_HOLD");
    assert(predicted.recovery_reason == "brief_gap" && predicted.ambiguous);
    input.target_phase = TargetPhase::Lost;
    assert(project_follow_b_status(input).target == "LOST");
    input.target_prediction_valid = false;
    input.target_phase = TargetPhase::Confirm;
    assert(project_follow_b_status(input).target == "LOST");
}
}  // namespace

int main() {
    scan_test();
    status_test();
    icd_test();
}
