// Confirmed/PREDICTED/유실과 RC·Offboard 이벤트의 명령 안전 게이트를 확인한다.
#include "argus/control/guidance/include/flight_guidance_controller.hpp"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace follow;
using namespace std::chrono_literals;

namespace {
FlightGuidanceConfig settings() {
    FlightGuidanceConfig c;
    c.desired_range_m = 5;
    c.range_deadband_m = 0.2;
    c.yaw_kp_deg_s_per_deg = 2;
    c.max_yaw_rate_deg_s = 10;
    c.yaw_alignment_deg = 5;
    c.forward_gain_m_s_per_m = 0.2;
    c.max_forward_m_s = 0.3;
    c.altitude_gain = 0.8;
    c.max_vertical_m_s = 0.2;
    c.edge_margin_fraction = 0.05;
    return c;
}
FlightGuidanceInput input(Clock::time_point now) {
    FlightGuidanceInput in;
    in.now = now;
    in.connected = in.armed = in.in_air = in.offboard = true;
    in.pose.have_position = in.pose.have_attitude = true;
    in.pose.position_at = in.pose.attitude_at = now;
    in.pose.down_m = -5;
    in.hold_down_m = -5;
    in.selection.phase = TargetSelectionSnapshot::Phase::Following;
    in.selection.command_epoch = 1;
    in.selection_command_epoch = 1;
    in.selection.target = SelectedTargetKey{2, 9};
    in.selection.state = TargetState::Confirmed;
    in.selection.fresh_real_detection = true;
    in.selection.selected_track =
        TrackRecordV1{9, 5, "rc_car", 0.9, 200, 150, 300, 300, "CONFIRMED", true, 1000000000};
    TracksV1 tracks;
    tracks.generation = 2;
    tracks.image_width = 640;
    tracks.image_height = 480;
    in.selection.latest_frame = tracks;
    in.geometry.range = {8.0, "METRIC", "fixture", true};
    in.geometry.bearing_valid = true;
    in.geometry.bearing_error_deg = 1;
    return in;
}
}  // namespace

void stop_result_contracts() {
    const auto now = Clock::now();
    const auto require = [](bool condition, const char* reason) {
        if (!condition) {
            throw std::runtime_error(reason);
        }
    };
    for (const bool success : {false, true}) {
        for (const bool hold : {false, true}) {
            FlightGuidanceController controller(settings());
            controller.explicit_start_committed(1);
            controller.request_operator_stop();
            auto result = controller.update(input(now));
            require(result.mission == "HOLD_TRANSITION" && result.reason == "operator_stop" &&
                        result.send_setpoint && result.request_offboard_stop_once,
                    "first stop must request zero and mode change once");
            result = controller.update(input(now));
            require(result.mission == "HOLD_WAIT" && !result.send_setpoint &&
                        !result.request_offboard_stop_once,
                    "repeated pending stop issued commands");
            controller.record_offboard_stop_result(success, hold);
            controller.record_offboard_stop_result(!success, !hold);
            result = controller.update(input(now));
            require(result.mission == (success && hold ? "HOLD_WAIT" : "FAULT") &&
                        !result.send_setpoint && !result.request_offboard_stop_once,
                    "stop outcome must require success and Hold; first recording must win");
            auto manual = input(now);
            manual.rc_override = true;
            require(controller.update(manual).mission == "MANUAL",
                    "RC must override recorded stop outcome");
            manual.rc_override = false;
            require(controller.update(manual).mission == "MANUAL",
                    "manual override must stay latched");
            controller.explicit_start_committed(1);
            require(controller.update(input(now)).mission == "FOLLOW_ACTIVE",
                    "explicit Start must clear stop/manual state");
            controller.request_operator_stop();
            require(controller.update(input(now)).request_offboard_stop_once,
                    "new Start must permit one new stop");
        }
    }
}

void stop_transition_contracts() {
    const auto now = Clock::now();
    const auto require = [](bool condition, const char* reason) {
        if (!condition) {
            throw std::runtime_error(reason);
        }
    };
    int stop_requests = 0;
    const auto tick = [&](FlightGuidanceController& controller, const char* event,
                          const FlightGuidanceInput& in, const char* mission, const char* reason,
                          bool send, bool request) {
        const auto result = controller.update(in);
        require(result.mission == mission && result.reason == reason &&
                    result.send_setpoint == send && result.request_offboard_stop_once == request &&
                    result.selection_command_epoch == in.selection_command_epoch,
                "stop transition output changed");
        const bool following = result.mission == "FOLLOW_ACTIVE";
        require(result.command.vx_m_s == (following ? 0.3 : 0.0) && result.command.vy_m_s == 0 &&
                    result.command.vz_m_s == 0 &&
                    result.command.yaw_rate_deg_s == (following ? 2.0 : 0.0),
                "command values before/after stop changed");
        const bool default_mode = result.mission == "MANUAL" || result.mission == "IDLE";
        require(result.command.mode == (default_mode ? "LOST_HOVER" : result.mission),
                "stop command mode changed");
        require(
            result.valid_until == (send ? in.now + settings().command_ttl : Clock::time_point{}),
            "stop command lifetime changed");
        stop_requests += result.request_offboard_stop_once;
        std::cout << event << " input=" << in.offboard << ',' << in.rc_override << ','
                  << in.selection_command_epoch << ',' << static_cast<int>(in.selection.state)
                  << " result=" << result.mission << ',' << result.reason << ','
                  << result.send_setpoint << ',' << result.request_offboard_stop_once << ','
                  << result.selection_command_epoch << " command=" << std::setprecision(17)
                  << result.command.vx_m_s << ',' << result.command.vy_m_s << ','
                  << result.command.vz_m_s << ',' << result.command.yaw_rate_deg_s << ','
                  << result.command.mode
                  << " ttl=" << (send ? (result.valid_until - in.now).count() : 0)
                  << " stop_requests=" << stop_requests << '\n';
    };
    for (const int trigger : {0, 1, 2}) {
        for (const bool success : {false, true}) {
            FlightGuidanceController controller(settings());
            stop_requests = 0;
            controller.record_offboard_stop_result(false, false);
            std::cout << "record_before_attempt ignored\n";
            controller.request_operator_stop();
            tick(controller, "stop_before_Start", input(now), "IDLE", "awaiting_explicit_start",
                 false, false);
            controller.explicit_start_committed(1);
            auto in = input(now);
            tick(controller, "Start_reset", in, "FOLLOW_ACTIVE", "confirmed_target", true, false);
            const char* first_reason = "selected_target_lost_or_stale";
            if (trigger == 0) {
                in.selection.state = TargetState::Lost;
            }
            if (trigger == 1) {
                controller.request_operator_stop();
                first_reason = "operator_stop";
            }
            if (trigger == 2) {
                in.selection_command_epoch = 2;
                first_reason = "operator_stop_or_selection_epoch_changed";
            }
            tick(controller, "first_stop", in, "HOLD_TRANSITION", first_reason, true, true);
            tick(controller, "same_input_repeat", in, "HOLD_WAIT",
                 trigger == 0 ? "resume_requires_operator_start" : first_reason, false, false);
            in = input(now + 1ms);
            const char* pending_reason =
                trigger == 1 ? "operator_stop" : "resume_requires_operator_start";
            tick(controller, "healthy_input_pending", in, "HOLD_WAIT", pending_reason, false,
                 false);
            controller.record_offboard_stop_result(success, success);
            std::cout << "record_result " << success << ',' << success << '\n';
            const char* completed_reason = trigger == 1
                                               ? "operator_stop"
                                               : (success ? "resume_requires_operator_start"
                                                          : "offboard_stop_failed_no_retry");
            tick(controller, "first_update_after_record", in, success ? "HOLD_WAIT" : "FAULT",
                 completed_reason, false, false);
            controller.record_offboard_stop_result(!success, !success);
            std::cout << "duplicate_record ignored\n";
            tick(controller, "duplicate_record_update", in, success ? "HOLD_WAIT" : "FAULT",
                 completed_reason, false, false);
            controller.request_operator_stop();
            tick(controller, "stop_while_stopped", in, success ? "HOLD_WAIT" : "FAULT",
                 "operator_stop", false, false);
            controller.request_manual_override();
            tick(controller, "manual_request", in, "MANUAL", "rc_override_latched", false, false);
            require(stop_requests == 1, "controller emitted more than one stop request per Start");
            controller.explicit_start_committed(2);
            in.selection.command_epoch = in.selection_command_epoch = 2;
            tick(controller, "new_epoch_Start", in, "FOLLOW_ACTIVE", "confirmed_target", true,
                 false);
            controller.request_operator_stop();
            tick(controller, "new_Start_stop", in, "HOLD_TRANSITION", "operator_stop", true, true);
            require(stop_requests == 2, "new Start failed to enable exactly one new stop");
        }
    }
    {
        FlightGuidanceController controller(settings());
        stop_requests = 0;
        controller.explicit_start_committed(1);
        auto in = input(now);
        in.offboard = false;
        tick(controller, "already_stopped_first", in, "HOLD_TRANSITION",
             "fc_or_mode_precondition_failed", false, false);
        in.offboard = true;
        tick(controller, "Offboard_return_no_auto_resume", in, "HOLD_WAIT",
             "resume_requires_operator_start", false, false);
        controller.record_offboard_stop_result(true, true);
        tick(controller, "suppressed_stop_record", in, "HOLD_WAIT",
             "resume_requires_operator_start", false, false);
        controller.record_offboard_stop_result(false, false);
        tick(controller, "suppressed_stop_duplicate_record", in, "HOLD_WAIT",
             "resume_requires_operator_start", false, false);
        require(stop_requests == 0, "already stopped input produced stop API requests");
    }
    {
        FlightGuidanceController controller(settings());
        stop_requests = 0;
        controller.explicit_start_committed(1);
        controller.request_operator_stop();
        auto in = input(now);
        in.rc_override = true;
        in.selection_command_epoch = 2;
        in.connected = false;
        in.selection.state = TargetState::Lost;
        tick(controller, "RC_precedes_stop_epoch_FC_target", in, "MANUAL", "rc_override_latched",
             false, false);
        controller.record_offboard_stop_result(false, false);
        in = input(now);
        tick(controller, "RC_release_latched", in, "MANUAL", "rc_override_latched", false, false);
        controller.explicit_start_committed(1);
        tick(controller, "Start_clears_manual_force_stop", in, "FOLLOW_ACTIVE", "confirmed_target",
             true, false);
        controller.request_operator_stop();
        tick(controller, "stop_after_manual_reset", in, "HOLD_TRANSITION", "operator_stop", true,
             true);
    }
    {
        FlightGuidanceController controller(settings());
        stop_requests = 0;
        controller.explicit_start_committed(1);
        controller.request_operator_stop();
        auto in = input(now);
        tick(controller, "pending_before_RC", in, "HOLD_TRANSITION", "operator_stop", true, true);
        in.rc_override = true;
        tick(controller, "RC_while_pending", in, "MANUAL", "rc_override_latched", false, false);
        controller.record_offboard_stop_result(false, false);
        std::cout << "record_failure_while_manual\n";
        in.rc_override = false;
        tick(controller, "manual_with_failed_stop", in, "MANUAL", "rc_override_latched", false,
             false);
        controller.explicit_start_committed(1);
        tick(controller, "Start_clears_pending_manual_result", in, "FOLLOW_ACTIVE",
             "confirmed_target", true, false);
    }
}

int main() {
    {
        auto c = settings();
        c.yaw_only = true;
        FlightGuidanceController yaw(c);
        auto in = input(Clock::now());
        assert(!yaw.update(in).send_setpoint);
        yaw.explicit_start_committed(1);
        const auto check_zero = [](const FlightGuidanceResult& out) {
            assert(out.command.vx_m_s == 0 && out.command.vy_m_s == 0 && out.command.vz_m_s == 0);
        };
        in.pose.down_m = -1;
        in.hold_down_m = -8;
        in.geometry.range = {50, "METRIC", "fixture", true};
        in.geometry.bearing_error_deg = 20;
        auto out = yaw.update(in);
        check_zero(out);
        assert(out.send_setpoint && out.command.yaw_rate_deg_s > 0 &&
               out.command.yaw_rate_deg_s <= c.max_yaw_rate_deg_s);
        assert(!out.altitude_target_down_m && out.reason == "yaw_only_confirmed_target");
        in.now += 50ms;
        in.pose.position_at = in.pose.attitude_at = in.now;
        in.geometry.bearing_error_deg = 0.1;
        out = yaw.update(in);
        check_zero(out);
        assert(out.command.yaw_rate_deg_s == 0);
        in.geometry.range.valid = false;
        in.geometry.bearing_error_deg = -10;
        out = yaw.update(in);
        check_zero(out);
        assert(out.command.yaw_rate_deg_s < 0);
        in.selection.state = TargetState::Predicted;
        out = yaw.update(in);
        check_zero(out);
        assert(out.command.yaw_rate_deg_s == 0);
        in.selection.state = TargetState::Confirmed;
        in.selection.recovery_pause = true;
        out = yaw.update(in);
        check_zero(out);
        assert(out.command.yaw_rate_deg_s == 0);
        in.selection.recovery_pause = false;
        in.selection.observation_valid_until = in.now + 40ms;
        out = yaw.update(in);
        check_zero(out);
        assert(out.mission == "GAP_HOLD" && out.command.yaw_rate_deg_s == 0);
        in.selection.observation_valid_until = in.now + 250ms;
        yaw.request_operator_stop();
        out = yaw.update(in);
        check_zero(out);
        assert(out.request_offboard_stop_once);
        out = yaw.update(in);
        assert(!out.send_setpoint);
        yaw.explicit_start_committed(1);
        in.rc_override = true;
        assert(yaw.update(in).mission == "MANUAL");
        in.rc_override = false;
        assert(yaw.update(in).mission == "MANUAL");
        c.ground_vector_follow = true;
        bool rejected = false;
        try {
            FlightGuidanceController mixed(c);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
        // 수치 피드백 검사: FC 동역학/영상 지연을 모델링한 실제 비행 성능 시험은 아니다.
        auto closed_config = settings();
        closed_config.yaw_only = true;
        FlightGuidanceController closed(closed_config);
        closed.explicit_start_committed(1);
        auto loop = input(Clock::now());
        double bearing = 15;
        for (int tick = 0; tick < 300; ++tick) {
            loop.now += 50ms;
            loop.pose.position_at = loop.pose.attitude_at = loop.now;
            loop.geometry.bearing_error_deg = bearing;
            const auto command = closed.update(loop);
            check_zero(command);
            bearing -= command.command.yaw_rate_deg_s * .05;
        }
        assert(std::abs(bearing) <= closed_config.yaw_only_bearing_deadband_deg);
    }
    {
        auto config = settings();
        config.ground_vector_follow = true;
        config.max_forward_m_s = 1.2;
        config.forward_gain_m_s_per_m = .6;
        config.yaw_alignment_deg = 25;
        config.max_horizontal_accel_m_s2 = .7;
        FlightGuidanceController vector(config);
        vector.explicit_start_committed(1);
        const Clock::time_point at{20s};
        auto in = input(at);
        in.geometry.north_m = 8;
        in.geometry.east_m = 1;
        auto out = vector.update(in);
        assert(out.command.vx_m_s > 0 && out.command.vy_m_s > 0);
        assert(std::hypot(out.command.vx_m_s, out.command.vy_m_s) <= .035 + 1e-9);
        for (int i = 1; i < 60; ++i) {
            in.now = at + i * 50ms;
            in.pose.position_at = in.pose.attitude_at = in.now;
            const auto previous = out.command;
            out = vector.update(in);
            assert(std::hypot(out.command.vx_m_s, out.command.vy_m_s) <= 1.2 + 1e-9);
            assert(std::hypot(out.command.vx_m_s - previous.vx_m_s,
                              out.command.vy_m_s - previous.vy_m_s) <= .035 + 1e-9);
        }
        in.selection.recovery_pause = true;
        out = vector.update(in);
        assert(out.command.vx_m_s == 0 && out.command.vy_m_s == 0);
        config.max_horizontal_accel_m_s2 = 0;
        FlightGuidanceController rotated(config);
        rotated.explicit_start_committed(1);
        in = input(at);
        in.pose.yaw_deg = 90;
        in.geometry.north_m = -1;
        in.geometry.east_m = 8;
        out = rotated.update(in);
        assert(out.command.vx_m_s > 0 && out.command.vy_m_s > 0);  // East前方, South右側
        in.geometry.range.valid = false;
        out = rotated.update(in);
        assert(out.command.vx_m_s == 0 && out.command.vy_m_s == 0);
        in.geometry.range.valid = true;
        in.geometry.north_m = 0;
        in.geometry.east_m = 4;
        out = rotated.update(in);
        assert(out.command.vx_m_s < 0 && std::abs(out.command.vy_m_s) < 1e-9);
        in.selection.observation_valid_until = at + 40ms;
        out = rotated.update(in);
        assert(out.mission == "GAP_HOLD" && out.command.vx_m_s == 0 && out.command.vy_m_s == 0);
    }
    {
        const Clock::time_point stamp{10s};
        FlightGuidanceController controller(settings());
        controller.explicit_start_committed(1);
        auto valid = input(stamp);
        valid.pose.position_at = valid.pose.attitude_at = stamp + 1ms;
        valid.pose.snapshot_at = stamp + 1ms;
        valid.selection.observation_valid_until = stamp + 250ms;
        auto result = controller.update(valid);
        assert(result.mission == "FOLLOW_ACTIVE" && result.valid_until <= stamp + 250ms);
        auto expiring = input(stamp + 210ms);
        expiring.selection.observation_valid_until = stamp + 250ms;
        result = controller.update(expiring);
        assert(result.mission == "GAP_HOLD" && !result.request_offboard_stop_once &&
               result.send_setpoint);
        assert(result.command.vx_m_s == 0 && result.command.vy_m_s == 0 &&
               result.command.yaw_rate_deg_s == 0);
        auto fresh = input(stamp + 230ms);
        fresh.selection.observation_valid_until = stamp + 480ms;
        assert(controller.update(fresh).mission == "FOLLOW_ACTIVE");
    }
    stop_result_contracts();
    stop_transition_contracts();
    const auto now = Clock::now();
    FlightGuidanceController idle_controller(settings());
    const auto idle = idle_controller.update(input(now));
    assert(idle.mission == "IDLE" && !idle.send_setpoint && !idle.request_offboard_stop_once);

    FlightGuidanceController controller(settings());
    controller.explicit_start_committed(1);
    auto result = controller.update(input(now));
    assert(result.mission == "FOLLOW_ACTIVE" && result.send_setpoint);
    assert(std::abs(result.command.vx_m_s - 0.3) < 1e-9);
    assert(std::abs(result.command.yaw_rate_deg_s - 2) < 1e-9);
    assert(result.command.vy_m_s == 0 && result.valid_until > now);

    auto no_range = input(now + 10ms);
    no_range.geometry.range = {NAN, "SCALE_ONLY", "range_not_metric", false};
    result = controller.update(no_range);
    assert(result.command.vx_m_s == 0 && result.command.yaw_rate_deg_s != 0);

    auto misaligned = input(now + 20ms);
    misaligned.geometry.bearing_error_deg = 20;
    result = controller.update(misaligned);
    assert(result.command.vx_m_s == 0);

    auto predicted = input(now + 30ms);
    predicted.selection.state = TargetState::Predicted;
    predicted.selection.fresh_real_detection = false;
    predicted.selection.selected_track->state = "PREDICTED";
    predicted.selection.selected_track->real_detection = false;
    result = controller.update(predicted);
    assert(result.mission == "PREDICTED" && result.command.vx_m_s == 0 &&
           result.command.vy_m_s == 0 && result.command.yaw_rate_deg_s == 0);

    auto lost = input(now + 40ms);
    lost.selection.state = TargetState::Lost;
    result = controller.update(lost);
    assert(result.request_offboard_stop_once && result.send_setpoint && result.command.vx_m_s == 0);
    result = controller.update(lost);
    assert(!result.request_offboard_stop_once);
    controller.record_offboard_stop_result(false, false);
    assert(controller.update(lost).mission == "FAULT");

    controller.explicit_start_committed(1);
    auto manual = input(now + 50ms);
    manual.rc_override = true;
    result = controller.update(manual);
    assert(result.mission == "MANUAL" && !result.send_setpoint &&
           !result.request_offboard_stop_once);
    manual.rc_override = false;
    controller.request_operator_stop();
    result = controller.update(manual);
    assert(result.mission == "MANUAL" && !result.send_setpoint &&
           !result.request_offboard_stop_once);
    return 0;
}
