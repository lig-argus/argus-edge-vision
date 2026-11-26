// 독립 tick에서 시작 준비·가이던스·만료/epoch 검사·PX4 단일 송신을 직렬화한다.
#include "argus/control/runtime/include/flight_control_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace follow {
bool FlightControlRuntime::handoff_ready(Clock::time_point now) const {
    const auto state = flight_state_source_.read();
    const auto pose = pose_source_.read();
    return inactive() && state.connected && pose.fresh(now) &&
           (!state.offboard || !state.rc_override) &&
           (state.hold || ((state.position || state.offboard) && now < position_handoff_until_));
}

bool FlightControlRuntime::prepare_handoff(Clock::time_point now) {
    const auto state = flight_state_source_.read();
    const auto pose = pose_source_.read();
    if (!inactive() || !state.connected || (state.offboard && state.rc_override) ||
        (!state.hold && !state.position && !state.offboard) || !pose.fresh(now)) {
        return false;
    }
    position_handoff_until_ = now + std::chrono::seconds(15);
    return true;
}

VehicleFlightStatus FlightControlRuntime::entry_state(VehicleFlightStatus state,
                                                      Clock::time_point now) const {
    // 명시적 Position 인계는 Offboard 진입 준비 동안에만 한시적으로 허용한다.
    // 추종 시작 후에는 실제 Position 복귀를 기존 수동 개입 경계로 감지한다.
    if ((phase_ == Phase::Idle || phase_ == Phase::Priming || phase_ == Phase::AwaitingOffboard) &&
        state.position && !state.offboard && now < position_handoff_until_) {
        state.hold = true;
        state.rc_override = false;
    }
    return state;
}

bool FlightControlRuntime::recover_after_explicit_stop(Clock::time_point now) {
    if (!config_.require_explicit_stop_recovery || !recovery_latched()) {
        return false;
    }
    const auto selection = selector_.snapshot(now);
    const auto flight = flight_state_source_.read();  // Do not apply Position handoff override.
    const auto pose = pose_source_.read();
    const auto checked = std::max(now, pose.snapshot_at);
    if (selection.phase != TargetSelectionSnapshot::Phase::Stopped || selection.target ||
        (observed_command_epoch_ && selection.command_epoch == *observed_command_epoch_) ||
        !flight.connected || !(flight.hold || (flight.offboard && flight.offboard_hover)) ||
        flight.rc_override || !pose.have_position || !pose.have_attitude ||
        pose.clock_domain != "host_monotonic" || checked < pose.position_at ||
        checked < pose.attitude_at || checked - pose.position_at > controller_.max_pose_age() ||
        checked - pose.attitude_at > controller_.max_pose_age()) {
        return false;
    }
    observed_command_epoch_ = selection.command_epoch;
    position_handoff_until_ = Clock::time_point{};
    stop_result_pending_ = false;
    set_phase(Phase::Idle);
    recovered_hold_ = true;
    return true;
}

bool FlightControlRuntime::reset_ground_context(Clock::time_point now) {
    const auto flight = flight_state_source_.read();
    // Offboard failsafe 뒤 착지·DISARM되면 PX4가 모드를 OFFBOARD로 되돌린다. DISARM·착지가
    // 확인되면 모드와 무관하게 송신 없이 초기화한다 (2026-11-06 실기).
    if (!flight.connected || flight.armed || !flight.ground_known || !flight.landed ||
        flight.rc_override) {
        return false;
    }
    selector_.execute(TargetCommand{"ground-context-reset", TargetCommandAction::Stop}, now);
    if (!selector_.reset_input_context()) {
        return false;
    }
    observed_command_epoch_ = selector_.snapshot(now).command_epoch;
    position_handoff_until_ = Clock::time_point{};
    stop_result_pending_ = false;
    recovered_hold_ = false;
    set_phase(Phase::Idle);
    return true;
}

void FlightControlRuntime::refresh_mode_guard() {
    if (!inactive()) {
        throw std::logic_error("mode change requires inactive runtime");
    }
    command_guard_.set_yaw_only(controller_.yaw_only());
    command_guard_.set_limits(controller_.command_limits());
    command_guard_.set_yaw_only_altitude_band(controller_.yaw_only_altitude_band(), &pose_source_);
}

FlightControlRuntime::FlightControlRuntime(
    TargetSelector& selector, const IVehicleStateSource& pose_source,
    const IVehicleFlightStatusSource& flight_state_source, const CalibratedGroundGeometry& geometry,
    FlightGuidanceController& controller, IVehiclePort& vehicle_port, IFlightModePort& mode_port,
    FlightControlRuntimeConfig config, FlightSafetyEventLatch* safety_events,
    std::recursive_mutex* context_mutex)
    : selector_(selector),
      pose_source_(pose_source),
      flight_state_source_(flight_state_source),
      geometry_(geometry),
      controller_(controller),
      command_guard_(selector, flight_state_source, vehicle_port, controller.yaw_only(),
                     context_mutex),
      vehicle_port_(vehicle_port),
      mode_port_(mode_port),
      config_(std::move(config)),
      safety_events_(safety_events) {
    command_guard_.set_limits(controller_.command_limits());
    command_guard_.set_yaw_only_altitude_band(controller_.yaw_only_altitude_band(), &pose_source_);
    const bool invalid_altitude =
        !std::isfinite(config_.ground_down_m) || !std::isfinite(config_.hold_down_m);
    const bool invalid_prestream = config_.prestream_setpoint_count == 0 ||
                                   config_.prestream_setpoint_count > 100 ||
                                   config_.prestream_interval.count() <= 0 ||
                                   config_.prestream_interval > std::chrono::seconds(1);
    const bool invalid_timeouts =
        config_.offboard_start_timeout.count() <= 0 || config_.hold_confirm_timeout.count() <= 0;
    if (invalid_altitude || invalid_prestream || invalid_timeouts) {
        throw std::invalid_argument("invalid flight control runtime configuration");
    }
}

void FlightControlRuntime::set_phase(Phase phase) {
    phase_ = phase;
    if (phase != Phase::Idle) {
        recovered_hold_ = false;
    }
    if (phase != Phase::Holding) {
        existing_hold_wait_ = false;
    }
    if (phase == Phase::Following || phase == Phase::Fault || phase == Phase::Manual) {
        position_handoff_until_ = Clock::time_point{};
    }
    if (phase == Phase::Idle || phase == Phase::Fault || phase == Phase::Manual) {
        vehicle_port_.revoke_commands();
    }
    if (phase == Phase::Fault || phase == Phase::Manual) {
        selector_.invalidate_recovery(phase == Phase::Manual ? "manual_override_latched"
                                                             : "flight_fault_latched");
    }
    following_active_.store(phase == Phase::Following, std::memory_order_release);
}

void FlightControlRuntime::set_phase(Phase phase, FlightControlRuntimeStatus& output,
                                     const char* mission, const std::string& reason) {
    set_phase(phase);
    output.mission = mission;
    output.reason = reason;
}

FlightGuidanceInput FlightControlRuntime::make_input(const TargetSelectionSnapshot& selection,
                                                     VehicleFlightStatus flight,
                                                     Clock::time_point now) const {
    FlightGuidanceInput input;
    input.now = now;
    input.selection = selection;
    input.selection_command_epoch = selection.command_epoch;
    input.pose = pose_source_.read();
    input.now = std::max({now, selection.snapshot_at, input.pose.snapshot_at});
    input.hold_down_m = config_.hold_down_m;
    input.connected = flight.connected;
    input.armed = flight.armed;
    input.in_air = flight.in_air;
    input.offboard = flight.offboard;
    input.rc_override = flight.rc_override;

    if (selection.latest_frame && selection.selected_track) {
        const auto& frame = *selection.latest_frame;
        std::optional<Pose> aligned;
        if (frame.exposure_time_valid && frame.exposure_monotonic_ns) {
            // 정렬 pose가 없으면 현재 자세를 시각 정렬된 값처럼 재사용하지 않는다.
            aligned = pose_source_.read_aligned(*frame.exposure_monotonic_ns, frame.clock_domain,
                                                geometry_.max_pose_bracket_distance());
        }
        // 노출 시각이 없으면 project가 미터 거리를 무효로 두고 보정 방향만 계산한다.
        input.geometry =
            geometry_.project(frame, *selection.selected_track, aligned ? *aligned : input.pose,
                              config_.ground_down_m, input.now, controller_.yaw_only());
        input.observation_pose = aligned;
        if (frame.ground_pose) {
            // producer가 동일한 지면 관측을 계산한 센서 정렬 FC pose다.
            // 원 host 수신/노출 시각을 발명하지 않고 좌표/자세만 투영에 사용한다.
            Pose observed;
            const auto& p = *frame.ground_pose;
            observed.north_m = p[0];
            observed.east_m = p[1];
            observed.down_m = p[2];
            observed.roll_deg = p[3];
            observed.pitch_deg = p[4];
            observed.yaw_deg = p[5];
            observed.have_position = observed.have_attitude = true;
            input.observation_pose = observed;
            // 2026-11-19 PAMPC #5: 같은 프레임의 producer 지면 기준 (선택 확장, 없으면 nullopt).
            input.producer_ground = producer_ground_of(frame);
        }
        // v2 지면 관측은 producer가 FC 센서 시각으로 이미 보정했다.
        // 수신 시각의 FC 보간을 별도로 읽되 노출 시각이나 TTL을 새로 만들지 않는다.
        if (!input.observation_pose && ground_tracks_schema(frame.schema) &&
            selection.selected_track->ground) {
            input.observation_pose =
                pose_source_.read_aligned(frame.sample_monotonic_ns, frame.clock_domain,
                                          geometry_.max_pose_bracket_distance());
        }
    }
    return input;
}

void FlightControlRuntime::begin_start(const TargetSelectionSnapshot& selection,
                                       const VehicleFlightStatus& flight, Clock::time_point now,
                                       FlightControlRuntimeStatus& output) {
    controller_.explicit_start_committed(selection.command_epoch);
    prestream_sent_ = 0;
    stop_result_pending_ = false;
    output.selection_command_epoch = selection.command_epoch;

    if (!selection.selected_track || !selection.latest_frame ||
        selection.state != TargetState::Confirmed || !selection.fresh_real_detection) {
        controller_.request_operator_stop();
        auto stop = controller_.update(make_input(selection, flight, now));
        if (stop.request_offboard_stop_once) {
            begin_stop(stop, selection, now, output);
        } else {
            set_phase(Phase::Fault, output, "FAULT", "start_requires_current_confirmed_target");
        }
        return;
    }
    if (!flight.connected || !flight.armed || !flight.in_air || flight.rc_override) {
        controller_.request_operator_stop();
        auto stop = controller_.update(make_input(selection, flight, now));
        if (stop.mission == "MANUAL") {
            set_phase(Phase::Manual, output, "MANUAL", stop.reason);
            return;
        }
        if (stop.request_offboard_stop_once) {
            begin_stop(stop, selection, now, output);
        } else {
            set_phase(Phase::Fault, output, "FAULT",
                      "start_requires_connected_armed_airborne_vehicle");
        }
        return;
    }
    if (flight.offboard) {
        set_phase(Phase::Following, output, "FOLLOW_ACTIVE", "explicit_start_in_existing_offboard");
        return;
    }
    if (!flight.hold) {
        set_phase(Phase::Fault, output, "FAULT", "start_requires_hold_or_offboard_mode");
        return;
    }
    const auto initial_pose = pose_source_.read();
    const auto checked_now = std::max(now, initial_pose.snapshot_at);
    if (!initial_pose.fresh(checked_now) || checked_now < initial_pose.position_at ||
        checked_now < initial_pose.attitude_at ||
        checked_now - initial_pose.position_at > controller_.max_pose_age() ||
        checked_now - initial_pose.attitude_at > controller_.max_pose_age()) {
        set_phase(Phase::Fault, output, "FAULT", "start_requires_fresh_pose_before_offboard");
        return;
    }

    // 무장/이륙은 수행하지 않는다. Start 요청과 지상 아닌 Hold 상태가 함께 있어야 setpoint 준비를
    // 시작한다.
    set_phase(Phase::Priming);
    next_prestream_at_ = now;
    transition_deadline_ = now + config_.offboard_start_timeout;
    output.mission = "OFFBOARD_PREPARING";
    output.reason = "explicit_start_waiting_for_setpoint_stream";
}

void FlightControlRuntime::begin_stop(const FlightGuidanceResult& result,
                                      const TargetSelectionSnapshot& selection,
                                      Clock::time_point now, FlightControlRuntimeStatus& output) {
    try {
        output.stop_trigger_reason = result.reason;
    } catch (...) {
    }
    output.selection_command_epoch = selection.command_epoch;
    if (result.mission == "MANUAL") {
        set_phase(Phase::Manual, output, "MANUAL", result.reason);
        return;
    }
    selector_.invalidate_recovery(result.reason);
    const auto stop_selection = selector_.snapshot(now);
    if (!result.request_offboard_stop_once) {
        if (flight_state_source_.read().hold) {
            // 이미 실제 Hold라면 중복 mode 요청 없이 재개 차단 상태로 끝낸다.
            controller_.record_offboard_stop_result(true, true);
            set_phase(Phase::Idle, output, "HOLD_CONFIRMED", "actual_hold_already_confirmed");
            return;
        }
        set_phase(Phase::Fault, output, "FAULT", result.reason);
        return;
    }

    bool zero_sent = true;
    if (result.send_setpoint) {
        const auto latest = selector_.snapshot(now);
        const auto flight = flight_state_source_.read();
        if (same_selection(stop_selection, latest) &&
            latest.command_epoch == result.selection_command_epoch && flight.connected &&
            flight.armed && flight.in_air && flight.offboard && !flight.rc_override) {
            // Stop 이후에는 이전 추종값 대신 현재 epoch의 영속도만 한 번 보낸다.
            zero_sent =
                vehicle_port_.send_until(FollowCommand{}, now + config_.prestream_interval * 3);
            output.command_sent = zero_sent;
        }
    }

    output.offboard_stop_requested = true;
    const bool stopped = mode_port_.stop_offboard();
    stop_result_ok_ = zero_sent && stopped;
    if (!stopped) {
        // FC 연결부가 관측 만료·송신 거부를 먼저 감지하면 자신이 Offboard 위치 유지를 이어받고
        // 이 추종기의 송신권을 폐기한다. 그 뒤의 정지 요청은 송신권 없음으로 거부되지만 기체는
        // 이미 멈춰 있다 (2026-11-04 실기). 실제 Hold·위치 유지가 보고될 때만 정상 정지로 본다.
        const auto taken_over = flight_state_source_.read();
        if (existing_hold_owner(taken_over)) {
            accept_existing_hold(taken_over, now, output);
            return;
        }
        if (taken_over.connected && taken_over.armed && taken_over.in_air && taken_over.offboard &&
            !taken_over.rc_override) {
            // 위치 유지 상태는 주기 발행이라 거부보다 늦게 도착할 수 있다. 송신 없이 기다린다.
            vehicle_port_.revoke_commands();
            transition_deadline_ = now + config_.hold_confirm_timeout;
            set_phase(Phase::Holding);
            existing_hold_wait_ = true;
            stop_result_pending_ = true;
            stop_result_ok_ = false;
            output.mission = "HOLD_WAIT";
            output.reason = "awaiting_existing_hold_owner";
            return;
        }
        controller_.record_offboard_stop_result(false, false);
        set_phase(Phase::Fault, output, "FAULT", "offboard_stop_request_failed_no_retry");
        return;
    }

    const auto state_after = flight_state_source_.read();
    if ((state_after.hold || (state_after.offboard && state_after.offboard_hover)) &&
        mode_port_.hold_result().value_or(false)) {
        controller_.record_offboard_stop_result(stop_result_ok_, stop_result_ok_);
        confirm_hold(state_after, now, output);
        return;
    }
    transition_deadline_ = now + config_.hold_confirm_timeout;
    set_phase(Phase::Holding);
    stop_result_pending_ = true;
    output.mission = "HOLD_WAIT";
    output.reason = result.reason;
}

void FlightControlRuntime::handle_command_epoch(const TargetSelectionSnapshot& selection,
                                                const VehicleFlightStatus& flight,
                                                Clock::time_point now,
                                                FlightControlRuntimeStatus& output) {
    if (!observed_command_epoch_ || selection.command_epoch != *observed_command_epoch_) {
        observed_command_epoch_ = selection.command_epoch;
        if (selection.phase == TargetSelectionSnapshot::Phase::Following &&
            config_.require_explicit_stop_recovery && recovery_latched()) {
            output.reason = "explicit_stop_and_actual_hold_required_before_restart";
            return;
        }
        if (selection.phase == TargetSelectionSnapshot::Phase::Following) {
            begin_start(selection, flight, now, output);
        } else if (phase_ == Phase::Following ||
                   (phase_ == Phase::AwaitingOffboard && flight.offboard)) {
            auto input = make_input(selection, flight, now);
            const auto stop = controller_.update(input);
            begin_stop(stop, selection, now, output);
        } else if (phase_ == Phase::AwaitingOffboard) {
            // Start 응답 대기 중 Stop이 오면 모드 확인을 기다리지 않고 취소 전환을 한 번 요청한다.
            output.offboard_stop_requested = true;
            if (!mode_port_.stop_offboard()) {
                set_phase(Phase::Fault, output, "FAULT", "cancel_offboard_start_failed_no_retry");
            } else if (flight_state_source_.read().hold) {
                set_phase(Phase::Idle, output, "HOLD_CONFIRMED",
                          "cancelled_offboard_start_in_hold");
            } else {
                set_phase(Phase::Holding);
                stop_result_ok_ = true;
                stop_result_pending_ = false;
                transition_deadline_ = now + config_.hold_confirm_timeout;
                output.mission = "HOLD_WAIT";
                output.reason = "awaiting_hold_after_start_cancellation";
            }
        } else if (phase_ == Phase::Priming) {
            set_phase(Phase::Idle, output, "STOPPED",
                      "operator_stop_cancelled_offboard_preparation");
        } else if (phase_ == Phase::Holding) {
            output.mission = "HOLD_WAIT";
            output.reason = "offboard_stop_already_requested";
        }
    }
}

void FlightControlRuntime::prepare_offboard(const TargetSelectionSnapshot& selection,
                                            const VehicleFlightStatus& flight,
                                            Clock::time_point now,
                                            FlightControlRuntimeStatus& output) {
    if (selection.phase != TargetSelectionSnapshot::Phase::Following ||
        selection.command_epoch != *observed_command_epoch_) {
        set_phase(Phase::Idle, output, "STOPPED", "selection_changed_before_offboard_start");
        return;
    }
    const bool invalid_target =
        selection.state != TargetState::Confirmed || !selection.fresh_real_detection;
    const auto current_pose = pose_source_.read();
    const auto checked_now = std::max(now, current_pose.snapshot_at);
    const bool invalid_pose = !current_pose.fresh(checked_now) ||
                              checked_now < current_pose.position_at ||
                              checked_now < current_pose.attitude_at ||
                              checked_now - current_pose.position_at > controller_.max_pose_age() ||
                              checked_now - current_pose.attitude_at > controller_.max_pose_age();
    const bool invalid_vehicle = !flight.connected || !flight.armed || !flight.in_air ||
                                 !flight.hold || flight.rc_override || invalid_pose;
    const bool bounded_recovery = config_.allow_recovery_during_priming &&
                                  selection.recovery_pause && selection.mission_target_id != 0 &&
                                  selection.target.has_value() && now < transition_deadline_;
    if ((invalid_target && !bounded_recovery) || invalid_vehicle ||
        (config_.allow_recovery_during_priming && now >= transition_deadline_)) {
        set_phase(Phase::Fault, output, "FAULT", "start_precondition_lost_before_offboard");
        return;
    }
    if (now < next_prestream_at_) {
        output.mission = "OFFBOARD_PREPARING";
        output.reason = "priming_setpoints";
        return;
    }
    if (!vehicle_port_.send_until(FollowCommand{}, now + config_.prestream_interval * 3)) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_prime_setpoint_failed_no_retry");
        return;
    }
    output.command_sent = true;
    ++prestream_sent_;
    next_prestream_at_ = now + config_.prestream_interval;
    if (prestream_sent_ < config_.prestream_setpoint_count) {
        output.mission = "OFFBOARD_PREPARING";
        output.reason = "priming_setpoints";
        return;
    }
    if (invalid_target) {
        output.mission = "OFFBOARD_PREPARING";
        output.reason = "zero_stream_waiting_for_confirmed_recovery";
        return;
    }
    if (!mode_port_.start_offboard()) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_start_request_failed_no_retry");
        return;
    }
    output.offboard_start_requested = true;
    set_phase(Phase::AwaitingOffboard);
    transition_deadline_ = now + config_.offboard_start_timeout;
    output.mission = "OFFBOARD_WAIT";
    output.reason = "awaiting_offboard_mode_confirmation";
    return;
}

void FlightControlRuntime::follow_target(const TargetSelectionSnapshot& selection,
                                         const VehicleFlightStatus& flight, Clock::time_point now,
                                         FlightControlRuntimeStatus& output) {
    auto input = make_input(selection, flight, now);
    const auto guidance_begin = Clock::now();
    auto result = controller_.update(input);
    output.command_generated_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(input.now.time_since_epoch()).count();
    output.guidance_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - guidance_begin).count();
    output.v21_motion = result.v21_motion;
    output.mission = result.mission;
    output.reason = result.reason;
    if (result.request_offboard_stop_once) {
        begin_stop(result, selection, now, output);
        return;
    }
    if (result.send_setpoint) {
        const auto guarded = command_guard_.send(result, selection, now);
        controller_.record_command_result(guarded == FlightCommandGuard::Result::Sent);
        if (guarded == FlightCommandGuard::Result::Sent) {
            output.command_sent = true;
            output.command = result.command;
            try {
                if (result.predictive_vertical && selection.latest_frame &&
                    selection.selected_track) {
                    const auto& frame = *selection.latest_frame;
                    PredictiveDecisionInput trace;
                    trace.vehicle_pose = input.pose;
                    trace.observation_pose = input.observation_pose;
                    trace.track = *selection.selected_track;
                    trace.generation = frame.generation;
                    trace.frame_id = frame.frame_id;
                    trace.command_epoch = selection.command_epoch;
                    trace.mission_target_id = selection.mission_target_id;
                    trace.frame_sample_ns = frame.sample_monotonic_ns;
                    trace.exposure_ns = frame.exposure_monotonic_ns;
                    trace.width = frame.image_width;
                    trace.height = frame.image_height;
                    trace.source = frame.source;
                    trace.timestamp_source = frame.timestamp_source;
                    trace.observation_pose_origin =
                        frame.ground_pose
                            ? "producer_ground_pose"
                            : (input.observation_pose ? "aligned_pose_history" : "unavailable");
                    trace.range_valid = input.geometry.range.valid;
                    trace.range_m = input.geometry.range.rho_m;
                    trace.range_quality =
                        input.geometry.range.quality ? input.geometry.range.quality : "";
                    trace.range_reason =
                        input.geometry.range.reason ? input.geometry.range.reason : "";
                    trace.bearing_valid = input.geometry.bearing_valid;
                    trace.bearing_error_deg = input.geometry.bearing_error_deg;
                    trace.ground_motion_valid = selection.ground_motion_valid;
                    trace.short_motion_valid = selection.short_motion.feedforward_valid;
                    trace.target_vn_m_s = selection.velocity_north_m_s;
                    trace.target_ve_m_s = selection.velocity_east_m_s;
                    trace.short_vn_m_s = selection.short_motion.velocity_north_m_s;
                    trace.short_ve_m_s = selection.short_motion.velocity_east_m_s;
                    trace.short_rmse_m = selection.short_motion.rmse_m;
                    trace.hold_down_m = input.hold_down_m;
                    output.predictive_input = std::move(trace);
                    output.predictive_vertical = result.predictive_vertical;
                }
            } catch (...) {
                output.predictive_diagnostic_capture_failed = true;
                // Best effort: diagnostics never change a command that was already sent.
            }
            output.pid_terms = result.pid_terms;
            output.altitude_target_down_m = result.altitude_target_down_m;
            output.command_sample_ns =
                selection.cv_gap_active
                    ? selection.cv_origin_sample_ns
                    : (selection.latest_frame ? selection.latest_frame->sample_monotonic_ns : 0);
            if (result.rear_goal && selection.selected_track && selection.selected_track->ground) {
                output.rear_goal = result.rear_goal;
                output.guidance_pose = input.pose;
                output.command_valid_until = result.valid_until;
                output.vehicle_reference_ned = std::array<double, 2>{
                    (*selection.selected_track->ground)[0], (*selection.selected_track->ground)[1]};
            }
            return;
        }
        // 명령 만료/epoch 변경은 폐기만 한다. 포트 실패는 한 번의 Hold 전이로 고정한다.
        const auto current = selector_.snapshot(now);
        if (guarded == FlightCommandGuard::Result::SelectionChanged ||
            !same_selection(selection, current)) {
            output.mission = "COMMAND_DISCARDED";
            output.reason = "selection_epoch_changed_before_vehicle_send";
            return;
        }
        controller_.request_operator_stop();
        const auto stop = controller_.update(input);
        output.reason = "vehicle_command_guard_or_send_failed";
        if (stop.request_offboard_stop_once) {
            begin_stop(stop, selection, now, output);
        } else {
            set_phase(Phase::Fault);
            output.mission = "FAULT";
        }
        return;
    }
    if (result.mission == "MANUAL") {
        set_phase(Phase::Manual);
        return;
    }
    if (result.mission == "FAULT" || result.mission == "HOLD_TRANSITION") {
        set_phase(Phase::Fault);
        return;
    }
}

bool FlightControlRuntime::confirm_offboard(const TargetSelectionSnapshot& selection,
                                            const VehicleFlightStatus& flight,
                                            Clock::time_point now,
                                            FlightControlRuntimeStatus& output) {
    if (selection.phase != TargetSelectionSnapshot::Phase::Following ||
        selection.command_epoch != *observed_command_epoch_) {
        set_phase(Phase::Fault, output, "FAULT", "selection_changed_after_offboard_start_request");
        return false;
    }
    const auto ack = mode_port_.start_result();
    if (ack && !*ack) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_start_ack_failed_no_retry");
        return false;
    }
    if (flight.offboard && ack.value_or(false)) {
        if (selection.state != TargetState::Confirmed || !selection.fresh_real_detection) {
            controller_.request_operator_stop();
            const auto stop = controller_.update(make_input(selection, flight, now));
            begin_stop(stop, selection, now, output);
            return false;
        }
        set_phase(Phase::Following);
        // 실제 모드를 확인한 tick은 그대로 이어서 추종 명령을 한 번 계산한다.
        return true;
    }
    if ((!flight.hold && !flight.offboard) || now >= transition_deadline_) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_mode_not_confirmed_no_retry");
        return false;
    }
    output.mission = "OFFBOARD_WAIT";
    output.reason = "awaiting_offboard_mode_confirmation";
    const auto pose = pose_source_.read();
    // 제어 tick 이후 도착한 표본은 실제 읽기 시각을 기준으로 검사한다.
    // 표본 자체의 시각이나 200ms 신선도 한계는 변경하지 않는다.
    const auto checked_now = std::max(now, pose.snapshot_at);
    if (!flight.connected || !flight.armed || !flight.in_air || flight.rc_override ||
        !pose.fresh(checked_now) || checked_now < pose.position_at ||
        checked_now < pose.attitude_at ||
        checked_now - pose.position_at > controller_.max_pose_age() ||
        checked_now - pose.attitude_at > controller_.max_pose_age()) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_entry_precondition_lost");
        return false;
    }
    if (!vehicle_port_.send_until(
            FollowCommand{},
            std::min(transition_deadline_, now + config_.prestream_interval * 3))) {
        set_phase(Phase::Fault, output, "FAULT", "offboard_entry_zero_send_failed");
        return false;
    }
    output.command_sent = true;
    return false;
}

bool FlightControlRuntime::existing_hold_owner(const VehicleFlightStatus& flight) {
    return flight.connected && flight.armed && flight.in_air && !flight.rc_override &&
           (flight.hold || (flight.offboard && flight.offboard_hover));
}

void FlightControlRuntime::accept_existing_hold(const VehicleFlightStatus& flight,
                                                Clock::time_point now,
                                                FlightControlRuntimeStatus& output) {
    stop_result_ok_ = true;
    stop_result_pending_ = false;
    controller_.record_offboard_stop_result(true, true);
    if (flight.offboard_hover) {
        // confirm_hold의 Offboard 위치 유지 성공과 같은 재개 조건을 남긴다.
        selector_.execute(TargetCommand{"local-hover-stop-" +
                                            std::to_string(selector_.snapshot(now).command_epoch),
                                        TargetCommandAction::Stop},
                          now);
        observed_command_epoch_ = selector_.snapshot(now).command_epoch;
    }
    set_phase(Phase::Idle, output, "HOLD_CONFIRMED",
              flight.offboard_hover ? "existing_offboard_hover_confirmed"
                                    : "existing_actual_hold_confirmed");
    recovered_hold_ = flight.offboard_hover;
}

void FlightControlRuntime::confirm_existing_hold(const VehicleFlightStatus& flight,
                                                 Clock::time_point now,
                                                 FlightControlRuntimeStatus& output) {
    if (existing_hold_owner(flight)) {
        accept_existing_hold(flight, now, output);
        return;
    }
    if (flight.rc_override) {
        stop_result_pending_ = false;
        controller_.record_offboard_stop_result(false, false);
        set_phase(Phase::Manual, output, "MANUAL", "hold_zero_bridge_precondition_lost");
        return;
    }
    if (now >= transition_deadline_ || !flight.connected || !flight.armed || !flight.in_air) {
        stop_result_pending_ = false;
        controller_.record_offboard_stop_result(false, false);
        set_phase(Phase::Fault, output, "FAULT", "offboard_stop_request_failed_no_retry");
        return;
    }
    output.mission = "HOLD_WAIT";
    output.reason = "awaiting_existing_hold_owner";
}

void FlightControlRuntime::confirm_hold(const VehicleFlightStatus& flight, Clock::time_point now,
                                        FlightControlRuntimeStatus& output) {
    if (existing_hold_wait_) {
        confirm_existing_hold(flight, now, output);
        return;
    }
    const auto ack = mode_port_.hold_result();
    if (ack && !*ack) {
        set_phase(Phase::Fault, output, "FAULT", "hold_ack_failed_no_retry");
        return;
    }
    if ((flight.hold || (flight.offboard && flight.offboard_hover)) && ack.value_or(false)) {
        if (stop_result_pending_) {
            controller_.record_offboard_stop_result(stop_result_ok_, stop_result_ok_);
            stop_result_pending_ = false;
        }
        if (stop_result_ok_ && flight.offboard_hover) {
            selector_.execute(
                TargetCommand{
                    "local-hover-stop-" + std::to_string(selector_.snapshot(now).command_epoch),
                    TargetCommandAction::Stop},
                now);
            observed_command_epoch_ = selector_.snapshot(now).command_epoch;
            recovered_hold_ = true;
        }
        set_phase(stop_result_ok_ ? Phase::Idle : Phase::Fault);
        output.mission = stop_result_ok_ ? "HOLD_CONFIRMED" : "FAULT";
        output.reason = stop_result_ok_
                            ? (flight.offboard_hover ? "offboard_hover_confirmed"
                                                     : "offboard_stopped_and_hold_confirmed")
                            : "zero_setpoint_failed";
    } else if (now >= transition_deadline_) {
        if (stop_result_pending_) {
            controller_.record_offboard_stop_result(false, false);
            stop_result_pending_ = false;
        }
        set_phase(Phase::Fault, output, "FAULT", "hold_not_confirmed_no_retry");
    } else {
        const auto pose = pose_source_.read();
        const auto checked_now = std::max(now, pose.snapshot_at);
        if (!stop_result_ok_) {
            vehicle_port_.revoke_commands();
        } else if (flight.connected && flight.armed && flight.in_air && !flight.rc_override &&
                   pose.fresh(checked_now) && checked_now >= pose.position_at &&
                   checked_now >= pose.attitude_at &&
                   checked_now - pose.position_at <= controller_.max_pose_age() &&
                   checked_now - pose.attitude_at <= controller_.max_pose_age()) {
            if (!flight.offboard || flight.offboard_hover) {
                // Actual Hold may precede its completion event. Send nothing,
                // but do not revoke the pending Hold request we are waiting for.
                // The FC owner retires its cached bridge when actual Hold arrives.
            } else if (!vehicle_port_.send_until(
                           FollowCommand{},
                           std::min(transition_deadline_, now + config_.prestream_interval * 3))) {
                set_phase(Phase::Fault, output, "FAULT", "hold_zero_bridge_send_failed");
                return;
            }
            output.command_sent = flight.offboard;
        } else {
            set_phase(flight.rc_override ? Phase::Manual : Phase::Fault, output,
                      flight.rc_override ? "MANUAL" : "FAULT",
                      "hold_zero_bridge_precondition_lost");
            return;
        }
        output.mission = "HOLD_WAIT";
        output.reason = "awaiting_hold_mode_confirmation";
    }
}

FlightControlRuntimeStatus FlightControlRuntime::step(Clock::time_point now) {
    if (now == Clock::time_point{}) {
        throw std::invalid_argument("control tick requires monotonic time");
    }
    FlightControlRuntimeStatus output;
    const auto manager_begin = Clock::now();
    selector_.advance(now);
    output.manager_advance_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - manager_begin).count();
    auto selection = selector_.snapshot(now);
    output.selection_command_epoch = selection.command_epoch;
    auto flight = entry_state(flight_state_source_.read(), now);

    // 1. 안전 사건과 새 선택/Start/Stop 명령을 이 제어 thread에서 처리한다.
    const auto safety_event =
        safety_events_ ? safety_events_->consume(selection.command_epoch) : FlightSafetyEvent::None;
    output.safety_event = safety_event;
    if (phase_ == Phase::Following) {
        // 감시 thread는 사건만 기록하고 mode/vehicle port는 이 제어 thread만 호출한다.
        switch (safety_event) {
            case FlightSafetyEvent::ManualOverride:
                controller_.request_manual_override();
                break;
            case FlightSafetyEvent::TargetUnavailable:
            case FlightSafetyEvent::OffboardLost:
            case FlightSafetyEvent::FlightStateInvalid:
                controller_.request_operator_stop();
                break;
            case FlightSafetyEvent::None:
                break;
        }
    }
    handle_command_epoch(selection, flight, now, output);

    // 2. 비활성·오류·수동 상태는 포트 송신 없이 끝낸다.
    if (phase_ == Phase::Idle) {
        if (recovered_hold_) {
            if (flight.connected && (flight.hold || (flight.offboard && flight.offboard_hover)) &&
                !flight.rc_override) {
                output.mission = "HOLD_CONFIRMED";
                output.reason = "explicit_stop_recovery_confirmed";
            } else {
                recovered_hold_ = false;
            }
        }
        if (output.mission == "IDLE" &&
            selection.phase == TargetSelectionSnapshot::Phase::Pending) {
            output.mission = "SELECTED";
            output.reason = "awaiting_explicit_start";
        }
        return output;
    }
    if (phase_ == Phase::Fault) {
        output.mission = "FAULT";
        if (output.reason == "awaiting_explicit_start") {
            output.reason = "explicit_retry_required";
        }
        return output;
    }
    if (phase_ == Phase::Manual) {
        output.mission = "MANUAL";
        output.reason = "manual_override_latched_operator_start_required";
        return output;
    }

    // 3. 최신 상태로 Offboard 준비와 실제 모드 응답을 확인한다.
    selection = selector_.snapshot(now);
    flight = entry_state(flight_state_source_.read(), now);
    if (phase_ == Phase::Priming) {
        prepare_offboard(selection, flight, now, output);
        return output;
    }

    if (phase_ == Phase::AwaitingOffboard && !confirm_offboard(selection, flight, now, output)) {
        return output;
    }

    // 4. 정지는 Hold 응답을 확인하고, 활성 추종은 guard를 거쳐 한 번 송신한다.
    if (phase_ == Phase::Holding) {
        confirm_hold(flight, now, output);
        return output;
    }

    if (phase_ == Phase::Following) {
        follow_target(selection, flight, now, output);
        return output;
    }
    return output;
}
}  // namespace follow
