// Follow/PREDICTED/Hold 출력을 한 제어 주기에서 계산하고 velocity 한도를 최종 적용한다.
#include "argus/control/guidance/include/flight_guidance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/guidance/include/cv_gap_visibility.hpp"
#include "argus/control/guidance/include/predictive_target_velocity.hpp"

namespace follow {

FlightGuidanceController::FlightGuidanceController(FlightGuidanceConfig config)
    : config_(config),
      forward_pid_(config.pid.forward, -config.max_forward_m_s, config.max_forward_m_s,
                   config.pid.anti_windup, config.pid.back_calculation),
      right_pid_(config.pid.right, -config.max_forward_m_s, config.max_forward_m_s,
                 config.pid.anti_windup, config.pid.back_calculation),
      yaw_pid_(config.pid.yaw, -config.max_yaw_rate_deg_s * std::acos(-1.) / 180,
               config.max_yaw_rate_deg_s * std::acos(-1.) / 180, config.pid.anti_windup,
               config.pid.back_calculation),
      altitude_pid_(config.pid.altitude, -config.max_vertical_m_s, config.max_vertical_m_s,
                    config.pid.anti_windup, config.pid.back_calculation),
      yaw_smoother_(config.pid.yaw_smoothing) {
    const bool v3_requested = config_.v3_pampc_point_profile != 0 ||
                              config_.v3_image_velocity_weight != 0 ||
                              config_.v3_predictive_integral || config_.v3_discard_recheck ||
                              config_.v32_sent_command_slew;
    if (v3_requested &&
        (!config_.predictive_follow || config_.outdoor_image_pid || config_.yaw_only)) {
        throw std::invalid_argument("V3 follow options require the predictive (non-PID) profile");
    }
    point_only_ = config_.v3_pampc_point_profile > 0;
    const bool invalid_range =
        !std::isfinite(config_.desired_range_m) || config_.desired_range_m <= 0 ||
        !std::isfinite(config_.range_deadband_m) || config_.range_deadband_m < 0;
    const bool invalid_yaw =
        !std::isfinite(config_.yaw_kp_deg_s_per_deg) || config_.yaw_kp_deg_s_per_deg <= 0 ||
        !std::isfinite(config_.max_yaw_rate_deg_s) || config_.max_yaw_rate_deg_s <= 0 ||
        !std::isfinite(config_.yaw_alignment_deg) || config_.yaw_alignment_deg <= 0;
    const bool invalid_forward =
        !std::isfinite(config_.forward_gain_m_s_per_m) || config_.forward_gain_m_s_per_m <= 0 ||
        !std::isfinite(config_.max_forward_m_s) || config_.max_forward_m_s <= 0;
    const bool invalid_backward =
        !std::isfinite(config_.max_backward_m_s) || config_.max_backward_m_s < 0 ||
        config_.max_backward_m_s > config_.max_forward_m_s ||
        (config_.max_backward_m_s > 0 && !config_.velocity_feedforward_follow &&
         !config_.outdoor_image_pid);
    const bool invalid_altitude =
        !std::isfinite(config_.altitude_gain) || config_.altitude_gain <= 0 ||
        !std::isfinite(config_.max_vertical_m_s) || config_.max_vertical_m_s <= 0;
    const bool invalid_fov = !std::isfinite(config_.edge_margin_fraction) ||
                             config_.edge_margin_fraction <= 0 ||
                             config_.edge_margin_fraction >= 0.5;
    const bool invalid_timing =
        config_.command_ttl.count() <= 0 || config_.max_pose_age.count() <= 0;
    const bool invalid_motion = !std::isfinite(config_.ground_center_offset_m) ||
                                config_.ground_center_offset_m < 0 ||
                                config_.ground_center_offset_m >= config_.desired_range_m ||
                                (config_.motion_ground_follow && !config_.ground_vector_follow);
    if (invalid_range || invalid_yaw || invalid_forward || invalid_backward || invalid_altitude ||
        invalid_fov || invalid_timing || invalid_motion) {
        throw std::invalid_argument("invalid flight guidance configuration");
    }
    if (config_.yaw_only) {
        if (config_.pid.enabled || config_.ground_vector_follow || config_.motion_ground_follow ||
            config_.observed_path_rear_follow || config_.body_standoff_follow ||
            config_.velocity_feedforward_follow || config_.camera_motion_guard ||
            config_.camera_descent_m != 0 || config_.reverse_range_margin_m != 0 ||
            config_.camera_min_height_m != 0 || config_.follow_side_offset_m != 0) {
            throw std::invalid_argument("yaw-only cannot be combined with translation guidance");
        }
        yaw_only_follower_.emplace(config_.yaw_kp_deg_s_per_deg, config_.max_yaw_rate_deg_s,
                                   config_.yaw_only_bearing_deadband_deg,
                                   config_.pid.yaw_smoothing);
    }
    if (config_.yaw_only_altitude_hold) {
        // 2026-11-18 데모 모드 고도 유지. 범위의 명목 높이 검사는 설정 읽기(hold_down_m)가 한다.
        if (!config_.yaw_only || !std::isfinite(config_.yaw_only_min_height_m) ||
            !std::isfinite(config_.yaw_only_max_height_m) || config_.yaw_only_min_height_m <= 0 ||
            config_.yaw_only_min_height_m >= config_.yaw_only_max_height_m ||
            !std::isfinite(config_.pid_ground_down_m)) {
            throw std::invalid_argument("invalid yaw-only altitude hold configuration");
        }
    }
    if (config_.observed_path_rear_follow) {
        if (!config_.motion_ground_follow) {
            throw std::invalid_argument("observed rear path requires motion ground guidance");
        }
        rear_goal_ = ObservedRearGoal(config_.desired_range_m - config_.ground_center_offset_m,
                                      config_.rear_path_sample_spacing_m);
    }
    if (config_.body_standoff_follow &&
        (!config_.motion_ground_follow || config_.observed_path_rear_follow)) {
        throw std::invalid_argument(
            "body standoff requires motion guidance and a single goal mode");
    }
    if (!std::isfinite(config_.reverse_range_margin_m) || config_.reverse_range_margin_m < 0 ||
        config_.reverse_range_margin_m > 1 ||
        (config_.reverse_range_margin_m > 0 && !config_.velocity_feedforward_follow)) {
        throw std::invalid_argument("invalid stop/reverse standoff margin");
    }
    if (!std::isfinite(config_.camera_descent_m) || config_.camera_descent_m < 0 ||
        config_.camera_descent_m > 1 ||
        (config_.camera_descent_m > 0 && !config_.velocity_feedforward_follow)) {
        throw std::invalid_argument("invalid camera descent margin");
    }
    if (!std::isfinite(config_.camera_min_height_m) || config_.camera_min_height_m < 0 ||
        config_.camera_min_height_m > 20 ||
        (config_.camera_min_height_m > 0 && !config_.velocity_feedforward_follow)) {
        throw std::invalid_argument("invalid camera minimum height");
    }
    if (!std::isfinite(config_.follow_side_offset_m) ||
        std::abs(config_.follow_side_offset_m) >=
            config_.desired_range_m - config_.ground_center_offset_m ||
        (config_.follow_side_offset_m != 0 &&
         (!config_.velocity_feedforward_follow || !config_.body_standoff_follow))) {
        throw std::invalid_argument("invalid side follow offset");
    }
    if (config_.reset_motion_on_stop_or_reverse && !config_.velocity_feedforward_follow) {
        throw std::invalid_argument("motion reset requires velocity feedforward guidance");
    }
    if (config_.velocity_feedforward_follow &&
        (!config_.pid.enabled || !config_.body_standoff_follow || !config_.camera_motion_guard)) {
        throw std::invalid_argument("velocity feedforward requires PID, standoff and camera guard");
    }
    if (config_.pid.enabled) {
        if ((!config_.outdoor_image_pid && !config_.observed_path_rear_follow &&
             !config_.body_standoff_follow) ||
            !std::isfinite(config_.pid_ground_down_m)) {
            throw std::invalid_argument("PID requires an observed rear goal and ground datum");
        }
        pid_projection_.emplace(config_.pid_camera);
    }
    if (!std::isfinite(config_.max_backward_accel_m_s2) || config_.max_backward_accel_m_s2 < 0 ||
        config_.max_backward_accel_m_s2 > 5 ||
        (config_.max_backward_accel_m_s2 > 0 && !config_.camera_motion_guard &&
         !config_.outdoor_image_pid)) {
        throw std::invalid_argument("invalid backward camera response limit");
    }
    if (config_.camera_motion_guard) {
        if (config_.max_horizontal_accel_m_s2 <= 0) {
            throw std::invalid_argument("camera constraint requires an acceleration limit");
        }
        camera_motion_guard_.emplace(config_.pid_camera, config_.edge_margin_fraction,
                                     config_.fov_velocity_p_acc, config_.ground_contact_anchor,
                                     config_.max_forward_m_s, config_.max_backward_accel_m_s2);
    }
    if (config_.outdoor_image_pid) {
        if (!config_.outdoor_profile || config_.predictive_follow || config_.yaw_only) {
            throw std::invalid_argument("image PID is an exclusive outdoor moving profile");
        }
        ImageCenterConfig c;
        c.width = config_.pid_camera.image_width;
        c.height = config_.pid_camera.image_height;
        c.fx = config_.pid_camera.fx;
        c.deadzone = config_.image_center_deadzone_fraction;
        c.forward = config_.pid.forward;
        c.yaw = config_.pid.yaw;
        c.altitude = config_.pid.altitude;
        c.max_forward = config_.max_forward_m_s;
        c.max_backward = config_.max_backward_m_s;
        c.max_vertical = config_.max_vertical_m_s;
        c.max_yaw = config_.max_yaw_rate_deg_s;
        c.acceleration = config_.max_horizontal_accel_m_s2;
        c.braking = std::max(c.acceleration, config_.max_backward_accel_m_s2);
        c.yaw_acceleration = config_.max_yaw_accel_deg_s2;
        c.ground_down = config_.pid_ground_down_m;
        c.target_height = config_.predictive_target_height_m;
        c.min_height = config_.predictive_min_height_m;
        c.max_height = config_.predictive_max_height_m;
        image_center_follower_.emplace(c);
    }
    if (config_.predictive_follow) {
        if (!config_.velocity_feedforward_follow || !config_.reset_motion_on_stop_or_reverse ||
            config_.yaw_only) {
            throw std::invalid_argument(
                "predictive following requires velocity guidance and motion reset");
        }
        PredictiveFollowConfig c;
        c.camera = config_.pid_camera;
        c.range_m = config_.desired_range_m;
        c.ground_down_m = config_.pid_ground_down_m;
        c.velocity_response = config_.fov_velocity_p_acc;
        c.max_speed_m_s = config_.max_forward_m_s;
        c.max_backward_m_s = config_.max_backward_m_s;
        c.max_vertical_m_s = config_.max_vertical_m_s;
        c.max_acceleration_m_s2 = config_.max_horizontal_accel_m_s2;
        c.braking_acceleration_m_s2 =
            std::max(config_.max_horizontal_accel_m_s2, config_.max_backward_accel_m_s2);
        c.max_yaw_rate_deg_s = config_.max_yaw_rate_deg_s;
        c.max_yaw_accel_deg_s2 = config_.max_yaw_accel_deg_s2;
        c.min_height_m = config_.predictive_min_height_m;
        c.max_height_m = config_.predictive_max_height_m;
        c.outdoor_profile = config_.outdoor_profile;
        c.target_height_m = config_.predictive_target_height_m;
        c.position_gain = config_.pid.forward.p;
        c.velocity_gain = config_.pid.forward.d;
        // V3: 예측 추종 I항(전진 PID의 I값)과 anti-windup. 설정 파일에서만 켠다.
        if (config_.v3_predictive_integral) {
            c.integral_gain = config_.pid.forward.i;
            c.integral_anti_windup = config_.pid.anti_windup;
        }
        c.sent_command_slew = config_.v32_sent_command_slew;
        c.pampc_point_profile = config_.v3_pampc_point_profile;
        c.image_velocity_weight = config_.v3_image_velocity_weight;
        c.side_offset_m = config_.follow_side_offset_m;
        c.height_anchor_m = config_.predictive_height_anchor_m;
        c.height_anchor_weight = config_.predictive_height_anchor_weight;
        c.height_anchor_deadband_m = config_.predictive_height_anchor_deadband_m;
        c.vertical_deadband_m_s = config_.predictive_vertical_deadband_m_s;
        c.vertical_deadband_release_m = config_.predictive_vertical_deadband_release_m;
        c.vertical_release_max_m_s = config_.predictive_vertical_release_max_m_s;
        c.gap_velocity_feedforward = config_.v35_gap_cv_feedforward;
        c.sent_command_slew_reconcile = config_.v35_slew_reconcile;
        c.point_range_deadband_m = config_.v35_point_range_deadband_m;
        c.point_range_near_deadband_m = config_.v35_point_range_near_deadband_m;
        c.point_bottom_margin = config_.v35_point_bottom_margin;
        c.point_tangential_weight = config_.v35_point_tangential_weight;
        c.plant_horizontal_response = config_.pampc_plant_horizontal_response;
        c.plant_vertical_response = config_.pampc_plant_vertical_response;
        c.use_producer_ground = config_.pampc_use_producer_ground;
        c.producer_ground_max_deviation_m = config_.pampc_producer_ground_max_deviation_m;
        c.producer_ground_hold_s = config_.pampc_producer_ground_hold_s;
        c.ground_datum_rate_m_s = config_.pampc_ground_datum_rate_m_s;
        c.ceiling_margin_m = config_.pampc_ceiling_margin_m;
        c.fixed_yaw_follow = config_.pampc_fixed_yaw_follow;
        c.fixed_yaw_realign_deg = config_.pampc_fixed_yaw_realign_deg;
        c.fixed_yaw_realign_rate_deg_s = config_.pampc_fixed_yaw_realign_rate_deg_s;
        if (!std::isfinite(config_.v35_bearing_realign_s + config_.v35_gap_yaw_decay_s) ||
            config_.v35_bearing_realign_s < 0 || config_.v35_bearing_realign_s > 5 ||
            config_.v35_gap_yaw_decay_s < 0 || config_.v35_gap_yaw_decay_s > 5) {
            throw std::invalid_argument("bearing realign and gap yaw decay must be 0..5 s");
        }
        predictive_follower_.emplace(c);
    }
}

void FlightGuidanceController::record_command_result(bool sent) {
    if (predictive_follower_) {
        predictive_follower_->complete_update(sent);
    }
}
void FlightGuidanceController::reset_pid() {
    if (image_center_follower_) {
        image_center_follower_->reset();
    }
    forward_pid_.reset();
    right_pid_.reset();
    yaw_pid_.reset();
    altitude_pid_.reset();
    yaw_smoother_.reset();
    pid_previous_at_ = {};
    if (yaw_only_follower_) {
        yaw_only_follower_->reset();
    }
    filtered_visual_error_.reset();
    filtered_descent_m_ = 0;
    side_los_sample_ns_ = 0;
    side_los_angle_rad_ = side_los_rate_deg_s_ = 0;
    reset_motion_previous_.reset();
    if (predictive_follower_) {
        predictive_follower_->reset_motion();
    }
}

void FlightGuidanceController::reset_pid_timing() {
    forward_pid_.reset();
    right_pid_.reset();
    yaw_pid_.reset();
    altitude_pid_.reset();
    yaw_smoother_.reset();
    pid_previous_at_ = {};
}

// 기존 공백 정지(아래 pause)와 같은 명령: 수평 0, 점 추종·하강·yaw 전용은 수직도 0. 같은 표적이
// 돌아오지 않으면 short_gap_hover 뒤 update가 HOLD로 넘긴다.
FlightGuidanceResult FlightGuidanceController::short_gap_pause(const FlightGuidanceInput& input) {
    reset_pid_timing();
    previous_command_at_ = input.now;
    previous_command_ = {};
    FlightGuidanceResult pause;
    pause.selection_command_epoch = input.selection_command_epoch;
    pause.valid_until = input.now + config_.command_ttl;
    if (input.selection.cv_gap_deadline) {
        pause.valid_until = std::min(pause.valid_until, *input.selection.cv_gap_deadline);
    }
    pause.send_setpoint = true;
    pause.mission = target_phase_name(input.selection.target_phase);
    pause.reason = "short_gap_hover";
    pause.command.mode = pause.mission;
    pause.command.vz_m_s =
        std::clamp(config_.altitude_gain * (input.hold_down_m - input.pose.down_m),
                   -config_.max_vertical_m_s, config_.max_vertical_m_s);
    if (point_only_ || config_.camera_descent_m > 0 || config_.yaw_only) {
        pause.command.vz_m_s = 0;
    }
    return pause;
}

void FlightGuidanceController::explicit_start_committed(std::uint64_t selection_command_epoch) {
    stop_state_ = StopState::NotAttempted;
    force_stop_ = false;
    manual_override_latched_ = false;
    started_ = true;
    start_command_epoch_ = selection_command_epoch;
    previous_command_at_ = {};
    previous_command_ = {};
    rear_goal_.reset();
    rear_sample_ns_ = 0;
    rear_motion_phase_ = ObservedMotionPhase::Unknown;
    side_follow_heading_.reset();
    reset_pid();
    short_gap_since_ = {};
    bearing_realign_since_ = {};
    last_follow_yaw_deg_s_ = 0;
    filtered_standoff_m_ = config_.desired_range_m - config_.ground_center_offset_m;
    if (predictive_follower_) {
        predictive_follower_->reset();
    }
    motion_clock_samples_.clear();
    motion_clock_rate_ = 1;
}
void FlightGuidanceController::request_operator_stop() {
    force_stop_ = true;
    rear_goal_.reset();
    rear_sample_ns_ = 0;
    reset_pid();
}
void FlightGuidanceController::request_manual_override() {
    manual_override_latched_ = true;
    rear_goal_.reset();
    rear_sample_ns_ = 0;
    reset_pid();
}
void FlightGuidanceController::record_offboard_stop_result(bool success,
                                                           bool actual_hold_confirmed) {
    if (stop_state_ != StopState::ResultPending) {
        return;
    }
    stop_state_ = success && actual_hold_confirmed ? StopState::Succeeded : StopState::Failed;
}

FlightGuidanceResult FlightGuidanceController::hold_once(const FlightGuidanceInput& input,
                                                         const char* reason) {
    FlightGuidanceResult result;
    rear_goal_.reset();
    rear_sample_ns_ = 0;
    reset_pid();
    short_gap_since_ = {};
    bearing_realign_since_ = {};
    result.mission = "HOLD_TRANSITION";
    if (stop_state_ != StopState::NotAttempted) {
        result.mission = stop_state_ == StopState::Failed ? "FAULT" : "HOLD_WAIT";
    }
    result.reason = reason;
    result.command.mode = result.mission;
    result.selection_command_epoch = input.selection_command_epoch;
    if (stop_state_ == StopState::NotAttempted) {
        stop_state_ = StopState::ResultPending;
        result.send_setpoint = result.request_offboard_stop_once =
            input.offboard && !input.rc_override;
        if (result.send_setpoint) {
            result.valid_until = input.now + config_.command_ttl;
        }
    }
    return result;
}

FlightGuidanceResult FlightGuidanceController::update(const FlightGuidanceInput& input) {
    if (input.now == Clock::time_point{}) {
        throw std::invalid_argument("control tick requires local monotonic time");
    }
    // 1. 명시 Start와 수동 개입/정지 상태를 먼저 처리한다.
    if (!started_) {
        FlightGuidanceResult result;
        result.mission =
            input.selection.phase == TargetSelectionSnapshot::Phase::Pending ? "SELECTED" : "IDLE";
        result.reason = "awaiting_explicit_start";
        result.selection_command_epoch = input.selection_command_epoch;
        return result;
    }
    // RC 개입은 다른 정지 원인보다 우선하며 수동 모드를 존중해 Offboard 명령을 만들지 않는다.
    if (manual_override_latched_ || input.rc_override) {
        reset_pid();
        manual_override_latched_ = true;
        FlightGuidanceResult result;
        result.mission = "MANUAL";
        result.reason = "rc_override_latched";
        result.selection_command_epoch = input.selection_command_epoch;
        return result;
    }
    if (input.selection.phase != TargetSelectionSnapshot::Phase::Following ||
        input.selection.command_epoch != start_command_epoch_ ||
        input.selection_command_epoch != start_command_epoch_) {
        return hold_once(input, "operator_stop_or_selection_epoch_changed");
    }
    if (force_stop_) {
        return hold_once(input, "operator_stop");
    }
    if (stop_state_ != StopState::NotAttempted) {
        return hold_once(input, stop_state_ == StopState::Failed
                                    ? "offboard_stop_failed_no_retry"
                                    : "resume_requires_operator_start");
    }
    // 2. 기체/pose와 선택한 표적이 현재 명령에 유효한지 확인한다.
    const bool invalid_flight_mode =
        !input.connected || !input.armed || !input.in_air || !input.offboard;
    const auto checked_now = std::max(input.now, input.pose.snapshot_at);
    const bool invalid_pose_time = !input.pose.fresh(checked_now) ||
                                   checked_now < input.pose.position_at ||
                                   checked_now < input.pose.attitude_at ||
                                   checked_now - input.pose.position_at > config_.max_pose_age ||
                                   checked_now - input.pose.attitude_at > config_.max_pose_age;
    if (invalid_flight_mode || invalid_pose_time) {
        return hold_once(input, "fc_or_mode_precondition_failed");
    }
    if (input.selection.recovery_pause && input.selection.target) {
        // 2026-11-17: 방위 재정렬(v35_bearing_realign_s, 실외 4종)의 시간은 실관측이 이어질 때만
        // 잰다. 재정렬 중 공백(NUC 약 0.5초 등)이 오면 공백 시간이 1.5초 예산을 깎아 복귀 직후
        // bearing_realign_timeout으로 끝날 수 있었다. 공백에 들어오면 처음부터 다시 잰다.
        if (bearing_realign_since_ != Clock::time_point{}) {
            std::cerr << "BEARING_REALIGN reset_by_gap" << std::endl;
            bearing_realign_since_ = {};
        }
        const auto deadline = input.selection.cv_gap_deadline;
        // Use the existing HOLD request reserve while the FC owner is still available.
        if (input.selection.cv_ten_frame_policy && deadline &&
            input.now + std::chrono::milliseconds{100} >= *deadline) {
            return hold_once(input, "v34_detection_gap_expired");
        }
        // 2026-11-09 사용자 결정(11-03 공백 CV 3초): 60프레임 전 기록이 없어도 마지막 저장
        // 속도로 CV 이동한다. 기록이 있으면 selection 속도가 그 기록 값이다. 0이면 움직이지 않는다.
        if (input.selection.cv_ten_frame_policy && deadline && input.selection.cv_gap_active &&
            predictive_follower_ && input.selection.cv_visibility_reference && !config_.yaw_only) {
            const auto& reference = *input.selection.cv_visibility_reference;
            const auto origin = Clock::time_point{std::chrono::nanoseconds{reference.sample_ns}};
            const double age = std::chrono::duration<double>(input.now - origin).count();
            const std::array<double, 2> shift{
                age * motion_clock_rate_ * input.selection.cv_velocity_north_m_s,
                age * motion_clock_rate_ * input.selection.cv_velocity_east_m_s};
            Pose observation;
            const auto& p = reference.observation_pose;
            observation.north_m = p[0];
            observation.east_m = p[1];
            observation.down_m = p[2];
            observation.roll_deg = p[3];
            observation.pitch_deg = p[4];
            observation.yaw_deg = p[5];
            observation.have_position = observation.have_attitude = true;
            // 2026-11-10: 켜면 CV 속도도 넘겨 예측 구간에서 표적이 계속 움직인다고 본다.
            const std::array<double, 2> gap_velocity =
                config_.v35_gap_cv_feedforward
                    ? std::array<double, 2>{input.selection.cv_velocity_north_m_s,
                                            input.selection.cv_velocity_east_m_s}
                    : std::array<double, 2>{0., 0.};
            if (config_.v35_gap_yaw_decay_s > 0) {
                // 2026-11-10 공백 중 yaw를 키우지 않고 마지막 실관측 추종 yaw에서 줄인다.
                constexpr double kGapYawFloorDegS = 10;
                predictive_follower_->set_yaw_rate_cap(
                    std::max(kGapYawFloorDegS,
                             std::abs(last_follow_yaw_deg_s_) *
                                 std::exp(-std::max(0., age) / config_.v35_gap_yaw_decay_s)));
            }
            const auto prediction = predictive_follower_->update(
                input.pose, observation, reference.box, gap_velocity, age, input.hold_down_m,
                .05 * motion_clock_rate_, motion_clock_rate_, false, std::nullopt, shift,
                reference.producer_ground);
            predictive_follower_->set_yaw_rate_cap(std::nullopt);
            if (!prediction) {
                const bool first = short_gap_since_ == Clock::time_point{};
                if (first || !config_.v35_gap_invalid_hover) {
                    std::cerr << "PREDICTIVE_REJECT path=gap_cv check="
                              << predictive_follower_->last_reject_reason()
                              << (config_.v35_gap_invalid_hover ? " action=short_gap_hover"
                                                                : " action=hold")
                              << std::endl;
                }
                if (config_.v35_gap_invalid_hover) {
                    if (first) {
                        short_gap_since_ = input.now;
                    }
                    if (input.now - short_gap_since_ < config_.short_gap_hover) {
                        return short_gap_pause(input);
                    }
                }
                return hold_once(input, "v34_gap_prediction_invalid");
            }
            // 공백 중에는 PID를 쓰지 않는다. 복귀 첫 틱이 공백 전 PID 시각으로 간격 오류를 내지
            // 않게 PID만 비운다. 예측 추종 상태는 유지한다 (2026-11-09 재현:
            // pid_control_interval_invalid).
            reset_pid_timing();
            FlightGuidanceResult result;
            result.selection_command_epoch = input.selection_command_epoch;
            result.valid_until = std::min(input.now + config_.command_ttl, *deadline);
            result.send_setpoint = true;
            result.mission = "GAP_CV";
            result.reason = "v34_cv60_pampc";
            result.command = prediction->command;
            result.command.mode = result.mission;
            previous_command_at_ = input.now;
            previous_command_ = result.command;
            return result;
        }
        if (input.selection.cv_ten_frame_policy) {
            // 2026-11-09 실기: NUC(약 0.5초)처럼 짧은 공백에서 CV 근거가 없다고 바로 HOLD로 넘겨
            // 추종이 풀렸다. 기준 관측이 없으면 short_gap_hover(기본 3초, 실제 끝은 공백 기한)
            // 동안 제자리를 지키고 같은 표적 복귀를 기다린다.
            if (short_gap_since_ == Clock::time_point{}) {
                short_gap_since_ = input.now;
            }
            if (input.now - short_gap_since_ < config_.short_gap_hover) {
                return short_gap_pause(input);
            }
            return hold_once(input, "v34_cv60_history_or_identity_unavailable");
        }
        reset_pid();
        const bool point_cv_history = point_only_ && input.selection.cv_ten_frame_policy;
        const bool allow_cv_control = !point_only_ || point_cv_history;
        // Allow the existing mode request to reach the owner before its hard 3s deadline.
        if (allow_cv_control && input.selection.cv_gap_deadline &&
            input.now + std::chrono::milliseconds{100} >= *input.selection.cv_gap_deadline) {
            return hold_once(input, "cv_gap_deadline_expired");
        }
        if (allow_cv_control && input.selection.cv_gap_active &&
            (!point_cv_history ||
             (input.selection.cv_history_frame_id && input.selection.cv_history_source_ns > 0)) &&
            input.selection.cv_gap_deadline && input.now < *input.selection.cv_gap_deadline &&
            !config_.yaw_only && !config_.outdoor_image_pid && std::isfinite(input.pose.yaw_deg)) {
            const double yaw = input.pose.yaw_deg * std::acos(-1.) / 180.;
            const double vn = input.selection.cv_velocity_north_m_s;
            const double ve = input.selection.cv_velocity_east_m_s;
            double forward = std::cos(yaw) * vn + std::sin(yaw) * ve;
            double right = -std::sin(yaw) * vn + std::cos(yaw) * ve;
            const double speed = std::hypot(forward, right);
            if (speed > config_.max_forward_m_s && speed > 0.) {
                forward *= config_.max_forward_m_s / speed;
                right *= config_.max_forward_m_s / speed;
            }
            if (point_cv_history) {
                forward = std::max(forward, -config_.max_backward_m_s);
            }
            if (previous_command_at_ != Clock::time_point{} &&
                config_.max_horizontal_accel_m_s2 > 0.) {
                const double dt = std::clamp(
                    std::chrono::duration<double>(input.now - previous_command_at_).count(), 0.,
                    .2);
                const double dv = config_.max_horizontal_accel_m_s2 * dt;
                const double dx = forward - previous_command_.vx_m_s;
                const double dy = right - previous_command_.vy_m_s;
                const double change = std::hypot(dx, dy);
                if (change > dv && change > 0.) {
                    forward = previous_command_.vx_m_s + dx * dv / change;
                    right = previous_command_.vy_m_s + dy * dv / change;
                }
            }
            FlightGuidanceResult predicted;
            predicted.selection_command_epoch = input.selection_command_epoch;
            predicted.valid_until =
                std::min(input.now + config_.command_ttl, *input.selection.cv_gap_deadline);
            predicted.send_setpoint = true;
            predicted.mission = "GAP_CV";
            predicted.reason = "constant_velocity_prediction";
            predicted.command.mode = predicted.mission;
            double chosen_scale = 1;
            bool chosen_lateral = false;
            double chosen_lateral_delta = 0;
            CvVisibilityDiagnostics visibility_trace;
            bool visibility_evaluated = false;
            bool visibility_missing = false;
            if (v21_enabled("ARGUS_V21_CV_VISIBILITY")) {
                std::optional<double> max_change;
                if (previous_command_at_ != Clock::time_point{} &&
                    config_.max_horizontal_accel_m_s2 > 0.) {
                    const double dt = std::clamp(
                        std::chrono::duration<double>(input.now - previous_command_at_).count(), 0.,
                        .2);
                    max_change = config_.max_horizontal_accel_m_s2 * dt;
                }
                std::optional<CvVisibilityChoice> choice;
                if (input.selection.cv_visibility_reference) {
                    choice = choose_cv_visibility(
                        config_.pid_camera, config_.pid_ground_down_m,
                        *input.selection.cv_visibility_reference, *input.selection.target,
                        input.pose, {vn, ve}, input.selection.cv_origin_sample_ns, input.now,
                        *input.selection.cv_gap_deadline, forward, right, previous_command_.vx_m_s,
                        previous_command_.vy_m_s, max_change, &visibility_trace,
                        config_.max_forward_m_s, config_.desired_range_m,
                        config_.max_horizontal_accel_m_s2,
                        std::max(config_.max_horizontal_accel_m_s2,
                                 config_.max_backward_accel_m_s2),
                        config_.fov_velocity_p_acc);
                    visibility_evaluated = true;
                }
                if (choice) {
                    forward = choice->forward_m_s;
                    right = choice->right_m_s;
                    chosen_scale = choice->scale;
                    chosen_lateral = choice->lateral;
                    chosen_lateral_delta = choice->lateral_delta_m_s;
                    predicted.reason = choice->lateral      ? "cv_visibility_lateral"
                                       : choice->scale < 1. ? "cv_visibility_scaled"
                                                            : "cv_visibility_evaluated";
                } else {
                    predicted.reason = "cv_visibility_unavailable";
                    visibility_missing = true;
                }
            }
            predicted.v21_motion =
                V21MotionDiagnostics{true,
                                     input.selection.box_motion_suspect,
                                     input.selection.cv_origin_sample_ns,
                                     std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         input.selection.cv_gap_deadline->time_since_epoch())
                                         .count(),
                                     vn,
                                     ve,
                                     chosen_scale};
            predicted.v21_motion->velocity_unconfirmed = input.selection.cv_velocity_unconfirmed;
            predicted.v21_motion->velocity_fit_sample_ns =
                input.selection.cv_velocity_fit_sample_ns;
            predicted.v21_motion->lateral = chosen_lateral;
            predicted.v21_motion->lateral_delta_m_s = chosen_lateral_delta;
            if (visibility_evaluated) {
                predicted.v21_motion->visibility_diagnostics = visibility_trace;
            }
            predicted.v21_motion->history_frame_id =
                input.selection.cv_history_frame_id.value_or(0);
            predicted.v21_motion->history_source_ns = input.selection.cv_history_source_ns;
            if (point_cv_history && visibility_missing) {
                // A missing projection is not permission for blind CV movement.
                forward = right = 0.;
                predicted.mission = predicted.command.mode = "GAP_HOLD";
                predicted.v21_motion->cv_gap_active = false;
            }
            if (point_cv_history) {
                forward = std::max(forward, -config_.max_backward_m_s);
            }
            predicted.command.vx_m_s = forward;
            predicted.command.vy_m_s = right;
            previous_command_at_ = input.now;
            previous_command_ = predicted.command;
            return predicted;
        }
        previous_command_at_ = input.now;
        previous_command_ = {};
        FlightGuidanceResult pause;
        pause.selection_command_epoch = input.selection_command_epoch;
        pause.valid_until = input.now + config_.command_ttl;
        pause.send_setpoint = true;
        pause.mission = target_phase_name(input.selection.target_phase);
        const auto& gap_reason = input.selection.recovery_reason;
        const bool missing_only = gap_reason == "target_input_gap" ||
                                  gap_reason == "selected_target_missing" ||
                                  gap_reason == "no_plausible_target_candidate";
        pause.reason = point_cv_history && missing_only && !input.selection.cv_history_frame_id
                           ? "cv_ten_frame_velocity_unavailable"
                           : gap_reason;
        pause.command.mode = pause.mission;
        pause.command.vz_m_s =
            std::clamp(config_.altitude_gain * (input.hold_down_m - input.pose.down_m),
                       -config_.max_vertical_m_s, config_.max_vertical_m_s);
        if (point_only_ || config_.camera_descent_m > 0 || config_.yaw_only) {
            pause.command.vz_m_s = 0;
        }
        return pause;
    }
    short_gap_since_ = {};  // 공백이 끝났다. 다음 공백은 처음부터 다시 잰다.
    const bool missing_target =
        !input.selection.target || !input.selection.selected_track || !input.selection.latest_frame;
    const bool mismatched_target =
        !missing_target &&
        (input.selection.latest_frame->generation != input.selection.target->generation ||
         input.selection.selected_track->track_id != input.selection.target->track_id ||
         input.selection.state == TargetState::Lost);
    if (missing_target || mismatched_target) {
        return hold_once(input, "selected_target_lost_or_stale");
    }

    FlightGuidanceResult result;
    result.selection_command_epoch = input.selection_command_epoch;
    result.valid_until = input.now + config_.command_ttl;
    result.send_setpoint = true;
    std::optional<std::array<PidTerms, 2>> stationary_brake_terms;
    result.altitude_target_down_m = input.hold_down_m;
    // 3. 고도 유지부터 계산한다. 예측 표적은 수평 명령을 만들지 않는다.
    result.command.vz_m_s =
        std::clamp(config_.altitude_gain * (input.hold_down_m - input.pose.down_m),
                   -config_.max_vertical_m_s, config_.max_vertical_m_s);
    if (config_.yaw_only) {
        result.command.vz_m_s = 0;
    }
    if (input.selection.observation_valid_until &&
        input.now + std::min(config_.command_ttl, std::chrono::milliseconds{50}) >=
            *input.selection.observation_valid_until) {
        // SDK의 반복 송신 전에 zero로 전환한다. motion 명령의 수명이 원래 관측 TTL을 넘지 않는다.
        result.mission = "GAP_HOLD";
        result.reason = "observation_expiry_reserve";
        // 2026-11-17 실외 비행(outdoor-low) 11:44:10: NUC 0.53초 공백에서 이 명령에 고도 보정
        // vz=-0.038이 남아, 근거 관측이 250ms를 넘자 FC 게이트웨이 송출 재검사가 source_stale로
        // 거부하고 송신권을 회수해 추종이 끝났다. 게이트웨이는 오래된 근거의 명령을 전부 0일
        // 때만 통과시킨다. 실내는 camera_descent_m>0 분기로 우연히 0이었다. 이제 항상 0으로 보낸다.
        result.command.vz_m_s = 0;
        result.command.mode = result.mission;
        previous_command_at_ = input.now;
        previous_command_ = {};
        reset_pid();
        return result;
    }
    if (input.selection.state == TargetState::Predicted) {
        reset_pid();
        result.mission = "PREDICTED";
        result.reason = "horizontal_guidance_paused";
        // 2026-11-17: 위 관측 만료 직전 명령과 같은 이유로 공백 정지 명령(short_gap_pause)과 같은
        // 조건에서 vz를 0으로 둔다 (실외 4종은 camera_descent_m=0이라 기존 조건으로는 남았다).
        if (point_only_ || config_.camera_descent_m > 0 || config_.yaw_only) {
            result.command.vz_m_s = 0;
        }
        result.command.mode = result.mission;
        return result;
    }
    if (input.selection.state != TargetState::Confirmed || !input.selection.fresh_real_detection ||
        !input.geometry.bearing_valid) {
        return hold_once(input, "confirmed_geometry_or_direction_invalid");
    }

    // 4. 실관측의 yaw를 계산하고 시야·정렬·미터 거리 조건이 맞으면 전진한다.
    const auto& target = *input.selection.selected_track;
    const auto& frame = *input.selection.latest_frame;
    if (frame.motion_sample_ns) {
        const double host = frame.sample_monotonic_ns / 1e9, sensor = *frame.motion_sample_ns / 1e9;
        if (motion_clock_samples_.empty() || host > motion_clock_samples_.back()[0]) {
            motion_clock_samples_.push_back({host, sensor});
            while (motion_clock_samples_.size() > 2 &&
                   host - motion_clock_samples_.front()[0] > .75) {
                motion_clock_samples_.pop_front();
            }
            const auto& first = motion_clock_samples_.front();
            if (host - first[0] >= .2) {
                const double rate = (sensor - first[1]) / (host - first[0]);
                if (!std::isfinite(rate) || rate < .1 || rate > 2) {
                    return hold_once(input, "motion_clock_rate_invalid");
                }
                motion_clock_rate_ = rate;
            }
        }
    } else {
        motion_clock_rate_ = 1;
    }
    const double time_rate = motion_clock_rate_;
    result.kinematic_time_rate = time_rate;
    result.mission = "FOLLOW_ACTIVE";
    if (input.selection.observation_valid_until) {
        result.valid_until = std::min(result.valid_until, *input.selection.observation_valid_until);
    }
    result.reason = "confirmed_target";
    result.command.mode = result.mission;
    result.command.yaw_rate_deg_s =
        std::clamp(config_.yaw_kp_deg_s_per_deg * input.geometry.bearing_error_deg,
                   -config_.max_yaw_rate_deg_s, config_.max_yaw_rate_deg_s);
    if (yaw_only_follower_) {
        const double dt =
            previous_command_at_ == Clock::time_point{}
                ? 0.05
                : std::clamp(
                      std::chrono::duration<double>(input.now - previous_command_at_).count(),
                      0.001, 0.2);
        result.command.yaw_rate_deg_s =
            yaw_only_follower_->update(input.geometry.bearing_error_deg, dt);
        result.reason = "yaw_only_confirmed_target";
        result.altitude_target_down_m.reset();
        // 2026-11-18 데모 모드: 확인된 실관측 추종 명령에서만 명목 높이를 지킨다(수평은 0).
        result.command.vx_m_s = result.command.vy_m_s = 0;
        result.command.vz_m_s = 0;
        if (config_.yaw_only_altitude_hold) {
            const double error_down_m = input.hold_down_m - input.pose.down_m;
            if (std::abs(error_down_m) > kYawOnlyAltitudeDeadbandM) {
                result.command.vz_m_s =
                    std::clamp(config_.altitude_gain * error_down_m, -config_.max_vertical_m_s,
                               config_.max_vertical_m_s);
            }
            result.altitude_target_down_m = input.hold_down_m;
        }
        previous_command_at_ = input.now;
        previous_command_ = result.command;
        return result;
    }
    if (image_center_follower_) {
        if (target.x1 < 2 || target.x2 > frame.image_width - 2 || target.y1 < 2 ||
            target.y2 > frame.image_height - 2) {
            return hold_once(input, "image_pid_target_touches_image_edge");
        }
        if (frame.image_width != config_.pid_camera.image_width ||
            frame.image_height != config_.pid_camera.image_height) {
            return hold_once(input, "image_pid_calibration_size_mismatch");
        }
        const double dt =
            previous_command_at_ == Clock::time_point{}
                ? .05
                : std::chrono::duration<double>(input.now - previous_command_at_).count();
        const auto command = image_center_follower_->update(input.pose, target, dt);
        if (!command) {
            return hold_once(input, "image_pid_state_invalid");
        }
        result.command = *command;
        result.command.mode = result.mission;
        result.reason = "outdoor_image_center_pid";
        result.altitude_target_down_m =
            config_.pid_ground_down_m - config_.predictive_target_height_m;
        result.pid_terms = image_center_follower_->terms();
        previous_command_at_ = input.now;
        previous_command_ = result.command;
        return result;
    }
    const double edge_x = frame.image_width * config_.edge_margin_fraction;
    const double edge_y = frame.image_height * config_.edge_margin_fraction;
    const bool fov_safe = target.x1 >= edge_x && target.x2 <= frame.image_width - edge_x &&
                          target.y1 >= edge_y && target.y2 <= frame.image_height - edge_y;
    const bool full_box = target.x1 >= 2 && target.x2 <= frame.image_width - 2 && target.y1 >= 2 &&
                          target.y2 <= frame.image_height - 2;
    const bool visible_contact = target.x1 >= 2 && target.x2 <= frame.image_width - 2 &&
                                 target.y2 >= 2 && target.y2 <= frame.image_height - 2 &&
                                 target.y2 - target.y1 >= 8;
    const bool visible_roof = target.x1 >= 2 && target.x2 <= frame.image_width - 2 &&
                              target.y1 >= 2 && target.y1 <= frame.image_height - 8 &&
                              target.y2 - target.y1 >= 8;
    const bool measured_anchor_visible =
        config_.predictive_follow && target.ground_anchor
            ? ((*target.ground_anchor)[0] == 0 ? visible_roof : visible_contact)
        : config_.ground_contact_anchor                                     ? visible_contact
        : config_.predictive_follow && config_.pid_camera.box_anchor_y == 0 ? visible_roof
                                                                            : full_box;
    if (camera_motion_guard_ && !measured_anchor_visible) {
        return hold_once(input, "measurement_anchor_touches_image_edge");
    }
    const bool aligned = std::abs(input.geometry.bearing_error_deg) <= config_.yaw_alignment_deg;
    const bool forward_aligned =
        (camera_motion_guard_ ? measured_anchor_visible : fov_safe) && aligned;
    const bool metric_range = input.geometry.range.valid &&
                              std::isfinite(input.geometry.range.rho_m) &&
                              input.geometry.range.rho_m >= 0 && input.geometry.range.quality &&
                              std::string_view(input.geometry.range.quality) == "METRIC";
    const bool horizontal_allowed = forward_aligned && metric_range;
    const auto admitted_velocity = [&]() {
        if (point_only_) {
            return std::array<double, 2>{0., 0.};
        }
        const auto& recent = input.selection.short_motion;
        const double yaw = input.pose.yaw_deg * std::acos(-1.) / 180;
        const double long_speed =
            std::hypot(input.selection.velocity_north_m_s, input.selection.velocity_east_m_s);
        const double long_forward = std::cos(yaw) * input.selection.velocity_north_m_s +
                                    std::sin(yaw) * input.selection.velocity_east_m_s;
        const bool long_valid =
            input.selection.ground_motion_valid && std::isfinite(long_speed) && long_speed >= .15;
        const bool far =
            input.geometry.range.valid && input.geometry.range.rho_m > config_.desired_range_m + .2;
        std::array<double, 2> v{};
        if (config_.predictive_follow) {
            return predictive_target_velocity(input.selection, config_.max_forward_m_s);
        }
        if (!recent.valid) {
            // 현재 실위치가 신선하고 먼 전방차인 경우에만 재확인 기간의 catch-up을 보조한다.
            if (far && long_valid && long_forward > .15) {
                v = {input.selection.velocity_north_m_s, input.selection.velocity_east_m_s};
            }
            return v;
        }
        if (recent.phase != ObservedMotionPhase::Moving || !recent.feedforward_valid) {
            return v;
        }
        v = {recent.velocity_north_m_s, recent.velocity_east_m_s};
        const double short_speed = std::hypot(v[0], v[1]);
        const double short_forward = std::cos(yaw) * v[0] + std::sin(yaw) * v[1];
        const double agreement =
            v[0] * input.selection.velocity_north_m_s + v[1] * input.selection.velocity_east_m_s;
        // 가까운 차의 과속 추정만 줄인다. 이미 뒤처진 전진차는 짧은 최신 속도로 따라잡는다.
        if (!(far && short_forward > .15) && long_valid && short_speed > 0 &&
            agreement >= .9 * short_speed * long_speed && long_speed < short_speed) {
            v[0] *= long_speed / short_speed;
            v[1] *= long_speed / short_speed;
        }
        return v;
    };
    double pid_dt = .05;
    if (config_.pid.enabled) {
        if (pid_previous_at_ != Clock::time_point{}) {
            pid_dt = std::chrono::duration<double>(input.now - pid_previous_at_).count();
            if (pid_dt <= 0 || pid_dt > .2) {
                return hold_once(input, "pid_control_interval_invalid");
            }
        }
        pid_previous_at_ = input.now;
        pid_dt *= time_rate;
        const auto& current_motion = input.selection.short_motion;
        if (!point_only_ && config_.reset_motion_on_stop_or_reverse && current_motion.valid) {
            const bool stopped = current_motion.phase == ObservedMotionPhase::Stopped &&
                                 (!reset_motion_previous_ ||
                                  reset_motion_previous_->phase != ObservedMotionPhase::Stopped);
            bool reversed = false;
            if (reset_motion_previous_ && reset_motion_previous_->valid &&
                reset_motion_previous_->feedforward_valid && current_motion.feedforward_valid) {
                const auto& previous = *reset_motion_previous_;
                reversed = previous.velocity_north_m_s * current_motion.velocity_north_m_s +
                               previous.velocity_east_m_s * current_motion.velocity_east_m_s <
                           -.5 *
                               std::hypot(previous.velocity_north_m_s, previous.velocity_east_m_s) *
                               std::hypot(current_motion.velocity_north_m_s,
                                          current_motion.velocity_east_m_s);
            }
            if (stopped || reversed) {
                forward_pid_.reset();
                right_pid_.reset();
                filtered_visual_error_.reset();
                latest_rear_goal_.velocity_north_m_s = latest_rear_goal_.velocity_east_m_s = 0;
                filtered_standoff_m_ = config_.desired_range_m - config_.ground_center_offset_m;
                // 표적 운동 계획만 폐기한다. 진행 중 yaw 명령의 slew 이력은 0으로 만들지 않는다.
                if (predictive_follower_) {
                    predictive_follower_->reset_plan();
                    predictive_follower_->reset_integral();
                }
            }
            if (current_motion.phase == ObservedMotionPhase::Stopped ||
                current_motion.feedforward_valid) {
                reset_motion_previous_ = current_motion;
            }
        }
        if (predictive_follower_) {
            // 2026-11-10 방위만 한도를 넘었고 접점·거리·관측은 유효하면 즉시 끝내지 않고 yaw로 다시
            // 맞춘다. 수평 0, point/하강 모드는 수직 0. 시간 안에 못 맞추면 기존처럼 HOLD.
            constexpr double kBearingRealignMaxDeg = 60;
            const bool view_ok = camera_motion_guard_ ? measured_anchor_visible : fov_safe;
            const double realign_bearing = input.geometry.bearing_error_deg;
            if (config_.v35_bearing_realign_s > 0 && !aligned && view_ok && metric_range &&
                input.observation_pose && target.ground && std::isfinite(realign_bearing) &&
                std::abs(realign_bearing) <= kBearingRealignMaxDeg) {
                if (bearing_realign_since_ == Clock::time_point{}) {
                    bearing_realign_since_ = input.now;
                    std::cerr << "BEARING_REALIGN start bearing_deg=" << realign_bearing
                              << std::endl;
                }
                if (std::chrono::duration<double>(input.now - bearing_realign_since_).count() >
                    config_.v35_bearing_realign_s) {
                    std::cerr << "BEARING_REALIGN timeout bearing_deg=" << realign_bearing
                              << std::endl;
                    return hold_once(input, "bearing_realign_timeout");
                }
                const double realign_dt =
                    previous_command_at_ == Clock::time_point{}
                        ? .05
                        : std::clamp(std::chrono::duration<double>(input.now - previous_command_at_)
                                         .count(),
                                     0., .2);
                const double wanted =
                    std::clamp(config_.yaw_kp_deg_s_per_deg * realign_bearing,
                               -config_.max_yaw_rate_deg_s, config_.max_yaw_rate_deg_s);
                const double yaw_step = config_.max_yaw_accel_deg_s2 > 0
                                            ? config_.max_yaw_accel_deg_s2 * realign_dt
                                            : std::numeric_limits<double>::infinity();
                const double realign_vz =
                    point_only_ || config_.camera_descent_m > 0 ? 0. : result.command.vz_m_s;
                result.command = FollowCommand{};
                result.command.vz_m_s = realign_vz;
                result.command.yaw_rate_deg_s =
                    std::clamp(wanted, previous_command_.yaw_rate_deg_s - yaw_step,
                               previous_command_.yaw_rate_deg_s + yaw_step);
                result.command.mode = result.mission;
                result.reason = "bearing_realign";
                reset_pid_timing();
                predictive_follower_->note_external_command({0., 0.}, realign_vz,
                                                            result.command.yaw_rate_deg_s);
                previous_command_at_ = input.now;
                previous_command_ = result.command;
                return result;
            }
            if (!horizontal_allowed || !input.observation_pose || !target.ground) {
                return hold_once(input, "predictive_observation_state_invalid");
            }
            if (bearing_realign_since_ != Clock::time_point{}) {
                std::cerr << "BEARING_REALIGN end bearing_deg=" << realign_bearing << std::endl;
                bearing_realign_since_ = {};
            }
            const double age =
                std::chrono::duration<double>(
                    input.now -
                    Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}})
                    .count();
            const auto velocity = admitted_velocity();
            const auto prediction = predictive_follower_->update(
                input.pose, *input.observation_pose, target, velocity, age, input.hold_down_m,
                pid_dt, time_rate, input.selection.recovery_reason == "target_reacquired",
                frame.motion_sample_ns, std::nullopt, input.producer_ground);
            if (!prediction) {
                std::cerr << "PREDICTIVE_REJECT path=follow check="
                          << predictive_follower_->last_reject_reason() << std::endl;
                return hold_once(input, "predictive_model_state_invalid");
            }
            if (predictive_follower_->fixed_yaw_realigning() != fixed_yaw_realign_active_) {
                fixed_yaw_realign_active_ = predictive_follower_->fixed_yaw_realigning();
                std::cerr << "FIXED_YAW_REALIGN " << (fixed_yaw_realign_active_ ? "start" : "end")
                          << " bearing_deg=" << input.geometry.bearing_error_deg << std::endl;
            }
            if (predictive_follower_->last_slew_reconciled()) {
                if (!slew_reconcile_active_) {
                    std::cerr << "PREDICTIVE_SLEW_RECONCILE start" << std::endl;
                }
                slew_reconcile_active_ = true;
                ++slew_reconcile_ticks_;
            } else if (slew_reconcile_active_) {
                std::cerr << "PREDICTIVE_SLEW_RECONCILE end ticks=" << slew_reconcile_ticks_
                          << std::endl;
                slew_reconcile_active_ = false;
                slew_reconcile_ticks_ = 0;
            }
            result.command = prediction->command;
            result.command.mode = result.mission;
            result.reason =
                point_only_ ? "current_poi_pampc_guidance" : "confirmed_target_predictive_guidance";
            result.rear_goal = RearGoal{
                prediction->goal[0],    prediction->goal[1], velocity[0], velocity[1], false,
                config_.desired_range_m};
            result.altitude_target_down_m = prediction->altitude_target_down_m;
            result.predictive_vertical = prediction->vertical_diagnostics;
            result.v21_motion = V21MotionDiagnostics{
                false, input.selection.box_motion_suspect, 0, 0, velocity[0], velocity[1], 1};
            last_follow_yaw_deg_s_ = result.command.yaw_rate_deg_s;
            previous_command_at_ = input.now;
            previous_command_ = result.command;
            return result;
        }
        if (config_.camera_descent_m == 0) {
            result.command.vz_m_s =
                altitude_pid_.update(input.pose.down_m, input.hold_down_m, pid_dt);
        }
        double normalized_x = (target.x1 + target.x2) / frame.image_width - 1;
        // 좁은 렌즈의 픽셀 확대율을 yaw loop 이득에 다시 곱하지 않는다.
        // 보정된 bearing(rad)→yaw rate(rad/s). 기존 수치계수와 단일deg변환은 유지한다.
        if (camera_motion_guard_) {
            normalized_x = input.geometry.bearing_error_deg * std::acos(-1.) / 180;
        }
        if (config_.body_standoff_follow) {
            // PixEagle mc_velocity_ground는 XY로 영상 오차를 제어하고 yawspeed=0이다.
            // 같은 가로 오차를 yaw/횡이동 두 loop가 동시에 추종하지 않도록 한다.
            if (!config_.velocity_feedforward_follow) {
                result.command.yaw_rate_deg_s = 0;
            } else {
                const auto velocity = admitted_velocity();
                const double vn = velocity[0], ve = velocity[1];
                const double n = input.geometry.north_m, e = input.geometry.east_m,
                             r2 = n * n + e * e;
                if (!input.pose.have_velocity || r2 < .01 || !std::isfinite(vn + ve)) {
                    return hold_once(input, "velocity_follow_state_invalid");
                }
                // LOS 각속도와 각도 오차: yaw 변경으로 가상의 횡목표를 만들지 않는다.
                double rate = (n * (ve - input.pose.velocity_east_m_s) -
                               e * (vn - input.pose.velocity_north_m_s)) /
                              r2 * 180 / std::acos(-1.);
                if (config_.follow_side_offset_m != 0) {
                    // 최근 실관측의 LOS 각속도는 이동축 추정이나 목표점 회전 속도와 구분한다.
                    const double angle = std::atan2(e, n), pi = std::acos(-1.);
                    const auto los_sample =
                        frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
                    if (los_sample != side_los_sample_ns_) {
                        const double dt = (los_sample - side_los_sample_ns_) / 1e9;
                        if (side_los_sample_ns_ > 0 && dt >= .01 && dt <= .25) {
                            const double observed = std::clamp(
                                std::remainder(angle - side_los_angle_rad_, 2 * pi) / dt * 180 / pi,
                                -config_.max_yaw_rate_deg_s, config_.max_yaw_rate_deg_s);
                            side_los_rate_deg_s_ +=
                                dt / (.05 + dt) * (observed - side_los_rate_deg_s_);
                        } else {
                            side_los_rate_deg_s_ = 0;
                        }
                        side_los_angle_rad_ = angle;
                        side_los_sample_ns_ = los_sample;
                    }
                    rate = side_los_rate_deg_s_;
                    const double age =
                        time_rate *
                        std::clamp(std::chrono::duration<double>(
                                       input.now - Clock::time_point{std::chrono::nanoseconds{
                                                       frame.sample_monotonic_ns}})
                                       .count(),
                                   0., .25);
                    const double bearing =
                        std::remainder(angle * 180 / pi + rate * age - input.pose.yaw_deg, 360.);
                    result.command.yaw_rate_deg_s = config_.yaw_kp_deg_s_per_deg * bearing;
                }
                const double raw =
                    std::clamp(result.command.yaw_rate_deg_s + rate, -config_.max_yaw_rate_deg_s,
                               config_.max_yaw_rate_deg_s);
                result.command.yaw_rate_deg_s = yaw_smoother_.update(
                    raw, pid_dt,
                    std::hypot(input.pose.velocity_north_m_s, input.pose.velocity_east_m_s));
            }
        } else {
            const double yaw_rad = yaw_pid_.update(-normalized_x, 0, pid_dt);
            result.command.yaw_rate_deg_s = yaw_smoother_.update(yaw_rad * 180 / std::acos(-1.),
                                                                 pid_dt, previous_command_.vx_m_s);
        }
        if (!horizontal_allowed) {
            forward_pid_.reset();
            right_pid_.reset();
        }
    }
    if (config_.ground_vector_follow && horizontal_allowed) {
        // FC 지면 NED 상대위치를 body 전방/우측으로 회전한다. GT/차량 경로는 읽지 않는다.
        if (!std::isfinite(input.geometry.north_m) || !std::isfinite(input.geometry.east_m)) {
            return hold_once(input, "ground_vector_invalid");
        }
        const double yaw = input.pose.yaw_deg * std::acos(-1.0) / 180.;
        const double forward =
            std::cos(yaw) * input.geometry.north_m + std::sin(yaw) * input.geometry.east_m;
        const double right =
            -std::sin(yaw) * input.geometry.north_m + std::cos(yaw) * input.geometry.east_m;
        if (config_.motion_ground_follow) {
            const double range = std::hypot(input.geometry.north_m, input.geometry.east_m);
            if (range < 1e-6) {
                return hold_once(input, "ground_motion_range_invalid");
            }
            double hn = input.geometry.north_m / range, he = input.geometry.east_m / range, vn = 0,
                   ve = 0;
            const double speed =
                std::hypot(input.selection.velocity_north_m_s, input.selection.velocity_east_m_s);
            if (input.selection.ground_motion_valid && std::isfinite(speed) && speed >= .15 &&
                speed <= 10) {
                vn = input.selection.velocity_north_m_s;
                ve = input.selection.velocity_east_m_s;
                hn = vn / speed;
                he = ve / speed;
            }
            double behind = config_.desired_range_m - config_.ground_center_offset_m;
            if (config_.velocity_feedforward_follow && config_.reverse_range_margin_m > 0) {
                const auto& recent = input.selection.short_motion;
                const double forward_speed = recent.valid
                                                 ? std::cos(yaw) * recent.velocity_north_m_s +
                                                       std::sin(yaw) * recent.velocity_east_m_s
                                                 : 0;
                const double reverse_fraction = std::clamp((-forward_speed - .15) / .45, 0., 1.);
                const double desired = behind + config_.reverse_range_margin_m * reverse_fraction;
                filtered_standoff_m_ += pid_dt / (.15 + pid_dt) * (desired - filtered_standoff_m_);
                behind = filtered_standoff_m_;
            }
            double en = input.geometry.north_m - behind * hn,
                   ee = input.geometry.east_m - behind * he;
            // g=p-behind*h: v_g=v-behind*(a-h*(h dot a))/|v|.
            // Only a fresh, admitted motion fit may rotate the rear goal; existing guards/caps
            // still apply below.
            if (config_.body_standoff_follow) {
                if (!target.ground) {
                    return hold_once(input, "body_standoff_ground_observation_required");
                }
                en = input.geometry.north_m - behind * std::cos(yaw);
                ee = input.geometry.east_m - behind * std::sin(yaw);
                const auto& recent = input.selection.short_motion;
                vn = recent.valid && recent.feedforward_valid ? recent.velocity_north_m_s : 0;
                ve = recent.valid && recent.feedforward_valid ? recent.velocity_east_m_s : 0;
                if (config_.velocity_feedforward_follow) {
                    const auto velocity = admitted_velocity();
                    vn = velocity[0];
                    ve = velocity[1];
                    if (!std::isfinite(vn + ve)) {
                        return hold_once(input, "vehicle_velocity_invalid");
                    }
                    const double speed = std::hypot(vn, ve);
                    if (speed > config_.max_forward_m_s) {
                        vn *= config_.max_forward_m_s / speed;
                        ve *= config_.max_forward_m_s / speed;
                    }
                    const double age =
                        std::chrono::duration<double>(
                            input.now -
                            Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}})
                            .count();
                    if (age < 0 || age > .25) {
                        return hold_once(input, "velocity_follow_observation_expired");
                    }
                    // 보상은 현재 명령 계산만 위한 것. 원 관측 시각/TTL/대상 기억은 그대로다.
                    const double n = input.geometry.north_m + vn * age * time_rate,
                                 e = input.geometry.east_m + ve * age * time_rate,
                                 r = std::hypot(n, e);
                    if (r < .1) {
                        return hold_once(input, "velocity_follow_range_invalid");
                    }
                    en = n - behind * n / r;
                    ee = e - behind * e / r;
                    if (config_.follow_side_offset_m != 0) {
                        if (!side_follow_heading_) {
                            side_follow_heading_ = std::array<double, 2>{n / r, e / r};
                        }
                        const auto previous = *side_follow_heading_;
                        const double fitted_speed = std::hypot(input.selection.velocity_north_m_s,
                                                               input.selection.velocity_east_m_s);
                        const double short_speed =
                            std::hypot(recent.velocity_north_m_s, recent.velocity_east_m_s);
                        const double agreement = std::abs(
                            recent.velocity_north_m_s * input.selection.velocity_north_m_s +
                            recent.velocity_east_m_s * input.selection.velocity_east_m_s);
                        if (recent.valid && recent.phase == ObservedMotionPhase::Moving &&
                            recent.direction_valid && recent.feedforward_valid &&
                            short_speed >= .3 && input.selection.ground_motion_valid &&
                            fitted_speed >= .3 && fitted_speed <= 10 &&
                            agreement >= .85 * short_speed * fitted_speed) {
                            double dn = input.selection.velocity_north_m_s / fitted_speed,
                                   de = input.selection.velocity_east_m_s / fitted_speed;
                            // 후진해도 관측 이동축을 뒤집지 않아 같은 쪽 간격을 유지한다.
                            if (dn * previous[0] + de * previous[1] < 0) {
                                dn = -dn;
                                de = -de;
                            }
                            const double alpha = pid_dt / (.5 + pid_dt);
                            const double angle = std::atan2(previous[0] * de - previous[1] * dn,
                                                            previous[0] * dn + previous[1] * de);
                            const double step =
                                std::clamp(alpha * angle, -.5 * pid_dt, .5 * pid_dt);
                            side_follow_heading_ = std::array<double, 2>{
                                previous[0] * std::cos(step) - previous[1] * std::sin(step),
                                previous[0] * std::sin(step) + previous[1] * std::cos(step)};
                        }
                        const auto axis = *side_follow_heading_;
                        const double side = config_.follow_side_offset_m,
                                     back = std::sqrt(behind * behind - side * side);
                        en = n - back * axis[0] - side * axis[1];
                        ee = e - back * axis[1] + side * axis[0];
                        // 곡선에서는 관측 축 회전에 따른 목표점 속도를 함께 반영한다.
                        const double turn =
                            (previous[0] * axis[1] - previous[1] * axis[0]) / pid_dt;
                        const double angular_rate = std::clamp(turn, -.5, .5);
                        vn += angular_rate * (back * axis[1] - side * axis[0]);
                        ve += angular_rate * (-back * axis[0] - side * axis[1]);
                    }
                }
                latest_rear_goal_ = {
                    input.pose.north_m + en, input.pose.east_m + ee, vn, ve, false, behind};
                result.rear_goal = latest_rear_goal_;
            } else if (config_.observed_path_rear_follow) {
                if (!target.ground) {
                    return hold_once(input, "rear_path_ground_observation_required");
                }
                if (rear_sample_ns_ != frame.sample_monotonic_ns) {
                    bool moving = input.selection.ground_motion_valid && std::isfinite(speed) &&
                                  speed >= .15 && speed <= 10;
                    double goal_speed = moving ? speed : 0;
                    std::array<double, 2> point{(*target.ground)[0], (*target.ground)[1]};
                    std::optional<std::array<double, 2>> observed_velocity;
                    const auto& recent = input.selection.short_motion;
                    const bool fast = input.selection.short_motion_enabled;
                    const auto phase =
                        fast && recent.valid ? recent.phase : ObservedMotionPhase::Unknown;
                    if (fast) {
                        moving = phase == ObservedMotionPhase::Moving && recent.direction_valid;
                        goal_speed = 0;
                        if (moving) {
                            const double short_speed =
                                std::hypot(recent.velocity_north_m_s, recent.velocity_east_m_s);
                            observed_velocity = std::array<double, 2>{recent.velocity_north_m_s,
                                                                      recent.velocity_east_m_s};
                            const double agreement =
                                input.selection.velocity_north_m_s * recent.velocity_north_m_s +
                                input.selection.velocity_east_m_s * recent.velocity_east_m_s;
                            const bool long_consistent = input.selection.ground_motion_valid &&
                                                         std::isfinite(speed) && speed >= .15 &&
                                                         speed <= 10 &&
                                                         agreement >= .5 * speed * short_speed &&
                                                         speed <= 2 * short_speed + .1;
                            goal_speed = recent.feedforward_valid
                                             ? (long_consistent ? speed : short_speed)
                                             : 0;
                            if (!long_consistent) {
                                point = {recent.north_m, recent.east_m};
                            }
                        } else if (recent.valid) {
                            point = {recent.north_m, recent.east_m};
                        }
                    }
                    // PID의 위치 반응을 속도 확인 대기에 묶지 않는다. 속도/방향만 회귀로 확인한다.
                    if (config_.pid.enabled) {
                        point = {(*target.ground)[0], (*target.ground)[1]};
                    }
                    if (config_.pid.enabled ||
                        !(fast && phase == ObservedMotionPhase::Stopped &&
                          rear_motion_phase_ == ObservedMotionPhase::Stopped &&
                          rear_sample_ns_ != 0)) {
                        latest_rear_goal_ = rear_goal_.update(
                            point, {input.geometry.north_m / range, input.geometry.east_m / range},
                            goal_speed, moving, observed_velocity,
                            fast && phase == ObservedMotionPhase::Stopped &&
                                rear_motion_phase_ != ObservedMotionPhase::Stopped);
                    }
                    if (fast && !moving) {
                        latest_rear_goal_.velocity_north_m_s = latest_rear_goal_.velocity_east_m_s =
                            0;
                    }
                    rear_motion_phase_ = phase;
                    rear_sample_ns_ = frame.sample_monotonic_ns;
                }
                if (config_.camera_motion_guard && target.ground) {
                    // 굽은/잡음 섞인 경로의 호길이만2.5m이면 실제 직선 간격이 줄어든다.
                    // 경로 뒤쪽 방향은 유지하며 사용자가 지정한 최소 직선 간격까지 연장한다.
                    double dn = latest_rear_goal_.north_m - (*target.ground)[0],
                           de = latest_rear_goal_.east_m - (*target.ground)[1];
                    double distance = std::hypot(dn, de);
                    if (distance < behind) {
                        if (distance < 1e-6) {
                            dn = -input.geometry.north_m / range;
                            de = -input.geometry.east_m / range;
                            distance = 1;
                        }
                        latest_rear_goal_.north_m = (*target.ground)[0] + behind * dn / distance;
                        latest_rear_goal_.east_m = (*target.ground)[1] + behind * de / distance;
                    }
                }
                en = latest_rear_goal_.north_m - input.pose.north_m;
                ee = latest_rear_goal_.east_m - input.pose.east_m;
                vn = latest_rear_goal_.velocity_north_m_s;
                ve = latest_rear_goal_.velocity_east_m_s;
                result.rear_goal = latest_rear_goal_;
            } else if (input.selection.ground_motion_valid &&
                       input.selection.ground_acceleration_valid && speed >= .15 && speed <= 10) {
                const double an = input.selection.acceleration_north_m_s2,
                             ae = input.selection.acceleration_east_m_s2;
                if (!std::isfinite(an) || !std::isfinite(ae)) {
                    return hold_once(input, "ground_acceleration_invalid");
                }
                const double longitudinal = hn * an + he * ae;
                vn -= behind * (an - hn * longitudinal) / speed;
                ve -= behind * (ae - he * longitudinal) / speed;
            }
            if (config_.velocity_feedforward_follow) {
                const auto& recent = input.selection.short_motion;
                const bool stopped = recent.valid && recent.phase == ObservedMotionPhase::Stopped;
                const std::array<double, 2> error{std::cos(yaw) * en + std::sin(yaw) * ee,
                                                  -std::sin(yaw) * en + std::cos(yaw) * ee};
                const double alpha = pid_dt / (.1 + pid_dt);
                if (!filtered_visual_error_) {
                    filtered_visual_error_ = error;
                } else {
                    for (std::size_t i = 0; i < 2; ++i) {
                        (*filtered_visual_error_)[i] +=
                            alpha * (error[i] - (*filtered_visual_error_)[i]);
                    }
                }
                const double drone_forward = std::cos(yaw) * input.pose.velocity_north_m_s +
                                             std::sin(yaw) * input.pose.velocity_east_m_s;
                const double drone_right = -std::sin(yaw) * input.pose.velocity_north_m_s +
                                           std::cos(yaw) * input.pose.velocity_east_m_s;
                const bool slow = std::hypot(drone_forward, drone_right) <= .05;
                const bool towards_goal =
                    slow || error[0] * drone_forward + error[1] * drone_right >= 0;
                // 정지 허용범위로 들어오면 실제 FC 속도로 D 제동한다. 목표에서 멀어지는 동안은
                // P/D를 유지한다.
                if (stopped &&
                    std::hypot(recent.velocity_north_m_s, recent.velocity_east_m_s) <= .15 &&
                    std::hypot(en, ee) <= config_.range_deadband_m && towards_goal) {
                    forward_pid_.reset();
                    right_pid_.reset();
                    filtered_visual_error_ = error;
                    result.command.vx_m_s = slow ? 0 : -config_.pid.forward.d * drone_forward;
                    result.command.vy_m_s = slow ? 0 : -config_.pid.right.d * drone_right;
                    stationary_brake_terms = std::array<PidTerms, 2>{
                        PidTerms{0, 0, result.command.vx_m_s, result.command.vx_m_s},
                        PidTerms{0, 0, result.command.vy_m_s, result.command.vy_m_s}};
                } else {
                    result.command.vx_m_s =
                        forward_pid_.update(-(*filtered_visual_error_)[0], 0, pid_dt, !stopped);
                    result.command.vy_m_s =
                        right_pid_.update(-(*filtered_visual_error_)[1], 0, pid_dt, !stopped);
                }
                result.command.vx_m_s += std::cos(yaw) * vn + std::sin(yaw) * ve;
                result.command.vy_m_s += -std::sin(yaw) * vn + std::cos(yaw) * ve;
            } else if (config_.pid.enabled) {
                const auto vehicle =
                    std::array<double, 2>{(*target.ground)[0], (*target.ground)[1]};
                const auto measured = pid_projection_->control_projection(
                    input.pose, vehicle, config_.pid_ground_down_m, !config_.ground_contact_anchor);
                Pose at_goal = input.pose;
                at_goal.north_m = latest_rear_goal_.north_m;
                at_goal.east_m = latest_rear_goal_.east_m;
                const auto desired = pid_projection_->control_projection(
                    at_goal, vehicle, config_.pid_ground_down_m, !config_.ground_contact_anchor);
                if (!measured || !desired) {
                    return hold_once(input, "pid_control_projection_outside_fov");
                }
                // 원본의 고정 setpoint처럼 오차를0주위에 놓는다. 같은 좌표 변환으로
                // measurement/reference가 함께 움직여도 오차0에서 D가 생기지 않는다.
                const auto& recent = input.selection.short_motion;
                const bool drone_still =
                    !input.pose.have_velocity ||
                    std::hypot(input.pose.velocity_north_m_s, input.pose.velocity_east_m_s) < .05;
                const bool stopped =
                    recent.valid && recent.phase == ObservedMotionPhase::Stopped &&
                    !recent.position_changed_since_stop && drone_still &&
                    std::hypot(recent.velocity_north_m_s, recent.velocity_east_m_s) <= .15;
                const double error_forward = std::cos(yaw) * en + std::sin(yaw) * ee;
                const double error_right = -std::sin(yaw) * en + std::cos(yaw) * ee;
                std::array<double, 2> residual{(*measured)[1] - (*desired)[1],
                                               (*desired)[0] - (*measured)[0]};
                if (config_.body_standoff_follow) {
                    // 20Hz 검출의 픽셀/계단 잡음이 미분 출력으로 증폭되지 않게 한다.
                    // 시정수100ms는 평활화 값이며 관측/명령 만료 시간과 독립이다.
                    const double alpha = pid_dt / (.1 + pid_dt);
                    if (!filtered_visual_error_) {
                        filtered_visual_error_ = residual;
                    } else {
                        for (std::size_t i = 0; i < 2; ++i) {
                            (*filtered_visual_error_)[i] +=
                                alpha * (residual[i] - (*filtered_visual_error_)[i]);
                        }
                    }
                    residual = *filtered_visual_error_;
                }
                if (stopped && std::abs(error_forward) <= config_.range_deadband_m) {
                    forward_pid_.reset();
                } else {
                    result.command.vx_m_s = forward_pid_.update(residual[0], 0, pid_dt);
                }
                if (stopped && std::abs(error_right) <= config_.range_deadband_m) {
                    right_pid_.reset();
                } else {
                    result.command.vy_m_s = right_pid_.update(residual[1], 0, pid_dt);
                }
                // 표시는 명령에 실제 사용한 전향항을 나타낸다. PID에는 차량 속도를 더하지 않는다.
                result.rear_goal->velocity_north_m_s = result.rear_goal->velocity_east_m_s = 0;
            } else {
                vn += std::abs(en) > config_.range_deadband_m ? config_.forward_gain_m_s_per_m * en
                                                              : 0;
                ve += std::abs(ee) > config_.range_deadband_m ? config_.forward_gain_m_s_per_m * ee
                                                              : 0;
                result.command.vx_m_s = std::cos(yaw) * vn + std::sin(yaw) * ve;
                result.command.vy_m_s = -std::sin(yaw) * vn + std::cos(yaw) * ve;
            }
            result.reason =
                stationary_brake_terms              ? "confirmed_target_stationary_velocity_braking"
                : config_.body_standoff_follow      ? "confirmed_target_body_standoff_guidance"
                : config_.observed_path_rear_follow ? "confirmed_target_observed_path_guidance"
                                                    : "confirmed_target_motion_guidance";
        } else {
            const double error = forward - config_.desired_range_m;
            result.command.vx_m_s = std::abs(error) > config_.range_deadband_m
                                        ? config_.forward_gain_m_s_per_m * error
                                        : 0.;
            result.command.vy_m_s = std::abs(right) > config_.range_deadband_m
                                        ? config_.forward_gain_m_s_per_m * right
                                        : 0.;
        }
        const double speed = std::hypot(result.command.vx_m_s, result.command.vy_m_s);
        if (speed > config_.max_forward_m_s) {
            result.command.vx_m_s *= config_.max_forward_m_s / speed;
            result.command.vy_m_s *= config_.max_forward_m_s / speed;
        }
    } else if (forward_aligned && metric_range) {
        const double distance_error = input.geometry.range.rho_m - config_.desired_range_m;
        if (distance_error > config_.range_deadband_m) {
            result.command.vx_m_s = std::clamp(config_.forward_gain_m_s_per_m * distance_error, 0.0,
                                               config_.max_forward_m_s);
        }
    }
    // 기본 모드는 횡속도0을 유지한다. 벡터 모드의 무효 관측/시야/정렬은 즉시 수평zero다.
    if (config_.max_backward_m_s > 0) {
        result.command.vx_m_s = std::max(result.command.vx_m_s, -config_.max_backward_m_s);
    }
    if (config_.max_horizontal_accel_m_s2 > 0 || config_.max_yaw_accel_deg_s2 > 0) {
        const double dt =
            previous_command_at_ == Clock::time_point{}
                ? 0.05
                : time_rate *
                      std::clamp(
                          std::chrono::duration<double>(input.now - previous_command_at_).count(),
                          0.0, 0.1);
        if (config_.max_horizontal_accel_m_s2 > 0) {
            const double dv = config_.max_horizontal_accel_m_s2 * dt;
            if (config_.ground_vector_follow && horizontal_allowed && !camera_motion_guard_) {
                const double dx = result.command.vx_m_s - previous_command_.vx_m_s;
                const double dy = result.command.vy_m_s - previous_command_.vy_m_s;
                const double delta = std::hypot(dx, dy);
                if (delta > dv) {
                    result.command.vx_m_s = previous_command_.vx_m_s + dx * dv / delta;
                    result.command.vy_m_s = previous_command_.vy_m_s + dy * dv / delta;
                }
            } else if (!config_.ground_vector_follow) {
                result.command.vx_m_s =
                    std::clamp(result.command.vx_m_s, previous_command_.vx_m_s - dv,
                               previous_command_.vx_m_s + dv);
            }
        }
        if (config_.max_yaw_accel_deg_s2 > 0) {
            const double dyaw = config_.max_yaw_accel_deg_s2 * dt;
            result.command.yaw_rate_deg_s =
                std::clamp(result.command.yaw_rate_deg_s, previous_command_.yaw_rate_deg_s - dyaw,
                           previous_command_.yaw_rate_deg_s + dyaw);
        }
        if (camera_motion_guard_ && horizontal_allowed && target.ground) {
            const auto constrained = camera_motion_guard_->limit(input.pose, target, result.command,
                                                                 config_.max_horizontal_accel_m_s2);
            if (!constrained) {
                return hold_once(input, "visual_motion_state_or_anchor_invalid");
            }
            result.command = constrained->command;
            // 실제 속도 기준 가드 뒤에도 명시 속도상한을 유지한다. 실제 가속도 보증은 아니다.
            if (config_.max_backward_m_s > 0) {
                result.command.vx_m_s = std::max(result.command.vx_m_s, -config_.max_backward_m_s);
            }
            if (constrained->limited) {
                result.reason = constrained->recovery ? "confirmed_target_fov_recovery"
                                                      : "confirmed_target_fov_acceleration_limited";
            }
        }
        previous_command_at_ = input.now;
        previous_command_ = result.command;
    }
    if (config_.camera_descent_m > 0) {
        if (horizontal_allowed) {
            const double yaw = input.pose.yaw_deg * std::acos(-1.) / 180;
            const auto& recent = input.selection.short_motion;
            const double car_forward = recent.valid ? std::cos(yaw) * recent.velocity_north_m_s +
                                                          std::sin(yaw) * recent.velocity_east_m_s
                                                    : 0;
            const double drone_forward = std::cos(yaw) * input.pose.velocity_north_m_s +
                                         std::sin(yaw) * input.pose.velocity_east_m_s;
            const double relative_closing = drone_forward - car_forward;
            const bool early_closing =
                relative_closing > .4 && target.y2 >= frame.image_height * .25;
            const double closing = target.y2 >= frame.image_height * .35 || early_closing
                                       ? std::clamp((relative_closing - .15) / .45, 0., 1.)
                                       : 0;
            const double lower_edge =
                std::clamp((target.y2 / frame.image_height - .6) / .25, 0., 1.);
            const double available_descent =
                config_.camera_min_height_m > 0
                    ? std::min(config_.camera_descent_m,
                               std::max(0., config_.pid_ground_down_m - input.hold_down_m -
                                                config_.camera_min_height_m))
                    : config_.camera_descent_m;
            const double desired_descent = available_descent * std::max(closing, lower_edge);
            // 상단 시야가 필요하면 이전 하강 기억을 비운다. 그 밖의 복귀는 완만하게 한다.
            if (target.y2 < frame.image_height * .35 && !early_closing) {
                filtered_descent_m_ = 0;
            } else if (desired_descent > filtered_descent_m_ + .01) {
                filtered_descent_m_ = desired_descent;
            } else if (desired_descent < filtered_descent_m_ - .01) {
                filtered_descent_m_ +=
                    pid_dt / (.6 + pid_dt) * (desired_descent - filtered_descent_m_);
            }
            if (desired_descent == 0 && filtered_descent_m_ < .01) {
                filtered_descent_m_ = 0;
            }
            filtered_descent_m_ = std::min(filtered_descent_m_, available_descent);
            const double target_down = input.hold_down_m + filtered_descent_m_;
            result.altitude_target_down_m = target_down;
            // 큰 참조 이동에서 포화 back-calculation의 I가 반대 보정을 남기지 않게 한다.
            const bool near_height = std::abs(input.pose.down_m - target_down) <= .05;
            result.command.vz_m_s =
                altitude_pid_.update(input.pose.down_m, target_down, pid_dt, near_height);
            if (input.pose.down_m >= input.hold_down_m + config_.camera_descent_m &&
                result.command.vz_m_s > 0) {
                result.command.vz_m_s = 0;
            }
            if (input.pose.down_m <= input.hold_down_m && result.command.vz_m_s < 0) {
                result.command.vz_m_s = 0;
            }
        } else {
            result.command.vz_m_s = 0;
            filtered_descent_m_ = 0;
        }
    }
    if (config_.pid.enabled) {
        result.pid_terms = std::array<PidTerms, 4>{forward_pid_.terms(), right_pid_.terms(),
                                                   yaw_pid_.terms(), altitude_pid_.terms()};
    }
    if (stationary_brake_terms && result.pid_terms) {
        result.pid_terms->at(0) = stationary_brake_terms->at(0);
        result.pid_terms->at(1) = stationary_brake_terms->at(1);
    }
    return result;
}
}  // namespace follow
