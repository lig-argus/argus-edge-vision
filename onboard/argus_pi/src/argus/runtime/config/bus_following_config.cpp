// 런타임 주소·추적기·보정·제어값을 환경 이름 분기 없이 외부 설정으로 조립한다.
#include "argus/runtime/config/include/bus_following_config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace follow {
namespace {
using Values = std::map<std::string, std::string>;

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

double number(const Values& values, const char* key) {
    const auto found = values.find(key);
    if (found == values.end()) {
        throw std::invalid_argument(std::string("missing config key: ") + key);
    }
    std::size_t used = 0;
    const double parsed = std::stod(found->second, &used);
    if (used != found->second.size() || !std::isfinite(parsed)) {
        throw std::invalid_argument(std::string("invalid finite number for config key: ") + key);
    }
    return parsed;
}

std::int64_t integer(const Values& values, const char* key) {
    const double parsed = number(values, key);
    if (std::floor(parsed) != parsed || parsed < 0 || parsed > 2147483647.0) {
        throw std::invalid_argument(std::string("invalid non-negative integer for config key: ") +
                                    key);
    }
    return static_cast<std::int64_t>(parsed);
}

std::chrono::milliseconds milliseconds(const Values& values, const char* key) {
    const auto value = integer(values, key);
    if (value == 0) {
        throw std::invalid_argument(std::string("duration must be positive: ") + key);
    }
    return std::chrono::milliseconds(value);
}

std::string string_value(const Values& values, const char* key) {
    const auto found = values.find(key);
    if (found == values.end() || found->second.empty()) {
        throw std::invalid_argument(std::string("missing config value: ") + key);
    }
    return found->second;
}

bool boolean(const Values& values, const char* key) {
    const auto value = string_value(values, key);
    if (value == "true") {
        return true;
    }
    if (value == "false") {
        return false;
    }
    throw std::invalid_argument(std::string("boolean must be true or false: ") + key);
}

std::vector<std::int32_t> class_ids(const Values& values) {
    std::istringstream stream(string_value(values, "target_class_ids"));
    std::vector<std::int32_t> ids;
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = trim(token);
        if (token.empty()) {
            throw std::invalid_argument("target_class_ids contains an empty item");
        }
        std::size_t used = 0;
        const auto id = std::stoll(token, &used);
        if (used != token.size() || id < 0 || id > 2147483647) {
            throw std::invalid_argument("target_class_ids must contain non-negative int32 values");
        }
        ids.push_back(static_cast<std::int32_t>(id));
    }
    if (ids.empty()) {
        throw std::invalid_argument("target_class_ids must not be empty");
    }
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
        throw std::invalid_argument("target_class_ids must not contain duplicates");
    }
    return ids;
}

template <std::size_t N>
std::array<double, N> number_array(const Values& values, const char* key) {
    std::istringstream stream(string_value(values, key));
    std::array<double, N> result{};
    std::string token;
    std::size_t index = 0;
    while (std::getline(stream, token, ',')) {
        if (index >= N) {
            throw std::invalid_argument(std::string("too many values for config key: ") + key);
        }
        std::size_t used = 0;
        token = trim(std::move(token));
        result[index] = std::stod(token, &used);
        if (used != token.size() || !std::isfinite(result[index])) {
            throw std::invalid_argument(std::string("invalid array value for config key: ") + key);
        }
        ++index;
    }
    if (index != N) {
        throw std::invalid_argument(std::string("wrong array length for config key: ") + key);
    }
    return result;
}

constexpr std::array<const char*, 174> kKeys{{"expected_fc_uid",
                                              "expected_fc_system_id",
                                              "expected_fc_component_id",
                                              "observations_sub_endpoint",
                                              "tracks_pub_endpoint",
                                              "tracks_sub_endpoint",
                                              "status_pub_endpoint",
                                              "command_rep_endpoint",
                                              "px4_connection_url",
                                              "target_class_ids",
                                              "image_width",
                                              "image_height",
                                              "fx",
                                              "fy",
                                              "cx",
                                              "cy",
                                              "camera_to_body",
                                              "camera_offset_body_m",
                                              "box_anchor_x",
                                              "box_anchor_y",
                                              "target_anchor_height_above_ground_m",
                                              "ground_down_m",
                                              "hold_down_m",
                                              "max_pose_bracket_ms",
                                              "max_pose_receive_age_ms",
                                              "desired_range_m",
                                              "range_deadband_m",
                                              "yaw_kp_deg_s_per_deg",
                                              "max_yaw_rate_deg_s",
                                              "yaw_alignment_deg",
                                              "forward_gain_m_s_per_m",
                                              "max_forward_m_s",
                                              "max_backward_m_s",
                                              "altitude_gain",
                                              "max_vertical_m_s",
                                              "edge_margin_fraction",
                                              "command_ttl_ms",
                                              "max_pose_age_ms",
                                              "input_score_floor",
                                              "bytetrack_high_score",
                                              "new_track_score",
                                              "lost_track_ttl_s",
                                              "max_prediction_gap_s",
                                              "max_observation_age_ms",
                                              "max_tracks_age_ms",
                                              "enable_rotation_cmc",
                                              "max_frame_interval_ms",
                                              "max_rotation_deg",
                                              "control_period_ms",
                                              "prestream_setpoint_count",
                                              "prestream_interval_ms",
                                              "offboard_start_timeout_ms",
                                              "hold_confirm_timeout_ms",
                                              "watchdog_period_ms",
                                              "enable_target_recovery",
                                              "target_recovery_max_gap_ms",
                                              "target_recovery_confirmation_frames",
                                              "target_recovery_center_gate_diagonals",
                                              "target_recovery_min_area_ratio",
                                              "target_recovery_max_area_ratio",
                                              "allow_recovery_during_priming",
                                              "external_tracks",
                                              "diagnostics_csv",
                                              "target_recovery_ground_gate_m",
                                              "max_horizontal_accel_m_s2",
                                              "max_yaw_accel_deg_s2",
                                              "ground_vector_follow",
                                              "motion_ground_follow",
                                              "ground_center_offset_m",
                                              "allow_competitor_departure",
                                              "prefer_selected_continuity",
                                              "trust_selected_id",
                                              "selected_box_jump_guard",
                                              "target_recovery_gate_growth_m_s",
                                              "target_recovery_gate_max_m",
                                              "trusted_return_check_ms",
                                              "thermal_identity",
                                              "thermal_identity_log",
                                              "thermal_min_contrast",
                                              "v35_thermal_anchor_gate",
                                              "v36_identity_context",
                                              "v36_new_id_relaxed",
                                              "v32_cv_ten_frame",
                                              "v32_sent_command_slew",
                                              "motion_history_s",
                                              "motion_history_min_s",
                                              "motion_history_min_samples",
                                              "motion_fit_max_rmse_m",
                                              "motion_fit_acceleration",
                                              "observed_path_rear_follow",
                                              "rear_path_sample_spacing_m",
                                              "short_motion_enabled",
                                              "short_motion_window_s",
                                              "short_motion_min_span_s",
                                              "short_motion_min_samples",
                                              "short_motion_confirmation_s",
                                              "short_motion_stop_speed_m_s",
                                              "short_motion_resume_speed_m_s",
                                              "short_motion_max_rmse_m",
                                              "yaw_only",
                                              "yaw_only_bearing_deadband_deg",
                                              "pixeagle_pid_enabled",
                                              "pid_forward_gains",
                                              "pid_right_gains",
                                              "pid_yaw_gains",
                                              "pid_altitude_gains",
                                              "pid_anti_windup",
                                              "pid_back_calculation",
                                              "pid_yaw_smoothing_enabled",
                                              "pid_yaw_deadzone_deg_s",
                                              "pid_yaw_max_change_deg_s2",
                                              "pid_yaw_alpha",
                                              "pid_yaw_speed_scaling",
                                              "pid_yaw_min_speed_m_s",
                                              "pid_yaw_max_speed_m_s",
                                              "pid_yaw_low_speed_factor",
                                              "camera_motion_guard",
                                              "fov_velocity_p_acc",
                                              "ground_contact_anchor",
                                              "body_standoff_follow",
                                              "velocity_feedforward_follow",
                                              "max_backward_accel_m_s2",
                                              "reverse_range_margin_m",
                                              "camera_descent_m",
                                              "camera_min_height_m",
                                              "follow_side_offset_m",
                                              "reset_motion_on_stop_or_reverse",
                                              "predictive_follow",
                                              "coherent_ground_anchor_motion",
                                              "predictive_min_height_m",
                                              "predictive_max_height_m",
                                              "outdoor_profile",
                                              "follow_controller_profile",
                                              "predictive_target_height_m",
                                              "predictive_height_anchor_m",
                                              "predictive_height_anchor_weight",
                                              "predictive_height_anchor_deadband_m",
                                              "predictive_vertical_deadband_m_s",
                                              "predictive_vertical_deadband_release_m",
                                              "predictive_vertical_release_max_m_s",
                                              "v35_gap_cv_feedforward",
                                              "v35_slew_reconcile",
                                              "v35_gap_invalid_hover",
                                              "v35_cv_reference_refresh",
                                              "v35_point_range_deadband_m",
                                              "v35_point_range_near_deadband_m",
                                              "v35_point_bottom_margin",
                                              "v35_point_tangential_weight",
                                              "pampc_plant_horizontal_response",
                                              "pampc_plant_vertical_response",
                                              "pampc_use_producer_ground",
                                              "pampc_model_log",
                                              "pampc_producer_ground_max_deviation_m",
                                              "pampc_producer_ground_hold_s",
                                              "pampc_ground_datum_rate_m_s",
                                              "pampc_ceiling_margin_m",
                                              "pampc_fixed_yaw_follow",
                                              "pampc_fixed_yaw_realign_deg",
                                              "pampc_fixed_yaw_realign_rate_deg_s",
                                              "v35_bearing_realign_s",
                                              "v35_gap_yaw_decay_s",
                                              "outdoor_image_pid",
                                              "image_center_deadzone_fraction",
                                              "estimated_speed_deadzone_m_s",
                                              "v3_pampc_point_profile",
                                              "v3_image_velocity_weight",
                                              "v3_predictive_integral",
                                              "v3_discard_recheck",
                                              "v3_box_motion_guard",
                                              "v3_fresh_contact_velocity",
                                              "short_gap_hover_ms",
                                              "yaw_only_altitude_hold",
                                              "yaw_only_min_height_m",
                                              "yaw_only_max_height_m"}};
Values read_values(std::istream& input) {
    // 1. 지원 key=value만 읽고 중복과 문법 오류를 거부한다.
    Values values;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(std::move(line));
        if (line.empty()) {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::invalid_argument("expected key=value at config line " +
                                        std::to_string(line_number));
        }
        const auto key = trim(line.substr(0, separator));
        const auto value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            throw std::invalid_argument("empty key or value at config line " +
                                        std::to_string(line_number));
        }
        if (std::find(kKeys.begin(), kKeys.end(), key) == kKeys.end()) {
            throw std::invalid_argument("unsupported config key: " + key);
        }
        if (!values.emplace(key, value).second) {
            throw std::invalid_argument("duplicate config key: " + key);
        }
    }
    return values;
}

void configure_geometry(BusFollowingConfig& config, const Values& values) {
    config.geometry.image_width = static_cast<std::uint32_t>(integer(values, "image_width"));
    config.geometry.image_height = static_cast<std::uint32_t>(integer(values, "image_height"));
    config.geometry.fx = number(values, "fx");
    config.geometry.fy = number(values, "fy");
    config.geometry.cx = number(values, "cx");
    config.geometry.cy = number(values, "cy");
    config.geometry.camera_to_body = number_array<9>(values, "camera_to_body");
    config.geometry.camera_offset_body_m = number_array<3>(values, "camera_offset_body_m");
    config.geometry.box_anchor_x = number(values, "box_anchor_x");
    config.geometry.box_anchor_y = number(values, "box_anchor_y");
    config.geometry.target_anchor_height_above_ground_m =
        number(values, "target_anchor_height_above_ground_m");
    config.geometry.max_pose_bracket_distance = milliseconds(values, "max_pose_bracket_ms");
    config.geometry.max_pose_receive_age = milliseconds(values, "max_pose_receive_age_ms");
    config.runtime.ground_down_m = number(values, "ground_down_m");
    config.runtime.hold_down_m = number(values, "hold_down_m");
}

void configure_guidance(BusFollowingConfig& config, const Values& values) {
    config.guidance.desired_range_m = number(values, "desired_range_m");
    config.guidance.range_deadband_m = number(values, "range_deadband_m");
    config.guidance.yaw_kp_deg_s_per_deg = number(values, "yaw_kp_deg_s_per_deg");
    config.guidance.max_yaw_rate_deg_s = number(values, "max_yaw_rate_deg_s");
    config.guidance.yaw_alignment_deg = number(values, "yaw_alignment_deg");
    config.guidance.forward_gain_m_s_per_m = number(values, "forward_gain_m_s_per_m");
    config.guidance.max_forward_m_s = number(values, "max_forward_m_s");
    config.guidance.altitude_gain = number(values, "altitude_gain");
    config.guidance.max_vertical_m_s = number(values, "max_vertical_m_s");
    config.guidance.edge_margin_fraction = number(values, "edge_margin_fraction");
    config.guidance.command_ttl = milliseconds(values, "command_ttl_ms");
    config.guidance.max_pose_age = milliseconds(values, "max_pose_age_ms");
    config.guidance.pid_camera = config.geometry;
    config.guidance.pid_ground_down_m = config.runtime.ground_down_m;
    if (values.count("camera_motion_guard")) {
        config.guidance.camera_motion_guard = boolean(values, "camera_motion_guard");
    }
    if (values.count("ground_contact_anchor")) {
        config.guidance.ground_contact_anchor = boolean(values, "ground_contact_anchor");
    }
    if (values.count("fov_velocity_p_acc")) {
        config.guidance.fov_velocity_p_acc = number(values, "fov_velocity_p_acc");
    }
    auto& pid = config.guidance.pid;
    if (values.count("pixeagle_pid_enabled")) {
        pid.enabled = boolean(values, "pixeagle_pid_enabled");
        if (pid.enabled) {
            const auto gains = [&](const char* key) {
                const auto g = number_array<3>(values, key);
                return PidGains{g[0], g[1], g[2]};
            };
            pid.forward = gains("pid_forward_gains");
            pid.right = gains("pid_right_gains");
            pid.yaw = gains("pid_yaw_gains");
            pid.altitude = gains("pid_altitude_gains");
            pid.anti_windup = boolean(values, "pid_anti_windup");
            pid.back_calculation = number(values, "pid_back_calculation");
            auto& y = pid.yaw_smoothing;
            y.enabled = boolean(values, "pid_yaw_smoothing_enabled");
            y.deadzone_deg_s = number(values, "pid_yaw_deadzone_deg_s");
            y.maximum_change_deg_s2 = number(values, "pid_yaw_max_change_deg_s2");
            y.alpha = number(values, "pid_yaw_alpha");
            y.speed_scaling = boolean(values, "pid_yaw_speed_scaling");
            y.minimum_speed_m_s = number(values, "pid_yaw_min_speed_m_s");
            y.maximum_speed_m_s = number(values, "pid_yaw_max_speed_m_s");
            y.low_speed_factor = number(values, "pid_yaw_low_speed_factor");
            config.guidance.pid_camera = config.geometry;
            config.guidance.pid_ground_down_m = config.runtime.ground_down_m;
        }
    }
}
}  // namespace

BusFollowingConfig parse_bus_following_config(std::istream& input) {
    const auto values = read_values(input);

    // 2. 통신 주소 → 추적 → 보정 → 제어 → CMC → 시간 설정 순서로 조립한다.
    BusFollowingConfig config;
    config.observations_sub_endpoint = string_value(values, "observations_sub_endpoint");
    config.tracks_pub_endpoint = string_value(values, "tracks_pub_endpoint");
    config.tracks_sub_endpoint = string_value(values, "tracks_sub_endpoint");
    config.status_pub_endpoint = string_value(values, "status_pub_endpoint");
    config.command_rep_endpoint = string_value(values, "command_rep_endpoint");
    config.px4_connection_url = string_value(values, "px4_connection_url");
    if (values.count("expected_fc_uid")) {
        config.expected_fc_uid = string_value(values, "expected_fc_uid");
    }
    for (auto key : {"expected_fc_system_id", "expected_fc_component_id"}) {
        if (values.count(key)) {
            const auto value = integer(values, key);
            if (value < 1 || value > 255) {
                throw std::invalid_argument("expected FC source ID must be 1..255");
            }
            if (std::string(key) == "expected_fc_system_id") {
                config.expected_fc_system_id = value;
            } else {
                config.expected_fc_component_id = value;
            }
        }
    }
    if (config.expected_fc_uid.size() != 36 ||
        config.expected_fc_uid.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
        throw std::invalid_argument("expected FC uid2 must be 18-byte hex");
    }
    std::transform(config.expected_fc_uid.begin(), config.expected_fc_uid.end(),
                   config.expected_fc_uid.begin(), [](unsigned char c) { return std::tolower(c); });

    const auto classes = class_ids(values);
    config.inbox.filter.accepted_class_ids = classes;
    config.inbox.capacity = 2;
    config.tracker.accepted_class_ids = classes;
    config.tracker.input_score_floor = static_cast<float>(number(values, "input_score_floor"));
    config.tracker.thresholds.high_score =
        static_cast<float>(number(values, "bytetrack_high_score"));
    config.tracker.thresholds.new_track_score =
        static_cast<float>(number(values, "new_track_score"));
    config.tracker.lost_track_ttl_s = number(values, "lost_track_ttl_s");
    config.tracker.max_prediction_gap_s = number(values, "max_prediction_gap_s");
    config.selector.max_tracks_age = milliseconds(values, "max_tracks_age_ms");
    config.selector.lost_track_ttl_s = config.tracker.lost_track_ttl_s;
    // 기존 설정 파일은 그대로 읽는다. 새 옵션을 명시한 경우에만 기본값을 덮어쓴다.
    auto& recovery = config.selector.recovery;
    if (values.count("enable_target_recovery")) {
        recovery.enabled = boolean(values, "enable_target_recovery");
    }
    if (values.count("target_recovery_max_gap_ms")) {
        recovery.max_gap = milliseconds(values, "target_recovery_max_gap_ms");
    }
    if (values.count("target_recovery_confirmation_frames")) {
        recovery.confirmation_frames =
            static_cast<std::uint32_t>(integer(values, "target_recovery_confirmation_frames"));
    }
    if (values.count("target_recovery_center_gate_diagonals")) {
        recovery.max_center_distance_diagonals =
            number(values, "target_recovery_center_gate_diagonals");
    }
    if (values.count("target_recovery_min_area_ratio")) {
        recovery.min_area_ratio = number(values, "target_recovery_min_area_ratio");
    }
    if (values.count("target_recovery_max_area_ratio")) {
        recovery.max_area_ratio = number(values, "target_recovery_max_area_ratio");
    }
    if (values.count("target_recovery_ground_gate_m")) {
        recovery.ground_gate_m = number(values, "target_recovery_ground_gate_m");
    }
    // 2026-11-10 가림 대응 (값 검사는 TargetManager가 한다)
    if (values.count("target_recovery_gate_growth_m_s")) {
        recovery.ground_gate_growth_m_s = number(values, "target_recovery_gate_growth_m_s");
    }
    if (values.count("target_recovery_gate_max_m")) {
        recovery.ground_gate_max_m = number(values, "target_recovery_gate_max_m");
    }
    if (values.count("trusted_return_check_ms")) {
        recovery.trusted_return_check = milliseconds(values, "trusted_return_check_ms");
    }
    if (values.count("allow_recovery_during_priming")) {
        config.runtime.allow_recovery_during_priming =
            boolean(values, "allow_recovery_during_priming");
    }
    if (values.count("external_tracks")) {
        config.external_tracks = boolean(values, "external_tracks");
    }
    if (values.count("diagnostics_csv")) {
        config.diagnostics_csv = string_value(values, "diagnostics_csv");
    }
    if (values.count("trust_selected_id")) {
        recovery.trust_selected_id = boolean(values, "trust_selected_id");
    }
    if (values.count("selected_box_jump_guard")) {
        recovery.selected_box_jump_guard = boolean(values, "selected_box_jump_guard");
    }
    if (values.count("thermal_identity") != 0) {
        const std::string mode = string_value(values, "thermal_identity");
        if (mode == "off") {
            recovery.thermal_identity = ThermalIdentityMode::Off;
        } else if (mode == "log") {
            recovery.thermal_identity = ThermalIdentityMode::Log;
        } else if (mode == "on") {
            recovery.thermal_identity = ThermalIdentityMode::On;
        } else {
            throw std::invalid_argument("thermal_identity must be off, log or on");
        }
    }
    if (values.count("thermal_identity_log") != 0) {
        recovery.thermal_identity_log = values.at("thermal_identity_log");
    }
    if (values.count("v35_thermal_anchor_gate") != 0) {
        recovery.v35_thermal_anchor_gate = boolean(values, "v35_thermal_anchor_gate");
    }
    // 2026-11-13 신원 판단 문맥 (target_memory.hpp 설명). trust_selected_id에서만 의미가 있다.
    if (values.count("v36_identity_context") != 0) {
        recovery.v36_identity_context = boolean(values, "v36_identity_context");
    }
    if (values.count("v36_new_id_relaxed") != 0) {
        recovery.v36_new_id_relaxed = boolean(values, "v36_new_id_relaxed");
        if (recovery.v36_new_id_relaxed && !recovery.v36_identity_context) {
            throw std::invalid_argument("v36_new_id_relaxed requires v36_identity_context");
        }
    }
    if (values.count("thermal_min_contrast") != 0) {
        recovery.thermal_min_contrast = std::stod(string_value(values, "thermal_min_contrast"));
        if (!std::isfinite(recovery.thermal_min_contrast) || recovery.thermal_min_contrast <= 0) {
            throw std::invalid_argument("thermal_min_contrast must be positive");
        }
    }
    if (values.count("v32_cv_ten_frame")) {
        recovery.v32_cv_ten_frame = boolean(values, "v32_cv_ten_frame");
    }
    if (values.count("v32_sent_command_slew")) {
        config.guidance.v32_sent_command_slew = boolean(values, "v32_sent_command_slew");
    }
    if (values.count("short_gap_hover_ms") != 0) {
        // 선택 키. 없으면 기본 3000ms. 0은 기존 동작(공백 즉시 HOLD)이라 milliseconds()의
        // 양수 검사 대신 정수로 읽는다.
        const auto hover = integer(values, "short_gap_hover_ms");
        if (hover < 0 || hover > 3000) {
            throw std::invalid_argument("short_gap_hover_ms must be 0..3000");
        }
        config.guidance.short_gap_hover = std::chrono::milliseconds{hover};
    }
    if (values.count("allow_competitor_departure")) {
        recovery.allow_competitor_departure = boolean(values, "allow_competitor_departure");
    }
    if (values.count("prefer_selected_continuity")) {
        recovery.prefer_selected_continuity = boolean(values, "prefer_selected_continuity");
    }
    recovery.motion_max_sample_gap = config.selector.max_tracks_age;
    if (values.count("motion_history_s")) {
        recovery.motion_history_s = number(values, "motion_history_s");
    }
    if (values.count("motion_history_min_s")) {
        recovery.motion_history_min_s = number(values, "motion_history_min_s");
    }
    if (values.count("motion_history_min_samples")) {
        recovery.motion_history_min_samples =
            static_cast<std::uint32_t>(integer(values, "motion_history_min_samples"));
    }
    if (values.count("motion_fit_max_rmse_m")) {
        recovery.motion_fit_max_rmse_m = number(values, "motion_fit_max_rmse_m");
    }
    if (values.count("motion_fit_acceleration")) {
        recovery.motion_fit_acceleration = boolean(values, "motion_fit_acceleration");
    }
    auto& short_motion = recovery.short_motion;
    short_motion.maximum_sample_gap_s =
        std::chrono::duration<double>(config.selector.max_tracks_age).count();
    if (values.count("short_motion_enabled")) {
        short_motion.enabled = boolean(values, "short_motion_enabled");
    }
    if (values.count("short_motion_window_s")) {
        short_motion.window_s = number(values, "short_motion_window_s");
    }
    if (values.count("short_motion_min_span_s")) {
        short_motion.minimum_span_s = number(values, "short_motion_min_span_s");
    }
    if (values.count("short_motion_min_samples")) {
        short_motion.minimum_samples =
            static_cast<std::uint32_t>(integer(values, "short_motion_min_samples"));
    }
    if (values.count("short_motion_confirmation_s")) {
        short_motion.confirmation_s = number(values, "short_motion_confirmation_s");
    }
    if (values.count("short_motion_stop_speed_m_s")) {
        short_motion.stop_speed_m_s = number(values, "short_motion_stop_speed_m_s");
    }
    if (values.count("short_motion_resume_speed_m_s")) {
        short_motion.resume_speed_m_s = number(values, "short_motion_resume_speed_m_s");
    }
    if (values.count("short_motion_max_rmse_m")) {
        short_motion.maximum_rmse_m = number(values, "short_motion_max_rmse_m");
    }
    if (values.count("ground_contact_anchor")) {
        recovery.ground_contact_anchor = boolean(values, "ground_contact_anchor");
    }
    (void)TargetManager(recovery);  // 런타임과 같은 범위 검사를 적용한다.

    configure_geometry(config, values);

    configure_guidance(config, values);
    if (values.count("yaw_only")) {
        config.guidance.yaw_only = boolean(values, "yaw_only");
    }
    if (values.count("yaw_only_bearing_deadband_deg")) {
        config.guidance.yaw_only_bearing_deadband_deg =
            number(values, "yaw_only_bearing_deadband_deg");
    }
    if (config.guidance.yaw_only) {
        if (!config.external_tracks) {
            throw std::invalid_argument("yaw-only requires explicit external UCMC tracks");
        }
        config.selector.require_local_monotonic_sample = true;
    }
    if (values.count("ground_vector_follow")) {
        config.guidance.ground_vector_follow = boolean(values, "ground_vector_follow");
    }
    if (values.count("motion_ground_follow")) {
        config.guidance.motion_ground_follow = boolean(values, "motion_ground_follow");
    }
    if (values.count("ground_center_offset_m")) {
        config.guidance.ground_center_offset_m = number(values, "ground_center_offset_m");
    }
    if (values.count("body_standoff_follow")) {
        config.guidance.body_standoff_follow = boolean(values, "body_standoff_follow");
    }
    if (values.count("velocity_feedforward_follow")) {
        config.guidance.velocity_feedforward_follow =
            boolean(values, "velocity_feedforward_follow");
    }
    if (values.count("max_backward_m_s")) {
        config.guidance.max_backward_m_s = number(values, "max_backward_m_s");
    }
    if (values.count("reverse_range_margin_m")) {
        config.guidance.reverse_range_margin_m = number(values, "reverse_range_margin_m");
    }
    if (values.count("camera_descent_m")) {
        config.guidance.camera_descent_m = number(values, "camera_descent_m");
    }
    if (values.count("camera_min_height_m")) {
        config.guidance.camera_min_height_m = number(values, "camera_min_height_m");
    }
    if (config.guidance.camera_min_height_m >
        config.runtime.ground_down_m - config.runtime.hold_down_m) {
        throw std::invalid_argument("camera minimum height exceeds nominal height");
    }
    if (config.guidance.camera_descent_m >=
            config.runtime.ground_down_m - config.runtime.hold_down_m &&
        config.guidance.camera_descent_m > 0) {
        throw std::invalid_argument("camera descent must leave positive ground clearance");
    }
    config.selector.use_fitted_ground_position = !config.guidance.pid.enabled;
    if (values.count("observed_path_rear_follow")) {
        config.guidance.observed_path_rear_follow = boolean(values, "observed_path_rear_follow");
    }
    if (values.count("rear_path_sample_spacing_m")) {
        config.guidance.rear_path_sample_spacing_m = number(values, "rear_path_sample_spacing_m");
    }
    if (values.count("max_horizontal_accel_m_s2")) {
        config.guidance.max_horizontal_accel_m_s2 = number(values, "max_horizontal_accel_m_s2");
    }
    if (values.count("max_yaw_accel_deg_s2")) {
        config.guidance.max_yaw_accel_deg_s2 = number(values, "max_yaw_accel_deg_s2");
    }
    if (values.count("max_backward_accel_m_s2")) {
        config.guidance.max_backward_accel_m_s2 = number(values, "max_backward_accel_m_s2");
    }
    if (config.guidance.max_horizontal_accel_m_s2 < 0 || config.guidance.max_yaw_accel_deg_s2 < 0) {
        throw std::invalid_argument("acceleration limits cannot be negative");
    }

    config.guidance.reset_motion_on_stop_or_reverse = recovery.reset_motion_on_stop_or_reverse;
    if (values.count("follow_side_offset_m")) {
        config.guidance.follow_side_offset_m = number(values, "follow_side_offset_m");
    }
    if (values.count("reset_motion_on_stop_or_reverse")) {
        recovery.reset_motion_on_stop_or_reverse =
            boolean(values, "reset_motion_on_stop_or_reverse");
    }
    if (values.count("predictive_follow")) {
        config.guidance.predictive_follow = boolean(values, "predictive_follow");
    }
    if (values.count("coherent_ground_anchor_motion")) {
        recovery.coherent_ground_anchor_motion = boolean(values, "coherent_ground_anchor_motion");
    }
    if (values.count("predictive_min_height_m")) {
        config.guidance.predictive_min_height_m = number(values, "predictive_min_height_m");
    }
    if (values.count("predictive_max_height_m")) {
        config.guidance.predictive_max_height_m = number(values, "predictive_max_height_m");
    }
    if (values.count("outdoor_profile")) {
        config.guidance.outdoor_profile = boolean(values, "outdoor_profile");
    }
    if (values.count("follow_controller_profile") != 0) {
        config.follow_controller_profile = string_value(values, "follow_controller_profile");
        if (!valid_follow_controller_profile(config.follow_controller_profile)) {
            throw std::invalid_argument(
                "follow_controller_profile must be predictive, predictive_low, predictive_m30, "
                "predictive_low_m30, demo or demo2");
        }
        const bool demo = demo_follow_controller_profile(config.follow_controller_profile);
        // 2026-11-18: 실내는 기본(predictive)과 데모 모드(demo), 실외는 기존 4종만 쓴다.
        // 2026-11-19: 실내 데모2(demo2, yaw 고정 이동 추종)도 실내에서만 쓴다.
        const bool demo2 = demo2_follow_controller_profile(config.follow_controller_profile);
        if (!config.guidance.outdoor_profile && config.follow_controller_profile != "predictive" &&
            !demo && !demo2) {
            throw std::invalid_argument(
                "indoor profiles must use follow_controller_profile=predictive, demo or demo2");
        }
        if (config.guidance.outdoor_profile && demo2) {
            throw std::invalid_argument("demo2 follow_controller_profile is indoor only");
        }
        if (config.guidance.outdoor_profile && demo) {
            throw std::invalid_argument("demo follow_controller_profile is indoor only");
        }
        if (demo && !config.guidance.yaw_only) {
            throw std::invalid_argument("demo follow_controller_profile requires yaw_only=true");
        }
    }
    // 2026-11-18 데모 모드 고도 유지: yaw 전용 추종 중에만 명목 높이(ground_down_m - hold_down_m)로
    // 수직 속도를 낸다. 수평 이동은 계속 0이다. 범위는 송신 직전 검사에도 쓴다.
    if (values.count("yaw_only_altitude_hold") != 0) {
        config.guidance.yaw_only_altitude_hold = boolean(values, "yaw_only_altitude_hold");
    }
    const bool band_keys =
        values.count("yaw_only_min_height_m") != 0 || values.count("yaw_only_max_height_m") != 0;
    if ((config.guidance.yaw_only_altitude_hold || band_keys) && !config.guidance.yaw_only) {
        throw std::invalid_argument("yaw-only altitude hold keys require yaw_only=true");
    }
    if (band_keys && !config.guidance.yaw_only_altitude_hold) {
        throw std::invalid_argument(
            "yaw_only_min/max_height_m require yaw_only_altitude_hold=true");
    }
    if (config.guidance.yaw_only_altitude_hold) {
        const double low = number(values, "yaw_only_min_height_m");
        const double high = number(values, "yaw_only_max_height_m");
        const double nominal = config.runtime.ground_down_m - config.runtime.hold_down_m;
        if (!(low > 0) || !(low < nominal) || !(nominal < high)) {
            throw std::invalid_argument(
                "yaw-only altitude hold requires 0 < min < nominal height < max");
        }
        if (!config.guidance.outdoor_profile && (low < 0.5 || high > 2.0)) {
            throw std::invalid_argument(
                "indoor yaw-only altitude hold band must stay within 0.5..2.0 m");
        }
        if (!(config.guidance.altitude_gain > 0) || !(config.guidance.max_vertical_m_s > 0)) {
            throw std::invalid_argument(
                "yaw-only altitude hold requires positive altitude_gain and max_vertical_m_s");
        }
        config.guidance.yaw_only_min_height_m = low;
        config.guidance.yaw_only_max_height_m = high;
    }
    if (values.count("estimated_speed_deadzone_m_s")) {
        const double threshold = number(values, "estimated_speed_deadzone_m_s");
        if (threshold < 0) {
            throw std::invalid_argument("speed deadzone must be nonnegative");
        }
        recovery.estimated_speed_deadzone_m_s = threshold;
    }
    if (values.count("predictive_target_height_m")) {
        config.guidance.predictive_target_height_m = number(values, "predictive_target_height_m");
    }
    // 2026-11-09 PAMPC point 고도 고정항·수직 무시 구간 (값 검사는 PredictiveFollower가 한다)
    if (values.count("predictive_height_anchor_m")) {
        config.guidance.predictive_height_anchor_m = number(values, "predictive_height_anchor_m");
    }
    if (values.count("predictive_height_anchor_weight")) {
        config.guidance.predictive_height_anchor_weight =
            number(values, "predictive_height_anchor_weight");
    }
    if (values.count("predictive_height_anchor_deadband_m")) {
        config.guidance.predictive_height_anchor_deadband_m =
            number(values, "predictive_height_anchor_deadband_m");
    }
    if (values.count("predictive_vertical_release_max_m_s")) {
        config.guidance.predictive_vertical_release_max_m_s =
            number(values, "predictive_vertical_release_max_m_s");
    }
    if (values.count("predictive_vertical_deadband_release_m")) {
        config.guidance.predictive_vertical_deadband_release_m =
            number(values, "predictive_vertical_deadband_release_m");
    }
    if (values.count("predictive_vertical_deadband_m_s")) {
        config.guidance.predictive_vertical_deadband_m_s =
            number(values, "predictive_vertical_deadband_m_s");
    }
    if (values.count("v35_gap_cv_feedforward")) {
        config.guidance.v35_gap_cv_feedforward = boolean(values, "v35_gap_cv_feedforward");
    }
    if (values.count("v35_slew_reconcile")) {
        config.guidance.v35_slew_reconcile = boolean(values, "v35_slew_reconcile");
    }
    if (values.count("v35_gap_invalid_hover")) {
        config.guidance.v35_gap_invalid_hover = boolean(values, "v35_gap_invalid_hover");
    }
    if (values.count("v35_point_range_deadband_m")) {
        config.guidance.v35_point_range_deadband_m = number(values, "v35_point_range_deadband_m");
    }
    if (values.count("v35_point_bottom_margin")) {
        config.guidance.v35_point_bottom_margin = number(values, "v35_point_bottom_margin");
    }
    if (values.count("v35_point_range_near_deadband_m")) {
        config.guidance.v35_point_range_near_deadband_m =
            number(values, "v35_point_range_near_deadband_m");
    }
    if (values.count("v35_point_tangential_weight")) {
        config.guidance.v35_point_tangential_weight = number(values, "v35_point_tangential_weight");
    }
    // 2026-11-19 PAMPC #3 rollout 기체 응답 이득, #5 producer 지면 기준 (모두 기본 꺼짐).
    // 범위 검사는 PredictiveFollower가 한다. 예측 추종(predictive_follow)에서만 의미가 있다.
    const bool pampc_model_keys = values.count("pampc_plant_horizontal_response") != 0 ||
                                  values.count("pampc_plant_vertical_response") != 0 ||
                                  values.count("pampc_use_producer_ground") != 0 ||
                                  values.count("pampc_producer_ground_max_deviation_m") != 0 ||
                                  values.count("pampc_producer_ground_hold_s") != 0 ||
                                  values.count("pampc_ground_datum_rate_m_s") != 0 ||
                                  values.count("pampc_ceiling_margin_m") != 0;
    if (values.count("pampc_plant_horizontal_response")) {
        config.guidance.pampc_plant_horizontal_response =
            number(values, "pampc_plant_horizontal_response");
    }
    if (values.count("pampc_plant_vertical_response")) {
        config.guidance.pampc_plant_vertical_response =
            number(values, "pampc_plant_vertical_response");
    }
    if (values.count("pampc_use_producer_ground")) {
        config.guidance.pampc_use_producer_ground = boolean(values, "pampc_use_producer_ground");
    }
    // 2026-11-19 검토 수정 안전 상수 (범위 검사는 PredictiveFollower, 기능이 켜져야 작용).
    if (values.count("pampc_producer_ground_max_deviation_m")) {
        config.guidance.pampc_producer_ground_max_deviation_m =
            number(values, "pampc_producer_ground_max_deviation_m");
    }
    if (values.count("pampc_producer_ground_hold_s")) {
        config.guidance.pampc_producer_ground_hold_s =
            number(values, "pampc_producer_ground_hold_s");
    }
    if (values.count("pampc_ground_datum_rate_m_s")) {
        config.guidance.pampc_ground_datum_rate_m_s = number(values, "pampc_ground_datum_rate_m_s");
    }
    if (values.count("pampc_ceiling_margin_m")) {
        config.guidance.pampc_ceiling_margin_m = number(values, "pampc_ceiling_margin_m");
    }
    // 진단 기록 스위치는 비예측 설정에도 둘 수 있다 (기록할 예측 tick이 없을 뿐이다).
    if (values.count("pampc_model_log")) {
        config.guidance.pampc_model_log = boolean(values, "pampc_model_log");
    }
    // 2026-11-19 실내 데모2 yaw 고정 추종 (범위 검사는 PredictiveFollower). demo2 이동 설정에서만
    // 켠다.
    if (values.count("pampc_fixed_yaw_follow")) {
        config.guidance.pampc_fixed_yaw_follow = boolean(values, "pampc_fixed_yaw_follow");
    }
    if (values.count("pampc_fixed_yaw_realign_deg")) {
        config.guidance.pampc_fixed_yaw_realign_deg = number(values, "pampc_fixed_yaw_realign_deg");
    }
    if (values.count("pampc_fixed_yaw_realign_rate_deg_s")) {
        config.guidance.pampc_fixed_yaw_realign_rate_deg_s =
            number(values, "pampc_fixed_yaw_realign_rate_deg_s");
    }
    {
        const bool demo2 = demo2_follow_controller_profile(config.follow_controller_profile);
        if (config.guidance.pampc_fixed_yaw_follow &&
            (!demo2 || !config.guidance.predictive_follow || config.guidance.yaw_only)) {
            throw std::invalid_argument(
                "pampc_fixed_yaw_follow requires follow_controller_profile=demo2 and "
                "predictive_follow=true");
        }
        if (demo2 && !config.guidance.yaw_only &&
            (!config.guidance.predictive_follow || !config.guidance.pampc_fixed_yaw_follow)) {
            throw std::invalid_argument(
                "demo2 moving profile requires predictive_follow=true and "
                "pampc_fixed_yaw_follow=true");
        }
    }
    if (pampc_model_keys && !config.guidance.predictive_follow) {
        throw std::invalid_argument(
            "pampc_plant_*, pampc_use_producer_ground and producer ground constants require "
            "predictive_follow=true");
    }
    if (values.count("v35_bearing_realign_s")) {
        config.guidance.v35_bearing_realign_s = number(values, "v35_bearing_realign_s");
    }
    if (values.count("v35_gap_yaw_decay_s")) {
        config.guidance.v35_gap_yaw_decay_s = number(values, "v35_gap_yaw_decay_s");
    }
    if (values.count("v35_cv_reference_refresh")) {
        recovery.v35_cv_reference_refresh = boolean(values, "v35_cv_reference_refresh");
    }
    if (values.count("outdoor_image_pid")) {
        config.guidance.outdoor_image_pid = boolean(values, "outdoor_image_pid");
    }
    if (values.count("image_center_deadzone_fraction")) {
        config.guidance.image_center_deadzone_fraction =
            number(values, "image_center_deadzone_fraction");
    }
    // V3 (userpc SITL 검증본) 추종 기능. 실내·실외 기존 방식(predictive) 설정에서만 쓴다.
    if (values.count("v3_pampc_point_profile")) {
        config.guidance.v3_pampc_point_profile =
            static_cast<int>(integer(values, "v3_pampc_point_profile"));
    }
    if (values.count("v3_image_velocity_weight")) {
        config.guidance.v3_image_velocity_weight = number(values, "v3_image_velocity_weight");
    }
    if (values.count("v3_predictive_integral")) {
        config.guidance.v3_predictive_integral = boolean(values, "v3_predictive_integral");
    }
    if (values.count("v3_discard_recheck")) {
        config.guidance.v3_discard_recheck = boolean(values, "v3_discard_recheck");
    }
    if (values.count("v3_box_motion_guard")) {
        recovery.v3_box_motion_guard = boolean(values, "v3_box_motion_guard");
    }
    if (values.count("v3_fresh_contact_velocity")) {
        recovery.v3_fresh_contact_velocity = boolean(values, "v3_fresh_contact_velocity");
        if (recovery.v3_fresh_contact_velocity && !recovery.v3_box_motion_guard) {
            throw std::invalid_argument("v3_fresh_contact_velocity requires v3_box_motion_guard");
        }
    }
    if ((recovery.v3_box_motion_guard || recovery.v3_fresh_contact_velocity) &&
        (!config.guidance.predictive_follow || config.guidance.outdoor_image_pid ||
         config.guidance.yaw_only)) {
        throw std::invalid_argument("V3 target options require the predictive (non-PID) profile");
    }
    config.guidance.reset_motion_on_stop_or_reverse = recovery.reset_motion_on_stop_or_reverse;
    (void)FlightGuidanceController(config.guidance);

    config.enable_rotation_cmc = boolean(values, "enable_rotation_cmc");
    config.rotation.calibration_width = config.geometry.image_width;
    config.rotation.calibration_height = config.geometry.image_height;
    config.rotation.fx = config.geometry.fx;
    config.rotation.fy = config.geometry.fy;
    config.rotation.cx = config.geometry.cx;
    config.rotation.cy = config.geometry.cy;
    config.rotation.camera_to_body = config.geometry.camera_to_body;
    const double max_frame_interval_ms = number(values, "max_frame_interval_ms");
    config.rotation.max_frame_interval_s = max_frame_interval_ms / 1000.0;
    config.rotation.max_rotation_deg = number(values, "max_rotation_deg");
    config.rotation.max_pose_bracket_distance = config.geometry.max_pose_bracket_distance;
    config.rotation.max_pose_receive_age = config.geometry.max_pose_receive_age;

    config.max_observation_age = milliseconds(values, "max_observation_age_ms");
    config.control_period = milliseconds(values, "control_period_ms");
    config.watchdog_period = milliseconds(values, "watchdog_period_ms");
    config.runtime.prestream_setpoint_count =
        static_cast<std::uint32_t>(integer(values, "prestream_setpoint_count"));
    config.runtime.prestream_interval = milliseconds(values, "prestream_interval_ms");
    config.runtime.offboard_start_timeout = milliseconds(values, "offboard_start_timeout_ms");
    config.runtime.hold_confirm_timeout = milliseconds(values, "hold_confirm_timeout_ms");

    const bool invalid_image =
        config.geometry.image_width == 0 || config.geometry.image_height == 0;
    const bool invalid_control_period = config.control_period < std::chrono::milliseconds(10) ||
                                        config.control_period > std::chrono::seconds(1);
    const bool invalid_observation_age = config.max_observation_age > std::chrono::seconds(5);
    const bool invalid_watchdog_period = config.watchdog_period < std::chrono::milliseconds(20) ||
                                         config.watchdog_period > std::chrono::milliseconds(50);
    const bool invalid_cmc_interval = max_frame_interval_ms <= 0 || max_frame_interval_ms > 5000;
    const bool invalid_prestream_count = config.runtime.prestream_setpoint_count < 1 ||
                                         config.runtime.prestream_setpoint_count > 100;
    if (invalid_image || invalid_control_period || invalid_observation_age ||
        invalid_watchdog_period || invalid_cmc_interval || invalid_prestream_count) {
        throw std::invalid_argument("bus following config value outside supported range");
    }
    if (config.guidance.pid.enabled) {
        if (config.control_period != std::chrono::milliseconds(50)) {
            throw std::invalid_argument("reference PID requires20Hz");
        }
        (void)FlightGuidanceController(config.guidance);
    }
    return config;
}

BusFollowingConfig load_bus_following_config(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open bus following config: " + path);
    }
    return parse_bus_following_config(input);
}
}  // namespace follow
