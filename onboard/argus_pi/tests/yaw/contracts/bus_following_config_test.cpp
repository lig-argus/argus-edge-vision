// 환경별 분기 없는 설정 주입과 필수/중복/미지원 값의 시작 전 거부를 확인한다.
#include "argus/runtime/config/include/bus_following_config.hpp"

#include <cassert>
#include <sstream>
#include <stdexcept>

namespace {
const char kValidConfig[] = R"CFG(
# 테스트 전용 값이며 운용 기본값으로 사용하지 않는다.
observations_sub_endpoint=tcp://127.0.0.1:5555
tracks_pub_endpoint=tcp://127.0.0.1:5556
tracks_sub_endpoint=tcp://127.0.0.1:5557
status_pub_endpoint=tcp://127.0.0.1:5559
command_rep_endpoint=tcp://127.0.0.1:5558
px4_connection_url=udpin://0.0.0.0:14540
target_class_ids=5
image_width=640
image_height=480
fx=500
fy=500
cx=320
cy=240
camera_to_body=0,0,1,1,0,0,0,1,0
camera_offset_body_m=0,0,0
box_anchor_x=0.5
box_anchor_y=1
target_anchor_height_above_ground_m=0
ground_down_m=0
hold_down_m=-5
max_pose_bracket_ms=100
max_pose_receive_age_ms=200
desired_range_m=5
range_deadband_m=0.2
yaw_kp_deg_s_per_deg=2
max_yaw_rate_deg_s=15
yaw_alignment_deg=5
forward_gain_m_s_per_m=0.2
max_forward_m_s=0.3
altitude_gain=0.8
max_vertical_m_s=0.2
edge_margin_fraction=0.05
command_ttl_ms=100
max_pose_age_ms=200
input_score_floor=0.1
bytetrack_high_score=0.3
new_track_score=0.4
lost_track_ttl_s=0.7
max_prediction_gap_s=0.5
max_observation_age_ms=250
max_tracks_age_ms=250
enable_rotation_cmc=false
max_frame_interval_ms=500
max_rotation_deg=35
control_period_ms=50
prestream_setpoint_count=15
prestream_interval_ms=100
offboard_start_timeout_ms=3000
hold_confirm_timeout_ms=2000
watchdog_period_ms=25
)CFG";

void must_reject(const std::string& text) {
    std::istringstream input(text);
    bool rejected = false;
    try {
        (void)follow::parse_bus_following_config(input);
    } catch (const std::exception&) {
        rejected = true;
    }
    assert(rejected);
}
}  // namespace

int main() {
    std::istringstream input(kValidConfig);
    const auto config = follow::parse_bus_following_config(input);
    assert(config.observations_sub_endpoint == "tcp://127.0.0.1:5555");
    assert(config.px4_connection_url == "udpin://0.0.0.0:14540");
    assert(config.inbox.filter.accepted_class_ids == std::vector<std::int32_t>{5});
    assert(config.tracker.accepted_class_ids == config.inbox.filter.accepted_class_ids);
    assert(config.geometry.image_width == 640 && config.geometry.fx == 500);
    assert(config.guidance.max_forward_m_s == 0.3 && config.runtime.prestream_setpoint_count == 15);
    assert(config.watchdog_period == std::chrono::milliseconds(25));
    assert(!config.enable_rotation_cmc);
    std::istringstream yaw_input(
        std::string(kValidConfig) +
        "external_tracks=true\nyaw_only=true\nyaw_only_bearing_deadband_deg=0.7\n");
    const auto yaw = follow::parse_bus_following_config(yaw_input);
    assert(yaw.guidance.yaw_only && yaw.guidance.yaw_only_bearing_deadband_deg == 0.7 &&
           yaw.selector.require_local_monotonic_sample);
    must_reject(std::string(kValidConfig) + "yaw_only=true\n");
    must_reject(std::string(kValidConfig) +
                "external_tracks=true\nyaw_only=true\nground_vector_follow=true\n");
    must_reject(std::string(kValidConfig) +
                "external_tracks=true\nyaw_only=true\nyaw_only_bearing_deadband_deg=-1\n");
    assert(!config.guidance.ground_vector_follow);
    std::istringstream vector_input(std::string(kValidConfig) + "ground_vector_follow=true\n");
    assert(follow::parse_bus_following_config(vector_input).guidance.ground_vector_follow);
    must_reject(std::string(kValidConfig) + "ground_vector_follow=auto\n");
    assert(!config.selector.recovery.enabled);
    assert(!config.selector.recovery.prefer_selected_continuity);
    std::istringstream continuity_input(std::string(kValidConfig) +
                                        "prefer_selected_continuity=true\n");
    assert(follow::parse_bus_following_config(continuity_input)
               .selector.recovery.prefer_selected_continuity);
    must_reject(std::string(kValidConfig) + "prefer_selected_continuity=auto\n");
    assert(config.selector.recovery.max_gap == std::chrono::milliseconds(2000));
    std::istringstream recovery_input(
        std::string(kValidConfig) +
        "enable_target_recovery=true\ntarget_recovery_max_gap_ms=1500\n"
        "target_recovery_confirmation_frames=4\ntarget_recovery_center_gate_diagonals=0.8\n"
        "target_recovery_min_area_ratio=0.6\ntarget_recovery_max_area_ratio=1.8\n");
    const auto recovery = follow::parse_bus_following_config(recovery_input).selector.recovery;
    assert(recovery.enabled && recovery.max_gap == std::chrono::milliseconds(1500));
    assert(recovery.confirmation_frames == 4 && recovery.max_center_distance_diagonals == 0.8);
    assert(recovery.min_area_ratio == 0.6 && recovery.max_area_ratio == 1.8);
    std::istringstream motion_input(
        std::string(kValidConfig) +
        "motion_history_s=2\nmotion_history_min_s=1.5\n"
        "motion_history_min_samples=20\nmotion_fit_max_rmse_m=0.2\nmotion_fit_acceleration=true\n");
    const auto motion = follow::parse_bus_following_config(motion_input).selector.recovery;
    assert(motion.motion_history_s == 2 && motion.motion_history_min_s == 1.5 &&
           motion.motion_history_min_samples == 20);
    assert(motion.motion_fit_max_rmse_m == .2 && motion.motion_fit_acceleration);
    std::istringstream short_input(
        std::string(kValidConfig) +
        "short_motion_enabled=true\nshort_motion_window_s=.6\nshort_motion_min_span_s=.45\nshort_"
        "motion_min_samples=7\nshort_motion_confirmation_s=.2\nshort_motion_stop_speed_m_s=."
        "1\nshort_motion_resume_speed_m_s=.2\nshort_motion_max_rmse_m=.08\n");
    const auto fast =
        follow::parse_bus_following_config(short_input).selector.recovery.short_motion;
    assert(fast.enabled && fast.window_s == .6 && fast.minimum_span_s == .45 &&
           fast.minimum_samples == 7 && fast.confirmation_s == .2 && fast.stop_speed_m_s == .1 &&
           fast.resume_speed_m_s == .2 && fast.maximum_rmse_m == .08);
    for (const char* invalid :
         {"short_motion_window_s=0\n", "short_motion_min_span_s=.5\n",
          "short_motion_min_samples=2\n", "short_motion_confirmation_s=-1\n",
          "short_motion_resume_speed_m_s=.1\n", "short_motion_max_rmse_m=0\n"}) {
        must_reject(std::string(kValidConfig) + invalid);
    }
    std::istringstream path_input(std::string(kValidConfig) +
                                  "ground_vector_follow=true\nmotion_ground_follow=true\nobserved_"
                                  "path_rear_follow=true\nrear_path_sample_spacing_m=.5\n");
    const auto path_config = follow::parse_bus_following_config(path_input);
    assert(path_config.guidance.observed_path_rear_follow &&
           path_config.guidance.rear_path_sample_spacing_m == .5);
    must_reject(std::string(kValidConfig) + "observed_path_rear_follow=true\n");
    must_reject(std::string(kValidConfig) +
                "ground_vector_follow=true\nmotion_ground_follow=true\nobserved_path_rear_follow="
                "true\nrear_path_sample_spacing_m=0\n");
    for (const char* invalid :
         {"motion_history_s=nan\n", "motion_history_s=0\n", "motion_history_s=6\n",
          "motion_history_min_s=.75\n", "motion_history_min_samples=2\n",
          "motion_history_min_samples=5.5\n", "motion_fit_max_rmse_m=0\n",
          "motion_fit_acceleration=auto\n"}) {
        must_reject(std::string(kValidConfig) + invalid);
    }
    for (const char* invalid :
         {"enable_target_recovery=auto\n", "target_recovery_max_gap_ms=0\n",
          "target_recovery_max_gap_ms=5001\n", "target_recovery_confirmation_frames=1\n",
          "target_recovery_confirmation_frames=2.5\n",
          "target_recovery_center_gate_diagonals=nan\n", "target_recovery_min_area_ratio=1.1\n",
          "target_recovery_max_area_ratio=0.9\n"}) {
        must_reject(std::string(kValidConfig) + invalid);
    }
    assert(follow::CalibratedGroundGeometry(config.geometry).max_pose_bracket_distance() ==
           std::chrono::milliseconds(100));
    auto custom_bracket = std::string(kValidConfig);
    const auto bracket_field = custom_bracket.find("max_pose_bracket_ms=100");
    custom_bracket.replace(bracket_field, std::string("max_pose_bracket_ms=100").size(),
                           "max_pose_bracket_ms=20");
    std::istringstream custom_input(custom_bracket);
    const auto custom = follow::parse_bus_following_config(custom_input);
    assert(follow::CalibratedGroundGeometry(custom.geometry).max_pose_bracket_distance() ==
           std::chrono::milliseconds(20));
    assert(custom.rotation.max_pose_bracket_distance == std::chrono::milliseconds(20));

    auto missing = std::string(kValidConfig);
    const auto field = missing.find("fx=500\n");
    missing.erase(field, std::string("fx=500\n").size());
    must_reject(missing);

    must_reject(std::string(kValidConfig) + "unknown_mode=gazebo\n");
    must_reject(std::string(kValidConfig) + "control_period_ms=25\n");
    auto invalid_boolean = std::string(kValidConfig);
    const auto toggle = invalid_boolean.find("enable_rotation_cmc=false");
    invalid_boolean.replace(toggle, std::string("enable_rotation_cmc=false").size(),
                            "enable_rotation_cmc=auto");
    must_reject(invalid_boolean);
    auto priority = std::string(kValidConfig);
    for (const char* field : {"input_score_floor=0.1\n", "fx=500\n"}) {
        priority.erase(priority.find(field), std::string(field).size());
    }
    try {
        std::istringstream broken(priority);
        (void)follow::parse_bus_following_config(broken);
        assert(false);
    } catch (const std::invalid_argument& error) {
        assert(std::string(error.what()) == "missing config key: input_score_floor");
    }

    const std::string pid_config =
        std::string(kValidConfig) +
        "ground_vector_follow=true\nmotion_ground_follow=true\nobserved_path_rear_follow=true\n"
        "pixeagle_pid_enabled=true\npid_forward_gains=4.5,0.1,1.5\npid_right_gains=4.5,0.1,1.5\n"
        "pid_yaw_gains=12,0.5,1.5\npid_altitude_gains=2,0.03,0.05\npid_anti_windup=true\npid_back_"
        "calculation=0.1\n"
        "pid_yaw_smoothing_enabled=true\npid_yaw_deadzone_deg_s=0.5\npid_yaw_max_change_deg_s2="
        "90\npid_yaw_alpha=0.7\n"
        "pid_yaw_speed_scaling=true\npid_yaw_min_speed_m_s=0.5\npid_yaw_max_speed_m_s=5\npid_yaw_"
        "low_speed_factor=0.5\n";
    std::istringstream pid_input(pid_config);
    const auto pid = follow::parse_bus_following_config(pid_input);
    assert(pid.guidance.pid.enabled && pid.guidance.pid.forward.p == 4.5 &&
           pid.guidance.pid.yaw.i == .5);
    assert(!pid.selector.use_fitted_ground_position && config.selector.use_fitted_ground_position);
    assert(pid.guidance.pid_camera.fx == pid.geometry.fx &&
           pid.guidance.pid_ground_down_m == pid.runtime.ground_down_m);
    auto velocity_config = pid_config;
    velocity_config.replace(velocity_config.find("observed_path_rear_follow=true"), 30,
                            "observed_path_rear_follow=false");
    std::istringstream velocity_input(
        velocity_config +
        "body_standoff_follow=true\nvelocity_feedforward_follow=true\ncamera_motion_guard="
        "true\nfov_velocity_p_acc=1.8\nmax_horizontal_accel_m_s2=1\n");
    assert(follow::parse_bus_following_config(velocity_input).guidance.velocity_feedforward_follow);
    must_reject(pid_config + "velocity_feedforward_follow=true\n");
    for (const auto change : {std::pair<std::string, std::string>{"pid_forward_gains=4.5,0.1,1.5",
                                                                  "pid_forward_gains=-1,0,0"},
                              {"pid_yaw_alpha=0.7", "pid_yaw_alpha=2"},
                              {"pid_altitude_gains=2,0.03,0.05", "pid_altitude_gains=nan,0,0"}}) {
        auto invalid = pid_config;
        invalid.replace(invalid.find(change.first), change.first.size(), change.second);
        must_reject(invalid);
    }
    return 0;
}
