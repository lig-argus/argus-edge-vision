// 고정 선택 target의 geometry·FC 상태를 제한된 PX4 velocity 후보로 바꾼다.
#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

#include "argus/control/contracts/include/control_intent.hpp"
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/geometry/include/camera_motion_guard.hpp"
#include "argus/control/guidance/include/cv_gap_visibility.hpp"
#include "argus/control/guidance/include/image_center_follower.hpp"
#include "argus/control/guidance/include/observed_rear_goal.hpp"
#include "argus/control/guidance/include/pid_controller.hpp"
#include "argus/control/guidance/include/predictive_follower.hpp"
#include "argus/control/guidance/include/yaw_only_follower.hpp"
#include "argus/control/guidance/include/yaw_rate_smoother.hpp"
#include "argus/control/target/include/target_selector.hpp"

namespace follow {

struct GuidancePidConfig {
    bool enabled{false};
    PidGains forward{4.5, .1, 1.5}, right{4.5, .1, 1.5}, yaw{12, .5, 1.5}, altitude{2, .03, .05};
    bool anti_windup{true};
    double back_calculation{.1};
    YawSmoothingConfig yaw_smoothing{};
};

struct FlightGuidanceConfig {
    double desired_range_m{0};
    double range_deadband_m{0};
    double yaw_kp_deg_s_per_deg{0};
    double max_yaw_rate_deg_s{0};
    double yaw_alignment_deg{0};
    double forward_gain_m_s_per_m{0};
    double max_forward_m_s{0};
    double altitude_gain{0};
    double max_vertical_m_s{0};
    double edge_margin_fraction{0};
    std::chrono::milliseconds command_ttl{100};
    std::chrono::milliseconds max_pose_age{200};
    double max_horizontal_accel_m_s2{0};
    double max_backward_accel_m_s2{
        0};  // camera guard의 후방/제동 속도 보정량 기준. 실제 가속도 보증 아님.
    double max_yaw_accel_deg_s2{0};
    bool ground_vector_follow{
        false};  // 비교용 opt-in. max_forward는 이 모드에서 수평 벡터 크기의 한도다.
    bool motion_ground_follow{false};
    bool observed_path_rear_follow{false};
    double camera_descent_m{0};        // 화면 하단/접근 압력에 대한 명목 높이의 최대 하강(m).
    double reverse_range_margin_m{0};  // 감속/정지/후진 때의 추가 간격(m).
    bool velocity_feedforward_follow{
        false};  // 미터 PID+관측 속도/LOS 목표+yaw 정렬. 기존 영상 PID와 분리한다.
    bool body_standoff_follow{false};  // 현재 기체 전방의 고정 간격에 영상 표적을 유지한다.
    double rear_path_sample_spacing_m{0.5};
    double ground_center_offset_m{0};  // 접점 투영→중심의 교정 사전값. GT 위치/ID 입력이 아니다.
    GuidancePidConfig pid{};
    CameraGroundCalibration pid_camera{};
    double pid_ground_down_m{0};
    bool camera_motion_guard{false};
    bool ground_contact_anchor{false};  // 생산자가 관측한 하단 접점→차량 중심 변환 계약.
    double fov_velocity_p_acc{0};  // 현재 FC의 MPC_XY_VEL_P_ACC 읽기값. 내부 PID를 변경하지 않는다.
    double max_backward_m_s{0};    // 0은 기존 상한 사용. 기존 aggregate 설정의 순서를 보존한다.
    double camera_min_height_m{0};  // 0은 기존 동작. 시야 하강의 지면 기준 최저 목표 높이.
    bool yaw_only{false};           // 명시 별도 모드: 모든 이동 속도 0, 보정 방향만 사용한다.
    double yaw_only_bearing_deadband_deg{0.5};
    // 2026-11-18 데모 모드: yaw 전용 FOLLOW_ACTIVE 명령에서만 명목 높이로 수직 속도를 낸다.
    // 다른 yaw 전용 명령(공백, 만료 직전, PREDICTED, 정지)은 계속 vz 0이다. 지면 기준 높이(m).
    bool yaw_only_altitude_hold{false};
    double yaw_only_min_height_m{0}, yaw_only_max_height_m{0};
    double follow_side_offset_m{0};  // 관측 이동축 옆의 추종 목표. 0은 기존 방사형 간격 추종.
    bool reset_motion_on_stop_or_reverse{false};
    bool predictive_follow{false};
    double predictive_min_height_m{1.2}, predictive_max_height_m{1.8};
    bool outdoor_profile{false};
    double predictive_target_height_m{0};
    // 2026-11-09 PAMPC point 모드 고도 고정항·수직 무시 구간. 0이면 기존 동작.
    double predictive_height_anchor_m{0};
    double predictive_height_anchor_weight{0};
    double predictive_height_anchor_deadband_m{0};
    double predictive_vertical_deadband_m_s{0};
    // 2026-11-16 수직 데드밴드 해제 여유(m). 0이면 기존 동작.
    double predictive_vertical_deadband_release_m{.1};
    double predictive_vertical_release_max_m_s{0};  // 2026-11-16 선택 상한 (0=없음)
    // 2026-11-10 GAP_CV 동안 CV 속도를 예측 추종기에 넘긴다(point 모드). 끄면 기존 동작.
    bool v35_gap_cv_feedforward{false};
    // 2026-11-10 송신 slew 한도 충돌 시 거부 대신 변화율 한도 안으로 줄여 보낸다. 끄면 기존 동작.
    bool v35_slew_reconcile{false};
    // 2026-11-10 GAP_CV 예측이 실패하면 바로 추종을 끝내지 않고 짧은 공백 Hover(short_gap_hover)로
    // 같은 표적 복귀를 기다린다. 공백 한도(3초)는 그대로. 끄면 기존 동작(HOLD).
    bool v35_gap_invalid_hover{false};
    // 2026-11-10 PAMPC point 거리 무시 구간(m). 0이면 기본 0.3.
    double v35_point_range_deadband_m{0};
    // 2026-11-16 가까운 쪽만 좁힌 거리 무시 구간(m). 0이면 위 값과 같다.
    double v35_point_range_near_deadband_m{0};
    // 2026-11-16 point 모드 화면 하단 시야 비용 시작 위치(정규화). 0이면 기존.
    double v35_point_bottom_margin{0};
    // 2026-11-10 PAMPC point 대상 둘레 방향 상대속도 벌점(0이면 기존 동작). 정지 대상 원 운동 억제.
    double v35_point_tangential_weight{0};
    // 2026-11-19 PAMPC #3: 예측 rollout 기체 응답 이득(1/s). 0이면 기존(수평 fov_velocity_p_acc,
    // 수직 2.0). PX4 가속 setpoint 모델(송신 slew, 시드)에는 쓰지 않는다.
    double pampc_plant_horizontal_response{0};
    double pampc_plant_vertical_response{0};
    // 2026-11-19 PAMPC #5: producer 지면 평면 down과 투영 자세를 계획기 기준으로 쓴다(기본 꺼짐).
    bool pampc_use_producer_ground{false};
    // 2026-11-19 PAMPC 진단 기록(decision.jsonl의 pampc_model과 설정 값). false면 기존 기록 그대로.
    bool pampc_model_log{false};
    // 2026-11-19 검토 수정 producer 기준 안전 상수 (PredictiveFollowConfig와 같은 기본값과 범위).
    double pampc_producer_ground_max_deviation_m{.2};
    double pampc_producer_ground_hold_s{5};
    double pampc_ground_datum_rate_m_s{.05};
    double pampc_ceiling_margin_m{.1};
    // 2026-11-19 실내 데모2 yaw 고정 추종 (PredictiveFollowConfig::fixed_yaw_*와 같은 뜻과 범위).
    bool pampc_fixed_yaw_follow{false};
    double pampc_fixed_yaw_realign_deg{0};
    double pampc_fixed_yaw_realign_rate_deg_s{0};
    // 2026-11-10 예측 추종 중 방위가 yaw_alignment_deg를 넘으면 바로 끝내지 않고 이 시간(초) 동안
    // 수평 0·yaw만으로 다시 맞춘다(실외 3건 종료 원인). 0이면 기존 동작(즉시 HOLD). 0~5초.
    double v35_bearing_realign_s{0};
    // 2026-11-10 GAP_CV 중 yaw 속도 상한을 마지막 실관측 추종 yaw에서 이 시정수(초)로 줄인다
    // (최소 10deg/s). 1468초 공백 중 −27→−60deg/s 과회전 대응. 0이면 기존 동작. 0~5초.
    double v35_gap_yaw_decay_s{0};
    bool outdoor_image_pid{false};
    double image_center_deadzone_fraction{.05};
    // V3 예측 추종 (predictive_follow=true인 실내·실외 기존 방식 설정에서만 켠다)
    int v3_pampc_point_profile{0};
    double v3_image_velocity_weight{0};
    bool v3_predictive_integral{false};
    bool v3_discard_recheck{false};
    bool v32_sent_command_slew{false};
    // 2026-11-09: CV 예측 근거가 없는 입력 공백(NUC 등)을 버티는 시간. 이 동안 수평 0 명령으로
    // 제자리를 지키고 같은 표적이 돌아오면 이어 간다. 사용자 결정(공백 3초 유지)으로 기본 3초이며
    // 실제 끝은 v34 공백 기한(약 2.9초)이 먼저 정한다. 0이면 기존처럼 바로 HOLD로 넘긴다.
    std::chrono::milliseconds short_gap_hover{3000};
};

// 2026-11-18 yaw 전용 고도 유지의 송신 직전 검사 범위. enabled=false면 yaw 전용 vz는 항상 0이다.
struct YawOnlyAltitudeBand {
    bool enabled{false};
    double max_vertical_m_s{0};
    double ground_down_m{0};  // NED down 지면 기준. 높이 = ground_down_m - pose.down_m
    double min_height_m{0}, max_height_m{0};
};

// 데모 모드 고도 유지 무시 구간(m). 명목 높이와의 차가 이 안이면 vz 0.
inline constexpr double kYawOnlyAltitudeDeadbandM = 0.05;

struct V21MotionDiagnostics {
    bool cv_gap_active{false}, box_motion_suspect{false};
    std::int64_t origin_sample_ns{0}, deadline_ns{0};
    double velocity_north_m_s{0}, velocity_east_m_s{0}, visibility_scale{1};
    bool velocity_unconfirmed{false}, lateral{false};
    std::int64_t velocity_fit_sample_ns{0};
    double lateral_delta_m_s{0};
    std::uint64_t history_frame_id{0};
    std::int64_t history_source_ns{0};
    std::optional<CvVisibilityDiagnostics> visibility_diagnostics;
};

struct FlightGuidanceInput {
    Clock::time_point now{};
    TargetSelectionSnapshot selection;
    std::uint64_t selection_command_epoch{0};
    Pose pose;
    double hold_down_m{0};
    bool connected{false};
    bool armed{false};
    bool in_air{false};
    bool offboard{false};
    bool rc_override{false};
    GroundProjection geometry;
    std::optional<Pose>
        observation_pose;  // v1 노출/v2 원 수신 시각의 보간 FC pose. 현재 pose로 대체하지 않는다.
    // 2026-11-19 PAMPC #5: observation_pose가 같은 프레임의 producer ground_pose일 때만 채운다.
    std::optional<ProducerGround> producer_ground;
};

struct FlightGuidanceResult {
    std::string mission{"IDLE"};
    std::string reason{"not_started"};
    FollowCommand command{};
    Clock::time_point valid_until{};
    std::uint64_t selection_command_epoch{0};
    bool send_setpoint{false};
    bool request_offboard_stop_once{false};
    std::optional<RearGoal>
        rear_goal;  // Read-only diagnostic of the goal actually used in this tick.
    std::optional<std::array<PidTerms, 4>> pid_terms;
    std::optional<double> altitude_target_down_m;
    std::optional<PredictiveVerticalDiagnostics> predictive_vertical;
    std::optional<V21MotionDiagnostics> v21_motion;
    double kinematic_time_rate{1};  // 진단: 센서 시간 초 / host 시간 초. 만료에는 사용하지 않는다.
};

// 독립 control tick에서 사용한다. FC mode/RC/stale 실패 시 자동 재개하지 않는다.
class FlightGuidanceController final {
public:
    explicit FlightGuidanceController(FlightGuidanceConfig config);
    void explicit_start_committed(std::uint64_t selection_command_epoch = 0);
    void request_operator_stop();
    void request_manual_override();
    void record_offboard_stop_result(bool success, bool actual_hold_confirmed);
    FlightGuidanceResult update(const FlightGuidanceInput& input);
    void record_command_result(bool sent);
    // 2026-11-16 요 시드 시선각 변화율 항 저역 필터 (0=꺼짐). 예측 추종기가 없으면 아무것도 안
    // 한다.
    void set_los_rate_filter(double tau_s) {
        if (predictive_follower_) {
            predictive_follower_->set_los_rate_filter(tau_s);
        }
    }
    const PredictiveIntegralDiagnostics* predictive_integral_diagnostics() const {
        return predictive_follower_ ? &predictive_follower_->integral_diagnostics() : nullptr;
    }
    bool yaw_only() const noexcept { return config_.yaw_only; }
    // V3: 선택 문맥 변경으로 버려진 명령을 최신 상태로 한 번만 다시 계산한다.
    bool v3_discard_recheck() const noexcept { return config_.v3_discard_recheck; }
    std::chrono::milliseconds max_pose_age() const noexcept { return config_.max_pose_age; }
    std::array<double, 3> command_limits() const noexcept {
        return {config_.max_forward_m_s, config_.max_vertical_m_s, config_.max_yaw_rate_deg_s};
    }
    YawOnlyAltitudeBand yaw_only_altitude_band() const noexcept {
        YawOnlyAltitudeBand band;
        band.enabled = config_.yaw_only && config_.yaw_only_altitude_hold;
        band.max_vertical_m_s = config_.max_vertical_m_s;
        band.ground_down_m = config_.pid_ground_down_m;
        band.min_height_m = config_.yaw_only_min_height_m;
        band.max_height_m = config_.yaw_only_max_height_m;
        return band;
    }

private:
    FlightGuidanceResult hold_once(const FlightGuidanceInput&, const char* reason);
    FlightGuidanceResult short_gap_pause(const FlightGuidanceInput&);
    void reset_pid();
    void reset_pid_timing();
    FlightGuidanceConfig config_;
    bool point_only_{false};
    enum class StopState { NotAttempted, ResultPending, Succeeded, Failed };
    StopState stop_state_{StopState::NotAttempted};
    bool force_stop_{false};
    bool manual_override_latched_{false};
    bool started_{false};
    std::uint64_t start_command_epoch_{0};
    Clock::time_point previous_command_at_{};
    FollowCommand previous_command_{};
    ObservedRearGoal rear_goal_;
    RearGoal latest_rear_goal_;
    std::int64_t rear_sample_ns_{0};
    ObservedMotionPhase rear_motion_phase_{ObservedMotionPhase::Unknown};
    PidController forward_pid_, right_pid_, yaw_pid_, altitude_pid_;
    YawRateSmoother yaw_smoother_;
    std::optional<YawOnlyFollower> yaw_only_follower_;
    std::optional<CalibratedGroundGeometry> pid_projection_;
    Clock::time_point pid_previous_at_{};
    Clock::time_point short_gap_since_{};
    Clock::time_point bearing_realign_since_{};
    double last_follow_yaw_deg_s_{0};
    std::optional<std::array<double, 2>> filtered_visual_error_;
    std::optional<CameraMotionGuard> camera_motion_guard_;
    double filtered_standoff_m_{0};
    double filtered_descent_m_{0};
    std::optional<std::array<double, 2>> side_follow_heading_;
    std::int64_t side_los_sample_ns_{0};
    double side_los_angle_rad_{0}, side_los_rate_deg_s_{0};
    std::optional<ShortMotionEstimate> reset_motion_previous_;
    std::optional<PredictiveFollower> predictive_follower_;
    // 2026-11-10 slew 충돌 줄여 보내기 기록(시작·끝만)
    bool slew_reconcile_active_{false};
    unsigned slew_reconcile_ticks_{0};
    bool fixed_yaw_realign_active_{false};  // 2026-11-19 데모2 재정렬 기록용
    std::optional<ImageCenterFollower> image_center_follower_;
    std::deque<std::array<double, 2>> motion_clock_samples_;
    double motion_clock_rate_{1};
};

}  // namespace follow
