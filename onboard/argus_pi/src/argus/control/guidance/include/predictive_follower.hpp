#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/control/contracts/include/control_intent.hpp"
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/guidance/include/pampc_point_settings.hpp"
namespace follow {
struct PredictiveFollowConfig {
    CameraGroundCalibration camera;
    double range_m{2.3}, ground_down_m{0}, velocity_response{1.8};
    double max_speed_m_s{.75}, max_backward_m_s{.65}, max_vertical_m_s{.7};
    double max_acceleration_m_s2{.6}, braking_acceleration_m_s2{1.8};
    double max_yaw_rate_deg_s{75}, max_yaw_accel_deg_s2{300};
    double min_height_m{1.2}, max_height_m{1.8};
    double position_gain{.8}, velocity_gain{.35};
    double integral_gain{0}, integral_limit_m_s{.10};
    bool integral_anti_windup{true};
    double side_offset_m{0};  // 총간격 원 위의 옆간격. 기존 설정과 동일한 부호.
    bool outdoor_profile{false};
    bool sent_command_slew{false};
    double target_height_m{0};  // 0: legacy reference, otherwise explicit ground-datum height.
    // V3 (userpc SITL 검증본) 추종 기능. 0이면 기존 동작. 설정 파일에서만 켠다.
    int pampc_point_profile{0};       // 0 끔, 1..4 PAMPC point 가중치 묶음 (V3 검증: 1)
    double image_velocity_weight{0};  // 0, .25, 1, 4 (point 모드에서는 묶음 값 사용)
    // 2026-11-09 point 모드 고도 고정항. weight 0이면 기존 동작(실내는 현재 높이, 실외는
    // target_height_m을 기존 point 가중치 0.3으로 기준 삼는다). 켜면 고정 높이에서 벗어난 만큼
    // weight × (오차 − deadband)² 비용을 쓴다. 실내는 height_anchor_m(지면 기준 m)을, 실외는
    // target_height_m을 고정 높이로 쓴다(실외는 height_anchor_m=0).
    double height_anchor_m{0};
    double height_anchor_weight{0};
    double height_anchor_deadband_m{0};
    // 2026-11-09 point 모드 수직 명령 무시 구간. 계획한 수직 속도가 이보다 작으면 0으로 보낸다
    // (가속 제한은 그대로라 0까지 천천히 줄어든다). 0이면 끔.
    double vertical_deadband_m_s{0};
    // 2026-11-16 사용자 지시(15:20 비행 고도 2.08m 이탈): 수직 데드밴드 해제 여유(m). 실내 고정
    // 높이 (height_anchor)에서 anchor deadband + 이 값보다 벗어났을 때 기준 쪽으로 가는 작은 수직
    // 속도는 0으로 바꾸지 않는다. 최대 높이 이상에서의 하강은 모든 모드에서 0으로 바꾸지 않는다.
    // 0이면 기존 동작.
    double vertical_deadband_release_m{.1};
    // 선택: 실내 고정 높이 이탈 tick의 기준 쪽 수직 속도 상한(m/s). 0이면 상한 없음(기본).
    // 주의: 켜면 데드밴드와 무관하게 이미 나가던 큰 수직 속도도 줄인다(시험: 0.26 -> 0.2).
    // 최대 높이 이상 하강(모든 모드)에는 쓰지 않는다.
    double vertical_release_max_m_s{0};
    // 2026-11-10 GAP_CV 속도 이어받기(사용자 확정). point 모드는 표적 속도를 0으로 보고 예측해
    // 공백 중 감속 계획을 세웠다. 켜면 공백(gap_displacement 있음)일 때만 넘겨받은 CV 속도를
    // 예측 구간의 표적 이동과 시작 명령의 기본값으로 쓴다. 끄면 기존 동작.
    bool gap_velocity_feedforward{false};
    // 2026-11-10 송신 slew에서 실제 속도 한도와 직전 송신 변화율 한도가 겹치지 않으면 기존에는
    // 결과를 버려 HOLD로 넘어갔다. 켜면 직전 송신값에서 변화율 한도만큼만 실제 속도 쪽으로
    // 옮겨 보낸다. 끄면 기존 동작(거부).
    bool sent_command_slew_reconcile{false};
    // 2026-11-10 point 모드 거리 무시 구간(m). 0이면 point 묶음 기본값(0.3). 10:08 비행 Replay에서
    // 0.4m로 1초대 전후 진동 에너지 -32%(열린 고리). 0.1~1.0만 허용.
    double point_range_deadband_m{0};
    // 2026-11-16 사용자 지시(반전 접근 때 하단 이탈): 가까운 쪽 거리 무시 구간만 따로 둔다(m).
    // 0이면 위 값과 같다(기존 대칭). 12:45 비행: 1.8±0.4라 1.4m까지 후진 없음, 높이 1.95m에서
    // 화면 하단 한계 지면거리 약 1.24m → 여유 0.15m로 표적 상실. 0.1..point_range_deadband 허용.
    double point_range_near_deadband_m{0};
    // 2026-11-16 사용자 지시(하단 시야 여유): point 모드 화면 하단 시야 비용이 시작되는 정규화 위치
    // (-1 위 끝 .. 1 아래 끝). 0이면 기존(약 0.725~0.75, y2 86~87.5%). 12:45 비행 상실은 y2가
    // 400px(83%)을 넘은 뒤 0.2초 만에 화면 밖. 0.3..0.75 허용, 기존 값보다 작을 때만 적용한다.
    double point_bottom_margin{0};
    // 2026-11-10 point 모드 대상 둘레(LOS 수직) 방향 상대속도 벌점 가중치. 0이면 기존 동작.
    // point 목표는 거리 방향만 보정해 둘레 이동에 비용이 거의 없었고, warm start가 그 옆속도를
    // 이어 써서 정지 대상 주위를 원으로 돌았다(실외 11:55·12:03, 약 20°/s). 0~50만 허용.
    double point_tangential_weight{0};
    // 2026-11-19 사용자 지시(PAMPC #3): 예측 rollout의 기체 응답 이득(1/s). 0이면 기존 값
    // (수평은 velocity_response, 수직은 2.0)을 쓴다. velocity_response(=PX4 MPC_XY_VEL_P_ACC 1.8)는
    // PX4 가속 setpoint 모델(송신 변화율 예산, 시드 이득)에 계속 쓰고, 이 값은 예측 자세/위치
    // 전파에만 쓴다. 2026-11-18 실내 8회 적합값은 수평 0.8~0.9, 수직 1.5. 0 또는 0.2..5만 허용.
    double plant_horizontal_response{0};
    double plant_vertical_response{0};
    // 2026-11-19 사용자 지시(PAMPC #5): producer가 지면 투영에 쓴 평면 down과 투영 자세를
    // 계획기에도 쓴다. false면 기존 동작(ground_down_m, observation pose). 값이 없거나 검사에
    // 걸리면 그 tick만 기존 동작으로 돌아가고 진단에 센다.
    bool use_producer_ground{false};
    // 2026-11-19 검토 수정 안전 상수 (use_producer_ground=true일 때만 작용).
    // 평면이 ground_down_m과 이보다 더 다르면 그 프레임 값은 쓰지 않는다 (m, 0.05..0.5).
    double producer_ground_max_deviation_m{.2};
    // 오래된 관측(Stale)이나 필드 공백(Absent) 때 마지막 평면을 유지하는 시간 (s, 0..10, 0=유지 안
    // 함).
    double producer_ground_hold_s{5};
    // 계획기에 적용하는 지면 기준의 변화율 상한 (m/s, 0..1, 0=제한 없음). 양방향, epoch 시작 포함.
    double ground_datum_rate_m_s{.05};
    // 고정 높이 + 무시 구간을 EKF 기준 상한보다 이만큼 아래로 둔다 (m, 0..0.3).
    double ceiling_margin_m{.1};
    // 2026-11-19 실내 데모2 yaw 고정 추종(사용자 지시). 켜면 계획의 모든 yaw 속도를 0으로 묶고,
    // 화면 좌우 맞춤은 옆이동(카메라 수평 방향에 수직)이 맡는다. 회전 중 광류 가짜 이동을 줄이려는
    // 목적이다. 카메라 방위 오차가 fixed_yaw_realign_deg를 넘으면 그 절반 아래로 돌아올 때까지만
    // fixed_yaw_realign_rate_deg_s 이하로 천천히 돌린다(0이면 재정렬 없음). point 모드에서만 쓴다.
    bool fixed_yaw_follow{false};
    double fixed_yaw_realign_deg{0};
    double fixed_yaw_realign_rate_deg_s{0};
};
// 2026-11-19 PAMPC #5 기준 선택 결과 (판단 기록용).
enum class PampcGroundSource : std::uint8_t { Legacy = 0, Producer = 1, Held = 2 };
enum class PampcGroundFallback : std::uint8_t {
    None = 0,       // producer 기준 사용 또는 기능 꺼짐
    Absent = 1,     // 필드 없음 (구버전 producer, 시각 정렬 pose 경로 등)
    NonFinite = 2,  // 비유한 값
    Deviation = 3,  // EKF 기준(ground_down_m)과 0.5m 넘게 다름
    Attitude = 4,   // 투영 자세가 관측 pose와 크게 다르거나 범위 밖
    Stale = 5,      // 관측 나이가 기준 시간(1초)을 넘음
};
struct PampcModelDiagnostics {
    PampcGroundSource datum_source{PampcGroundSource::Legacy};
    PampcGroundSource attitude_source{PampcGroundSource::Legacy};
    PampcGroundFallback fallback{PampcGroundFallback::None};
    double ground_datum_down_m{0};   // 이번 tick 계획기가 쓴 지면 down (m, NED)
    double target_datum_down_m{0};   // 변화율 제한 전 목표 기준 (기능 꺼짐이면 0)
    bool height_ref_clamped{false};  // EKF 상한 여유 때문에 기준 높이를 줄였는가
    double height_ref_limit_m{0};    // 이번 tick 기준 높이 상한 (기능 꺼짐이면 0)
    std::uint64_t held_ticks{0}, height_ref_clamped_ticks{0};    // 누적
    std::optional<double> producer_ground_down_m;                // 받은 값 (있을 때)
    std::optional<std::array<double, 3>> producer_attitude_deg;  // 받은 투영 자세 (있을 때)
    std::array<double, 3> projection_attitude_deg{};             // 이번 tick t0 재투영에 쓴 자세
    double plant_horizontal_response{0}, plant_vertical_response{0};
    std::uint64_t producer_ticks{0}, fallback_ticks{0};  // 누적 (reset 때 유지)
    // 영상 예측 (정규화 x, 중심 y) 1, 3, 5, 10, 20 단계. 영상 속도 비용이 꺼져 있으면 0이다.
    std::array<std::array<double, 2>, 5> image_forecast{};
    bool image_forecast_valid{false};
    // 선택된 계획 3 블록 (vn, ve, vd m/s, yaw rate deg/s, NED).
    std::array<std::array<double, 4>, 3> plan_blocks{};
};
// 선택 과정의 관측값이다. 명령 소유권이나 TTL에는 영향을 주지 않는다.
struct PredictiveVerticalDiagnostics {
    double proposed_down_m_s;
    double limited_down_m_s;
    double final_down_m_s;
    double previous_down_m_s;
    double vertical_slew_m_s2;
    double height_reference_m;
    double best_score;
    double height_cost;
    double visibility_cost;
    double boundary_cost;
    double other_cost;
    // Best coarse-grid score for each initial vertical seed, before fine search. These do
    // not include the optional horizontal/yaw visibility refinements.
    std::optional<double> up_candidate_score;
    std::optional<double> zero_candidate_score;
    std::optional<double> down_candidate_score;
    unsigned safety_flags;
    double vertical_center_cost{0};
    PampcModelDiagnostics model{};  // 2026-11-19 PAMPC #3, #5 진단 (동작에 쓰지 않는다)
};
struct PredictiveFollowResult {
    FollowCommand command;
    std::array<double, 2> goal;
    double altitude_target_down_m;
    PredictiveVerticalDiagnostics vertical_diagnostics;
};
struct PredictiveIntegralDiagnostics {
    std::array<double, 2> committed{}, proposed{}, error{}, first_requested{}, first_limited{};
    std::array<double, 2> seed_requested{}, seed_limited{};
    std::array<std::array<double, 2>, 3> forecast{};
    bool enabled{false}, pending{false}, saturated{false};
    std::int64_t sample_ns{0};
};
// 2026-11-16 요 흔들림 분석용 진단(동작 불변). 각 탐색 단계 뒤 첫 블록 요(rad/s).
// source: 초기 계획 0=시드, 1=정지 계획, 2=warm start.
struct PredictiveYawTrace {
    double los_rate{0}, p_term{0}, seed{0}, seed_score{0}, best_score{0}, previous{0};
    double after_init{0}, after_coarse{0}, after_search{0}, after_visibility{0}, after_point{0};
    double after_contract{0}, final{0};
    double range_rate{0}, approach_ff{0}, visibility{0};  // m/s, m/s(기체 후퇴), 시야 위험
    int source{0};
};
struct PredictiveImageVelocityDiagnostics {
    double weight{0}, cost{0};
    unsigned steps{0};
    std::array<double, 2> initial_center{};
    std::array<std::array<double, 4>, 20> forecast{};  // x,y,dot_x/s,dot_y/s
};
// 실제 관측/FC 상태만 사용하는 작은 direct-shooting 제어기.
// OFF: 3×0.4초 입력/1.2초 예측. point: 0.4+0.4+1.2초 입력/2초 예측, 0.1초 적분.
// 원본 thrust/body-rate NMPC 재현은 아니다.
class PredictiveFollower final {
public:
    explicit PredictiveFollower(PredictiveFollowConfig);
    void reset();
    void reset_motion();
    void reset_plan();
    void reset_integral();
    void complete_update(bool sent);
    const PredictiveIntegralDiagnostics& integral_diagnostics() const {
        return integral_diagnostics_;
    }
    const PredictiveImageVelocityDiagnostics& image_velocity_diagnostics() const {
        return image_velocity_diagnostics_;
    }
    // 2026-11-10 마지막 update()가 결과를 내지 못한 검사 이름(결과를 냈으면 빈 문자열).
    const std::string& last_reject_reason() const { return last_reject_reason_; }
    // 마지막 update()에서 송신 slew 두 한도가 겹치지 않아 변화율 한도 안으로 줄여 보냈는가.
    const PredictiveYawTrace& yaw_trace() const { return yaw_trace_; }
    bool last_slew_reconciled() const { return last_slew_reconciled_; }
    // 2026-11-19 데모2 yaw 고정 추종에서 마지막 update()가 느린 재정렬 회전을 허용했는가.
    bool fixed_yaw_realigning() const { return fixed_yaw_realigning_; }
    // 2026-11-10 다음 update()의 최종 yaw 속도 상한(deg/s). nullopt면 제한 없음(기존 동작).
    // 내부 yaw 기준(previous_yaw_rate_)도 제한된 값으로 남겨 다음 계획과 송신값이 어긋나지 않는다.
    void set_yaw_rate_cap(std::optional<double> cap_deg_s);
    // 2026-11-16 사용자 지시(요 흔들림): 요 시드의 시선각 변화율(los_rate) 항만 1차 저역 필터로
    // 거른다. tau_s 0이면 꺼짐(기존). 근거: 11/16 비행 요 고주파 흔들림의 대부분이 이 항(가까운
    // 거리에서 상대속도 잡음/거리²), 결정 기록 근사로 τ 0.1s면 고주파 약 58% 감소. 방위 오차 P항은
    // 그대로.
    void set_los_rate_filter(double tau_s);
    // 2026-11-16 요 흔들림 개선 후보 (모두 기본 꺼짐, point 모드에서만 의미가 있다).
    // ① plan_hysteresis: 직전 계획(warm)보다 비용이 이 비율 이상 좋아질 때만 다른 계획으로 바꾼다
    // (0..0.5). ② stop_keeps_seed_yaw: 정지 계획 후보도 요는 시드(LOS 피드백)를 유지하고 이동만
    // 멈춘다. ④ point_yaw_follows_seed: point 모드도 일반 모드처럼 요는 시드를 따른다 (warm 요를
    // 시드로,
    //    좌표 탐색에서 요 차원 제외). 시야 확보용 요 후보 탐색은 그대로 둔다.
    void set_yaw_plan_options(double plan_hysteresis, bool stop_keeps_seed_yaw,
                              bool point_yaw_follows_seed);
    // 2026-11-16 사용자 지시(반전 접근 때 하단 이탈): point 모드에서 영상 거리 변화율로 표적이
    // 다가오면(거리율 < -threshold) 그 속도 × gain만큼 표적 반대 방향으로 시드 속도를 더한다.
    // gain 0 = 꺼짐(기존). 먼 쪽 데드밴드 밖(멀어진 상태)에서는 쓰지 않는다. 크기는 최대 후진속도
    // 제한.
    void set_approach_feedforward(double gain, double threshold_m_s);
    // 2026-11-10 추종기 밖에서 만든 명령(방위 재정렬 등)을 보냈을 때 송신 기준을 맞춘다.
    // world_ne_m_s: 송신한 수평 속도(NED), yaw_rate_deg_s: 송신한 yaw 속도, vertical_m_s: 송신 vz.
    void note_external_command(std::array<double, 2> world_ne_m_s, double vertical_m_s,
                               double yaw_rate_deg_s);
    std::optional<PredictiveFollowResult> update(
        const Pose& current, const Pose& observation, const TrackRecordV1&,
        std::array<double, 2> velocity, double age_s, double nominal_down_m, double dt_s,
        double time_rate = 1, bool recovered = false,
        std::optional<std::int64_t> feature_clock_ns = {},
        std::optional<std::array<double, 2>> gap_displacement = {},
        std::optional<ProducerGround> producer_ground = {});
    // 2026-11-19 PAMPC #5 기준 검사 상한 (단위 m, deg, s).
    static constexpr double kProducerAttitudeMaxTiltDeg = 45.;
    static constexpr double kProducerAttitudeMaxDifferenceDeg = 15.;
    static constexpr double kProducerGroundMaxAgeS = 1.;

private:
    std::uint64_t producer_ground_ticks_{0}, producer_ground_fallback_ticks_{0};
    std::uint64_t producer_ground_held_ticks_{0}, height_ref_clamped_ticks_{0};
    // 2026-11-19 검토 수정: FC epoch 안에서 유지하는 지면 기준 상태 (reset_motion에도 유지).
    struct HeldDatum {
        double plane;
        std::int64_t accepted_ns;
    };
    std::optional<HeldDatum> held_datum_;
    std::optional<double> applied_datum_;
    std::optional<std::uint64_t> datum_epoch_;
    std::optional<std::array<double, 2>> successful_world_command_, pending_world_command_;
    std::int64_t successful_command_ns_{0}, pending_command_ns_{0};
    PredictiveFollowConfig config_;
    PampcPointSettings point_;
    std::array<double, 2> integral_{};
    std::optional<std::array<double, 2>> pending_integral_;
    PredictiveIntegralDiagnostics integral_diagnostics_;
    CalibratedGroundGeometry projection_;
    double image_velocity_weight_{0};
    PredictiveImageVelocityDiagnostics image_velocity_diagnostics_;
    using Control =
        std::array<double, 4>;  // NED vn,ve,vd(m/s),yaw rate(rad/s). 송신 때만 body로 바꾼다.
    using Plan = std::array<Control, 3>;
    std::optional<Plan> previous_plan_;
    std::optional<std::array<double, 2>> offset_axis_;
    double previous_yaw_rate_{0};
    std::optional<double> previous_vertical_command_;
    std::string last_reject_reason_;
    bool last_slew_reconciled_{false};
    PredictiveYawTrace yaw_trace_;  // 2026-11-16 분석용
    std::optional<double> yaw_rate_cap_deg_s_;
    double los_rate_filter_s_{0};                        // 2026-11-16: 0 = 꺼짐
    double plan_hysteresis_{0};                          // 2026-11-16 ①
    double range_near_deadband_{0};                      // 2026-11-16 가까운 쪽 거리 무시 구간
    double bottom_margin_{0};                            // 2026-11-16 하단 시야 여유 (0=기존)
    double approach_gain_{0}, approach_threshold_{.05};  // 2026-11-16 접근 감쇠
    bool stop_keeps_seed_yaw_{false};                    // 2026-11-16 ②
    bool point_yaw_follows_seed_{false};                 // 2026-11-16 ④
    std::optional<double> filtered_los_rate_;            // 필터 상태 (rad/s)
    bool fixed_yaw_realigning_{false};                   // 2026-11-19 데모2 yaw 고정: 재정렬 중인가
    std::optional<double> pending_vertical_command_;
    std::int64_t successful_vertical_ns_{0};
    std::int64_t pending_vertical_ns_{0};
    struct BoxExtent {
        std::int64_t sample_ns;
        double width_depth;
    };
    std::optional<BoxExtent> previous_box_extent_;
    double box_width_growth_{0};
    std::int64_t box_growth_started_ns_{0};
    struct VisibleShape {
        std::int64_t sample_ns;
        std::int64_t track_id;
        double width_depth, height_depth;
        std::array<double, 3> target;
        std::array<double, 2> velocity;
        std::array<double, 3> ground_covariance;
        std::array<double, 2> image_bias;
    };
    std::optional<VisibleShape> visible_shape_;
    struct FeatureObservation {
        std::int64_t clock_ns, track_id;
        std::array<double, 3> point;
        std::array<double, 2> bias;
    };
    std::optional<FeatureObservation> feature_observation_;
    std::array<double, 3>
        feature_rate_{};  // Image anchor u/v and radial range disturbance, not world velocity.
};
}  // namespace follow
