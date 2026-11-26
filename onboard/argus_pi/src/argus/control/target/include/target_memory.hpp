// 선택한 차량의 기억과 재획득 설정이다. 원본 표본 시각과 로컬 수신 시각을 섞지 않는다.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/common/time/include/time.hpp"
#include "argus/control/target/include/cv_ten_frame_memory.hpp"
#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/tracking/motion/include/short_motion_estimator.hpp"

namespace follow {

struct SelectedTargetKey {
    std::uint64_t generation{0};
    std::int64_t track_id{0};
};

// An old real sample used only to estimate visibility during a bounded CV gap.
// It is never a new detection and does not extend the observation lease.
struct CvVisibilityReference {
    SelectedTargetKey binding;
    TrackRecordV1 box;
    std::array<double, 6> observation_pose;
    std::int64_t sample_ns{0};
    std::uint32_t image_width{0}, image_height{0};
    // 2026-11-19 PAMPC #5: 같은 기준 프레임의 producer 지면 기준 (구버전 producer면 없음).
    std::optional<ProducerGround> producer_ground;
};

enum class TargetPhase { Unselected, Locked, GapHold, Reacquire, Confirm, Lost };
const char* target_phase_name(TargetPhase phase);

enum class ThermalIdentityMode : std::uint8_t { Off, Log, On };

struct TargetRecoveryConfig {
    bool ground_contact_anchor{false};
    bool enabled{false};
    std::chrono::milliseconds max_gap{2000};
    std::uint32_t confirmation_frames{3};
    double max_center_distance_diagonals{1.0};
    double min_area_ratio{0.5};
    double max_area_ratio{2.0};
    double ground_gate_m{2.0};
    // 2026-11-10 가림 대응(사용자 확정): 공백이 길수록 CV 예측 오차가 커지므로 지면 게이트를
    // ground_gate_m + growth × 공백(초)로 키우고 max에서 멈춘다. growth 0이면 기존 고정 게이트.
    double ground_gate_growth_m_s{0};
    double ground_gate_max_m{0};
    // trust_selected_id에서만 쓴다. 같은 native ID가 이 시간보다 오래 비었다가 돌아오면 CV 예측
    // 위치에서 위 지면 게이트 안일 때만 바로 믿는다. 밖이면 일반 후보 검색으로 넘긴다. 0이면 끔.
    std::chrono::milliseconds trusted_return_check{0};
    bool allow_competitor_departure{false};
    // 명시적으로 켠 경우에만 새 경쟁 대상의 초기 판정을 사용한다.
    bool prefer_selected_continuity{false};
    double motion_history_s{0.75}, motion_history_min_s{0.3};
    std::uint32_t motion_history_min_samples{5};
    double motion_fit_max_rmse_m{0.4};
    bool motion_fit_acceleration{false};
    std::chrono::milliseconds motion_max_sample_gap{250};
    ShortMotionConfig short_motion;
    bool reset_motion_on_stop_or_reverse{false};
    bool retain_motion_on_brief_gap{
        false};  // 예측모드의 실관측 회귀만 보존. 공백 제어는 계속 중립.
    bool coherent_ground_anchor_motion{
        false};  // 검증된 중심 관측의 기준 전환만 연속성 검사. 기본 비활성.
    bool trust_selected_id{false};
    // trust_selected_id에서만 쓴다. 같은 native ID라도 직전 수용 박스와 겹치지 않고 멀리 튄 박스는
    // 그 프레임만 버리고, 튐이 짧은 창을 넘어 이어지면 대상을 잃은 것으로 본다(기본 끔).
    bool selected_box_jump_guard{false};
    // 2026-11-09 열 외형 같은 대상 확인 (trust_selected_id에서만). off=기존 동작, log=기록만,
    // on=다른 후보가 선택 대상의 열 분포 기억과 더 닮으면(상대 비교) 그 프레임을 버리고,
    // 0.5초 넘게 이어지면 대상을 잃은 것으로 본다.
    ThermalIdentityMode thermal_identity{ThermalIdentityMode::Off};
    // 비교 기록 파일(비어 있으면 없음)과 판단 최소 대비(RAW 단위, 이보다 약하면 판단하지 않음).
    std::string thermal_identity_log;
    double thermal_min_contrast{40};
    // 2026-11-10 열 외형 원래 기준 고정·갱신 제한(on에서만 판단). 선택 시점 외형을 따로 보관하고,
    // 유사도 0.85 이상일 때만 기억을 갱신한다. 원래 기준과 갱신 기준 둘 다 0.75 미만이면 비교
    // 후보가 없어도 불일치로 본다(실내 15:28 다른 물체가 같은 ID를 받아 기억이 그쪽으로 갱신됐다).
    // 끄면 기존 동작(상대 비교만, 매 프레임 갱신).
    bool v35_thermal_anchor_gate{false};
    // 2026-11-13 신원 판단 문맥(사용자 확정). 2026-11-16 사용자 확정 변경 "ID를 최대한 믿는다":
    // 켜면
    // - 선택 ID가 이번 프레임에 있으면 열 외형으로 보류·해제하지 않는다(사라졌다 돌아와도, 외형이
    //   급변해도, 다른 차량과 겹쳤다 떨어져도). 유사도는 thermal-identity 기록에만 남긴다.
    // - 열 기억은 매 프레임 새 외형으로 천천히 갱신하고, 사라졌다 돌아오면 돌아온 외형으로 바꾼다.
    //   같은 종류 다른 차량과 박스가 가까울 때만 갱신을 쉰다(섞인 외형 방지).
    // - 공백 뒤 다른 ID는 따라가지 않는다(v36_new_id_relaxed를 켜면 조건부 허용).
    // - 박스 튐 검사·지면 예측 복귀 검사는 이 설정과 별개로 그대로 동작한다.
    bool v36_identity_context{false};
    // 다른 ID 이어받기 완화(기본 끔): 같은 종류 후보가 하나뿐이고 예측 위치 안이며 열 외형이
    // 기억과 강하게 닮을 때만 새 ID를 표적으로 이어받는다.
    bool v36_new_id_relaxed{false};
    bool v32_cv_ten_frame{false};  // Optional in V3.2; tested V3.2 settings leave this off.
    double estimated_speed_deadzone_m_s{0};
    // V3: 잘린/급변 박스를 가림 의심으로 보고 속도 적합을 멈춘다 (기본 끔: 메인 기존 동작).
    bool v3_box_motion_guard{false};
    // V3: 가림 의심 중 직전 실제 접지 관측의 속도를 CV 근거로 쓴다 (box 가드가 켜져야 동작).
    bool v3_fresh_contact_velocity{false};
    // 2026-11-10 박스 의심이 길게 이어지면 GAP_CV 출발점(cv_visibility_reference)이 갱신되지 않아
    // 공백 시작 즉시 거부됐다(10:08 비행, 의심 3.5초). 켜면 기준이 1초보다 오래됐을 때 의심 중인
    // 실제 탐지로도 기준을 새로 잡는다. 끄면 기존 동작.
    bool v35_cv_reference_refresh{false};
};

struct TargetMemory {
    CvTenFrameMemory cv_ten_frame;
    std::uint64_t mission_target_id{0};
    SelectedTargetKey binding;
    std::optional<CvVisibilityReference> cv_visibility_reference;
    TrackRecordV1 last_real_track;
    TracksV1 last_real_frame;
    std::int64_t last_real_sample_ns{0};
    Clock::time_point last_real_received_at{};
    double velocity_x_px_s{0}, velocity_y_px_s{0};
    double velocity_north_m_s{0}, velocity_east_m_s{0};
    double acceleration_north_m_s2{0}, acceleration_east_m_s2{0};
    double fitted_north_m{0}, fitted_east_m{0};
    bool ground_acceleration_valid{false};
    bool ground_motion_valid{false};
    double ground_motion_rmse_m{0};
    ShortMotionEstimate short_motion;
    // Identity-only evidence: never exported as control/CV velocity quality.
    ShortMotionEstimate identity_short_motion;
    bool box_motion_suspect{false};
    double cv_velocity_north_m_s{0}, cv_velocity_east_m_s{0};
    bool cv_velocity_trusted{false};
    bool cv_last_observed_moving{false};
    bool cv_velocity_unconfirmed{false};
    std::int64_t cv_velocity_fit_sample_ns{0};
    std::string source, clock_domain, timestamp_source;
    bool exposure_time_valid{false};
    std::uint32_t image_width{0}, image_height{0};
};

struct TargetRecoverySnapshot {
    std::optional<TargetMemory> memory;
    TargetPhase phase{TargetPhase::Unselected};
    std::uint64_t revision{0};
    std::string reason{"unselected"};
    bool pause_commands{false};
};

}  // namespace follow
