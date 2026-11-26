// 표본 검사 → 단절 기한 → 후보 검사 → 연속 확인 → 정상 기억 갱신 순서로 처리한다.
#include "argus/control/target/include/target_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/target/include/cv_motion_cache.hpp"

namespace follow {
namespace {
constexpr std::size_t kMaxCompetitors = 32;
// thermal_identity 상대 비교 여유 (제안, 2026-11-10 비슷하게 데운 두 탱크 RAW14 측정): 선택 대상이
// 다른 탱크보다 히스토그램 교집합이 최소 0.086, 보통 0.12 높았다(1/4 축소까지 유지).
constexpr double kThermalPreferenceMargin = .05;
constexpr double kThermalReferenceBlend = .1;
// v35_thermal_anchor_gate (제안, 2026-11-10 실외 9735·실내 797 정상 프레임: 0.80 미만은 최장
// 0.13초, 실내 15:28 다른 물체는 0.69~0.78이 1초 넘게 이어짐). 갱신 하한은 정상 p1(0.84~0.88) 근처.
constexpr double kThermalUpdateMinSimilarity = .85;
// 2026-11-10 16:06 실내 비행: 0.80에서 진짜 차가 0.5초 넘게 하한 아래로 3번 머물러 추종이 2번 끊김.
// 0.75 아래는 최장 0.24초(2179프레임). 15:28 다른 물체는 0.69~0.78이 1초 넘게 이어졌다.
constexpr double kThermalLowSimilarity = .75;
constexpr std::int64_t kThermalMismatchWindowNs = 500000000LL;
// selected_box_jump_guard 값. 2026-11-09 실외 기록에서 정상 이동은 50ms에 약 40px, 다른 물체로의
// 1프레임 튐은 200~400px이었다. 허용 중심 이동은 직전 박스 대각선의 0.35배에 초당 3배를 더한다.
constexpr double kBoxJumpMinIou = .1;
constexpr double kBoxJumpBaseDiagonals = .35;
constexpr double kBoxJumpDiagonalsPerSecond = 3.;
constexpr std::int64_t kBoxJumpWindowNs = 500000000LL;
// v35_cv_reference_refresh: GAP_CV 출발점을 이보다 오래 두지 않는다 (GAP_CV 한도 3초보다 짧게).
constexpr std::int64_t kCvReferenceRefreshNs = 1000000000LL;
// v36_identity_context 값 (2026-11-13 사용자 확정). 근거: Pi thermal-identity 기록 33,586프레임에서
// 같은 종류 경쟁차 유사도 중앙값 0.89·선택 0.95, (경쟁 − 선택) ≥0.05가 5%, ≥0.10이 2%, ≥0.15가
// 거의 0%. 선택 혼자 p5 0.77. 가까워짐 판정은 박스 기록이 없어 경험값이다.
constexpr double kV36NearGapDiagonals = .5;  // 박스 간격 ≤ 큰 박스 대각선 × 0.5
constexpr std::int64_t kV36ReturnGapNs =
    200000000LL;                               // 이보다 오래 비면 "사라졌다 돌아옴"(기억 교체)
constexpr double kV36ReferenceBlend = .03;     // 매 프레임 갱신 비율 (가까운 차량 있으면 쉼)
constexpr std::size_t kV36RelaxedFrames = 10;  // 다른 ID 완화: 확인 프레임
constexpr double kV36RelaxedSimilarity = .92;  // 다른 ID 완화: 유사도 중앙값 하한

double v36_median(std::vector<double> values) {
    if (values.empty()) {
        return -1;
    }
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    return n % 2 ? values[n / 2] : (values[(n / 2) - 1] + values[n / 2]) / 2;
}

double jump_box_iou(const TrackRecordV1& a, const TrackRecordV1& b) {
    const double width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    const double height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);
    const double overlap = width > 0 && height > 0 ? width * height : 0;
    const double area = ((a.x2 - a.x1) * (a.y2 - a.y1)) + ((b.x2 - b.x1) * (b.y2 - b.y1)) - overlap;
    return area > 0 ? overlap / area : 0;
}
}  // namespace

// 진단용 단계 이름이다. 알 수 없는 값은 LOST로 떨어진다.
const char* target_phase_name(TargetPhase phase) {
    switch (phase) {
        case TargetPhase::Unselected:
            return "UNSELECTED";
        case TargetPhase::Locked:
            return "LOCKED";
        case TargetPhase::GapHold:
            return "GAP_HOLD";
        case TargetPhase::Reacquire:
            return "REACQUIRE";
        case TargetPhase::Confirm:
            return "CONFIRM";
        case TargetPhase::Lost:
            return "LOST";
    }
    return "LOST";
}

// 시간, 확인 횟수, 게이트, 운동 이력 중 하나라도 범위를 벗어나면 구성을 거부한다.
TargetManager::TargetManager(TargetRecoveryConfig config, ReacquisitionTrace* trace)
    : config_(config),
      reacquirer_(config, trace),
      short_motion_(config.short_motion),
      identity_motion_(config.short_motion) {
    if (config.trust_selected_id && !config.enabled) {
        throw std::invalid_argument("selected-ID trust requires target recovery");
    }
    if (!std::isfinite(config.estimated_speed_deadzone_m_s) ||
        config.estimated_speed_deadzone_m_s < 0) {
        throw std::invalid_argument("invalid estimated speed deadzone");
    }
    // max_gap은 0 초과 5초 이하여야 한다.
    const bool invalid_time =
        config.max_gap.count() <= 0 || config.max_gap > std::chrono::seconds(5);
    // confirmation_frames는 2 이상 30 이하여야 한다.
    const bool invalid_count = config.confirmation_frames < 2 || config.confirmation_frames > 30;
    // 중심 거리, 면적 비, 지면 게이트의 유한성과 범위를 검사한다.
    const bool invalid_gate =
        !std::isfinite(config.max_center_distance_diagonals) ||
        config.max_center_distance_diagonals <= 0 || !std::isfinite(config.min_area_ratio) ||
        !std::isfinite(config.max_area_ratio) || config.min_area_ratio <= 0 ||
        config.min_area_ratio > 1 || config.max_area_ratio < 1 ||
        !std::isfinite(config.ground_gate_m) || config.ground_gate_m <= 0 ||
        config.ground_gate_m > 10 || !std::isfinite(config.ground_gate_growth_m_s) ||
        config.ground_gate_growth_m_s < 0 || config.ground_gate_growth_m_s > 5 ||
        (config.ground_gate_growth_m_s > 0 &&
         (!std::isfinite(config.ground_gate_max_m) ||
          config.ground_gate_max_m < config.ground_gate_m || config.ground_gate_max_m > 10)) ||
        config.trusted_return_check.count() < 0 ||
        config.trusted_return_check > std::chrono::seconds(5) ||
        (config.trusted_return_check.count() > 0 && !config.trust_selected_id);
    // 운동 이력 길이, 최소 표본, 적합 RMSE, 표본 간격을 검사한다.
    const bool invalid_motion =
        !std::isfinite(config.motion_history_s) || config.motion_history_s <= 0 ||
        config.motion_history_s > 5 || !std::isfinite(config.motion_history_min_s) ||
        config.motion_history_min_s <= 0 ||
        config.motion_history_min_s >= config.motion_history_s ||
        config.motion_history_min_samples < 3 || config.motion_history_min_samples > 128 ||
        !std::isfinite(config.motion_fit_max_rmse_m) || config.motion_fit_max_rmse_m <= 0 ||
        config.motion_fit_max_rmse_m > 2 || config.motion_max_sample_gap.count() <= 0 ||
        config.motion_max_sample_gap > std::chrono::seconds(1);
    if (invalid_time || invalid_count || invalid_gate || invalid_motion) {
        throw std::invalid_argument("invalid target recovery configuration");
    }
}

// 단계가 바뀔 때만 revision을 올린다. Locked가 아니면 운동 기억을 지운다.
// 짧은 공백 유지가 켜져 있으면 Lost와 Unselected가 아닐 때만 그 기억을 남긴다.
void TargetManager::set_phase(TargetPhase phase, const char* reason) {
    if (phase_ != phase) {
        ++revision_;
    }
    const bool brief_continuity = brief_continuity_memory_ && (phase == TargetPhase::GapHold ||
                                                               phase == TargetPhase::Reacquire ||
                                                               phase == TargetPhase::Confirm);
    if (!brief_continuity && phase != TargetPhase::Locked) {
        brief_continuity_memory_.reset();
    }
    if (phase != TargetPhase::Locked && !brief_continuity) {
        for (auto& competitor : competitors_) {
            competitor.selected_continuity_proven = false;
        }
        if (!config_.retain_motion_on_brief_gap || phase == TargetPhase::Lost ||
            phase == TargetPhase::Unselected) {
            ground_history_.clear();
            short_motion_.reset();
            identity_motion_.reset();
            box_motion_guard_.reset();
            reset_previous_moving_velocity_.reset();
        }
        if (memory_) {
            memory_->ground_motion_valid = false;
            memory_->ground_acceleration_valid = false;
        }
        if (memory_) {
            memory_->short_motion = {};
            memory_->identity_short_motion = {};
        }
    }
    phase_ = phase;
    reason_ = reason;
}

void TargetManager::reset_confirmation() {
    candidate_key_.reset();
    confirmation_track_.reset();
    confirmation_count_ = 0;
}

// 새 임무 ID를 발급한 뒤 start로 확정 기억과 Locked를 만든다.
void TargetManager::select(const TracksV1& frame, const TrackRecordV1& track,
                           Clock::time_point received_at) {
    memory_ = TargetMemory{};
    memory_->mission_target_id = ++next_mission_id_;
    ++revision_;
    start(frame, track, received_at);
}

void TargetManager::start(const TracksV1& frame, const TrackRecordV1& track,
                          Clock::time_point received_at) {
    if (!memory_) {
        throw std::logic_error("target start requires a selection");
    }
    brief_continuity_memory_.reset();
    // 명시 Start는 최신 입력 맥락을 다시 승인한다. 자동 복귀에서는 이 함수를 호출하지 않는다.
    memory_->source = frame.source;
    memory_->clock_domain = frame.clock_domain;
    memory_->timestamp_source = frame.timestamp_source;
    memory_->exposure_time_valid = frame.exposure_time_valid;
    memory_->image_width = frame.image_width;
    memory_->image_height = frame.image_height;
    memory_->last_real_received_at = {};
    memory_->cv_velocity_north_m_s = memory_->cv_velocity_east_m_s = 0;
    memory_->cv_velocity_trusted = false;
    memory_->cv_last_observed_moving = false;
    memory_->cv_velocity_unconfirmed = false;
    memory_->cv_velocity_fit_sample_ns = 0;
    memory_->cv_ten_frame.reset();
    memory_->cv_visibility_reference.reset();
    memory_->velocity_x_px_s = memory_->velocity_y_px_s = 0;
    memory_->velocity_north_m_s = memory_->velocity_east_m_s = 0;
    memory_->acceleration_north_m_s2 = memory_->acceleration_east_m_s2 = 0;
    ground_history_.clear();
    short_motion_.reset();
    identity_motion_.reset();
    box_motion_guard_.reset();
    fresh_contact_velocity_.reset();
    reset_previous_moving_velocity_.reset();
    competitors_.clear();
    box_jump_since_ns_.reset();
    thermal_mismatch_since_ns_.reset();
    thermal_reference_ = track.thermal;
    thermal_anchor_ = track.thermal;
    thermal_low_similarity_ = false;
    v36_reset();
    remember(frame, track, received_at);
    last_frame_id_ = frame.frame_id;
    last_sample_ns_ = frame.sample_monotonic_ns;
    reset_confirmation();
    set_phase(TargetPhase::Locked, "confirmed_target");
}

// 운용자 정지다. 기억과 경쟁차, 확인 상태를 없애고 Unselected로 돌아간다.
void TargetManager::stop() {
    memory_.reset();
    ground_history_.clear();
    short_motion_.reset();
    identity_motion_.reset();
    box_motion_guard_.reset();
    fresh_contact_velocity_.reset();
    reset_previous_moving_velocity_.reset();
    competitors_.clear();
    box_jump_since_ns_.reset();
    thermal_reference_.reset();
    thermal_anchor_.reset();
    thermal_low_similarity_ = false;
    thermal_mismatch_since_ns_.reset();
    v36_reset();
    reset_confirmation();
    ++revision_;
    set_phase(TargetPhase::Unselected, "operator_stop");
}

// 기억은 남기고 확인만 끊은 채 Lost로 보낸다. 선택이 없으면 아무 일도 하지 않는다.
void TargetManager::invalidate(const char* reason) {
    if (!memory_) {
        return;
    }
    reset_confirmation();
    fresh_contact_velocity_.reset();
    box_jump_since_ns_.reset();
    thermal_mismatch_since_ns_.reset();
    v36_reset();
    set_phase(TargetPhase::Lost, reason);
}

void TargetManager::v36_reset() {
    v36_relaxed_key_.reset();
    v36_relaxed_similarity_.clear();
    v36_last_seen_ns_ = 0;
}

bool TargetManager::v36_thermal_usable(const TrackRecordV1& track) const {
    return track.thermal.has_value() &&
           track.thermal.value_or(ThermalSignature{}).at(1) >= config_.thermal_min_contrast;
}

// 기억 또는 트랙의 열 외형이 판단에 쓸 수 없으면 -1.
double TargetManager::v36_similarity(const std::optional<ThermalSignature>& reference,
                                     const TrackRecordV1& track) const {
    if (!reference || reference->at(1) < config_.thermal_min_contrast ||
        !v36_thermal_usable(track)) {
        return -1;
    }
    return compare_thermal(*reference, track.thermal.value_or(ThermalSignature{})).histogram;
}

std::pair<double, std::int64_t> TargetManager::v36_best_other(
    const std::optional<ThermalSignature>& reference, const TracksV1& frame,
    const TrackRecordV1& track) const {
    double best = -1;
    std::int64_t best_id = -1;
    for (const auto& other : frame.tracks) {
        if (other.track_id == track.track_id || other.class_id != track.class_id ||
            other.label != track.label || !other.real_detection) {
            continue;
        }
        const double similarity = v36_similarity(reference, other);
        if (similarity > best) {
            best = similarity;
            best_id = other.track_id;
        }
    }
    return {best, best_id};
}

// 같은 종류 실검출 다른 트랙의 박스가 선택 박스와 겹치거나, 두 박스 사이 간격이 큰 박스 대각선의
// kV36NearGapDiagonals배 이내이면 true.
bool TargetManager::v36_competitor_near(const TracksV1& frame, const TrackRecordV1& track) const {
    const double track_diagonal = std::hypot(track.x2 - track.x1, track.y2 - track.y1);
    for (const auto& other : frame.tracks) {
        if (other.track_id == track.track_id || other.class_id != track.class_id ||
            other.label != track.label || !other.real_detection) {
            continue;
        }
        const double gap_x =
            std::max(0., std::max(track.x1, other.x1) - std::min(track.x2, other.x2));
        const double gap_y =
            std::max(0., std::max(track.y1, other.y1) - std::min(track.y2, other.y2));
        const double diagonal =
            std::max(track_diagonal, std::hypot(other.x2 - other.x1, other.y2 - other.y1));
        if (std::hypot(gap_x, gap_y) <= kV36NearGapDiagonals * diagonal) {
            return true;
        }
    }
    return false;
}

void TargetManager::v36_log(const TracksV1& frame, const TrackRecordV1& track, const char* context,
                            double selected, double other, std::int64_t other_id,
                            const char* decision) {
    if (config_.thermal_identity_log.empty()) {
        return;
    }
    if (!thermal_log_) {
        thermal_log_ = std::make_shared<std::ofstream>(config_.thermal_identity_log, std::ios::app);
    }
    if (!thermal_log_->good()) {
        return;
    }
    *thermal_log_ << std::setprecision(6) << R"({"sample_ns":)" << frame.sample_monotonic_ns
                  << R"(,"frame_id":)" << frame.frame_id << R"(,"track_id":)" << track.track_id
                  << R"(,"mode":"v36","context":")" << context << R"(","has_thermal":)"
                  << (track.thermal ? "true" : "false") << R"(,"selected_similarity":)" << selected
                  << R"(,"best_other_id":)" << other_id << R"(,"best_other_similarity":)" << other
                  << R"(,"decision":")" << decision << '"';
    if (track.thermal) {
        *thermal_log_ << R"(,"signature":[)";
        for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
            *thermal_log_ << (i ? "," : "") << track.thermal->at(i);
        }
        *thermal_log_ << "]";
    }
    *thermal_log_ << "}\n";
    thermal_log_->flush();
}

// 2026-11-13 사용자 확정 신원 판단 (target_memory.hpp v36_identity_context 설명).
// 2026-11-16 사용자 확정 변경: "아이디를 최대한 믿는다. ID가 안 바뀌면 바꾸지 말고 계속 새것으로
// 갱신한다." 14:58 실내 비행에서 대상이 돌아서며(박스 폭 188→44px) 탐지가 1초 끊긴 뒤 같은 ID
// 20으로 돌아왔으나, 복귀 검증 유사도 0.60~0.66(하한 0.70)으로 거부돼 상실됐다. 이제 선택 ID가
// 그대로 있으면 열 외형으로 보류·해제하지 않는다(복귀 검증·급변 하한·겹침 뒤 바뀜 판단 모두
// 기록만). 기억은 매 프레임 새 외형으로 갱신하고, 사라졌다 돌아오면 돌아온 외형으로 바로 바꾼다.
// 다른 같은 종류 차량과 박스가 가까울 때만 섞인 외형이 들어가지 않게 갱신을 쉰다.
// 상자 튐 검사(selected_box_jump_guard)와 지면 예측 복귀 검사(trusted_return_check)는 그대로다.
TargetManager::V36Decision TargetManager::v36_selected_identity(const TracksV1& frame,
                                                                const TrackRecordV1& track) {
    const std::int64_t now = frame.sample_monotonic_ns;
    // "사라짐"은 화면에 보이지 않은 시간으로 잰다.
    const std::int64_t last_seen =
        v36_last_seen_ns_ > 0 ? v36_last_seen_ns_ : memory_->last_real_sample_ns;
    v36_last_seen_ns_ = now;
    const bool returned = now - last_seen >= kV36ReturnGapNs;
    const bool near = v36_competitor_near(frame, track);

    // 진단용 값: 지금 기억·처음 기억과의 유사도, 가장 닮은 다른 같은 종류 차량.
    const double selected = v36_similarity(thermal_reference_, track);
    const double anchor = v36_similarity(thermal_anchor_, track);
    const auto [other, other_id] = v36_best_other(thermal_reference_, frame, track);
    const bool low = selected >= 0 && std::max(selected, anchor) < kThermalLowSimilarity;
    thermal_mismatch_since_ns_.reset();

    const char* context = returned ? "return" : (near ? "near" : "clear");
    const char* decision = "trusted_id_updated";
    if (v36_thermal_usable(track)) {
        const ThermalSignature current = track.thermal.value_or(ThermalSignature{});
        if (!thermal_anchor_ || thermal_anchor_->at(1) < config_.thermal_min_contrast) {
            thermal_anchor_ = current;
        }
        if (returned || !thermal_reference_ ||
            thermal_reference_->at(1) < config_.thermal_min_contrast) {
            thermal_reference_ = current;
            decision = "trusted_id_reference_replaced";
        } else if (near) {
            decision = "trusted_id_near_not_updated";
        } else {
            ThermalSignature blended = *thermal_reference_;
            for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
                blended.at(i) += kV36ReferenceBlend * (current.at(i) - blended.at(i));
            }
            thermal_reference_ = blended;
            if (low) {
                decision = "trusted_id_low_updated";
            }
        }
    } else {
        decision = "trusted_id_no_thermal";
    }
    v36_log(frame, track, context, selected, other, other_id, decision);
    return V36Decision::Accept;
}

// 직전 수용 박스와 겹침이 거의 없고 시간에 비해 중심이 너무 멀면 다른 물체로 본다.
// 수용이 끊긴 지 kBoxJumpWindowNs를 넘으면 허용 범위를 더 늘리지 않는다.
bool TargetManager::selected_box_jumped(const TracksV1& frame, const TrackRecordV1& track) const {
    if (!memory_) {
        return false;
    }
    const auto& previous = memory_->last_real_track;
    const auto elapsed_ns = frame.sample_monotonic_ns - memory_->last_real_sample_ns;
    if (elapsed_ns <= 0 || previous.x2 <= previous.x1 || previous.y2 <= previous.y1) {
        return false;
    }
    if (jump_box_iou(previous, track) >= kBoxJumpMinIou) {
        return false;
    }
    const double elapsed_s = static_cast<double>(std::min(elapsed_ns, kBoxJumpWindowNs)) / 1e9;
    const double diagonal = std::hypot(previous.x2 - previous.x1, previous.y2 - previous.y1);
    const double shift = std::hypot(((track.x1 + track.x2) - (previous.x1 + previous.x2)) / 2,
                                    ((track.y1 + track.y2) - (previous.y1 + previous.y2)) / 2);
    return shift > diagonal * (kBoxJumpBaseDiagonals + (kBoxJumpDiagonalsPerSecond * elapsed_s));
}

// 선택 대상의 열 분포 기억과 같은 종류 후보들을 비교한다(상대 비교, 2026-11-10).
// 다른 후보가 선택 ID보다 kThermalPreferenceMargin 이상 더 닮았을 때만 불일치로 본다.
// 두 대상이 비슷하면 차이가 작아 스스로 판단하지 않는다. 대비가 약하거나 열 정보가 없는
// 후보(NUC 직후 등)는 비교에서 빠진다. 불일치가 아니면 기억을 천천히 갱신한다.
bool TargetManager::selected_thermal_mismatch(const TracksV1& frame, const TrackRecordV1& track) {
    const auto usable = [this](const TrackRecordV1& candidate) {
        return candidate.thermal.has_value() &&
               candidate.thermal.value_or(ThermalSignature{}).at(1) >= config_.thermal_min_contrast;
    };
    const bool selected_usable = usable(track);
    const bool reference_usable =
        thermal_reference_.has_value() &&
        thermal_reference_.value_or(ThermalSignature{}).at(1) >= config_.thermal_min_contrast;
    double selected_similarity = -1;
    double best_other_similarity = -1;
    std::int64_t best_other_id = -1;
    int compared_others = 0;
    if (selected_usable && reference_usable) {
        const ThermalSignature reference = thermal_reference_.value_or(ThermalSignature{});
        selected_similarity =
            compare_thermal(reference, track.thermal.value_or(ThermalSignature{})).histogram;
        for (const auto& other : frame.tracks) {
            if (other.track_id == track.track_id || other.class_id != track.class_id ||
                other.label != track.label || !other.real_detection || !usable(other)) {
                continue;
            }
            ++compared_others;
            const double similarity =
                compare_thermal(reference, other.thermal.value_or(ThermalSignature{})).histogram;
            if (similarity > best_other_similarity) {
                best_other_similarity = similarity;
                best_other_id = other.track_id;
            }
        }
    }
    const bool relative_mismatch =
        compared_others > 0 &&
        best_other_similarity - selected_similarity >= kThermalPreferenceMargin;
    // 2026-11-10 원래 기준(선택 시점) 유사도와 절대 하한. 둘 중 더 닮은 쪽마저 하한 미만이면
    // 비교 후보가 없어도 같은 대상으로 보지 않는다.
    const bool anchor_usable =
        thermal_anchor_.has_value() &&
        thermal_anchor_.value_or(ThermalSignature{}).at(1) >= config_.thermal_min_contrast;
    double anchor_similarity = -1;
    if (config_.v35_thermal_anchor_gate && selected_usable && anchor_usable) {
        anchor_similarity = compare_thermal(thermal_anchor_.value_or(ThermalSignature{}),
                                            track.thermal.value_or(ThermalSignature{}))
                                .histogram;
    }
    const bool low_similarity =
        config_.v35_thermal_anchor_gate && selected_similarity >= 0 &&
        std::max(selected_similarity, anchor_similarity) < kThermalLowSimilarity;
    const bool mismatch = relative_mismatch || low_similarity;
    thermal_low_similarity_ = low_similarity && !relative_mismatch;
    // 갱신 여부: 기존은 불일치가 아니면 항상. 켜면 유사도가 충분히 높을 때만.
    const bool update_reference = !mismatch && selected_usable &&
                                  (!config_.v35_thermal_anchor_gate || !reference_usable ||
                                   selected_similarity >= kThermalUpdateMinSimilarity);
    if (!config_.thermal_identity_log.empty()) {
        if (!thermal_log_) {
            thermal_log_ =
                std::make_shared<std::ofstream>(config_.thermal_identity_log, std::ios::app);
        }
        if (thermal_log_->good()) {
            *thermal_log_ << std::setprecision(6) << R"({"sample_ns":)" << frame.sample_monotonic_ns
                          << R"(,"frame_id":)" << frame.frame_id << R"(,"track_id":)"
                          << track.track_id << R"(,"mode":")"
                          << (config_.thermal_identity == ThermalIdentityMode::On ? "on" : "log")
                          << R"(","has_thermal":)" << (track.thermal ? "true" : "false")
                          << R"(,"reference":)" << (reference_usable ? "true" : "false")
                          << R"(,"selected_similarity":)" << selected_similarity << R"(,"others":)"
                          << compared_others << R"(,"best_other_id":)" << best_other_id
                          << R"(,"best_other_similarity":)" << best_other_similarity
                          << R"(,"mismatch":)" << (mismatch ? "true" : "false");
            if (config_.v35_thermal_anchor_gate) {
                // 2026-11-10 하한·갱신 기준 조정용: 원래 기준 유사도, 하한 판단, 갱신 여부, 현재
                // 외형.
                *thermal_log_ << R"(,"anchor_similarity":)" << anchor_similarity
                              << R"(,"low_similarity":)" << (low_similarity ? "true" : "false")
                              << R"(,"updated":)" << (update_reference ? "true" : "false");
                if (track.thermal) {
                    *thermal_log_ << R"(,"signature":[)";
                    for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
                        *thermal_log_ << (i ? "," : "") << track.thermal->at(i);
                    }
                    *thermal_log_ << "]";
                }
            }
            *thermal_log_ << "}\n";
            thermal_log_->flush();
        }
    }
    if (update_reference) {
        const ThermalSignature current = track.thermal.value_or(ThermalSignature{});
        if (!anchor_usable) {
            // 선택 시점에 열 정보가 없었으면 첫 유효 외형을 원래 기준으로 삼는다.
            thermal_anchor_ = current;
        }
        if (!reference_usable) {
            thermal_reference_ = current;
        } else {
            ThermalSignature blended = thermal_reference_.value_or(ThermalSignature{});
            for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
                blended.at(i) += kThermalReferenceBlend * (current.at(i) - blended.at(i));
            }
            thermal_reference_ = blended;
        }
    }
    return mismatch;
}

void TargetManager::pause_ambiguous(const char* reason) {
    if (!memory_ || phase_ == TargetPhase::Lost) {
        return;
    }
    // 대상을 구분하지 못한 공백은 동일 binding의 짧은 관측 누락과 다르다.
    brief_continuity_memory_.reset();
    ground_history_.clear();
    short_motion_.reset();
    identity_motion_.reset();
    box_motion_guard_.reset();
    reset_previous_moving_velocity_.reset();
    fresh_contact_velocity_.reset();
    reset_confirmation();
    set_phase(TargetPhase::GapHold, reason);
}

// 같은 generation의 track id만 이미 아는 경쟁차로 본다.
bool TargetManager::known_competitor(const TracksV1& frame, const TrackRecordV1& track) const {
    return std::any_of(competitors_.begin(), competitors_.end(), [&](const auto& competitor) {
        return competitor.key.generation == frame.generation &&
               competitor.key.track_id == track.track_id;
    });
}

// Locked이고 짧은 운동이 유효할 때만, 예측이 서로 바뀐 연결보다 분리된 신뢰 경쟁차를
// 표적 후보에서 뺀다.
std::optional<TracksV1> TargetManager::position_identity_candidates(
    const TracksV1& frame, std::chrono::milliseconds max_age) const {
    if (!memory_ || !config_.prefer_selected_continuity || !config_.allow_competitor_departure ||
        !config_.short_motion.enabled || phase_ != TargetPhase::Locked ||
        frame.generation != memory_->binding.generation || !memory_->last_real_track.ground) {
        return std::nullopt;
    }
    const auto gap = frame.sample_monotonic_ns - memory_->last_real_sample_ns;
    const auto max_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(max_age).count();
    const auto stamp = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
    const double dt =
        (stamp - memory_->last_real_frame.motion_sample_ns.value_or(memory_->last_real_sample_ns)) /
        1e9;
    if (gap <= 0 || gap > max_ns || dt <= 0 || dt > config_.short_motion.maximum_sample_gap_s) {
        return std::nullopt;
    }
    const auto fresh = [&](const TrackRecordV1& t) {
        return t.ground && t.state == "CONFIRMED" && t.confidence >= .3 && t.real_detection &&
               t.last_real_detection_monotonic_ns == frame.sample_monotonic_ns &&
               t.class_id == memory_->last_real_track.class_id &&
               t.label == memory_->last_real_track.label;
    };
    const auto selected = std::find_if(
        frame.tracks.begin(), frame.tracks.end(),
        [&](const auto& t) { return t.track_id == memory_->binding.track_id && fresh(t); });
    if (selected == frame.tracks.end() ||
        selected->ground_anchor != memory_->last_real_track.ground_anchor) {
        return std::nullopt;
    }
    const auto iou = [](const TrackRecordV1& a, const TrackRecordV1& b) {
        const double intersection = std::max(0., std::min(a.x2, b.x2) - std::max(a.x1, b.x1)) *
                                    std::max(0., std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
        const double area =
            (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - intersection;
        return area > 0 ? intersection / area : 0.;
    };
    const auto& prior = memory_->last_real_track;
    const double tolerance = config_.short_motion.maximum_rmse_m;
    const auto distance = [](const TrackRecordV1& t, double n, double e) {
        return std::hypot((*t.ground)[0] - n, (*t.ground)[1] - e);
    };
    const double own_error = distance(*selected, (*prior.ground)[0], (*prior.ground)[1]);
    if (iou(*selected, prior) < .5 || own_error > 2 * tolerance) {
        return std::nullopt;
    }
    // 알려진 신뢰 차량이 사라졌을 때의 보호를 새 위치 전용 근거로 대체하지 않는다.
    for (const auto& known : competitors_) {
        if (known.trusted && !known.departed &&
            std::none_of(frame.tracks.begin(), frame.tracks.end(), [&](const auto& t) {
                return known.key.generation == frame.generation &&
                       t.track_id == known.key.track_id && fresh(t);
            })) {
            return std::nullopt;
        }
    }
    for (const auto& other : frame.tracks) {
        if (other.track_id == selected->track_id || other.class_id != prior.class_id ||
            other.label != prior.label) {
            continue;
        }
        if (!fresh(other) || iou(*selected, other) > 0 || iou(prior, other) > 0 ||
            distance(other, (*prior.ground)[0], (*prior.ground)[1]) - own_error < 4 * tolerance) {
            return std::nullopt;
        }
        const auto known =
            std::find_if(competitors_.begin(), competitors_.end(), [&](const auto& k) {
                return k.key.generation == frame.generation && k.key.track_id == other.track_id;
            });
        if (known == competitors_.end()) {
            if (competitors_.size() >= kMaxCompetitors) {
                return std::nullopt;
            }
            continue;
        }
        const auto age = frame.sample_monotonic_ns - known->real_sample_ns;
        const double other_dt = (stamp - known->motion_sample_ns) / 1e9;
        if (known->departed || !known->last_track.ground || age <= 0 || age > max_ns ||
            other_dt <= 0 || other_dt > config_.short_motion.maximum_sample_gap_s ||
            iou(other, known->last_track) < .5) {
            return std::nullopt;
        }
        const auto& m = known->short_motion;
        const double n =
            m.valid ? m.north_m + m.velocity_north_m_s * other_dt : (*known->last_track.ground)[0];
        const double e =
            m.valid ? m.east_m + m.velocity_east_m_s * other_dt : (*known->last_track.ground)[1];
        if (distance(other, n, e) > 2 * tolerance) {
            return std::nullopt;
        }
    }
    auto result = frame;
    result.tracks = {*selected};
    if (std::getenv("ARGUS_TRIAL_TRACE") && identity_diagnostic_frame_ != frame.frame_id) {
        identity_diagnostic_frame_ = frame.frame_id;
        std::cerr << "TM_POSITION_IDENTITY frame=" << frame.frame_id
                  << " selected=" << selected->track_id
                  << " source_ns=" << frame.sample_monotonic_ns
                  << " peers=" << frame.tracks.size() - 1 << " own_error=" << own_error
                  << " motion_valid=" << memory_->short_motion.valid
                  << " box_suspect=" << memory_->box_motion_suspect << '\n';
    }
    return result;
}

const ShortMotionEstimate& TargetManager::identity_motion(const TargetMemory& memory) const {
    if (config_.trust_selected_id) {
        return memory.short_motion;
    }
    return config_.prefer_selected_continuity ? memory.identity_short_motion : memory.short_motion;
}

TracksV1 TargetManager::ground_candidates(const TracksV1& frame,
                                          std::chrono::milliseconds max_tracks_age) const {
    auto candidates = frame;
    if (config_.trust_selected_id && !memory_->short_motion.valid) {
        const auto identity = position_identity_candidates(frame, max_tracks_age);
        return identity ? *identity : candidates;
    }
    if (!config_.allow_competitor_departure || !config_.short_motion.enabled ||
        phase_ != TargetPhase::Locked || !identity_motion(*memory_).valid) {
        return candidates;
    }
    const auto fresh = [&](const TrackRecordV1& track) {
        return track.ground && track.state == "CONFIRMED" && track.confidence >= .3 &&
               track.real_detection &&
               track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns &&
               track.class_id == memory_->last_real_track.class_id &&
               track.label == memory_->last_real_track.label;
    };
    const auto target =
        std::find_if(frame.tracks.begin(), frame.tracks.end(), [&](const auto& track) {
            return frame.generation == memory_->binding.generation &&
                   track.track_id == memory_->binding.track_id && fresh(track);
        });
    if (target == frame.tracks.end()) {
        return candidates;
    }
    const auto stamp = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
    const auto max_age_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count();
    const double identity_window =
        std::min(1., config_.short_motion.window_s + config_.short_motion.maximum_sample_gap_s);
    const auto identity_age_ns = max_age_ns + std::int64_t(config_.short_motion.window_s * 1e9);
    const auto predict = [&](const ShortMotionEstimate& motion, std::int64_t real_ns,
                             std::int64_t motion_ns, std::int64_t allowed_age_ns,
                             double allowed_gap_s) -> std::optional<std::array<double, 2>> {
        // 실관측 나이와 운동 표본 간격이 허용 창을 넘으면 예측을 쓰지 않는다.
        const auto age = frame.sample_monotonic_ns - real_ns;
        const double dt = (stamp - motion_ns) / 1e9;
        if (!motion.valid || age <= 0 || age > allowed_age_ns || dt <= 0 || dt > allowed_gap_s) {
            return std::nullopt;
        }
        return std::array<double, 2>{motion.north_m + motion.velocity_north_m_s * dt,
                                     motion.east_m + motion.velocity_east_m_s * dt};
    };
    const auto target_prediction =
        predict(identity_motion(*memory_), memory_->last_real_sample_ns,
                memory_->last_real_frame.motion_sample_ns.value_or(memory_->last_real_sample_ns),
                max_age_ns, config_.short_motion.maximum_sample_gap_s);
    if (!target_prediction) {
        return candidates;
    }
    const auto distance = [](const TrackRecordV1& track, const std::array<double, 2>& predicted) {
        return std::hypot((*track.ground)[0] - predicted[0], (*track.ground)[1] - predicted[1]);
    };
    const double tolerance = config_.short_motion.maximum_rmse_m;
    const double target_error = distance(*target, *target_prediction);
    if (target_error > 2 * tolerance) {
        return candidates;
    }
    const auto box_iou = [](const TrackRecordV1& a, const TrackRecordV1& b) {
        const double width = std::max(0., std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
        const double height = std::max(0., std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
        const double intersection = width * height;
        const double area =
            (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - intersection;
        return area > 0 ? intersection / area : 0.;
    };
    // 새 경쟁차의 이력이 쌓이는 동안에도 원선택의 명확한 실관측 연결만 보존한다.
    if (config_.prefer_selected_continuity) {
        auto selected_only = frame;
        selected_only.tracks = {*target};
        // 내부 선별을 실제 후보 진단 기록과 섞지 않는다.
        const auto selected = TargetReacquirer{config_}.find_candidate(*memory_, selected_only);
        if (selected.kind == ReacquisitionResult::Kind::Unique &&
            box_iou(*target, memory_->last_real_track) >= .5) {
            for (const auto& other : frame.tracks) {
                if (other.track_id == target->track_id || !fresh(other)) {
                    continue;
                }
                const auto known = std::find_if(
                    competitors_.begin(), competitors_.end(), [&](const auto& competitor) {
                        return competitor.key.generation == frame.generation &&
                               competitor.key.track_id == other.track_id;
                    });
                // 기억할 공간이 없으면 초기 적용 기한을 추적할 수 없다.
                if (known == competitors_.end() && competitors_.size() >= kMaxCompetitors) {
                    continue;
                }
                if (known != competitors_.end()) {
                    const auto first_age = frame.sample_monotonic_ns - known->first_real_sample_ns;
                    if (known->first_real_sample_ns <= 0 || first_age < 0 ||
                        first_age > max_age_ns || (known->trusted && known->short_motion.valid)) {
                        continue;
                    }
                }
                if (box_iou(*target, other) == 0 && box_iou(memory_->last_real_track, other) == 0 &&
                    distance(other, *target_prediction) - target_error >= 4 * tolerance) {
                    candidates.tracks.erase(
                        std::remove_if(
                            candidates.tracks.begin(), candidates.tracks.end(),
                            [&](const auto& track) { return track.track_id == other.track_id; }),
                        candidates.tracks.end());
                }
            }
        }
    }
    for (const auto& competitor : competitors_) {
        if (!competitor.trusted) {
            continue;
        }
        const auto other =
            std::find_if(frame.tracks.begin(), frame.tracks.end(), [&](const auto& track) {
                return frame.generation == competitor.key.generation &&
                       track.track_id == competitor.key.track_id && fresh(track);
            });
        if (other == frame.tracks.end()) {
            continue;
        }
        const auto prediction =
            predict(competitor.short_motion, competitor.real_sample_ns, competitor.motion_sample_ns,
                    identity_age_ns, identity_window);
        if (!prediction) {
            continue;
        }
        const double other_error = distance(*other, *prediction);
        const double keep = target_error + other_error;
        const double swap = distance(*other, *target_prediction) + distance(*target, *prediction);
        const auto& prior = memory_->last_real_track;
        const auto& other_prior = competitor.last_track;
        const double own_image = box_iou(*target, prior);
        const double other_image = box_iou(*other, other_prior);
        const double swapped_image = box_iou(*target, other_prior) + box_iou(*other, prior);
        // 지면 불확실성이 겹쳐도 두 ID의 영상 연결이 명확하면 현재 선택을 유지한다.
        // 교환 쪽 지면 대응이 더 좋거나 실제 상자가 섞이면 이 예외를 쓰지 않는다.
        const bool image_continuity =
            config_.prefer_selected_continuity &&
            competitor.real_sample_ns == memory_->last_real_sample_ns &&
            frame.sample_monotonic_ns - competitor.real_sample_ns <= max_age_ns &&
            own_image >= .5 && other_image >= .5 && box_iou(*target, *other) < .5 &&
            box_iou(prior, other_prior) < .5 && own_image + other_image - swapped_image >= .5 &&
            swap >= keep;
        if (other_error <= 2 * tolerance && (swap - keep >= 4 * tolerance || image_continuity)) {
            candidates.tracks.erase(
                std::remove_if(
                    candidates.tracks.begin(), candidates.tracks.end(),
                    [&](const auto& track) { return track.track_id == other->track_id; }),
                candidates.tracks.end());
        }
    }
    return candidates;
}

// 이번 프레임에 없는 경쟁차가 원표적과 다른 경로인지 짧은 가림에서만 판정한다.
bool TargetManager::missing_competitor_is_separate(const TracksV1& frame,
                                                   const CompetitorMemory& competitor,
                                                   std::chrono::milliseconds max_tracks_age) const {
    const bool recovering_identity = brief_continuity_memory_ && (phase_ != TargetPhase::Locked ||
                                                                  !identity_motion(*memory_).valid);
    const auto& evidence = recovering_identity ? *brief_continuity_memory_ : *memory_;
    if (!config_.allow_competitor_departure || !config_.short_motion.enabled ||
        !competitor.trusted || (phase_ != TargetPhase::Locked && !brief_continuity_memory_) ||
        !identity_motion(evidence).valid || !competitor.short_motion.valid) {
        return false;
    }
    const auto target =
        std::find_if(frame.tracks.begin(), frame.tracks.end(), [&](const auto& track) {
            return frame.generation == evidence.binding.generation &&
                   track.track_id == evidence.binding.track_id && track.ground &&
                   track.state == "CONFIRMED" && track.confidence >= .3 && track.real_detection &&
                   track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns &&
                   track.class_id == evidence.last_real_track.class_id &&
                   track.label == evidence.last_real_track.label;
        });
    if (target == frame.tracks.end()) {
        return false;
    }
    const auto stamp = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
    const double target_dt =
        (stamp - evidence.last_real_frame.motion_sample_ns.value_or(evidence.last_real_sample_ns)) /
        1e9;
    const double other_dt = (stamp - competitor.motion_sample_ns) / 1e9;
    const auto max_age_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count();
    const auto target_age = frame.sample_monotonic_ns - evidence.last_real_sample_ns;
    const auto other_age = frame.sample_monotonic_ns - competitor.real_sample_ns;
    const double window =
        std::min(1., config_.short_motion.window_s + config_.short_motion.maximum_sample_gap_s);
    if (target_age <= 0 || target_age > max_age_ns || target_dt <= 0 ||
        target_dt > config_.short_motion.maximum_sample_gap_s || other_age <= 0 ||
        other_age > max_age_ns + std::int64_t(config_.short_motion.window_s * 1e9) ||
        other_dt <= 0 || other_dt > window) {
        return false;
    }
    const auto error = [&](const ShortMotionEstimate& motion, double dt) {
        return std::hypot((*target->ground)[0] - motion.north_m - motion.velocity_north_m_s * dt,
                          (*target->ground)[1] - motion.east_m - motion.velocity_east_m_s * dt);
    };
    const double own = error(identity_motion(evidence), target_dt);
    // 경쟁차 예측은 짧은 가림의 신원 배제에만 쓴다. 원표적 명령은 현재 실관측으로만 갱신한다.
    return own <= 2 * config_.short_motion.maximum_rmse_m &&
           error(competitor.short_motion, other_dt) - own >=
               4 * config_.short_motion.maximum_rmse_m;
}

// 같은 물리 경로의 두 실관측이 ID만 맞바꾼 경우에만 두 기억을 함께 연결한다.
bool TargetManager::reconcile_identity_swap(const TracksV1& frame, Clock::time_point received_at,
                                            std::chrono::milliseconds max_tracks_age) {
    if (!config_.prefer_selected_continuity || !config_.allow_competitor_departure ||
        phase_ != TargetPhase::Locked || !identity_motion(*memory_).valid ||
        frame.generation != memory_->binding.generation) {
        return false;
    }
    const auto age =
        received_at - Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
    const auto max_age_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count();
    const auto own_gap = frame.sample_monotonic_ns - memory_->last_real_sample_ns;
    const auto stamp = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
    const double own_dt =
        (stamp - memory_->last_real_frame.motion_sample_ns.value_or(memory_->last_real_sample_ns)) /
        1e9;
    if (age < Clock::duration::zero() || age > max_tracks_age || own_gap <= 0 ||
        own_gap > max_age_ns || own_dt <= 0 || own_dt > config_.short_motion.maximum_sample_gap_s) {
        return false;
    }
    const auto fresh = [&](const TrackRecordV1& t) {
        return t.ground && t.real_detection && t.state == "CONFIRMED" && t.confidence >= .3 &&
               t.last_real_detection_monotonic_ns == frame.sample_monotonic_ns &&
               t.class_id == memory_->last_real_track.class_id &&
               t.label == memory_->last_real_track.label;
    };
    if (std::count_if(frame.tracks.begin(), frame.tracks.end(), fresh) != 2) {
        return false;
    }
    const auto old_id = memory_->binding.track_id;
    const auto old_native =
        std::find_if(frame.tracks.begin(), frame.tracks.end(),
                     [&](const auto& t) { return t.track_id == old_id && fresh(t); });
    if (old_native == frame.tracks.end()) {
        return false;
    }
    const auto chosen = reacquirer_.find_candidate(*memory_, frame);
    if (chosen.kind != ReacquisitionResult::Kind::Unique || !chosen.candidate ||
        chosen.candidate->track_id == old_id || !fresh(*chosen.candidate)) {
        return false;
    }
    const auto separate_boxes = [](const TrackRecordV1& a, const TrackRecordV1& b) {
        return a.x2 <= b.x1 || b.x2 <= a.x1 || a.y2 <= b.y1 || b.y2 <= a.y1;
    };
    const auto distance = [](const TrackRecordV1& t, const std::array<double, 2>& point) {
        return std::hypot((*t.ground)[0] - point[0], (*t.ground)[1] - point[1]);
    };
    const auto& own = identity_motion(*memory_);
    const std::array<double, 2> own_prediction{own.north_m + own.velocity_north_m_s * own_dt,
                                               own.east_m + own.velocity_east_m_s * own_dt};
    const double tolerance = config_.short_motion.maximum_rmse_m;
    for (auto& other : competitors_) {
        if (!other.trusted || other.departed || other.key.generation != frame.generation ||
            other.key.track_id != chosen.candidate->track_id || !other.last_track.ground) {
            continue;
        }
        const auto other_age = frame.sample_monotonic_ns - other.real_sample_ns;
        const double dt = (stamp - other.motion_sample_ns) / 1e9;
        if (other_age <= 0 || other_age > max_age_ns || dt <= 0 ||
            dt > std::min(1., config_.short_motion.window_s +
                                  config_.short_motion.maximum_sample_gap_s)) {
            return false;
        }
        auto prediction = other.ground;
        if (other.short_motion.valid) {
            prediction = {other.short_motion.north_m + other.short_motion.velocity_north_m_s * dt,
                          other.short_motion.east_m + other.short_motion.velocity_east_m_s * dt};
        } else if (other.short_motion.rmse_m > tolerance ||
                   other.real_sample_ns != memory_->last_real_sample_ns) {
            return false;
        }
        // 경쟁 차량 쪽의 형상·위치 대응도 기존 후보 게이트로 검사한다.
        TargetMemory other_memory;
        other_memory.last_real_track = other.last_track;
        other_memory.last_real_sample_ns = other.real_sample_ns;
        other_memory.last_real_frame.motion_sample_ns = other.motion_sample_ns;
        if (other.short_motion.valid) {
            other_memory.velocity_north_m_s = other.short_motion.velocity_north_m_s;
            other_memory.velocity_east_m_s = other.short_motion.velocity_east_m_s;
        }
        auto only_other = frame;
        only_other.tracks = {*old_native};
        if (!reacquirer_.find_candidate(other_memory, only_other).candidate ||
            !separate_boxes(*chosen.candidate, *old_native) ||
            !separate_boxes(memory_->last_real_track, other.last_track) ||
            distance(*chosen.candidate, {(*old_native->ground)[0], (*old_native->ground)[1]}) <
                6 * tolerance) {
            return false;
        }
        const double selected_error = distance(*chosen.candidate, own_prediction);
        const double other_error = distance(*old_native, prediction);
        const double keep =
            distance(*old_native, own_prediction) + distance(*chosen.candidate, prediction);
        if (selected_error > 2 * tolerance || other_error > 2 * tolerance ||
            keep - selected_error - other_error < 4 * tolerance) {
            return false;
        }
        // selector 잠금 안에서 함께 갱신한다. 물리 표적/경쟁차의 이력과 원시각은 보존한다.
        memory_->binding.track_id = chosen.candidate->track_id;
        memory_->last_real_track.track_id = chosen.candidate->track_id;
        for (auto& t : memory_->last_real_frame.tracks) {
            t.track_id = chosen.candidate->track_id;
        }
        other.key.track_id = old_id;
        other.last_track.track_id = old_id;
        other.selected_continuity_proven = false;
        ++revision_;
        return true;
    }
    return false;
}

// 새 실관측의 동일 binding·짧은 운동·유일한 후보가 모두 일치해야 연속성 증거다.
bool TargetManager::selected_continuity_is_unique(const TracksV1& frame,
                                                  Clock::time_point received_at,
                                                  std::chrono::milliseconds max_tracks_age) const {
    const bool recovering_identity = brief_continuity_memory_ && (phase_ != TargetPhase::Locked ||
                                                                  !identity_motion(*memory_).valid);
    const auto& evidence = recovering_identity ? *brief_continuity_memory_ : *memory_;
    if (!config_.prefer_selected_continuity || !config_.allow_competitor_departure ||
        (phase_ != TargetPhase::Locked && !brief_continuity_memory_) ||
        !identity_motion(evidence).valid || frame.generation != evidence.binding.generation) {
        return false;
    }
    const auto age =
        received_at - Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
    const auto gap = frame.sample_monotonic_ns - evidence.last_real_sample_ns;
    const double dt =
        (frame.motion_sample_ns.value_or(frame.sample_monotonic_ns) -
         evidence.last_real_frame.motion_sample_ns.value_or(evidence.last_real_sample_ns)) /
        1e9;
    if (age < Clock::duration::zero() || age > max_tracks_age || gap <= 0 ||
        gap > std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count() ||
        dt <= 0 || dt > config_.short_motion.maximum_sample_gap_s) {
        return false;
    }
    const auto unique =
        reacquirer_.find_candidate(evidence, ground_candidates(frame, max_tracks_age));
    if (unique.kind != ReacquisitionResult::Kind::Unique || !unique.candidate ||
        unique.candidate->track_id != evidence.binding.track_id ||
        known_competitor(frame, *unique.candidate) || !unique.candidate->ground ||
        unique.candidate->state != "CONFIRMED" || unique.candidate->confidence < .3) {
        return false;
    }
    const auto& motion = identity_motion(evidence);
    return std::hypot(
               (*unique.candidate->ground)[0] - motion.north_m - motion.velocity_north_m_s * dt,
               (*unique.candidate->ground)[1] - motion.east_m - motion.velocity_east_m_s * dt) <=
           2 * config_.short_motion.maximum_rmse_m;
}

// 가장자리로 벗어난 경쟁차는 일관으로 본다. 아직 신뢰하지 않는 ID의 실종은 표적을 막지 않는다.
// 그 외의 실종이나 위치 이탈은 불일치다.
bool TargetManager::competitors_consistent(const TracksV1& frame,
                                           std::chrono::milliseconds max_tracks_age,
                                           bool selected_continuity) {
    return std::all_of(competitors_.begin(), competitors_.end(), [&](auto& competitor) {
        const auto current =
            std::find_if(frame.tracks.begin(), frame.tracks.end(), [&](const auto& track) {
                return competitor.key.generation == frame.generation &&
                       competitor.key.track_id == track.track_id && track.real_detection &&
                       track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns;
            });
        if (config_.allow_competitor_departure && current != frame.tracks.end() &&
            current->last_real_detection_monotonic_ns >
                competitor.last_track.last_real_detection_monotonic_ns) {
            const auto& old = competitor.last_track;
            const double dx = (current->x1 + current->x2 - old.x1 - old.x2) * .5;
            const double dy = (current->y1 + current->y2 - old.y1 - old.y2) * .5;
            const bool at_edge = (current->x1 <= 1 && dx < -.25) ||
                                 (current->x2 >= frame.image_width - 1 && dx > .25) ||
                                 (current->y1 <= 1 && dy < -.25) ||
                                 (current->y2 >= frame.image_height - 1 && dy > .25);
            bool outward = at_edge;
            competitor.exit_boundary_reached = false;
            if (config_.short_motion.enabled && competitor.trusted && current->confidence >= .3) {
                const double stamp =
                    frame.motion_sample_ns.value_or(frame.sample_monotonic_ns) / 1e9;
                auto& history = competitor.image_history;
                if (history.empty() || stamp > history.back()[0]) {
                    history.push_back({stamp, (current->x1 + current->x2) * .5,
                                       (current->y1 + current->y2) * .5});
                }
                while (history.size() > 32 ||
                       (!history.empty() &&
                        stamp - history.front()[0] > config_.short_motion.window_s)) {
                    history.pop_front();
                }
                if (history.size() >= 3 &&
                    stamp - history.front()[0] >= config_.short_motion.minimum_span_s) {
                    double st = 0, stt = 0, sx = 0, sy = 0, stx = 0, sty = 0;
                    for (const auto& p : history) {
                        const double t = p[0] - stamp;
                        st += t;
                        stt += t * t;
                        sx += p[1];
                        sy += p[2];
                        stx += t * p[1];
                        sty += t * p[2];
                    }
                    const auto count = history.size();
                    const double n = count, denominator = n * stt - st * st;
                    if (denominator > 1e-9) {
                        const double vx = (n * stx - st * sx) / denominator,
                                     vy = (n * sty - st * sy) / denominator;
                        // 모서리에서는 두 출구가 동시에 가능하다. 방향별 실관측 증거를 따로
                        // 보관한다.
                        constexpr double edge = 4;  // producer2pixel + 정수 끝점 예측 여유2pixel
                        const double horizon =
                            std::chrono::duration<double>(max_tracks_age).count();
                        const double cadence =
                            std::max(history[count - 1][0] - history[count - 2][0],
                                     history[count - 2][0] - history[count - 3][0]);
                        const std::array<double, 4> speeds{-vx, vx, -vy, vy};
                        const std::array<double, 4> distances{
                            current->x1, frame.image_width - current->x2, current->y1,
                            frame.image_height - current->y2};
                        for (std::size_t i = 0; i < speeds.size(); ++i) {
                            auto& observed = competitor.exit_observations[i];
                            observed = speeds[i] * horizon >= 2 ? observed + 1 : 0;
                            if (observed < config_.confirmation_frames) {
                                continue;
                            }
                            const double eta = std::max(0., distances[i] - edge) / speeds[i];
                            // 부분 잘림은 관측 사이에 발생하며, 3실표본 추세와 최근 두 간격의
                            // 경계를 모두 확인한다.
                            competitor.exit_boundary_reached =
                                competitor.exit_boundary_reached || eta <= 2 * cadence;
                        }
                    }
                }
            }
            competitor.departure_observations = outward ? competitor.departure_observations + 1 : 0;
            competitor.exit_boundary_reached =
                competitor.exit_boundary_reached ||
                (at_edge && competitor.departure_observations >= config_.confirmation_frames);
            if (competitor.trusted) {
                competitor.last_track = *current;
            }
        }
        if (config_.allow_competitor_departure && current == frame.tracks.end() &&
            (config_.short_motion.enabled ? competitor.exit_boundary_reached
                                          : competitor.departure_observations >= 3)) {
            competitor.departed = true;
        }
        // 실제 새 프레임3개의 edge/outward 증거만 사용한다. 단순시간만료/중앙실종은 여전히 HOLD다.
        if (config_.allow_competitor_departure && competitor.departed) {
            return true;
        }
        // 잠정 다른 ID도 재결합은 금지하지만, 실종만으로 정상 표적을 막지 않는다.
        // 실제 후보 모호함은 아래 TargetReacquirer가 기존대로 검사한다.
        if (config_.allow_competitor_departure && !competitor.trusted) {
            return true;
        }
        if (current == frame.tracks.end() || !current->ground) {
            const bool separate = missing_competitor_is_separate(frame, competitor, max_tracks_age);
            // 경쟁차의 오래된 위치를 외삽하지 않는다. 유효했던 분리 이후 선택차의
            // 매 실관측으로만 증거를 잇고, 공백·모호함·binding 변경에서는 끊는다.
            const auto other_age = frame.sample_monotonic_ns - competitor.real_sample_ns;
            const auto identity_age =
                std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count() +
                std::int64_t(config_.short_motion.window_s * 1e9);
            // 짧게 나타난 경쟁차는 운동 회귀가 준비되기 전에 사라질 수 있다.
            // 직전 같은 실프레임에서 분리됐고 선택차가 그 경로를 잇는 경우만 시작한다.
            const auto& prior = memory_->last_real_track;
            const auto& other = competitor.last_track;
            const bool distinct_boxes = prior.x2 <= other.x1 || other.x2 <= prior.x1 ||
                                        prior.y2 <= other.y1 || other.y2 <= prior.y1;
            const auto target = std::find_if(
                frame.tracks.begin(), frame.tracks.end(),
                [&](const auto& t) { return t.track_id == memory_->binding.track_id && t.ground; });
            const bool short_pair =
                !competitor.short_motion.valid &&
                competitor.short_motion.rmse_m <= config_.short_motion.maximum_rmse_m &&
                memory_->last_real_sample_ns == competitor.real_sample_ns && distinct_boxes &&
                other_age > 0 &&
                other_age <=
                    std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count() &&
                target != frame.tracks.end() &&
                std::hypot((*target->ground)[0] - competitor.ground[0],
                           (*target->ground)[1] - competitor.ground[1]) >=
                    6 * config_.short_motion.maximum_rmse_m;
            // 아직 유효한 경쟁 경로가 접근을 보이면 연속성 기억으로 덮지 않는다.
            if (selected_continuity && current == frame.tracks.end() &&
                (separate || short_pair ||
                 (competitor.selected_continuity_proven &&
                  (other_age > identity_age || !competitor.short_motion.valid)))) {
                competitor.selected_continuity_proven = true;
                return true;
            }
            competitor.selected_continuity_proven = false;
            return separate;
        }
        competitor.selected_continuity_proven = false;
        return std::hypot((*current->ground)[0] - competitor.ground[0],
                          (*current->ground)[1] - competitor.ground[1]) <= config_.ground_gate_m;
    });
}

// 같은 클래스의 다른 실검출을 최대 32대까지 기억하고, 연속 확인 뒤에만 trusted로 올린다.
void TargetManager::remember_competitors(const TracksV1& frame) {
    if (!memory_->last_real_track.ground) {
        return;
    }
    for (const auto& track : frame.tracks) {
        if (track.track_id == memory_->binding.track_id || !track.ground || !track.real_detection ||
            track.last_real_detection_monotonic_ns != frame.sample_monotonic_ns ||
            track.class_id != memory_->last_real_track.class_id ||
            track.label != memory_->last_real_track.label) {
            continue;
        }
        const auto found =
            std::find_if(competitors_.begin(), competitors_.end(), [&](const auto& old) {
                return old.key.generation == frame.generation && old.key.track_id == track.track_id;
            });
        if (found != competitors_.end()) {
            if (config_.allow_competitor_departure) {
                // 고정 비교 core의 high-score(.3)와 기존 연속 확인/위치 gate를 사용한다.
                // native ID가 긴 공백 뒤 약한 검출에 붙어도 새 신뢰 증거로 세지 않는다.
                const auto& previous = found->last_track;
                const auto gap =
                    frame.sample_monotonic_ns - previous.last_real_detection_monotonic_ns;
                // 직전 실검출과 250ms 이내이고 지면 게이트 안일 때만 확인을 이어 간다.
                const bool consistent =
                    gap > 0 && gap <= 250000000 && previous.ground &&
                    std::hypot((*track.ground)[0] - (*previous.ground)[0],
                               (*track.ground)[1] - (*previous.ground)[1]) <= config_.ground_gate_m;
                if (track.confidence >= .3) {
                    found->confirmation_observations =
                        consistent ? found->confirmation_observations + 1 : 1;
                    if (found->confirmation_observations >= config_.confirmation_frames) {
                        found->trusted = true;
                    }
                } else {
                    found->confirmation_observations = 0;
                }
                found->last_track = track;
                found->departed = false;
            }
            found->ground = {(*track.ground)[0], (*track.ground)[1]};
            if (config_.allow_competitor_departure && config_.short_motion.enabled) {
                if (track.confidence < .3) {
                    found->motion.reset();
                    found->short_motion = {};
                } else {
                    found->short_motion = found->motion.update(
                        frame.motion_sample_ns.value_or(frame.sample_monotonic_ns), found->ground);
                }
                found->real_sample_ns = frame.sample_monotonic_ns;
                found->motion_sample_ns =
                    frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
            }
        } else if (competitors_.size() < kMaxCompetitors) {
            CompetitorMemory added;
            added.key = {frame.generation, track.track_id};
            added.first_real_sample_ns = frame.sample_monotonic_ns;
            added.ground = {(*track.ground)[0], (*track.ground)[1]};
            if (config_.allow_competitor_departure) {
                added.last_track = track;
                added.confirmation_observations = track.confidence >= .3 ? 1 : 0;
            }
            if (config_.allow_competitor_departure && config_.short_motion.enabled) {
                auto identity_motion = config_.short_motion;
                identity_motion.window_s =
                    std::min(1., identity_motion.window_s + identity_motion.maximum_sample_gap_s);
                identity_motion.maximum_sample_gap_s = identity_motion.window_s;
                added.motion = ShortMotionEstimator{identity_motion};
                added.real_sample_ns = frame.sample_monotonic_ns;
                added.motion_sample_ns = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
                if (track.confidence >= .3) {
                    added.short_motion = added.motion.update(added.motion_sample_ns, added.ground);
                }
            }
            competitors_.push_back(added);
        }
    }
}

// 확정 실관측으로 픽셀 속도, 지면 운동, 경쟁차 기억을 갱신한다.
void TargetManager::remember(const TracksV1& frame, const TrackRecordV1& track,
                             Clock::time_point received_at) {
    // 새 실관측을 수용하면 튐 판정 기준도 이 관측으로 바뀐다.
    box_jump_since_ns_.reset();
    auto& memory = *memory_;
    memory.ground_motion_valid = false;
    memory.ground_acceleration_valid = false;
    // 직전 실관측이 있을 때만 영상 중심의 픽셀 속도를 구한다.
    if (memory.last_real_received_at != Clock::time_point{}) {
        const double dt_s =
            static_cast<double>(frame.sample_monotonic_ns - memory.last_real_sample_ns) / 1e9;
        if (dt_s > 0) {
            memory.velocity_x_px_s =
                ((track.x1 + track.x2) - (memory.last_real_track.x1 + memory.last_real_track.x2)) *
                0.5 / dt_s;
            memory.velocity_y_px_s =
                ((track.y1 + track.y2) - (memory.last_real_track.y1 + memory.last_real_track.y2)) *
                0.5 / dt_s;
        }
    }
    // 기존 V2.1 환경변수 또는 V3 설정(실내·실외 기존 방식)에서 box 움직임 가드를 켠다.
    const bool guard_enabled =
        v21_enabled("ARGUS_V21_BOX_MOTION_GUARD") || config_.v3_box_motion_guard;
    if (memory.last_real_received_at != Clock::time_point{} &&
        (memory.binding.generation != frame.generation ||
         memory.binding.track_id != track.track_id)) {
        box_motion_guard_.reset();
        fresh_contact_velocity_.reset();
        memory.cv_visibility_reference.reset();
        memory.cv_ten_frame.reset();
    }
    if (!guard_enabled) {
        box_motion_guard_.reset();
    }
    // V3: 가림 의심 중 쓸 새 관측 기반 속도 추정 (설정에서만, box 가드가 켜진 경우).
    const bool fresh_contact_enabled = guard_enabled && config_.v3_fresh_contact_velocity;
    if (fresh_contact_enabled) {
        fresh_contact_velocity_.observe(
            frame, track,
            std::chrono::duration_cast<std::chrono::nanoseconds>(received_at.time_since_epoch())
                .count());
    } else {
        fresh_contact_velocity_.reset();
    }
    // Box shape is a velocity-quality hint, not an identity verdict. Fit separate
    // identity evidence from original real samples; never feed it into control/CV.
    const bool identity_binding_changed = memory.last_real_received_at != Clock::time_point{} &&
                                          (memory.binding.generation != frame.generation ||
                                           memory.binding.track_id != track.track_id);
    bool identity_anchor_coherent = false;
    const bool identity_anchor_changed =
        memory.last_real_track.ground_anchor != track.ground_anchor;
    if (config_.coherent_ground_anchor_motion && identity_anchor_changed &&
        memory.identity_short_motion.valid && track.ground && !identity_binding_changed) {
        const auto previous_ns =
            memory.last_real_frame.motion_sample_ns.value_or(memory.last_real_sample_ns);
        const auto current_ns = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
        const double dt = (current_ns - previous_ns) / 1e9;
        const auto& motion = memory.identity_short_motion;
        const double residual =
            std::hypot((*track.ground)[0] - motion.north_m - motion.velocity_north_m_s * dt,
                       (*track.ground)[1] - motion.east_m - motion.velocity_east_m_s * dt);
        identity_anchor_coherent =
            dt > 0 && dt <= .25 &&
            residual <= std::min(.2, 2 * config_.short_motion.maximum_rmse_m);
    }
    if (identity_binding_changed || (identity_anchor_changed && !identity_anchor_coherent) ||
        !track.ground) {
        identity_motion_.reset();
        memory.identity_short_motion = {};
    }
    if (track.ground) {
        memory.identity_short_motion =
            identity_motion_.update(frame.motion_sample_ns.value_or(frame.sample_monotonic_ns),
                                    {(*track.ground)[0], (*track.ground)[1]});
    }
    const bool was_suspect = memory.box_motion_suspect;
    memory.box_motion_suspect = guard_enabled && box_motion_guard_.suspect(frame, track);
    if (memory.box_motion_suspect && !was_suspect) {
        memory.cv_ten_frame.freeze_before(frame.frame_id);
    }
    if (!memory.box_motion_suspect) {
        memory.cv_ten_frame.clear_frozen();
    }
    const bool reference_stale =
        !memory.cv_visibility_reference ||
        frame.sample_monotonic_ns - memory.cv_visibility_reference->sample_ns >
            kCvReferenceRefreshNs;
    if (guard_enabled &&
        (!memory.box_motion_suspect || (config_.v35_cv_reference_refresh && reference_stale)) &&
        frame.ground_pose && track.ground && frame.clock_domain == "host_monotonic" &&
        frame.sample_monotonic_ns > 0 && track.real_detection && track.x1 > 0 && track.y1 > 0 &&
        track.x2 < frame.image_width && track.y2 < frame.image_height) {
        memory.cv_visibility_reference = CvVisibilityReference{{frame.generation, track.track_id},
                                                               track,
                                                               *frame.ground_pose,
                                                               frame.sample_monotonic_ns,
                                                               frame.image_width,
                                                               frame.image_height,
                                                               producer_ground_of(frame)};
    }
    if (memory.box_motion_suspect) {
        if (fresh_contact_enabled && fresh_contact_velocity_.can_refresh()) {
            // 잘린 박스 대신 직전 실제 접지 관측으로 추정한 속도를 CV 근거로 쓴다.
            const auto& fresh = fresh_contact_velocity_.latest();
            const bool stopped = fresh.phase == ObservedMotionPhase::Stopped;
            memory.cv_velocity_north_m_s = stopped ? 0. : fresh.velocity_north_m_s;
            memory.cv_velocity_east_m_s = stopped ? 0. : fresh.velocity_east_m_s;
            memory.cv_velocity_trusted = true;
            memory.cv_last_observed_moving = !stopped;
            memory.cv_velocity_unconfirmed = false;
            memory.cv_velocity_fit_sample_ns = fresh.fit_sample_ns;
        }
        // A clipped box is still a real detection; its ground point is unsuitable for velocity fit.
        ground_history_.clear();
        short_motion_.reset();
        reset_previous_moving_velocity_.reset();
        memory.short_motion = {};
    } else if (track.ground) {
        const bool anchor_changed = memory.last_real_track.ground_anchor != track.ground_anchor;
        bool coherent_anchor = false;
        if (config_.coherent_ground_anchor_motion && anchor_changed &&
            memory.last_real_track.ground && memory.short_motion.valid &&
            memory.last_real_frame.generation == frame.generation &&
            memory.last_real_track.track_id == track.track_id &&
            memory.last_real_track.class_id == track.class_id &&
            memory.last_real_track.label == track.label) {
            const auto previous_ns =
                memory.last_real_frame.motion_sample_ns.value_or(memory.last_real_sample_ns);
            const auto current_ns = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
            const double dt = (current_ns - previous_ns) / 1e9;
            const auto& previous = *memory.last_real_track.ground;
            const auto& velocity = memory.short_motion;
            const double residual =
                std::hypot((*track.ground)[0] - previous[0] - velocity.velocity_north_m_s * dt,
                           (*track.ground)[1] - previous[1] - velocity.velocity_east_m_s * dt);
            // 같은 ID와 클래스에서 0.25초 이내이고, 잔차가 0.2m 또는 RMSE의 2배 이하면
            // 기준점이 바뀌어도 연속으로 본다.
            coherent_anchor = dt > 0 && dt <= .25 &&
                              residual <= std::min(.2, 2 * config_.short_motion.maximum_rmse_m);
        }
        if (memory.last_real_received_at != Clock::time_point{} &&
            ((anchor_changed && !coherent_anchor) ||
             (config_.retain_motion_on_brief_gap &&
              (memory.binding.generation != frame.generation ||
               memory.binding.track_id != track.track_id)))) {
            // 새 ID·큰 기준 차이는 초기화한다. 검증된 같은 중심의 새 실관측만 연속 회귀할 수 있다.
            ground_history_.clear();
            short_motion_.reset();
            reset_previous_moving_velocity_.reset();
        }
        const auto motion_ns = frame.motion_sample_ns.value_or(frame.sample_monotonic_ns);
        memory.short_motion =
            short_motion_.update(motion_ns, {(*track.ground)[0], (*track.ground)[1]});
        const bool allow_raw =
            guard_enabled && frame.ground_pose && v21_enabled("ARGUS_V23_CV_RAW_MOTION");
        update_cv_motion_cache(memory, memory.short_motion, allow_raw);
        const auto& current = memory.short_motion;
        const double new_speed = std::hypot(current.velocity_north_m_s, current.velocity_east_m_s);
        const bool stopped = current.valid && current.phase == ObservedMotionPhase::Stopped &&
                             !(config_.retain_motion_on_brief_gap && current.departure_pending);
        bool reversed = false;
        if (config_.reset_motion_on_stop_or_reverse) {
            if (stopped) {
                reset_previous_moving_velocity_.reset();
            } else if (current.valid && current.feedforward_valid) {
                if (reset_previous_moving_velocity_) {
                    const auto& previous = *reset_previous_moving_velocity_;
                    // 이전 이동 속도와의 내적이 두 속도 크기 곱의 -0.5배보다 작으면 역전이다.
                    reversed = previous[0] * current.velocity_north_m_s +
                                   previous[1] * current.velocity_east_m_s <
                               -.5 * std::hypot(previous[0], previous[1]) * new_speed;
                }
                reset_previous_moving_velocity_ =
                    std::array<double, 2>{current.velocity_north_m_s, current.velocity_east_m_s};
            }
        }
        // 정지로 확인되거나 역전이면 지면 이력과 속도를 지운다.
        if (config_.reset_motion_on_stop_or_reverse && (stopped || reversed)) {
            ground_history_.clear();
            memory.velocity_north_m_s = memory.velocity_east_m_s = 0;
            memory.acceleration_north_m_s2 = memory.acceleration_east_m_s2 = 0;
        }
        const double stamp = static_cast<double>(motion_ns) / 1e9;
        if (!ground_history_.empty() &&
            stamp - ground_history_.back()[0] >
                std::chrono::duration<double>(config_.motion_max_sample_gap).count()) {
            ground_history_.clear();
        }
        ground_history_.push_back({stamp, (*track.ground)[0], (*track.ground)[1]});
        while (ground_history_.size() > 256 ||
               stamp - ground_history_.front()[0] > config_.motion_history_s) {
            ground_history_.pop_front();
        }
        // 새로 관측한 위치만 쓴다. 직교하는 이차 잔차로 두 표본 미분의 잡음을 피한다.
        if (ground_history_.size() >= config_.motion_history_min_samples &&
            stamp - ground_history_.front()[0] >= config_.motion_history_min_s) {
            double sum_t = 0, sum_tt = 0, sum_n = 0, sum_e = 0, sum_tn = 0, sum_te = 0;
            for (const auto& point : ground_history_) {
                const double t = point[0] - stamp;
                sum_t += t;
                sum_tt += t * t;
                sum_n += point[1];
                sum_e += point[2];
                sum_tn += t * point[1];
                sum_te += t * point[2];
            }
            const double count = ground_history_.size();
            const double denominator = count * sum_tt - sum_t * sum_t;
            if (denominator > 1e-9) {
                memory.velocity_north_m_s = (count * sum_tn - sum_t * sum_n) / denominator;
                memory.velocity_east_m_s = (count * sum_te - sum_t * sum_e) / denominator;
                const double mean_t = sum_t / count, mean_n = sum_n / count, mean_e = sum_e / count;
                double cn = 0, ce = 0, quadratic_intercept = 0, quadratic_slope = 0;
                if (config_.motion_fit_acceleration) {
                    double sum_ttt = 0;
                    for (const auto& point : ground_history_) {
                        sum_ttt += std::pow(point[0] - stamp, 3);
                    }
                    quadratic_slope = (count * sum_ttt - sum_t * sum_tt) / denominator;
                    quadratic_intercept = sum_tt / count - quadratic_slope * mean_t;
                    double qq = 0, qn = 0, qe = 0;
                    for (const auto& point : ground_history_) {
                        const double t = point[0] - stamp,
                                     q = t * t - quadratic_intercept - quadratic_slope * t;
                        qq += q * q;
                        qn += q * point[1];
                        qe += q * point[2];
                    }
                    if (qq > 1e-9) {
                        cn = qn / qq;
                        ce = qe / qq;
                        memory.ground_acceleration_valid = true;
                    }
                }
                memory.fitted_north_m =
                    mean_n - memory.velocity_north_m_s * mean_t - cn * quadratic_intercept;
                memory.fitted_east_m =
                    mean_e - memory.velocity_east_m_s * mean_t - ce * quadratic_intercept;
                memory.velocity_north_m_s -= cn * quadratic_slope;
                memory.velocity_east_m_s -= ce * quadratic_slope;
                memory.acceleration_north_m_s2 = 2 * cn;
                memory.acceleration_east_m_s2 = 2 * ce;
                double residual = 0;
                for (const auto& point : ground_history_) {
                    const double t = point[0] - stamp;
                    residual += std::pow(point[1] - memory.fitted_north_m -
                                             memory.velocity_north_m_s * t - cn * t * t,
                                         2) +
                                std::pow(point[2] - memory.fitted_east_m -
                                             memory.velocity_east_m_s * t - ce * t * t,
                                         2);
                }
                memory.ground_motion_rmse_m = std::sqrt(residual / count);
                memory.ground_motion_valid =
                    std::isfinite(memory.velocity_north_m_s + memory.velocity_east_m_s) &&
                    memory.ground_motion_rmse_m <= config_.motion_fit_max_rmse_m;
                memory.ground_acceleration_valid =
                    memory.ground_acceleration_valid && memory.ground_motion_valid &&
                    std::isfinite(memory.acceleration_north_m_s2 + memory.acceleration_east_m_s2);
            }
        }
    } else {
        ground_history_.clear();
        short_motion_.reset();
        reset_previous_moving_velocity_.reset();
        memory.short_motion = {};
    }
    if (memory.binding.generation != frame.generation ||
        memory.binding.track_id != track.track_id) {
        ++revision_;
    }
    const auto& motion = memory.short_motion;
    if (guard_enabled && config_.v32_cv_ten_frame && !memory.box_motion_suspect &&
        track.real_detection &&
        track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns && motion.valid &&
        motion.feedforward_valid && motion.phase == ObservedMotionPhase::Moving &&
        !motion.departure_pending) {
        memory.cv_ten_frame.remember(frame.frame_id, frame.sample_monotonic_ns,
                                     motion.fit_sample_ns, motion.velocity_north_m_s,
                                     motion.velocity_east_m_s, true, true, false);
    }
    memory.binding = {frame.generation, track.track_id};
    memory.last_real_track = track;
    memory.last_real_frame = frame;
    memory.last_real_frame.tracks = {track};
    memory.last_real_sample_ns = frame.sample_monotonic_ns;
    memory.last_real_received_at =
        ground_tracks_schema(frame.schema) && frame.clock_domain == "host_monotonic"
            ? Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}}
            : received_at;
    remember_competitors(frame);
}

// 상태를 바꾸지 않는 투영이다. 단절 기한과 입력 공백을 결과 단계에만 반영한다.
TargetRecoverySnapshot TargetManager::snapshot(Clock::time_point now,
                                               std::chrono::milliseconds max_tracks_age) const {
    TargetRecoverySnapshot result{memory_, phase_, revision_, reason_, false};
    if (result.memory && config_.estimated_speed_deadzone_m_s > 0) {
        auto& output = *result.memory;
        const auto zero_small = [&](double& north, double& east) {
            const bool small = std::hypot(north, east) < config_.estimated_speed_deadzone_m_s;
            if (small) {
                north = east = 0;
            }
            return small;
        };
        if (zero_small(output.velocity_north_m_s, output.velocity_east_m_s)) {
            output.acceleration_north_m_s2 = output.acceleration_east_m_s2 = 0;
        }
        zero_small(output.short_motion.velocity_north_m_s, output.short_motion.velocity_east_m_s);
        if (zero_small(output.cv_velocity_north_m_s, output.cv_velocity_east_m_s)) {
            output.cv_velocity_trusted = false;
        }
    }
    if (!memory_ || phase_ == TargetPhase::Lost) {
        return result;
    }
    const auto age = now - memory_->last_real_received_at;
    if (age < Clock::duration::zero() || age >= config_.max_gap) {
        result.phase = TargetPhase::Lost;
        result.reason = "recovery_deadline_expired";
    } else if (phase_ == TargetPhase::Locked && age > max_tracks_age) {
        // 관찰은 상태를 수정하지 않는다. 다음 tick 전에도 오래된 정상 명령을 차단한다.
        result.phase = TargetPhase::GapHold;
        result.reason = "target_input_gap";
    }
    result.pause_commands = result.phase == TargetPhase::GapHold ||
                            result.phase == TargetPhase::Reacquire ||
                            result.phase == TargetPhase::Confirm;
    return result;
}

// snapshot이 가리키는 단계가 현재와 다르면 그 단계로 실제 전이를 수행한다.
void TargetManager::advance(Clock::time_point now, std::chrono::milliseconds max_tracks_age) {
    if (brief_continuity_memory_ &&
        (now < brief_continuity_memory_->last_real_received_at ||
         now - brief_continuity_memory_->last_real_received_at > max_tracks_age)) {
        brief_continuity_memory_.reset();
        if (phase_ != TargetPhase::Locked) {
            set_phase(phase_, "brief_continuity_expired");
        }
    }
    const auto view = snapshot(now, max_tracks_age);
    if (view.phase != phase_) {
        if (view.phase == TargetPhase::Lost) {
            invalidate(view.reason.c_str());
        } else {
            set_phase(view.phase, view.reason.c_str());
        }
    }
}

// 표본 검사, 단절 기한, 후보 검사, 연속 확인, 정상 기억 갱신 순서로 한 프레임을 처리한다.
void TargetManager::observe(const TracksV1& frame, Clock::time_point received_at,
                            std::chrono::milliseconds max_tracks_age) {
    if (!memory_ || phase_ == TargetPhase::Lost) {
        return;
    }
    if (frame.source != memory_->source || frame.clock_domain != memory_->clock_domain ||
        frame.timestamp_source != memory_->timestamp_source ||
        frame.exposure_time_valid != memory_->exposure_time_valid ||
        frame.image_width != memory_->image_width || frame.image_height != memory_->image_height) {
        invalidate("target_input_context_changed");
        return;
    }
    if (frame.frame_id <= last_frame_id_ || frame.sample_monotonic_ns <= last_sample_ns_) {
        return;
    }
    advance(received_at, max_tracks_age);
    if (phase_ == TargetPhase::Lost) {
        return;
    }
    if (frame.sample_monotonic_ns - memory_->last_real_sample_ns >=
        std::chrono::duration_cast<std::chrono::nanoseconds>(config_.max_gap).count()) {
        invalidate("recovery_sample_gap_expired");
        return;
    }
    last_frame_id_ = frame.frame_id;
    last_sample_ns_ = frame.sample_monotonic_ns;

    if (config_.trust_selected_id) {
        const auto current =
            std::find_if(frame.tracks.begin(), frame.tracks.end(), [&](const auto& t) {
                return frame.generation == memory_->binding.generation &&
                       t.track_id == memory_->binding.track_id &&
                       t.class_id == memory_->last_real_track.class_id &&
                       t.label == memory_->last_real_track.label && t.state == "CONFIRMED" &&
                       t.real_detection &&
                       t.last_real_detection_monotonic_ns == frame.sample_monotonic_ns;
            });
        const auto sampled = Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
        // 2026-11-10 가림 대응: 오래 비었다가 돌아온 같은 native ID는 CV 예측 지면 위치에서
        // 게이트 안일 때만 바로 믿는다. 추적기가 가려진 차를 예측으로 붙잡는 동안 다른 차가
        // 그 ID를 물려받는 경우를 막는다. 밖이면 이 프레임은 일반 후보 검색으로 넘긴다.
        bool trusted_return_rejected = false;
        if (current != frame.tracks.end() && config_.trusted_return_check.count() > 0 &&
            memory_->last_real_track.ground && current->ground &&
            frame.sample_monotonic_ns - memory_->last_real_sample_ns >
                std::chrono::duration_cast<std::chrono::nanoseconds>(config_.trusted_return_check)
                    .count()) {
            const auto prediction =
                reacquirer_.predicted_ground_distance(*memory_, frame, *current);
            trusted_return_rejected =
                !prediction || prediction->distance_m > reacquirer_.ground_gate(prediction->gap_s);
            if (std::getenv("ARGUS_TRIAL_TRACE")) {
                std::cerr << "TM_TRUSTED_RETURN frame=" << frame.frame_id
                          << " selected=" << current->track_id
                          << " distance_m=" << (prediction ? prediction->distance_m : -1.)
                          << " gate_m="
                          << (prediction ? reacquirer_.ground_gate(prediction->gap_s) : -1.)
                          << " rejected=" << trusted_return_rejected << '\n';
            }
        }
        if (trusted_return_rejected) {
            reset_confirmation();
            if (phase_ == TargetPhase::Locked) {
                set_phase(TargetPhase::GapHold, "selected_id_return_outside_ground_gate");
            }
        } else if (current != frame.tracks.end() && received_at >= sampled &&
                   received_at - sampled <= max_tracks_age) {
            if (config_.selected_box_jump_guard && memory_) {
                const auto elapsed_ns = frame.sample_monotonic_ns - memory_->last_real_sample_ns;
                // 튐 없이 길게 비었던 경우는 기존처럼 받아들인다.
                const bool checked =
                    elapsed_ns <= kBoxJumpWindowNs || box_jump_since_ns_.has_value();
                // 2026-11-16 사용자 지시 "3초 동안 계속 비교판정, 실패 한 번 나와도": 튐이 이어져도
                // 버리지 않고 매 프레임 다시 판정한다. 공백 끝은 GAP_CV 3초 기한
                // (v34_detection_gap_expired)과 복구 한도(max_gap)가 정한다. 예전에는 튐이 0.5초를
                // 넘으면 selected_id_box_jump_persisted로 바로 상실했다.
                if (checked && selected_box_jumped(frame, *current)) {
                    if (!box_jump_since_ns_) {
                        box_jump_since_ns_ = frame.sample_monotonic_ns;
                    }
                    reset_confirmation();
                    set_phase(TargetPhase::GapHold, "selected_id_box_jump");
                    return;
                }
                box_jump_since_ns_.reset();
            }
            if (config_.v36_identity_context &&
                config_.thermal_identity != ThermalIdentityMode::Off && memory_) {
                if (v36_selected_identity(frame, *current) != V36Decision::Accept) {
                    return;
                }
            } else if (config_.thermal_identity != ThermalIdentityMode::Off && memory_) {
                const bool mismatch = selected_thermal_mismatch(frame, *current);
                if (mismatch && config_.thermal_identity == ThermalIdentityMode::On) {
                    if (!thermal_mismatch_since_ns_) {
                        thermal_mismatch_since_ns_ = frame.sample_monotonic_ns;
                    }
                    if (frame.sample_monotonic_ns - *thermal_mismatch_since_ns_ >
                        kThermalMismatchWindowNs) {
                        invalidate(thermal_low_similarity_
                                       ? "selected_id_thermal_low_similarity_persisted"
                                       : "selected_id_thermal_mismatch_persisted");
                        return;
                    }
                    reset_confirmation();
                    set_phase(TargetPhase::GapHold, thermal_low_similarity_
                                                        ? "selected_id_thermal_low_similarity"
                                                        : "selected_id_thermal_mismatch");
                    return;
                }
                thermal_mismatch_since_ns_.reset();
            }
            remember(frame, *current, received_at);
            reset_confirmation();
            brief_continuity_memory_.reset();
            set_phase(TargetPhase::Locked, "selected_native_id_trusted");
            if (std::getenv("ARGUS_TRIAL_TRACE")) {
                std::cerr << "TM_NATIVE_ID_TRUST frame=" << frame.frame_id
                          << " selected=" << current->track_id
                          << " source_ns=" << frame.sample_monotonic_ns
                          << " peers=" << frame.tracks.size() - 1
                          << " motion_valid=" << memory_->short_motion.valid
                          << " box_suspect=" << memory_->box_motion_suspect << '\n';
            }
            return;
        }
        if (phase_ == TargetPhase::Locked) {
            set_phase(TargetPhase::GapHold, "selected_id_absent_compare_candidates");
        }
    }

    if (phase_ == TargetPhase::Locked && ground_tracks_schema(frame.schema) &&
        memory_->last_real_track.ground &&
        std::all_of(competitors_.begin(), competitors_.end(),
                    [](const auto& other) { return other.departed; }) &&
        frame.tracks.empty() &&
        frame.sample_monotonic_ns - memory_->last_real_sample_ns <
            std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count()) {
        // 짧은 단일 차량 미검출은 아직 유효한 원래 관측을 보존한다. 시각/확인 횟수는 갱신하지
        // 않는다.
        return;
    }

    if (frame.tracks.empty() && config_.prefer_selected_continuity &&
        identity_motion(*memory_).valid &&
        (brief_continuity_memory_ ||
         (phase_ == TargetPhase::Locked &&
          std::all_of(competitors_.begin(), competitors_.end(), [](const auto& other) {
              return other.departed || !other.trusted || other.selected_continuity_proven;
          })))) {
        // 빈 입력은 중립으로 처리한다. 복귀 근거도 새 실관측 없이 250ms 넘게 보존하지 않는다.
        if (!brief_continuity_memory_ || phase_ == TargetPhase::Locked) {
            brief_continuity_memory_ = memory_;
        }
        reset_confirmation();
        set_phase(TargetPhase::GapHold, "selected_target_missing");
        return;
    }

    const bool identity_reconciled =
        !config_.trust_selected_id && reconcile_identity_swap(frame, received_at, max_tracks_age);

    // 이미 알던 다른 차량의 ID가 사라지거나 위치가 뒤바뀌면 같은 native ID도 원래 표적이라는 증거가
    // 아니다. ambiguous 구간의 경쟁 차량 기억은 갱신하지 않는다. GT/외형 임베딩을 사용하지 않는다.
    if (!config_.trust_selected_id && !frame.tracks.empty() &&
        !competitors_consistent(
            frame, max_tracks_age,
            selected_continuity_is_unique(frame, received_at, max_tracks_age))) {
        pause_ambiguous("known_competitor_identity_unresolved");
        return;
    }

    if (phase_ == TargetPhase::Locked) {
        // v2에서는 native ID가 같아도 위치/운동/복수 후보를 다시 확인한다.
        if (memory_->last_real_track.ground) {
            const auto plausible =
                reacquirer_.find_candidate(*memory_, ground_candidates(frame, max_tracks_age));
            if (plausible.candidate && known_competitor(frame, *plausible.candidate)) {
                pause_ambiguous("candidate_is_known_other_vehicle");
                return;
            }
            if (plausible.kind == ReacquisitionResult::Kind::Ambiguous) {
                pause_ambiguous("locked_ground_candidates_ambiguous");
                return;
            }
            if (!plausible.candidate ||
                plausible.candidate->track_id != memory_->binding.track_id ||
                frame.generation != memory_->binding.generation) {
                set_phase(TargetPhase::GapHold, "native_binding_requires_ground_confirmation");
            }
        }
    }
    if (phase_ == TargetPhase::Locked) {
        for (const auto& track : frame.tracks) {
            if (frame.generation == memory_->binding.generation &&
                track.track_id == memory_->binding.track_id &&
                track.class_id == memory_->last_real_track.class_id &&
                track.label == memory_->last_real_track.label && track.real_detection &&
                track.last_real_detection_monotonic_ns == frame.sample_monotonic_ns) {
                remember(frame, track, received_at);
                if (identity_reconciled) {
                    if (brief_continuity_memory_) {
                        brief_continuity_memory_ = memory_;
                    }
                    set_phase(TargetPhase::Locked, "target_id_reconciled_by_continuity");
                }
                return;
            }
        }
        set_phase(TargetPhase::GapHold, "selected_target_missing");
    }

    // GapHold는 후보 검색으로 넘긴다. 같은 후보가 confirmation_frames만큼 연속이면
    // Locked로 복귀한다.
    if (phase_ == TargetPhase::GapHold) {
        set_phase(TargetPhase::Reacquire, "searching_target_candidates");
    }
    if (confirmation_track_ &&
        frame.sample_monotonic_ns - confirmation_track_->last_real_detection_monotonic_ns >
            std::chrono::duration_cast<std::chrono::nanoseconds>(max_tracks_age).count()) {
        reset_confirmation();
    }
    const auto* continuity =
        candidate_key_ && candidate_key_->generation == frame.generation && confirmation_track_
            ? &*confirmation_track_
            : nullptr;
    const auto& reference = brief_continuity_memory_ ? *brief_continuity_memory_ : *memory_;
    const auto result = reacquirer_.find_candidate(reference, frame, continuity);
    if (result.candidate && known_competitor(frame, *result.candidate)) {
        pause_ambiguous("candidate_is_known_other_vehicle");
        return;
    }
    if (result.kind == ReacquisitionResult::Kind::Ambiguous) {
        pause_ambiguous("reacquisition_ambiguous");
        return;
    }
    if (!result.candidate) {
        reset_confirmation();
        set_phase(TargetPhase::GapHold, "no_plausible_target_candidate");
        return;
    }
    const SelectedTargetKey key{frame.generation, result.candidate->track_id};
    // v36_identity_context: 공백 뒤 다른 ID는 따라가지 않는다. 완화 정책을 켜면 같은 종류 후보가
    // 하나뿐이고 열 외형이 기억과 강하게 닮을 때만(kV36RelaxedFrames 중앙값) 이어받는다.
    bool v36_lock_ready = true;
    if (config_.v36_identity_context && (key.generation != memory_->binding.generation ||
                                         key.track_id != memory_->binding.track_id)) {
        const auto same_kind =
            std::count_if(frame.tracks.begin(), frame.tracks.end(), [&](const TrackRecordV1& t) {
                return t.class_id == result.candidate->class_id &&
                       t.label == result.candidate->label && t.real_detection;
            });
        const double similarity = v36_similarity(thermal_reference_, *result.candidate);
        if (!config_.v36_new_id_relaxed || same_kind != 1 || similarity < 0) {
            v36_relaxed_key_.reset();
            v36_relaxed_similarity_.clear();
            reset_confirmation();
            set_phase(TargetPhase::GapHold, config_.v36_new_id_relaxed
                                                ? "different_id_relaxed_conditions_unmet"
                                                : "different_id_not_followed");
            return;
        }
        if (!v36_relaxed_key_ || v36_relaxed_key_->generation != key.generation ||
            v36_relaxed_key_->track_id != key.track_id) {
            v36_relaxed_key_ = key;
            v36_relaxed_similarity_.clear();
        }
        v36_relaxed_similarity_.push_back(similarity);
        if (v36_relaxed_similarity_.size() >= kV36RelaxedFrames &&
            v36_median(v36_relaxed_similarity_) < kV36RelaxedSimilarity) {
            reset_confirmation();
            set_phase(TargetPhase::GapHold, "different_id_relaxed_similarity_low");
            return;
        }
        v36_lock_ready = v36_relaxed_similarity_.size() >= kV36RelaxedFrames;
    }
    if (!candidate_key_ || candidate_key_->generation != key.generation ||
        candidate_key_->track_id != key.track_id) {
        reset_confirmation();
        candidate_key_ = key;
    }
    bool brief_motion_ready = !brief_continuity_memory_;
    if (brief_continuity_memory_ && result.candidate->ground &&
        key.generation == memory_->binding.generation &&
        key.track_id == memory_->binding.track_id) {
        // 확인 중 받은 실제 표본도 회귀에 남기되, 명령용 기억과 원시각은 확정 전 갱신하지 않는다.
        if (config_.prefer_selected_continuity) {
            // Confirmation samples can rebuild control speed only if the same
            // box-quality guard permits them. Identity evidence remains separate.
            auto quality_guard = box_motion_guard_;
            const bool suspect =
                (v21_enabled("ARGUS_V21_BOX_MOTION_GUARD") || config_.v3_box_motion_guard) &&
                quality_guard.suspect(frame, *result.candidate);
            if (suspect) {
                short_motion_.reset();
            } else {
                short_motion_.update(
                    frame.motion_sample_ns.value_or(frame.sample_monotonic_ns),
                    {(*result.candidate->ground)[0], (*result.candidate->ground)[1]});
            }
        }
        const auto motion =
            (!config_.trust_selected_id && config_.prefer_selected_continuity ? identity_motion_
                                                                              : short_motion_)
                .update(frame.motion_sample_ns.value_or(frame.sample_monotonic_ns),
                        {(*result.candidate->ground)[0], (*result.candidate->ground)[1]});
        brief_motion_ready = motion.valid;
        auto& evidence = *brief_continuity_memory_;
        auto& identity = config_.prefer_selected_continuity ? evidence.identity_short_motion
                                                            : evidence.short_motion;
        if (motion.valid) {
            identity = motion;
        } else {
            identity.north_m = (*result.candidate->ground)[0];
            identity.east_m = (*result.candidate->ground)[1];
        }
        evidence.last_real_track = *result.candidate;
        evidence.last_real_frame = frame;
        evidence.last_real_sample_ns = frame.sample_monotonic_ns;
        evidence.last_real_received_at =
            Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
    }
    ++confirmation_count_;
    confirmation_track_ = result.candidate;
    set_phase(TargetPhase::Confirm, "confirming_target_candidate");
    if (confirmation_count_ >= config_.confirmation_frames && brief_motion_ready &&
        v36_lock_ready) {
        if (config_.v36_identity_context && v36_relaxed_key_) {
            // 새 ID를 이어받았다: 이후 판단의 기준을 새 binding으로 옮긴다.
            v36_relaxed_key_.reset();
            v36_relaxed_similarity_.clear();
        }
        remember(frame, *result.candidate, received_at);
        reset_confirmation();
        set_phase(TargetPhase::Locked, "target_reacquired");
    }
}

}  // namespace follow
