// 임무 표적 기억을 한 곳에서 소유한다. 호출자는 기존 선택기 잠금을 잡고 상태 변경을 직렬화한다.
#pragma once

#include <deque>
#include <fstream>
#include <memory>
#include <utility>
#include <vector>

#include "argus/control/target/include/box_motion_guard.hpp"
#include "argus/control/target/include/fresh_contact_velocity_estimator.hpp"
#include "argus/control/target/include/target_reacquirer.hpp"

namespace follow {

class TargetManager final {
public:
    explicit TargetManager(TargetRecoveryConfig config = {}, ReacquisitionTrace* trace = nullptr);
    // Selector가 envelope·레코드·순서를 검증한 자료를 기존 mutex 아래 전달한다.
    void select(const TracksV1& frame, const TrackRecordV1& track, Clock::time_point received_at);
    void start(const TracksV1& frame, const TrackRecordV1& track, Clock::time_point received_at);
    void stop();
    void invalidate(const char* reason);
    void pause_ambiguous(const char* reason);
    void observe(const TracksV1& frame, Clock::time_point received_at,
                 std::chrono::milliseconds max_tracks_age);
    void advance(Clock::time_point now, std::chrono::milliseconds max_tracks_age);
    TargetRecoverySnapshot snapshot(Clock::time_point now,
                                    std::chrono::milliseconds max_tracks_age) const;

private:
    void set_phase(TargetPhase phase, const char* reason);
    void remember(const TracksV1& frame, const TrackRecordV1& track, Clock::time_point received_at);
    void reset_confirmation();
    void remember_competitors(const TracksV1& frame);
    bool reconcile_identity_swap(const TracksV1& frame, Clock::time_point received_at,
                                 std::chrono::milliseconds max_tracks_age);
    bool selected_continuity_is_unique(const TracksV1& frame, Clock::time_point received_at,
                                       std::chrono::milliseconds max_tracks_age) const;
    bool competitors_consistent(const TracksV1& frame, std::chrono::milliseconds max_tracks_age,
                                bool selected_continuity = false);
    bool known_competitor(const TracksV1& frame, const TrackRecordV1& track) const;
    TracksV1 ground_candidates(const TracksV1& frame,
                               std::chrono::milliseconds max_tracks_age) const;
    std::optional<TracksV1> position_identity_candidates(
        const TracksV1& frame, std::chrono::milliseconds max_tracks_age) const;
    mutable std::uint64_t identity_diagnostic_frame_{0};
    const ShortMotionEstimate& identity_motion(const TargetMemory& memory) const;
    bool selected_box_jumped(const TracksV1& frame, const TrackRecordV1& track) const;
    // 열 외형이 기억과 크게 다르면 true. 기록·기억 갱신도 여기서 한다 (2026-11-09).
    bool selected_thermal_mismatch(const TracksV1& frame, const TrackRecordV1& track);
    // v36_identity_context: 선택 ID가 이번 프레임에 있을 때의 신원 판단.
    // Accept=기억 갱신·Locked 진행, Hold=이번 프레임 보류(단계 설정됨), Release=대상 버림(설정됨).
    enum class V36Decision { Accept, Hold, Release };
    V36Decision v36_selected_identity(const TracksV1& frame, const TrackRecordV1& track);
    bool v36_competitor_near(const TracksV1& frame, const TrackRecordV1& track) const;
    bool v36_thermal_usable(const TrackRecordV1& track) const;
    double v36_similarity(const std::optional<ThermalSignature>& reference,
                          const TrackRecordV1& track) const;
    // 같은 종류 실검출 다른 트랙 중 기억과 가장 닮은 값(없으면 -1)과 그 ID.
    std::pair<double, std::int64_t> v36_best_other(const std::optional<ThermalSignature>& reference,
                                                   const TracksV1& frame,
                                                   const TrackRecordV1& track) const;
    void v36_log(const TracksV1& frame, const TrackRecordV1& track, const char* context,
                 double selected, double other, std::int64_t other_id, const char* decision);
    void v36_reset();
    TargetRecoveryConfig config_;
    TargetReacquirer reacquirer_;
    ShortMotionEstimator short_motion_;
    ShortMotionEstimator identity_motion_;
    BoxMotionGuard box_motion_guard_;
    FreshContactVelocityEstimator fresh_contact_velocity_;
    std::optional<TargetMemory> memory_;
    TargetPhase phase_{TargetPhase::Unselected};
    // 제어용 운동 추정과 별개인 복귀 식별 근거다. 원실관측의 250ms까지만 쓴다.
    std::optional<TargetMemory> brief_continuity_memory_;

    std::string reason_{"unselected"};
    std::uint64_t next_mission_id_{0}, revision_{0}, last_frame_id_{0};
    std::int64_t last_sample_ns_{0};
    // selected_box_jump_guard: 마지막 수용 이후 튄 박스를 버리기 시작한 표본 시각.
    std::optional<std::int64_t> box_jump_since_ns_;
    // thermal_identity: 대상의 열 외형 기억과 불일치가 시작된 표본 시각.
    std::optional<ThermalSignature> thermal_reference_;
    // 2026-11-10 선택 시점의 열 외형(고정). v35_thermal_anchor_gate에서만 판단에 쓴다.
    std::optional<ThermalSignature> thermal_anchor_;
    // 마지막 불일치가 절대 유사도 하한 때문인가 (상대 비교 불일치와 사유를 구분해 기록).
    bool thermal_low_similarity_{false};
    std::optional<std::int64_t> thermal_mismatch_since_ns_;
    std::shared_ptr<std::ofstream> thermal_log_;
    // 선택 ID가 마지막으로 화면에 보인 표본 시각(보류한 프레임 포함). 0이면 아직 없음.
    std::int64_t v36_last_seen_ns_{0};
    // 다른 ID 완화 정책의 후보별 유사도 기록.
    std::optional<SelectedTargetKey> v36_relaxed_key_;
    std::vector<double> v36_relaxed_similarity_;
    std::optional<SelectedTargetKey> candidate_key_;
    std::optional<TrackRecordV1> confirmation_track_;
    std::uint32_t confirmation_count_{0};
    std::deque<std::array<double, 3>> ground_history_;  // sample seconds, north, east
    std::optional<std::array<double, 2>> reset_previous_moving_velocity_;
    struct CompetitorMemory {
        SelectedTargetKey key;
        std::array<double, 2> ground;
        TrackRecordV1 last_track;
        std::uint32_t departure_observations{0};
        bool departed{false};
        // D2는 native CONFIRMED 표지만으로 경쟁 기억을 신뢰하지 않는다.
        std::uint32_t confirmation_observations{0};
        bool trusted{false};
        bool selected_continuity_proven{false};
        ShortMotionEstimator motion;
        ShortMotionEstimate short_motion;
        std::int64_t real_sample_ns{0}, motion_sample_ns{0};
        std::int64_t first_real_sample_ns{0};
        std::deque<std::array<double, 3>> image_history;
        bool exit_boundary_reached{false};
        std::array<std::uint32_t, 4> exit_observations{};
    };
    bool missing_competitor_is_separate(const TracksV1& frame, const CompetitorMemory& competitor,
                                        std::chrono::milliseconds max_tracks_age) const;
    std::vector<CompetitorMemory> competitors_;
};

}  // namespace follow
