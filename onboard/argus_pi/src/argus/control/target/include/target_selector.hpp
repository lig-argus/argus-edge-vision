// 운용자 명령을 처리하고 선택한 임무 표적의 최신 상태를 제공한다.
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "argus/common/time/include/time.hpp"
#include "argus/control/target/include/target_manager.hpp"
#include "argus/tracking/contracts/include/target_state.hpp"
#include "argus/tracking/contracts/include/tracks_v1.hpp"

namespace follow {

enum class TargetCommandAction { Select, Start, Stop, SelectAndStart };
struct TargetCommand {
    std::string request_id;
    TargetCommandAction action{TargetCommandAction::Stop};
    std::uint64_t generation{0};
    std::int64_t track_id{0};
    std::string expected_uid;
    // native vMy 선택 검증용. 기존 target command v1/v2 wire에는 추가하지 않는다.
    std::optional<std::int32_t> class_id;
    std::optional<std::string> label;
    std::optional<std::string> sensor;
};
struct TargetCommandResult {
    std::string request_id;
    bool accepted{false};
    std::string status;
    std::string reason;
};
struct TargetSelectionSnapshot {
    enum class Phase { Stopped, Pending, Following } phase{Phase::Stopped};
    std::optional<SelectedTargetKey> target;
    std::optional<TracksV1> latest_frame;
    std::optional<TrackRecordV1> selected_track;
    TargetState state{TargetState::Lost};
    bool fresh_real_detection{false};
    double age_since_real_s{0};
    // Start/Stop 경계를 구분해 중단 직전 계산된 명령을 송신 전에 폐기한다.
    std::uint64_t command_epoch{0};
    std::uint64_t mission_target_id{0};
    std::uint64_t target_revision{0};
    TargetPhase target_phase{TargetPhase::Unselected};
    std::string recovery_reason;
    bool recovery_pause{false};
    // Actual observation validity used by control; never extend it for status reporting.
    std::optional<Clock::time_point> observation_valid_until;
    // Reporting-only target retention deadline. NOT a motion command/observation lease.
    std::optional<Clock::time_point> prediction_valid_until;
    Clock::time_point snapshot_at{};
    bool ground_motion_valid{false};
    double velocity_north_m_s{0}, velocity_east_m_s{0};
    bool ground_acceleration_valid{false};
    double acceleration_north_m_s2{0}, acceleration_east_m_s2{0};
    bool short_motion_enabled{false};
    ShortMotionEstimate short_motion;
    bool box_motion_suspect{false}, cv_gap_active{false};
    double cv_velocity_north_m_s{0}, cv_velocity_east_m_s{0};
    std::int64_t cv_origin_sample_ns{0};
    bool cv_velocity_unconfirmed{false};
    std::int64_t cv_velocity_fit_sample_ns{0};
    std::optional<Clock::time_point> cv_gap_deadline;
    std::optional<CvVisibilityReference> cv_visibility_reference;
    bool cv_ten_frame_policy{false};
    std::optional<std::uint64_t> cv_history_frame_id;
    std::int64_t cv_history_source_ns{0};
    // 상태 표시용 신원만 보존한다. 오래된 관측/제어 입력을 신선하게 만들지 않는다.
    std::optional<std::int32_t> identity_class_id;
    std::string identity_label, identity_source;
};

inline bool same_selection(const TargetSelectionSnapshot& a, const TargetSelectionSnapshot& b) {
    if (a.phase != b.phase || a.command_epoch != b.command_epoch ||
        a.mission_target_id != b.mission_target_id || a.target_revision != b.target_revision ||
        a.target_phase != b.target_phase || a.recovery_pause != b.recovery_pause ||
        a.cv_gap_active != b.cv_gap_active || a.cv_origin_sample_ns != b.cv_origin_sample_ns ||
        a.cv_gap_deadline != b.cv_gap_deadline || a.box_motion_suspect != b.box_motion_suspect ||
        (a.box_motion_suspect && (a.cv_velocity_north_m_s != b.cv_velocity_north_m_s ||
                                  a.cv_velocity_east_m_s != b.cv_velocity_east_m_s)) ||
        a.target.has_value() != b.target.has_value()) {
        return false;
    }
    return !a.target || (a.target->generation == b.target->generation &&
                         a.target->track_id == b.target->track_id);
}

struct TargetSelectorConfig {
    std::chrono::milliseconds max_tracks_age{250};
    double lost_track_ttl_s{0.7};
    double overlap_iou_reject{0.65};
    std::size_t dedup_capacity{256};
    TargetRecoveryConfig recovery;
    bool use_fitted_ground_position{
        true};  // 기존 경로 제어용. 시각 PID는 현재 실관측 위치를 사용한다.
    bool require_local_monotonic_sample{
        false};  // Pi 감시에서만 같은 호스트 원표본 나이를 강제한다.
    bool allow_idle_camera_context_switch{false};
};

// latest tracks의 도착 신선도와 원본 표본 TTL을 각각 확인한다. 둘은 서로 다른 clock이다.
class TargetSelector final {
public:
    explicit TargetSelector(TargetSelectorConfig config = {}, ReacquisitionTrace* trace = nullptr);
    bool update(const TracksV1& tracks, Clock::time_point local_received_at);
    TargetCommandResult execute(const TargetCommand& command, Clock::time_point now = Clock::now());
    TargetSelectionSnapshot snapshot(Clock::time_point now = Clock::now()) const;
    void advance(Clock::time_point now);
    void invalidate_recovery(const std::string& reason);
    // Explicit stopped-only reset after a producer/source/FC epoch restart.
    bool reset_input_context();

private:
    friend class BusFollowingApplication;
    void stop_for_shutdown(Clock::time_point now);
    struct CachedCommand {
        TargetCommand command;
        TargetCommandResult result;
    };
    bool tracks_fresh(Clock::time_point now) const;
    const TrackRecordV1* find_target(const TracksV1& message, const SelectedTargetKey& key) const;
    bool overlapping_candidate(const TracksV1& message, const TrackRecordV1& selected) const;
    TargetCommandResult apply(const TargetCommand& command, Clock::time_point now);

    const TargetSelectorConfig config_;
    mutable std::mutex mutex_;
    std::optional<TracksV1> latest_;
    Clock::time_point latest_received_at_{};
    std::optional<SelectedTargetKey> pending_;
    std::optional<SelectedTargetKey> active_;
    std::uint64_t command_epoch_{0};
    std::map<std::string, CachedCommand> command_cache_;
    TargetManager target_manager_;
};

}  // namespace follow
