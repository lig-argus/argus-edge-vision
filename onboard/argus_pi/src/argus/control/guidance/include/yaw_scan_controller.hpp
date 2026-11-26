#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/common/time/include/time.hpp"

namespace follow {

struct YawScanConfig {
    double rate_deg_s{10};
    std::chrono::milliseconds timeout{60000};
    std::chrono::milliseconds max_yaw_age{200};
    double maximum_actual_rate_deg_s{75};
    // Keep last: existing aggregate initializers retain their full-turn default.
    double sweep_deg{360};
};

// Actual yaw progress is signed and unwrapped. Timer duration never proves a turn.
class YawScanController final {
public:
    enum class State : std::uint8_t {
        Idle,
        Scanning,
        Candidate,
        Completed,
        Stopped,
        Fault,
        TimedOut
    };
    explicit YawScanController(YawScanConfig config = {});
    void start(double yaw_deg, Clock::time_point sampled_at, Clock::time_point now,
               std::uint64_t frame_id);
    // health includes UID, writer ownership, fresh FC/video, actual HOLD and no RC.
    bool update(double yaw_deg, Clock::time_point sampled_at, Clock::time_point now, bool health,
                bool fresh_candidate, std::uint64_t frame_id);
    void stop(double current_yaw_deg);
    void fail(const std::string& reason);
    // Display-only estimate, never an execution deadline or completion predicate.
    std::optional<double> remaining_seconds(Clock::time_point now) const;
    bool active() const { return state_ == State::Scanning; }
    State state() const { return state_; }
    YawScanConfig config() const { return config_; }
    double progress_deg() const { return progress_deg_; }
    double heading_deg() const { return heading_deg_; }
    const std::string& reason() const { return reason_; }

private:
    YawScanConfig config_;
    State state_{State::Idle};
    Clock::time_point begin_, sample_;
    std::uint64_t first_frame_id_{0};
    double start_yaw_{0}, last_yaw_{0}, progress_deg_{0}, heading_deg_{0};
    std::string reason_{"idle"};
};

// 조회 전용 enum 이름. UI의 최종 통합 상태 코드를 정의하지 않는다.
inline const char* yaw_scan_state_name(YawScanController::State state) {
    switch (state) {
        case YawScanController::State::Idle:
            return "Idle";
        case YawScanController::State::Scanning:
            return "Scanning";
        case YawScanController::State::Candidate:
            return "Candidate";
        case YawScanController::State::Completed:
            return "Completed";
        case YawScanController::State::Stopped:
            return "Stopped";
        case YawScanController::State::Fault:
            return "Fault";
        case YawScanController::State::TimedOut:
            return "TimedOut";
    }
    return "Unknown";
}
}  // namespace follow
