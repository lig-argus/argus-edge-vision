#include "argus/control/guidance/include/yaw_scan_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace follow {
YawScanController::YawScanController(YawScanConfig config) : config_(config) {
    if (!std::isfinite(config.sweep_deg) || config.sweep_deg <= 0 || config.sweep_deg > 360 ||
        !std::isfinite(config.rate_deg_s) || config.rate_deg_s == 0 ||
        std::abs(config.rate_deg_s) > 75 ||
        std::abs(config.rate_deg_s) > config.maximum_actual_rate_deg_s ||
        !std::isfinite(config.maximum_actual_rate_deg_s) || config.maximum_actual_rate_deg_s <= 0 ||
        config.timeout.count() <= 0 || config.max_yaw_age.count() <= 0 ||
        config.timeout <=
            std::chrono::duration<double>(config.sweep_deg / std::abs(config.rate_deg_s))) {
        throw std::invalid_argument("invalid yaw scan configuration");
    }
}
std::optional<double> YawScanController::remaining_seconds(Clock::time_point now) const {
    if (!active() || now < begin_ || now < sample_ || now - sample_ > config_.max_yaw_age) {
        return std::nullopt;
    }
    const double timeout_left =
        std::max(0.0, std::chrono::duration<double>(config_.timeout - (now - begin_)).count());
    const double rotation_left = std::max(0.0, config_.sweep_deg - std::max(0.0, progress_deg_)) /
                                 std::abs(config_.rate_deg_s);
    return std::min(timeout_left, rotation_left);
}

void YawScanController::start(double yaw, Clock::time_point sample, Clock::time_point now,
                              std::uint64_t frame_id) {
    if (active() || !std::isfinite(yaw) || sample == Clock::time_point{} || now < sample ||
        now - sample > config_.max_yaw_age) {
        throw std::invalid_argument("scan requires a fresh yaw sample and inactive scan");
    }
    state_ = State::Scanning;
    begin_ = now;
    sample_ = sample;
    first_frame_id_ = frame_id;
    start_yaw_ = last_yaw_ = heading_deg_ = yaw;
    progress_deg_ = 0;
    reason_ = "scanning";
}
bool YawScanController::update(double yaw, Clock::time_point sample, Clock::time_point now,
                               bool health, bool candidate, std::uint64_t frame_id) {
    if (!active()) {
        return false;
    }
    if (!health || !std::isfinite(yaw) || now < begin_ || now < sample || sample < sample_ ||
        now - sample > config_.max_yaw_age) {
        fail("scan_health_or_timestamp_invalid");
        return false;
    }
    const double direction = config_.rate_deg_s > 0 ? 1 : -1;
    if (sample > sample_) {
        const double dt = std::chrono::duration<double>(sample - sample_).count();
        const double change = std::remainder(yaw - last_yaw_, 360.0);
        if (sample - sample_ > config_.max_yaw_age ||
            std::abs(change) > (config_.maximum_actual_rate_deg_s * dt) + 1) {
            fail("yaw_discontinuity");
            return false;
        }
        // Net signed progress prevents alternating jitter from completing a turn.
        progress_deg_ += direction * change;
        last_yaw_ = yaw;
        sample_ = sample;
    }
    // ICD: 제한 시간은 정상 탐색 종료다. 건강/시각/연속성 검사를 통과한 경우에만
    // 현재 heading으로 정지 요청을 만들며, 실제 종료 수락은 호출자의 ACK 확인 뒤다.
    if (now - begin_ >= config_.timeout) {
        heading_deg_ = yaw;
        state_ = State::TimedOut;
        reason_ = "scan_time_limit_completed";
        return true;
    }
    if (candidate && frame_id > first_frame_id_) {
        heading_deg_ = yaw;
        state_ = State::Candidate;
        reason_ = "candidate_requires_explicit_selection_and_start";
        return true;
    }
    if (progress_deg_ >= config_.sweep_deg) {
        heading_deg_ = yaw;
        state_ = State::Completed;
        reason_ = config_.sweep_deg == 360 ? "actual_turn_completed" : "actual_sweep_completed";
        return true;
    }
    // Advance the reference at the requested rate despite actuator lag. Bound the
    // lookahead to two command intervals so a stalled vehicle cannot accumulate
    // a distant heading (or reverse across the shortest-heading boundary).
    // Only measured signed yaw above can complete the sweep.
    const double rate = std::abs(config_.rate_deg_s);
    const double lead = std::max(1.0, rate * .5);
    const double measured = std::max(0.0, progress_deg_);
    const double scheduled = rate * std::chrono::duration<double>(now - begin_).count();
    const double reference =
        std::min(config_.sweep_deg + lead,
                 std::min(std::max(measured, scheduled) + lead, measured + 2 * lead));
    heading_deg_ = std::remainder(start_yaw_ + direction * reference, 360.0);
    return true;
}
void YawScanController::stop(double current_yaw_deg) {
    if (!std::isfinite(current_yaw_deg)) {
        fail("invalid_stop_yaw");
        return;
    }
    heading_deg_ = current_yaw_deg;
    state_ = State::Stopped;
    reason_ = "operator_stop";
}
void YawScanController::fail(const std::string& reason) {
    state_ = State::Fault;
    reason_ = reason;
}
}  // namespace follow
