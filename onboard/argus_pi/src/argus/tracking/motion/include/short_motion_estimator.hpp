#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>

namespace follow {
enum class ObservedMotionPhase { Unknown, Stopped, Moving };
struct ShortMotionConfig {
    bool enabled{false};
    double window_s{.5}, minimum_span_s{.4}, confirmation_s{.15};
    std::uint32_t minimum_samples{6};
    double stop_speed_m_s{.15}, resume_speed_m_s{.25}, maximum_rmse_m{.1};
    double maximum_sample_gap_s{.25};
};
struct ShortMotionEstimate {
    ObservedMotionPhase phase{ObservedMotionPhase::Unknown};
    bool valid{false};
    bool direction_valid{false};
    bool feedforward_valid{false};
    bool position_changed_since_stop{false};  // 새 실위치의 변화. 확정 속도/방향을 뜻하지 않는다.
    bool departure_pending{false};            // 새 실위치/회귀속도로 출발 확인 대기 중.
    double north_m{0}, east_m{0}, velocity_north_m_s{0}, velocity_east_m_s{0}, rmse_m{0};
    // Motion-clock timestamp of the last actual regression, not the host observation TTL.
    std::int64_t fit_sample_ns{0};
};
// Only newly admitted real ground points enter this short, independent linear fit.
class ShortMotionEstimator final {
public:
    explicit ShortMotionEstimator(ShortMotionConfig config = {});
    ShortMotionEstimate update(std::int64_t sample_ns, std::array<double, 2> point);
    void reset();

private:
    ShortMotionConfig config_;
    std::deque<std::array<double, 3>> points_;
    ShortMotionEstimate latest_;
    std::optional<ObservedMotionPhase> pending_;
    double pending_at_s_{0};
    std::optional<std::array<double, 2>> stopped_at_;
};
}  // namespace follow
