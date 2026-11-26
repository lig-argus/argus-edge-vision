#pragma once

#include <array>
#include <deque>
#include <optional>

namespace follow {
struct RearGoal {
    double north_m{0}, east_m{0}, velocity_north_m_s{0}, velocity_east_m_s{0};
    bool initial_extension{true};
    double standoff_m{0};  // LOS/body 목표에만 기록. 경로 호길이와 구분한다.
};

// A distance behind the observed vehicle path. Unobserved initial history extends the first LOS
// only.
class ObservedRearGoal final {
public:
    ObservedRearGoal(double distance_m = 5, double sample_spacing_m = .5);
    RearGoal update(std::array<double, 2> point, std::array<double, 2> initial_direction,
                    double speed_m_s, bool moving,
                    std::optional<std::array<double, 2>> observed_velocity = std::nullopt,
                    bool commit_stop_anchor = false);
    void reset();

private:
    double distance_m_, sample_spacing_m_;
    std::array<double, 2> initial_direction_{1, 0};
    std::array<double, 2> forward_direction_{1, 0};
    bool reversing_{false};
    std::deque<std::array<double, 2>> points_;
};
}  // namespace follow
