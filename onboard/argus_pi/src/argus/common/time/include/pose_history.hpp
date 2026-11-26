// 같은 단조 clock 안에서 프레임 표본 시각에 맞춘 FC pose를 보간한다.
#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "argus/vehicle/contracts/include/vehicle_state.hpp"

namespace follow {

class PoseHistory final {
public:
    explicit PoseHistory(std::size_t capacity = 256);
    void offer(const Pose& pose);
    std::optional<Pose> interpolate(std::int64_t sample_monotonic_ns,
                                    const std::string& clock_domain,
                                    std::chrono::nanoseconds max_bracket_distance) const;

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<Pose> samples_;
};

}  // namespace follow
