#include <chrono>

#include "argus/gcs/core/clock.hpp"

namespace argus::gcs::core {

qint64 SteadyClock::nowMonotonicNs() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace argus::gcs::core
