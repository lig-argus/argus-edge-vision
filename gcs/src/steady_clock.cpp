#include <chrono>

#include "argus/gcs/clock.hpp"

namespace argus::gcs {

qint64 SteadyClock::nowMonotonicNs() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace argus::gcs
