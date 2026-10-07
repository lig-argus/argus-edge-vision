#include "argus/ports/IClock.hpp"
#include <ctime>
#include <stdexcept>
namespace argus {
static std::uint64_t stamp(clockid_t kind) {
    timespec value{};
    if (clock_gettime(kind, &value))
        throw std::runtime_error("clock_gettime failed");
    return std::uint64_t(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}
ClockStamp SystemClock::now() const {
    return {stamp(CLOCK_REALTIME), stamp(CLOCK_MONOTONIC)};
}
} // namespace argus
