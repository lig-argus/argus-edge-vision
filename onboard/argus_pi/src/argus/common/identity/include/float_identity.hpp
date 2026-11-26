// MAVLink float32 매개변수로 정확하게 왕복하는 양수 24bit 식별자.
#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace follow {
inline constexpr std::uint32_t kFloatIdentityMax = 16777215;
inline bool valid_float_identity(std::uint64_t value) {
    return value > 0 && value <= kFloatIdentityMax;
}
class TrackingIdentityError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
inline std::uint32_t identity_from_float(float value) {
    if (!std::isfinite(value) || value < 1 || value > kFloatIdentityMax ||
        std::trunc(value) != value) {
        throw TrackingIdentityError("MAVLink identity must be an exact integer in 1..16777215");
    }
    return static_cast<std::uint32_t>(value);
}
class FloatTrackIdSequence {
public:
    explicit FloatTrackIdSequence(std::uint32_t first = 1) : next_(first) {}
    std::int64_t allocate() {
        auto current = next_.load();
        for (;;) {
            if (!valid_float_identity(current)) {
                throw TrackingIdentityError("track_id exhausted; no wrap or reuse allowed");
            }
            if (next_.compare_exchange_weak(current, current + 1)) {
                return current;
            }
        }
    }

private:
    std::atomic<std::uint32_t> next_;
};
}  // namespace follow
