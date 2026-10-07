#pragma once
#include <cstdint>
namespace argus {
struct ClockStamp {
    std::uint64_t unix_ns{}, monotonic_ns{};
};
class IClock {
  public:
    virtual ~IClock() = default;
    virtual ClockStamp now() const = 0;
};
class SystemClock final : public IClock {
  public:
    ClockStamp now() const override;
};
} // namespace argus
