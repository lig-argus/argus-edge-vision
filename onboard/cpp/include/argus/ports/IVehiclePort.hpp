#pragma once
#include <cstdint>
#include <string>
namespace argus {
struct ControlIntent {
    std::uint64_t epoch{}, expires_monotonic_ns{};
    double forward{}, right{}, down{}, yaw_rate{};
};
class IVehiclePort {
  public:
    virtual ~IVehiclePort() = default;
    virtual void submit(const ControlIntent &) = 0;
}; // Contract only. This migration opens NO PX4 connection and generates NO commands.
} // namespace argus
