#include "argus/mavlink_gateway/sysfs_cpu_temperature.hpp"

#include <cstdint>
#include <fstream>
#include <optional>

namespace argus::mavlink_gateway {

std::optional<double> SysfsCpuTemperature::celsius() const {
    std::ifstream file(path_);
    std::int64_t millidegrees = 0;
    if (!(file >> millidegrees)) {
        return std::nullopt;
    }
    return static_cast<double>(millidegrees) / 1000.0;
}

}  // namespace argus::mavlink_gateway
