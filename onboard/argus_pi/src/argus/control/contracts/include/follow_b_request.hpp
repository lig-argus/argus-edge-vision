// B adapter request semantics; no MAVLink IDs or parameter positions are assigned here.
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "argus/control/guidance/include/yaw_scan_controller.hpp"

namespace follow {
struct ScanRequest {
    std::optional<double> yaw_range_rad;
    std::optional<double> yaw_rate_rad_s;
    std::optional<double> timeout_s;
};

inline YawScanConfig parse_scan_request(const ScanRequest& request, YawScanConfig defaults = {}) {
    constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
    auto specified = [](const std::optional<double>& value) {
        if (!value || std::isnan(*value)) {
            return false;
        }
        if (!std::isfinite(*value)) {
            throw std::invalid_argument("scan parameter must be finite or NaN default");
        }
        return true;
    };
    auto config = defaults;
    if (specified(request.yaw_range_rad)) {
        constexpr double kPi = 3.14159265358979323846;
        auto range = *request.yaw_range_rad;
        // PR27의 XML 상한을 float32로 보낸 경우만 안전 상한 2π로 정규화한다.
        // 실제 회전 범위 360도 제한은 늘리지 않는다.
        if (range > 2 * kPi && range <= 6.2832) {
            range = 2 * kPi;
        }
        config.sweep_deg = range * kDegreesPerRadian;
    }
    if (specified(request.yaw_rate_rad_s)) {
        constexpr double kLimit = 75.0 / kDegreesPerRadian;
        auto rate = *request.yaw_rate_rad_s;
        if (std::abs(rate) > kLimit &&
            std::abs(rate) <= static_cast<double>(static_cast<float>(1.309))) {
            rate = std::copysign(kLimit, rate);
        }
        config.rate_deg_s = rate * kDegreesPerRadian;
    }
    if (specified(request.timeout_s)) {
        const double seconds = *request.timeout_s;
        const long double milliseconds = std::ceil(static_cast<long double>(seconds) * 1000);
        if (seconds <= 0 || milliseconds > std::numeric_limits<std::int64_t>::max()) {
            throw std::invalid_argument("scan timeout is out of millisecond range");
        }
        config.timeout = std::chrono::milliseconds{static_cast<std::int64_t>(milliseconds)};
    }
    // Share core validation with native execution. No command or socket is opened.
    (void)YawScanController{config};
    return config;
}
}  // namespace follow
