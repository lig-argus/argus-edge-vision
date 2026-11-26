// RPi CPU 온도를 읽는 인터페이스
// 규약: docs/spec/icd.md 3.1 (ARGUS_STATUS.cpu_temp)
// 구현: SysfsCpuTemperature
// 사용: ArgusStatusReporter
#ifndef ARGUS_ONBOARD_CPU_TEMPERATURE_HPP_
#define ARGUS_ONBOARD_CPU_TEMPERATURE_HPP_

#include <optional>

namespace argus::mavlink_gateway {

class ICpuTemperature {
public:
    ICpuTemperature() = default;
    virtual ~ICpuTemperature() = default;
    ICpuTemperature(const ICpuTemperature&) = delete;
    ICpuTemperature& operator=(const ICpuTemperature&) = delete;
    ICpuTemperature(ICpuTemperature&&) = delete;
    ICpuTemperature& operator=(ICpuTemperature&&) = delete;

    // 현재 CPU 온도 (섭씨). 읽지 못하면 nullopt
    virtual std::optional<double> celsius() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_CPU_TEMPERATURE_HPP_
