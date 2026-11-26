// Linux thermal zone 파일에서 CPU 온도를 읽는 ICpuTemperature 구현
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 ArgusStatusReporter 에 전달
#ifndef ARGUS_ONBOARD_SYSFS_CPU_TEMPERATURE_HPP_
#define ARGUS_ONBOARD_SYSFS_CPU_TEMPERATURE_HPP_

#include <optional>
#include <string>
#include <utility>

#include "argus/mavlink_gateway/cpu_temperature.hpp"

namespace argus::mavlink_gateway {

// RPi 의 CPU 온도 파일. 값은 밀리도 단위의 정수 한 줄이다 (Linux thermal sysfs)
inline constexpr const char* kDefaultThermalZonePath = "/sys/class/thermal/thermal_zone0/temp";

// 호출할 때마다 파일을 읽는다 (1 Hz 송신에서만 부르므로 보관하지 않는다).
// 파일이 없거나(macOS 등) 정수가 아니면 nullopt
class SysfsCpuTemperature : public ICpuTemperature {
public:
    explicit SysfsCpuTemperature(std::string path = kDefaultThermalZonePath)
        : path_(std::move(path)) {}

    std::optional<double> celsius() const override;

private:
    std::string path_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SYSFS_CPU_TEMPERATURE_HPP_
