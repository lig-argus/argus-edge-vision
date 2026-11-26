// 기체의 최신 위치·자세 사본을 제공하는 읽기 전용 인터페이스다.
#pragma once
#include <chrono>
#include <optional>
#include <string>

#include "argus/vehicle/contracts/include/vehicle_state.hpp"
namespace follow {
class IVehicleStateSource {
public:
    virtual ~IVehicleStateSource() = default;
    // 호출자는 반환된 Pose의 fresh()로 위치와 자세의 수신 상태를 확인한다.
    virtual Pose read() const = 0;
    // 구현이 시각 정렬을 제공하지 않으면 nullopt를 돌려 억지로 최신 자세를 끼워 맞추지 않는다.
    virtual std::optional<Pose> read_aligned(std::int64_t, const std::string&,
                                             std::chrono::nanoseconds) const {
        return std::nullopt;
    }
};
}  // namespace follow
