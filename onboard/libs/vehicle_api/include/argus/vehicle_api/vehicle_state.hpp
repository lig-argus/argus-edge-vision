// 기체 상태(자세, 위치, 비행 모드)를 조회하는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, docs/design/uml/class_follow.md
// 구현: onboard FC 통신의 ZeroMQ 어댑터(vehicle_api_zmq)
//      테스트용 FakeVehicleState(vehicle_api_testing)
// 사용: 추종 제어, 추적기
#ifndef ARGUS_VEHICLE_API_VEHICLE_STATE_HPP_
#define ARGUS_VEHICLE_API_VEHICLE_STATE_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// attitude_at() 이 반환하는 자세의 수신 시각과 요청 시각의 최대 차 (docs/spec/icd.md 2장)
inline constexpr std::int64_t kAttitudeMatchToleranceNs = 50'000'000;

class IVehicleState {
public:
    using ModeEventHandler = std::function<void(const ModeEvent&)>;

    IVehicleState() = default;
    virtual ~IVehicleState() = default;
    IVehicleState(const IVehicleState&) = delete;
    IVehicleState& operator=(const IVehicleState&) = delete;
    IVehicleState(IVehicleState&&) = delete;
    IVehicleState& operator=(IVehicleState&&) = delete;

    // 가장 최근 기체 상태. FC 메시지를 아직 하나도 받지 못했으면 nullopt.
    // 값이 얼마나 오래됐는지는 호출자가 received_monotonic_ns 로 판단한다
    virtual std::optional<VehicleState> latest() const = 0;

    // 받은 자세 중 received_monotonic_ns 가 monotonic_ns 에 가장 가까운 것.
    // 차가 kAttitudeMatchToleranceNs 를 넘거나 보관한 자세가 없으면 nullopt.
    // 쓰는 시점의 수신 후 경과 시간(200 ms 이내) 검사는 호출자가 한다
    virtual std::optional<Attitude> attitude_at(std::int64_t monotonic_ns) const = 0;

    // 실제 비행 모드가 바뀔 때마다 handler 를 부른다. 여러 번 등록하면 모두 부른다.
    // handler 는 구현이 받은 메시지를 처리할 때 불린다.
    // ZeroMQ 구현은 bus 의 spin_once() 안에서, spin_once() 를 부른 스레드에서 부른다.
    // 다른 메시지 처리가 밀리지 않게 오래 걸리는 일을 하지 않는다.
    virtual void on_mode_event(ModeEventHandler handler) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_VEHICLE_STATE_HPP_
