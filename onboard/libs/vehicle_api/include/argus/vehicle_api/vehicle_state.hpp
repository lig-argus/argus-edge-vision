// 기체 상태(자세, 위치, 비행 모드)를 조회하는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, docs/design/uml/class_follow.md,
//      docs/design/uml/class_tracker.md (선택 순서)
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

// *_at() 이 반환하는 값의 수신 시각과 요청 시각의 최대 차 (docs/spec/icd.md 2장)
inline constexpr std::int64_t kTelemetryMatchToleranceNs = 50'000'000;

// *_at() 이 고르는 값의 수신 시각과 조회 시점의 최대 차 (docs/design/uml/class_tracker.md)
inline constexpr std::int64_t kTelemetryMaxAgeNs = 200'000'000;

// 프레임 시각에 맞는 값을 찾는 조회(*_at)의 공통 규칙
// 1. 구현이 자기 시계로 조회 시점을 얻어, 그 시점에서 kTelemetryMaxAgeNs 이내에 받은 값만 남긴다.
// 2. 남은 값 중 received_monotonic_ns 가 monotonic_ns 에 가장 가까운 것을 고른다.
//    프레임 앞뒤 어느 쪽 값이든 쓸 수 있다. 두 값을 보간하지 않는다.
// 3. 고른 값과 monotonic_ns 의 차가 kTelemetryMatchToleranceNs 를 넘거나 남은 값이 없으면 nullopt.
// 조회 함수는 대상 시각 하나만 받고 최근 값의 보관과 조회 시점은 구현이 맡는다.
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
    // 값이 얼마나 오래됐는지는 호출자가 각 값의 received_monotonic_ns 로 판단한다
    virtual std::optional<VehicleState> latest() const = 0;

    // monotonic_ns(프레임 수신 시각)에 맞는 자세. 위 공통 규칙으로 고른다
    virtual std::optional<Attitude> attitude_at(std::int64_t monotonic_ns) const = 0;

    // monotonic_ns 에 맞는 local NED 위치. 위 공통 규칙으로 고른다
    virtual std::optional<LocalPosition> local_position_at(std::int64_t monotonic_ns) const = 0;

    // monotonic_ns 에 맞는 하향 거리. 위 공통 규칙으로 고른다
    virtual std::optional<DownwardDistance> downward_distance_at(
        std::int64_t monotonic_ns) const = 0;

    // 실제 비행 모드가 바뀔 때마다 handler 를 부른다. 여러 번 등록하면 모두 부른다.
    // handler 는 구현이 받은 메시지를 처리할 때 불린다.
    // ZeroMQ 구현은 IBusPoller::process_messages() 안에서, 그 함수를 부른 스레드에서 부른다.
    // 다른 메시지 처리가 밀리지 않게 오래 걸리는 일을 하지 않는다.
    virtual void on_mode_event(ModeEventHandler handler) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_VEHICLE_STATE_HPP_
