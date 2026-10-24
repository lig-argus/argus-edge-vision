// 기체 상태(자세, 위치, 비행 모드)를 읽는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, 5.1 fc/attitude·fc/state·fc/mode_event,
//      docs/design/uml/class_follow.md, docs/design/uml/class_tracker.md (선택 순서)
// 구현: (다음 PR) ZmqVehicleTelemetrySubscriber (vehicle_api_zmq. 조회 시점은 core::IClock),
//      테스트용 FakeVehicleTelemetry (vehicle_api_testing)
// 사용: 추종 제어, 추적기
#ifndef ARGUS_VEHICLE_API_VEHICLE_TELEMETRY_HPP_
#define ARGUS_VEHICLE_API_VEHICLE_TELEMETRY_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/core/subscription.hpp"
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
class IVehicleTelemetry {
public:
    using FlightModeEventHandler = std::function<void(const FlightModeEvent&)>;

    IVehicleTelemetry() = default;
    virtual ~IVehicleTelemetry() = default;
    IVehicleTelemetry(const IVehicleTelemetry&) = delete;
    IVehicleTelemetry& operator=(const IVehicleTelemetry&) = delete;
    IVehicleTelemetry(IVehicleTelemetry&&) = delete;
    IVehicleTelemetry& operator=(IVehicleTelemetry&&) = delete;

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
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 다른 메시지 처리가 밀리지 않게 오래 걸리는 일을 하지 않는다.
    // 반환된 Subscription 이 사라지면 그 handler 는 더 불리지 않는다.
    [[nodiscard]] virtual core::Subscription subscribe_flight_mode_events(
        FlightModeEventHandler handler) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_VEHICLE_TELEMETRY_HPP_
