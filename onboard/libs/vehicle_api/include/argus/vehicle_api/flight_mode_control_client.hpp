// 비행 모드 전환을 요청하는 인터페이스 (호출용. 응답은 handler 로 비동기로 받는다)
// 규약: docs/spec/process_interfaces.md 3장(요청 공통 필드), 5.2 control/mode_request
// 구현: (다음 PR) ZmqFlightModeControlClient (vehicle_api_zmq, bus::IRpcClient 로 보냄),
//      테스트용 FakeFlightModeControlClient (vehicle_api_testing)
// 사용: 추종 제어(FollowController)
#ifndef ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <optional>

#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// 호출하는 쪽이 한 스레드 루프 안에 있어 응답을 기다리며 멈출 수 없으므로, 요청을 보내고 바로
// 반환하며 응답은 handler 로 받는다. 받는 쪽(onboard)이 구현하는 IFlightModeControl 과 짝이다.
// - handler 는 core::IPoller::process_events() 안에서 정확히 한 번 불린다.
//   응답이 timeout 안에 오지 않으면 nullopt 로 불린다.
// - 반환된 Subscription 이 먼저 사라지면 handler 는 불리지 않는다. 보낸 요청은 취소되지 않는다.
// - 응답은 수락 여부다. 실제 모드 전환은 IVehicleTelemetry::subscribe_flight_mode_events 로
// 확인한다.
class IFlightModeControlClient {
public:
    using ReplyHandler = std::function<void(std::optional<FlightModeReply>)>;

    IFlightModeControlClient() = default;
    virtual ~IFlightModeControlClient() = default;
    IFlightModeControlClient(const IFlightModeControlClient&) = delete;
    IFlightModeControlClient& operator=(const IFlightModeControlClient&) = delete;
    IFlightModeControlClient(IFlightModeControlClient&&) = delete;
    IFlightModeControlClient& operator=(IFlightModeControlClient&&) = delete;

    [[nodiscard]] virtual core::Subscription request_offboard(const RequestHeader& header,
                                                              std::chrono::milliseconds timeout,
                                                              ReplyHandler on_reply) = 0;

    [[nodiscard]] virtual core::Subscription request_hold(const RequestHeader& header,
                                                          std::chrono::milliseconds timeout,
                                                          ReplyHandler on_reply) = 0;

    // Hold 상태에서 절대 heading(rad, NED, 북쪽 0) 으로 회전 (docs/spec/icd.md E3)
    [[nodiscard]] virtual core::Subscription request_heading(const RequestHeader& header,
                                                             double heading_rad,
                                                             std::chrono::milliseconds timeout,
                                                             ReplyHandler on_reply) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_CLIENT_HPP_
