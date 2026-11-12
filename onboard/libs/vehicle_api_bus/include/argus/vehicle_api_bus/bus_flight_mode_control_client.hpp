// 비행 모드 전환을 control/mode_request 로 요청하는 IFlightModeControlClient 구현
// 규약: docs/spec/process_interfaces.md 5.2 control/mode_request
// 사용: 추종 제어 프로세스의 main() 이 만들어 FollowController 에 IFlightModeControlClient 로
// 넘긴다
// 응답 클래스: BusFlightModeControlServer
#ifndef ARGUS_VEHICLE_API_BUS_BUS_FLIGHT_MODE_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_FLIGHT_MODE_CONTROL_CLIENT_HPP_

#include <chrono>
#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/flight_mode_control_client.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"

namespace argus::vehicle_api_bus {

// 형식이 맞지 않는 응답은 통신 구현이 폐기하고 응답이 오지 않은 것과 같이 알리므로, on_reply 는
// nullopt 로 불린다. on_reply 가 비어 있으면 요청을 보내지 않고 빈 Subscription 을 반환한다
class BusFlightModeControlClient : public vehicle_api::IFlightModeControlClient {
public:
    // bus 가 요청하는 쪽을 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusFlightModeControlClient> create(bus::IBus& bus);

    [[nodiscard]] core::Subscription request_offboard(const vehicle_api::RequestHeader& header,
                                                      std::chrono::milliseconds timeout,
                                                      ReplyHandler on_reply) override;
    [[nodiscard]] core::Subscription request_hold(const vehicle_api::RequestHeader& header,
                                                  std::chrono::milliseconds timeout,
                                                  ReplyHandler on_reply) override;

private:
    using Requester = bus::Requester<FlightModeRequest, vehicle_api::FlightModeReply>;

    explicit BusFlightModeControlClient(std::unique_ptr<Requester> requester);

    core::Subscription send(const FlightModeRequest& request, std::chrono::milliseconds timeout,
                            ReplyHandler on_reply);

    std::unique_ptr<Requester> requester_;
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_FLIGHT_MODE_CONTROL_CLIENT_HPP_
