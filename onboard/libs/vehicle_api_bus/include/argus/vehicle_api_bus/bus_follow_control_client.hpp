// 추종, Scan 명령을 follow/command 로 요청하는 IFollowControlClient 구현
// 규약: docs/spec/process_interfaces.md 5.2 follow/command, 5.2.1 follow/result
// 사용: mavlink_gateway 프로세스의 main() 이 만들어 명령 처리 클래스에 IFollowControlClient 로
// 넘긴다 응답 클래스: BusFollowControlServer
#ifndef ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_CLIENT_HPP_

#include <chrono>
#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"

namespace argus::vehicle_api_bus {

// 형식이 맞지 않는 응답은 통신 구현이 폐기하고 응답이 오지 않은 것과 같이 알리므로, on_result 는
// nullopt 로 불린다. on_result 가 비어 있으면 요청을 보내지 않고 빈 Subscription 을 반환한다
class BusFollowControlClient : public vehicle_api::IFollowControlClient {
public:
    // bus 가 요청하는 쪽을 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusFollowControlClient> create(bus::IBus& bus);

    [[nodiscard]] core::Subscription track_start(const vehicle_api::RequestHeader& header,
                                                 const vehicle_api::TrackTarget& target,
                                                 std::chrono::milliseconds timeout,
                                                 ResultHandler on_result) override;
    [[nodiscard]] core::Subscription stop(const vehicle_api::RequestHeader& header,
                                          vehicle_api::StopReason reason,
                                          std::chrono::milliseconds timeout,
                                          ResultHandler on_result) override;
    [[nodiscard]] core::Subscription scan_start(const vehicle_api::RequestHeader& header,
                                                const vehicle_api::ScanRequest& scan,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) override;

private:
    using Requester = bus::Requester<FollowCommandRequest, vehicle_api::FollowResult>;

    explicit BusFollowControlClient(std::unique_ptr<Requester> requester);

    core::Subscription send(const FollowCommandRequest& request, std::chrono::milliseconds timeout,
                            ResultHandler on_result);

    std::unique_ptr<Requester> requester_;
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_CLIENT_HPP_
