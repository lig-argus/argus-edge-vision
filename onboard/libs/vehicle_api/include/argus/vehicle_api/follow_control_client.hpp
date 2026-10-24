// 추종, 스캔 명령을 보내는 인터페이스 (호출용. 응답은 handler 로 비동기로 받는다)
// 규약: docs/spec/process_interfaces.md 5.2 follow/command, 5.2.1 follow/result
// 구현: (다음 PR) ZmqFollowControlClient (vehicle_api_zmq, bus::IRpcClient 로 보냄),
//      테스트용 FakeFollowControlClient (vehicle_api_testing)
// 사용: onboard 의 TrackStartHandler, TrackStopHandler, ScanStartHandler, GcsLinkMonitor
#ifndef ARGUS_VEHICLE_API_FOLLOW_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_FOLLOW_CONTROL_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <optional>

#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// 호출하는 쪽이 한 스레드 루프 안에 있어 응답을 기다리며 멈출 수 없으므로, 요청을 보내고 바로
// 반환하며 응답은 handler 로 받는다. 받는 쪽(추종 제어)이 구현하는 IFollowControl 과 짝이다.
// - handler 는 core::IPoller::process_events() 안에서 정확히 한 번 불린다.
//   응답이 timeout 안에 오지 않으면 nullopt 로 불린다.
// - 반환된 Subscription 이 먼저 사라지면 handler 는 불리지 않는다.
// - 첫 결과만 이 handler 로 온다. 이후 진행은 follow/status 의 command_results 에서 같은
//   command_id 로 찾는다 (IFollowControl 의 공통 계약과 같다).
class IFollowControlClient {
public:
    using ResultHandler = std::function<void(std::optional<FollowResult>)>;

    IFollowControlClient() = default;
    virtual ~IFollowControlClient() = default;
    IFollowControlClient(const IFollowControlClient&) = delete;
    IFollowControlClient& operator=(const IFollowControlClient&) = delete;
    IFollowControlClient(IFollowControlClient&&) = delete;
    IFollowControlClient& operator=(IFollowControlClient&&) = delete;

    [[nodiscard]] virtual core::Subscription track_start(const RequestHeader& header,
                                                         const TrackTarget& target,
                                                         std::chrono::milliseconds timeout,
                                                         ResultHandler on_result) = 0;

    [[nodiscard]] virtual core::Subscription stop(const RequestHeader& header, StopReason reason,
                                                  std::chrono::milliseconds timeout,
                                                  ResultHandler on_result) = 0;

    [[nodiscard]] virtual core::Subscription scan_start(const RequestHeader& header,
                                                        const ScanRequest& scan,
                                                        std::chrono::milliseconds timeout,
                                                        ResultHandler on_result) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FOLLOW_CONTROL_CLIENT_HPP_
