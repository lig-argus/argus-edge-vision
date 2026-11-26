// tracking/status 토픽을 구독하는 ITrackingStatusSource, ITrackingStatusEvents 구현
// 규약: docs/spec/process_interfaces.md 5.1.3. 조회 규칙은
//      argus/perception_api/tracking_status_source.hpp, 알림 규칙은
//      argus/perception_api/tracking_status_events.hpp
// 사용: mavlink_gateway 의 main() 이 만들어, 사용하는 쪽에 필요한 인터페이스로만 넘긴다
#ifndef ARGUS_PERCEPTION_API_BUS_BUS_TRACKING_STATUS_SUBSCRIBER_HPP_
#define ARGUS_PERCEPTION_API_BUS_BUS_TRACKING_STATUS_SUBSCRIBER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/tracking_status_events.hpp"
#include "argus/perception_api/tracking_status_source.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::perception_api_bus {

// 카메라(source)마다 가장 최근 값을 보관한다. 추적기는 카메라마다 하나이며 각자 발행한다
class BusTrackingStatusSubscriber : public perception_api::ITrackingStatusSource,
                                    public perception_api::ITrackingStatusEvents {
public:
    // clock 은 받은 시각을 얻는 데 쓴다. 이 객체보다 오래 살아야 한다.
    // bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusTrackingStatusSubscriber> create(bus::IBus& bus,
                                                               const core::IClock& clock);

    std::optional<perception_api::ReceivedTrackingStatus> latest(
        const std::string& source) const override;
    [[nodiscard]] core::Subscription subscribe(TrackingStatusHandler handler) override;

private:
    // Subscription 이 이 객체보다 오래 살아도 안전하도록 받은 값과 handler 목록은 공유 객체에 두고,
    // Subscription 과 bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    struct State {
        std::map<std::string, perception_api::ReceivedTrackingStatus> latest;  // source 마다
        std::map<std::uint64_t, TrackingStatusHandler> handlers;               // 등록한 순서
        std::uint64_t next_id = 1;
    };

    BusTrackingStatusSubscriber(
        std::shared_ptr<State> state,
        std::unique_ptr<bus::Subscriber<perception_api::TrackingStatus>> subscriber,
        core::Subscription subscription);

    static void on_status(State& state, const perception_api::TrackingStatus& status,
                          std::int64_t arrived_monotonic_ns);

    std::shared_ptr<State> state_;
    std::unique_ptr<bus::Subscriber<perception_api::TrackingStatus>> subscriber_;
    core::Subscription subscription_;  // 이 객체가 사라지면 해제된다
};

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_BUS_TRACKING_STATUS_SUBSCRIBER_HPP_
