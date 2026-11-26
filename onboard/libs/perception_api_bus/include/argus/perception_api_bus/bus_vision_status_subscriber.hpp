// vision/status 토픽을 구독하는 IVisionStatusSource 구현
// 규약: docs/spec/process_interfaces.md 5.1.7
//      조회 규칙은 argus/perception_api/vision_status_source.hpp
// 사용: mavlink_gateway 의 main() 이 만들어 IVisionStatusSource 로 넘긴다
#ifndef ARGUS_PERCEPTION_API_BUS_BUS_VISION_STATUS_SUBSCRIBER_HPP_
#define ARGUS_PERCEPTION_API_BUS_BUS_VISION_STATUS_SUBSCRIBER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api/vision_status_source.hpp"

namespace argus::perception_api_bus {

// 카메라(source)마다 가장 최근 값을 보관한다
class BusVisionStatusSubscriber : public perception_api::IVisionStatusSource {
public:
    // clock 은 받은 시각을 얻는 데 쓴다. 이 객체보다 오래 살아야 한다.
    // bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusVisionStatusSubscriber> create(bus::IBus& bus,
                                                             const core::IClock& clock);

    std::optional<perception_api::ReceivedVisionStatus> latest(
        const std::string& source) const override;

private:
    // bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    using Latest = std::map<std::string, perception_api::ReceivedVisionStatus>;  // source 마다

    BusVisionStatusSubscriber(
        std::shared_ptr<Latest> latest,
        std::unique_ptr<bus::Subscriber<perception_api::VisionStatus>> subscriber,
        core::Subscription subscription);

    std::shared_ptr<Latest> latest_;
    std::unique_ptr<bus::Subscriber<perception_api::VisionStatus>> subscriber_;
    core::Subscription subscription_;  // 이 객체가 사라지면 해제된다
};

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_BUS_VISION_STATUS_SUBSCRIBER_HPP_
