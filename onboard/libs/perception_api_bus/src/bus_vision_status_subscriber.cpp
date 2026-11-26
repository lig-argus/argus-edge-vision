#include "argus/perception_api_bus/bus_vision_status_subscriber.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "argus/perception_api_bus/message_types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {

std::unique_ptr<BusVisionStatusSubscriber> BusVisionStatusSubscriber::create(
    bus::IBus& bus, const core::IClock& clock) {
    auto subscriber = bus::make_subscriber<perception_api::VisionStatus>(
        bus, protocol::kVisionStatus, kVisionStatusType, protocol::kVisionStatusQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto latest = std::make_shared<Latest>();
    const std::weak_ptr<Latest> weak = latest;
    // clock 은 이 객체보다 오래 살고, 이 handler 는 이 객체가 사라질 때 함께 해제된다
    core::Subscription subscription =
        subscriber->subscribe([weak, &clock](const perception_api::VisionStatus& status,
                                             const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                (*locked)[status.source] =
                    perception_api::ReceivedVisionStatus{status, clock.now_monotonic_ns()};
            }
        });
    return std::unique_ptr<BusVisionStatusSubscriber>(new BusVisionStatusSubscriber(
        std::move(latest), std::move(subscriber), std::move(subscription)));
}

BusVisionStatusSubscriber::BusVisionStatusSubscriber(
    std::shared_ptr<Latest> latest,
    std::unique_ptr<bus::Subscriber<perception_api::VisionStatus>> subscriber,
    core::Subscription subscription)
    : latest_(std::move(latest)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

std::optional<perception_api::ReceivedVisionStatus> BusVisionStatusSubscriber::latest(
    const std::string& source) const {
    const auto found = latest_->find(source);
    if (found == latest_->end()) {
        return std::nullopt;
    }
    return found->second;
}

}  // namespace argus::perception_api_bus
