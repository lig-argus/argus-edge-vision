#include "argus/video_api_bus/bus_video_stream_info_subscriber.hpp"

#include <memory>
#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/video_api_bus/message_types.hpp"

namespace argus::video_api_bus {

std::unique_ptr<BusVideoStreamInfoSubscriber> BusVideoStreamInfoSubscriber::create(
    bus::IBus& bus, const core::IClock& clock) {
    auto subscriber = bus::make_subscriber<video_api::VideoStreamInfo>(
        bus, protocol::kVideoStreamInfo, kVideoStreamInfoType,
        protocol::kVideoStreamInfoQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto state = std::make_shared<State>();
    const std::weak_ptr<State> weak = state;
    // clock 은 이 객체보다 오래 살고, 이 handler 는 이 객체가 사라질 때 함께 해제된다
    core::Subscription subscription = subscriber->subscribe(
        [weak, &clock](const video_api::VideoStreamInfo& info, const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                locked->latest = video_api::ReceivedVideoStreamInfo{info, clock.now_monotonic_ns()};
            }
        });
    return std::unique_ptr<BusVideoStreamInfoSubscriber>(new BusVideoStreamInfoSubscriber(
        std::move(state), std::move(subscriber), std::move(subscription)));
}

BusVideoStreamInfoSubscriber::BusVideoStreamInfoSubscriber(
    std::shared_ptr<State> state,
    std::unique_ptr<bus::Subscriber<video_api::VideoStreamInfo>> subscriber,
    core::Subscription subscription)
    : state_(std::move(state)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

std::optional<video_api::ReceivedVideoStreamInfo> BusVideoStreamInfoSubscriber::latest() const {
    return state_->latest;
}

}  // namespace argus::video_api_bus
