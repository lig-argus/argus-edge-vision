#include "argus/video_sender/bus_frame_source.hpp"

#include <utility>

namespace argus::video_sender {

std::unique_ptr<BusFrameSource> BusFrameSource::create(bus::IBus& bus) {
    auto subscriber = perception_api_bus::create_frame_subscriber(bus);
    if (!subscriber) {
        return nullptr;
    }
    auto handlers = std::make_shared<Handlers>();
    const std::weak_ptr<Handlers> weak = handlers;
    core::Subscription subscription = subscriber->subscribe(
        [weak](const perception_api::Frame& frame, const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                locked->notify(frame);
            }
        });
    return std::unique_ptr<BusFrameSource>(
        new BusFrameSource(std::move(subscriber), std::move(handlers), std::move(subscription)));
}

BusFrameSource::BusFrameSource(std::unique_ptr<perception_api_bus::BusFrameSubscriber> subscriber,
                               std::shared_ptr<Handlers> handlers, core::Subscription subscription)
    : subscriber_(std::move(subscriber)),
      handlers_(std::move(handlers)),
      subscription_(std::move(subscription)) {}

core::Subscription BusFrameSource::subscribe(FrameHandler handler) {
    if (!handler) {
        return {};
    }
    return handlers_->subscribe(std::move(handler));
}

}  // namespace argus::video_sender
