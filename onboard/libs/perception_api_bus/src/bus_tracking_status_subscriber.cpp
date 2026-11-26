#include "argus/perception_api_bus/bus_tracking_status_subscriber.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "argus/perception_api_bus/message_types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {

std::unique_ptr<BusTrackingStatusSubscriber> BusTrackingStatusSubscriber::create(
    bus::IBus& bus, const core::IClock& clock) {
    auto subscriber = bus::make_subscriber<perception_api::TrackingStatus>(
        bus, protocol::kTrackingStatus, kTrackingStatusType, protocol::kTrackingStatusQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto state = std::make_shared<State>();
    const std::weak_ptr<State> weak = state;
    // clock 은 이 객체보다 오래 살고, 이 handler 는 이 객체가 사라질 때 함께 해제된다
    core::Subscription subscription =
        subscriber->subscribe([weak, &clock](const perception_api::TrackingStatus& status,
                                             const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                on_status(*locked, status, clock.now_monotonic_ns());
            }
        });
    return std::unique_ptr<BusTrackingStatusSubscriber>(new BusTrackingStatusSubscriber(
        std::move(state), std::move(subscriber), std::move(subscription)));
}

BusTrackingStatusSubscriber::BusTrackingStatusSubscriber(
    std::shared_ptr<State> state,
    std::unique_ptr<bus::Subscriber<perception_api::TrackingStatus>> subscriber,
    core::Subscription subscription)
    : state_(std::move(state)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

std::optional<perception_api::ReceivedTrackingStatus> BusTrackingStatusSubscriber::latest(
    const std::string& source) const {
    const auto found = state_->latest.find(source);
    if (found == state_->latest.end()) {
        return std::nullopt;
    }
    return found->second;
}

core::Subscription BusTrackingStatusSubscriber::subscribe(TrackingStatusHandler handler) {
    if (!handler) {
        std::cerr << "[perception_api_bus] BusTrackingStatusSubscriber::subscribe: 빈 handler\n";
        return {};
    }
    const std::uint64_t id = state_->next_id++;
    state_->handlers.emplace(id, std::move(handler));
    const std::weak_ptr<State> weak = state_;
    return core::Subscription([weak, id] {
        if (const auto locked = weak.lock()) {
            locked->handlers.erase(id);
        }
    });
}

void BusTrackingStatusSubscriber::on_status(State& state,
                                            const perception_api::TrackingStatus& status,
                                            std::int64_t arrived_monotonic_ns) {
    const perception_api::ReceivedTrackingStatus received{status, arrived_monotonic_ns};
    state.latest[status.source] = received;
    // handler 가 등록을 해제해도 안전하도록 복사해서 부른다
    const auto handlers = state.handlers;
    for (const auto& [id, handler] : handlers) {
        handler(received);
    }
}

}  // namespace argus::perception_api_bus
