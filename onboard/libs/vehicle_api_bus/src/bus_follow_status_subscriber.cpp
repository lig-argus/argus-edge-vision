#include "argus/vehicle_api_bus/bus_follow_status_subscriber.hpp"

#include <cstdint>
#include <memory>
#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusFollowStatusSubscriber> BusFollowStatusSubscriber::create(
    bus::IBus& bus, const core::IClock& clock) {
    auto subscriber = bus::make_subscriber<vehicle_api::FollowStatus>(
        bus, protocol::kFollowStatus, kFollowStatusType, protocol::kFollowStatusQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto state = std::make_shared<State>();
    const std::weak_ptr<State> weak = state;
    // clock 은 이 객체보다 오래 살고, 이 handler 는 이 객체가 사라질 때 함께 해제된다
    core::Subscription subscription = subscriber->subscribe(
        [weak, &clock](const vehicle_api::FollowStatus& status, const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                on_status(*locked, status, clock.now_monotonic_ns());
            }
        });
    return std::unique_ptr<BusFollowStatusSubscriber>(new BusFollowStatusSubscriber(
        std::move(state), std::move(subscriber), std::move(subscription)));
}

BusFollowStatusSubscriber::BusFollowStatusSubscriber(
    std::shared_ptr<State> state,
    std::unique_ptr<bus::Subscriber<vehicle_api::FollowStatus>> subscriber,
    core::Subscription subscription)
    : state_(std::move(state)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

std::optional<vehicle_api::ReceivedFollowStatus> BusFollowStatusSubscriber::latest() const {
    return state_->latest;
}

core::Subscription BusFollowStatusSubscriber::subscribe(FollowStatusHandler handler) {
    if (!handler) {
        detail::log_error("BusFollowStatusSubscriber::subscribe: 빈 handler");
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

void BusFollowStatusSubscriber::on_status(State& state, const vehicle_api::FollowStatus& status,
                                          std::int64_t arrived_monotonic_ns) {
    state.latest = vehicle_api::ReceivedFollowStatus{status, arrived_monotonic_ns};
    // handler 가 등록을 해제해도 안전하도록 복사해서 부른다
    const auto handlers = state.handlers;
    const vehicle_api::ReceivedFollowStatus received = *state.latest;
    for (const auto& [id, handler] : handlers) {
        handler(received);
    }
}

}  // namespace argus::vehicle_api_bus
