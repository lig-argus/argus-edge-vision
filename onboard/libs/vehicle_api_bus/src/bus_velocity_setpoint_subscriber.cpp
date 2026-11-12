#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVelocitySetpointSubscriber> BusVelocitySetpointSubscriber::create(
    bus::IBus& bus) {
    auto subscriber = bus::make_subscriber<vehicle_api::VelocitySetpoint>(
        bus, protocol::kControlSetpoint, kControlSetpointType,
        protocol::kControlSetpointQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto state = std::make_shared<State>();
    const std::weak_ptr<State> weak = state;
    core::Subscription subscription = subscriber->subscribe(
        [weak](const vehicle_api::VelocitySetpoint& setpoint, const bus::MessageInfo& info) {
            if (const auto locked = weak.lock()) {
                on_message(*locked, setpoint, info);
            }
        });
    return std::unique_ptr<BusVelocitySetpointSubscriber>(new BusVelocitySetpointSubscriber(
        std::move(subscriber), std::move(state), std::move(subscription)));
}

BusVelocitySetpointSubscriber::BusVelocitySetpointSubscriber(std::unique_ptr<Subscriber> subscriber,
                                                             std::shared_ptr<State> state,
                                                             core::Subscription subscription)
    : state_(std::move(state)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

core::Subscription BusVelocitySetpointSubscriber::subscribe(Handler handler) {
    if (!handler) {
        detail::log_error("BusVelocitySetpointSubscriber::subscribe: 빈 handler");
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

std::uint64_t BusVelocitySetpointSubscriber::lost_count() const { return state_->seq.lost_count(); }

void BusVelocitySetpointSubscriber::on_message(State& state,
                                               const vehicle_api::VelocitySetpoint& setpoint,
                                               const bus::MessageInfo& info) {
    state.seq.observe(info.sequence);
    // handler 가 등록을 해제해도 안전하도록 복사해서 부른다
    const auto handlers = state.handlers;
    for (const auto& [id, handler] : handlers) {
        handler(setpoint);
    }
}

}  // namespace argus::vehicle_api_bus
