#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"

#include <optional>
#include <utility>

#include "argus/protocol/topics.hpp"
#include "control_setpoint_codec.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {

std::unique_ptr<BusVelocitySetpointSubscriber> BusVelocitySetpointSubscriber::create(
    bus::IBus& bus) {
    auto subscriber =
        bus.create_subscriber(protocol::kControlSetpoint, protocol::kControlSetpointQueueDepth);
    if (!subscriber) {
        return nullptr;
    }
    auto state = std::make_shared<State>();
    const std::weak_ptr<State> weak = state;
    core::Subscription subscription = subscriber->subscribe([weak](const bus::BusMessage& message) {
        if (const auto locked = weak.lock()) {
            on_message(*locked, message);
        }
    });
    return std::unique_ptr<BusVelocitySetpointSubscriber>(new BusVelocitySetpointSubscriber(
        std::move(subscriber), std::move(state), std::move(subscription)));
}

BusVelocitySetpointSubscriber::BusVelocitySetpointSubscriber(
    std::unique_ptr<bus::ITopicSubscriber> subscriber, std::shared_ptr<State> state,
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

void BusVelocitySetpointSubscriber::on_message(State& state, const bus::BusMessage& message) {
    // control/setpoint 에는 바이너리가 없다
    const std::optional<Decoded<vehicle_api::VelocitySetpoint>> decoded =
        message.binary.empty() ? decode_control_setpoint(message.body) : std::nullopt;
    if (!decoded) {
        detail::log_error("control/setpoint: 형식이 맞지 않는 메시지 폐기");
        return;
    }
    state.seq.observe(decoded->seq);
    // handler 가 등록을 해제해도 안전하도록 복사해서 부른다
    const auto handlers = state.handlers;
    for (const auto& [id, handler] : handlers) {
        handler(decoded->value);
    }
}

}  // namespace argus::vehicle_api_bus
