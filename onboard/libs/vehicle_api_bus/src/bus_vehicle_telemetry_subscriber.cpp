#include "argus/vehicle_api_bus/bus_vehicle_telemetry_subscriber.hpp"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::vehicle_api_bus {
namespace {

// 최근 값 목록에 value 를 넣는다. 직전 값과 수신 시각이 같으면 같은 FC 메시지이므로 넣지 않는다
// (fc/state 는 값이 바뀌지 않아도 주기마다 같은 값을 다시 싣는다). 넣은 뒤 value 보다
// kTelemetryMaxAgeNs 넘게 오래된 값을 지운다
template <typename T>
void remember(std::deque<T>& history, const T& value) {
    if (!history.empty() && history.back().received_monotonic_ns == value.received_monotonic_ns) {
        return;
    }
    history.push_back(value);
    while (value.received_monotonic_ns - history.front().received_monotonic_ns >
           vehicle_api::kTelemetryMaxAgeNs) {
        history.pop_front();
    }
}

// 조회 공통 규칙 (argus/vehicle_api/vehicle_telemetry.hpp): now_ns 에서 kTelemetryMaxAgeNs 이내에
// 받은 값 중 수신 시각이 monotonic_ns 에 가장 가까운 값을 고르고, 그 차가
// kTelemetryMatchToleranceNs 를 넘으면 nullopt
template <typename T>
std::optional<T> select_at(const std::deque<T>& history, std::int64_t monotonic_ns,
                           std::int64_t now_ns) {
    const T* best = nullptr;
    std::int64_t best_difference_ns = 0;
    for (const T& value : history) {
        if (now_ns - value.received_monotonic_ns > vehicle_api::kTelemetryMaxAgeNs) {
            continue;
        }
        const std::int64_t difference_ns = std::llabs(value.received_monotonic_ns - monotonic_ns);
        if (best == nullptr || difference_ns < best_difference_ns) {
            best = &value;
            best_difference_ns = difference_ns;
        }
    }
    if (best == nullptr || best_difference_ns > vehicle_api::kTelemetryMatchToleranceNs) {
        return std::nullopt;
    }
    return *best;
}

// 토픽을 구독하고, 메시지가 오면 state 가 살아 있는 동안 handler 를 부르게 한다. 구독 등록은
// subscriptions 에 넣는다. bus 가 구독자를 만들지 못하면 nullptr
template <typename T, typename State>
std::unique_ptr<bus::Subscriber<T>> subscribe_topic(
    bus::IBus& bus, std::string_view topic, const bus::MessageType& type, std::int32_t queue_depth,
    const std::shared_ptr<State>& state, void (*handler)(State&, const T&, const bus::MessageInfo&),
    std::vector<core::Subscription>& subscriptions) {
    auto subscriber = bus::make_subscriber<T>(bus, topic, type, queue_depth);
    if (!subscriber) {
        return nullptr;
    }
    const std::weak_ptr<State> weak = state;
    subscriptions.push_back(
        subscriber->subscribe([weak, handler](const T& message, const bus::MessageInfo& info) {
            if (const auto locked = weak.lock()) {
                handler(*locked, message, info);
            }
        }));
    return subscriber;
}

}  // namespace

std::unique_ptr<BusVehicleTelemetrySubscriber> BusVehicleTelemetrySubscriber::create(
    bus::IBus& bus, const core::IClock& clock) {
    auto state = std::make_shared<State>();
    Subscribers subscribers;
    subscribers.attitude = subscribe_topic(bus, protocol::kFcAttitude, kFcAttitudeType,
                                           protocol::kFcAttitudeQueueDepth, state, &on_attitude,
                                           subscribers.subscriptions);
    subscribers.state =
        subscribe_topic(bus, protocol::kFcState, kFcStateType, protocol::kFcStateQueueDepth, state,
                        &on_state, subscribers.subscriptions);
    subscribers.mode_event = subscribe_topic(bus, protocol::kFcModeEvent, kFcModeEventType,
                                             protocol::kFcModeEventQueueDepth, state,
                                             &on_mode_event, subscribers.subscriptions);
    if (!subscribers.attitude || !subscribers.state || !subscribers.mode_event) {
        return nullptr;
    }
    return std::unique_ptr<BusVehicleTelemetrySubscriber>(
        new BusVehicleTelemetrySubscriber(clock, std::move(state), std::move(subscribers)));
}

BusVehicleTelemetrySubscriber::BusVehicleTelemetrySubscriber(const core::IClock& clock,
                                                             std::shared_ptr<State> state,
                                                             Subscribers subscribers)
    : clock_(clock), state_(std::move(state)), subscribers_(std::move(subscribers)) {}

std::optional<vehicle_api::VehicleState> BusVehicleTelemetrySubscriber::latest() const {
    return state_->latest;
}

std::optional<vehicle_api::Attitude> BusVehicleTelemetrySubscriber::attitude_at(
    std::int64_t monotonic_ns) const {
    return select_at(state_->attitudes, monotonic_ns, clock_.now_monotonic_ns());
}

std::optional<vehicle_api::LocalPosition> BusVehicleTelemetrySubscriber::local_position_at(
    std::int64_t monotonic_ns) const {
    return select_at(state_->locals, monotonic_ns, clock_.now_monotonic_ns());
}

std::optional<vehicle_api::DownwardDistance> BusVehicleTelemetrySubscriber::downward_distance_at(
    std::int64_t monotonic_ns) const {
    return select_at(state_->downward_distances, monotonic_ns, clock_.now_monotonic_ns());
}

core::Subscription BusVehicleTelemetrySubscriber::subscribe_flight_mode_events(
    FlightModeEventHandler handler) {
    if (!handler) {
        detail::log_error(
            "BusVehicleTelemetrySubscriber::subscribe_flight_mode_events: 빈 handler");
        return {};
    }
    const std::uint64_t id = state_->next_id++;
    state_->mode_event_handlers.emplace(id, std::move(handler));
    const std::weak_ptr<State> weak = state_;
    return core::Subscription([weak, id] {
        if (const auto locked = weak.lock()) {
            locked->mode_event_handlers.erase(id);
        }
    });
}

std::uint64_t BusVehicleTelemetrySubscriber::lost_attitude_count() const {
    return state_->attitude_seq.lost_count();
}

std::uint64_t BusVehicleTelemetrySubscriber::lost_state_count() const {
    return state_->state_seq.lost_count();
}

std::uint64_t BusVehicleTelemetrySubscriber::lost_mode_event_count() const {
    return state_->mode_event_seq.lost_count();
}

void BusVehicleTelemetrySubscriber::on_attitude(State& state, const vehicle_api::Attitude& attitude,
                                                const bus::MessageInfo& info) {
    state.attitude_seq.observe(info.sequence);
    remember(state.attitudes, attitude);
    if (!state.latest) {
        state.latest.emplace();
    }
    state.latest->attitude = attitude;
}

void BusVehicleTelemetrySubscriber::on_state(State& state,
                                             const vehicle_api::VehicleState& received,
                                             const bus::MessageInfo& info) {
    state.state_seq.observe(info.sequence);
    if (!state.latest) {
        state.latest.emplace();
    }
    // 메시지에 없는 값(FC 에서 아직 받지 못함)은 직전 값을 유지한다. fc/state 는 자세를 나르지
    // 않으므로 received.attitude 는 읽지 않는다
    if (received.local) {
        remember(state.locals, *received.local);
        state.latest->local = received.local;
    }
    if (received.downward_distance) {
        remember(state.downward_distances, *received.downward_distance);
        state.latest->downward_distance = received.downward_distance;
    }
    if (received.global) {
        state.latest->global = received.global;
    }
    if (received.status) {
        state.latest->status = received.status;
    }
}

void BusVehicleTelemetrySubscriber::on_mode_event(State& state,
                                                  const vehicle_api::FlightModeEvent& event,
                                                  const bus::MessageInfo& info) {
    state.mode_event_seq.observe(info.sequence);
    // handler 가 등록을 해제해도 안전하도록 복사해서 부른다
    const auto handlers = state.mode_event_handlers;
    for (const auto& [id, handler] : handlers) {
        handler(event);
    }
}

}  // namespace argus::vehicle_api_bus
