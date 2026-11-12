// fc/attitude, fc/state, fc/mode_event 토픽을 구독하는 IVehicleTelemetry 구현
// 규약: docs/spec/process_interfaces.md 5.1, 조회 규칙은 argus/vehicle_api/vehicle_telemetry.hpp
// 사용: 추종 제어 프로세스와 추적기의 main() 이 만들어 IVehicleTelemetry 로 넘긴다
// 발행 클래스: BusVehicleTelemetryPublisher
#ifndef ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_SUBSCRIBER_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_SUBSCRIBER_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "argus/bus/bus.hpp"
#include "argus/bus/message.hpp"
#include "argus/bus/topic_subscriber.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/vehicle_telemetry.hpp"
#include "argus/vehicle_api_bus/seq_tracker.hpp"

namespace argus::vehicle_api_bus {

class BusVehicleTelemetrySubscriber : public vehicle_api::IVehicleTelemetry {
public:
    // clock 은 조회 시점을 얻는 데 쓴다. 이 객체보다 오래 살아야 한다.
    // bus 가 구독자 3개 중 하나라도 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에
    // 남긴다)
    static std::unique_ptr<BusVehicleTelemetrySubscriber> create(bus::IBus& bus,
                                                                 const core::IClock& clock);

    std::optional<vehicle_api::VehicleState> latest() const override;
    std::optional<vehicle_api::Attitude> attitude_at(std::int64_t monotonic_ns) const override;
    std::optional<vehicle_api::LocalPosition> local_position_at(
        std::int64_t monotonic_ns) const override;
    std::optional<vehicle_api::DownwardDistance> downward_distance_at(
        std::int64_t monotonic_ns) const override;
    [[nodiscard]] core::Subscription subscribe_flight_mode_events(
        FlightModeEventHandler handler) override;

    // 토픽마다 처음 받은 메시지 이후에 손실된 메시지 수 (seq 로 센다). 발행자가 다시 시작해 seq 가
    // 작아진 경우는 세지 않는다
    std::uint64_t lost_attitude_count() const;
    std::uint64_t lost_state_count() const;
    std::uint64_t lost_mode_event_count() const;

private:
    // Subscription 이 이 객체보다 오래 살아도 안전하도록 받은 값과 handler 목록은 공유 객체에 두고,
    // Subscription 과 bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    struct State {
        // 조회(*_at)에 쓰는 최근 값. 받은 순서. 가장 최근 값보다 kTelemetryMaxAgeNs 넘게 오래된
        // 값은 지운다
        std::deque<vehicle_api::Attitude> attitudes;
        std::deque<vehicle_api::LocalPosition> locals;
        std::deque<vehicle_api::DownwardDistance> downward_distances;
        std::optional<vehicle_api::VehicleState> latest;
        std::map<std::uint64_t, FlightModeEventHandler> mode_event_handlers;  // 등록한 순서
        std::uint64_t next_id = 1;
        SeqTracker attitude_seq;
        SeqTracker state_seq;
        SeqTracker mode_event_seq;
    };

    BusVehicleTelemetrySubscriber(const core::IClock& clock, std::shared_ptr<State> state,
                                  std::vector<std::unique_ptr<bus::ITopicSubscriber>> subscribers,
                                  std::vector<core::Subscription> subscriptions);

    static void on_attitude(State& state, const bus::BusMessage& message);
    static void on_state(State& state, const bus::BusMessage& message);
    static void on_mode_event(State& state, const bus::BusMessage& message);

    const core::IClock& clock_;
    std::shared_ptr<State> state_;
    std::vector<std::unique_ptr<bus::ITopicSubscriber>> subscribers_;
    std::vector<core::Subscription> subscriptions_;  // bus 구독. 이 객체가 사라지면 해제된다
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_SUBSCRIBER_HPP_
