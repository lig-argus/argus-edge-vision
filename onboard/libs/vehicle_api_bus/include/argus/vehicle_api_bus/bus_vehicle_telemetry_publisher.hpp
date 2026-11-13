// 기체 상태를 fc/attitude, fc/state, fc/mode_event 토픽으로 발행한다
// 규약: docs/spec/process_interfaces.md 5.1
// 사용: onboard 프로세스. FC 메시지를 값 타입으로 바꾸는 일과 발행 주기는 사용하는 코드가 맡는다
// 구독 클래스: BusVehicleTelemetrySubscriber
#ifndef ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_PUBLISHER_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_PUBLISHER_HPP_

#include <cstdint>
#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/topic_publisher.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_bus {

class BusVehicleTelemetryPublisher {
public:
    // bus 가 발행자 3개 중 하나라도 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에
    // 남긴다)
    static std::unique_ptr<BusVehicleTelemetryPublisher> create(bus::IBus& bus);

    // 아래 함수 3개는 토픽마다 seq 를 1 증가시켜 발행하고, 발행했으면 true 를 반환한다. 실패해도
    // 기다리거나 다시 보내지 않는다. 실패가 시작될 때와 복구될 때 한 번씩 로그에 남긴다

    // fc/attitude. received_monotonic_ns 는 FC 메시지를 처음 받은 시각이며 덮어쓰지 않는다
    bool publish_attitude(const vehicle_api::Attitude& attitude);

    // fc/state. state 의 local, downward_distance, global, status 를 보낸다. attitude 는 보내지
    // 않는다 (자세는 publish_attitude 로 보낸다)
    bool publish_state(const vehicle_api::VehicleState& state);

    // fc/mode_event. 실제 비행 모드가 바뀔 때마다 한 번 부른다
    bool publish_mode_event(const vehicle_api::FlightModeEvent& event);

private:
    // 토픽 하나의 발행자와 그 토픽의 seq, 직전 발행 결과
    struct Topic {
        std::unique_ptr<bus::ITopicPublisher> publisher;
        std::uint64_t next_seq = 1;
        bool failing = false;
    };

    BusVehicleTelemetryPublisher(Topic attitude, Topic state, Topic mode_event);

    static bool publish(Topic& topic, const bus::Bytes& body);

    Topic attitude_;
    Topic state_;
    Topic mode_event_;
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_VEHICLE_TELEMETRY_PUBLISHER_HPP_
