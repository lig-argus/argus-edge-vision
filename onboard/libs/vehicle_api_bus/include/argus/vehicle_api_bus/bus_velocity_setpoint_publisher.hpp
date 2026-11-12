// 속도 setpoint 를 control/setpoint 토픽으로 발행하는 IVelocitySetpointSink 구현
// 규약: docs/spec/process_interfaces.md 5.1 control/setpoint
// 사용: 추종 제어 프로세스의 main() 이 만들어 FollowController 에 IVelocitySetpointSink 로 넘긴다
// 구독 클래스: BusVelocitySetpointSubscriber
#ifndef ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_PUBLISHER_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_PUBLISHER_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/velocity_setpoint_sink.hpp"

namespace argus::vehicle_api_bus {

class BusVelocitySetpointPublisher : public vehicle_api::IVelocitySetpointSink {
public:
    // bus 가 발행자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusVelocitySetpointPublisher> create(bus::IBus& bus);

    // 호출마다 한 번 발행한다 (발행 번호는 bus 가 붙인다). 발행에 실패해도 기다리거나 다시 보내지
    // 않는다. 구독하는 mavlink_gateway 는 valid_until_monotonic_ns 가 지나도록 새 값이 없으면 안전
    // 동작을 한다
    // (process_interfaces.md 6장). 실패가 시작될 때와 복구될 때 한 번씩 로그에 남긴다
    void send(const vehicle_api::VelocitySetpoint& setpoint) override;

private:
    using Publisher = bus::Publisher<vehicle_api::VelocitySetpoint>;

    explicit BusVelocitySetpointPublisher(std::unique_ptr<Publisher> publisher);

    std::unique_ptr<Publisher> publisher_;
    bool publish_failing_ = false;
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_PUBLISHER_HPP_
