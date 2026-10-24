// 속도 setpoint 를 받는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, 5.1 control/setpoint
// 구현: (다음 PR) 추종 제어 프로세스의 ZmqVelocitySetpointSink (control/setpoint 발행),
//      onboard 의 MavlinkVelocitySetpointSink (FC 로 SET_POSITION_TARGET_LOCAL_NED 송신),
//      테스트용 FakeVelocitySetpointSink (vehicle_api_testing)
// 사용: 추종 제어(FollowController), onboard 의 ZmqVelocitySetpointReceiver
#ifndef ARGUS_VEHICLE_API_VELOCITY_SETPOINT_SINK_HPP_
#define ARGUS_VEHICLE_API_VELOCITY_SETPOINT_SINK_HPP_

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

class IVelocitySetpointSink {
public:
    IVelocitySetpointSink() = default;
    virtual ~IVelocitySetpointSink() = default;
    IVelocitySetpointSink(const IVelocitySetpointSink&) = delete;
    IVelocitySetpointSink& operator=(const IVelocitySetpointSink&) = delete;
    IVelocitySetpointSink(IVelocitySetpointSink&&) = delete;
    IVelocitySetpointSink& operator=(IVelocitySetpointSink&&) = delete;

    // 제어 주기(20 Hz)마다 부른다. 보내기를 기다리지 않고 바로 돌아온다.
    // onboard 는 valid_until_monotonic_ns 가 지난 setpoint 를 쓰지 않는다.
    virtual void send(const VelocitySetpoint& setpoint) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_VELOCITY_SETPOINT_SINK_HPP_
