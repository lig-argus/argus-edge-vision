// 속도 setpoint 를 보내는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, 5.1 control/setpoint
// 구현: onboard FC 통신의 ZeroMQ 어댑터(vehicle_api_zmq)
//      테스트용 FakeVelocitySetpoint(vehicle_api_testing)
// 사용: 추종 제어
#ifndef ARGUS_VEHICLE_API_VELOCITY_SETPOINT_HPP_
#define ARGUS_VEHICLE_API_VELOCITY_SETPOINT_HPP_

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

class IVelocitySetpoint {
public:
    IVelocitySetpoint() = default;
    virtual ~IVelocitySetpoint() = default;
    IVelocitySetpoint(const IVelocitySetpoint&) = delete;
    IVelocitySetpoint& operator=(const IVelocitySetpoint&) = delete;
    IVelocitySetpoint(IVelocitySetpoint&&) = delete;
    IVelocitySetpoint& operator=(IVelocitySetpoint&&) = delete;

    // 제어 주기(20 Hz)마다 부른다. 보내기를 기다리지 않고 바로 돌아온다.
    // onboard 는 valid_until_monotonic_ns 가 지난 setpoint 를 쓰지 않는다.
    virtual void send(const VelocitySetpoint& setpoint) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_VELOCITY_SETPOINT_HPP_
