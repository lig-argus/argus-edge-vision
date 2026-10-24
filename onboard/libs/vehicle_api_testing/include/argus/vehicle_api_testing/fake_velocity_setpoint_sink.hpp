// 테스트용 IVelocitySetpointSink. 받은 setpoint 를 순서대로 기록한다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_VELOCITY_SETPOINT_SINK_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_VELOCITY_SETPOINT_SINK_HPP_

#include <vector>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/velocity_setpoint_sink.hpp"

namespace argus::vehicle_api_testing {

class FakeVelocitySetpointSink : public vehicle_api::IVelocitySetpointSink {
public:
    void send(const vehicle_api::VelocitySetpoint& setpoint) override { sent_.push_back(setpoint); }

    const std::vector<vehicle_api::VelocitySetpoint>& sent() const { return sent_; }

private:
    std::vector<vehicle_api::VelocitySetpoint> sent_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_VELOCITY_SETPOINT_SINK_HPP_
