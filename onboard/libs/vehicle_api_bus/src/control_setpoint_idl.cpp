#include "control_setpoint_idl.hpp"

namespace argus::vehicle_api_bus {

void to_idl_control_setpoint(const vehicle_api::VelocitySetpoint& setpoint,
                             idl::ControlSetpoint& sample) {
    sample.forward_m_s(setpoint.forward_m_s);
    sample.right_m_s(setpoint.right_m_s);
    sample.down_m_s(setpoint.down_m_s);
    sample.yaw_rate_rad_s(setpoint.yaw_rate_rad_s);
    sample.valid_until_monotonic_ns(setpoint.valid_until_monotonic_ns);
}

bool from_idl_control_setpoint(const idl::ControlSetpoint& sample,
                               vehicle_api::VelocitySetpoint& setpoint) {
    setpoint.forward_m_s = sample.forward_m_s();
    setpoint.right_m_s = sample.right_m_s();
    setpoint.down_m_s = sample.down_m_s();
    setpoint.yaw_rate_rad_s = sample.yaw_rate_rad_s();
    setpoint.valid_until_monotonic_ns = sample.valid_until_monotonic_ns();
    return true;
}

}  // namespace argus::vehicle_api_bus
