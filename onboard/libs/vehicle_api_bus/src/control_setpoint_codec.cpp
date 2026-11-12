#include "control_setpoint_codec.hpp"

#include "argus_msg/control_setpoint.pb.h"

namespace argus::vehicle_api_bus {

bus::Bytes encode_control_setpoint(const vehicle_api::VelocitySetpoint& setpoint,
                                   std::uint64_t seq) {
    msg::ControlSetpoint message;
    message.set_seq(seq);
    message.set_forward_m_s(setpoint.forward_m_s);
    message.set_right_m_s(setpoint.right_m_s);
    message.set_down_m_s(setpoint.down_m_s);
    message.set_yaw_rate_rad_s(setpoint.yaw_rate_rad_s);
    message.set_valid_until_monotonic_ns(setpoint.valid_until_monotonic_ns);
    return to_bytes(message);
}

std::optional<Decoded<vehicle_api::VelocitySetpoint>> decode_control_setpoint(
    const bus::Bytes& body) {
    msg::ControlSetpoint message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    Decoded<vehicle_api::VelocitySetpoint> decoded;
    decoded.seq = message.seq();
    decoded.value.forward_m_s = message.forward_m_s();
    decoded.value.right_m_s = message.right_m_s();
    decoded.value.down_m_s = message.down_m_s();
    decoded.value.yaw_rate_rad_s = message.yaw_rate_rad_s();
    decoded.value.valid_until_monotonic_ns = message.valid_until_monotonic_ns();
    return decoded;
}

}  // namespace argus::vehicle_api_bus
