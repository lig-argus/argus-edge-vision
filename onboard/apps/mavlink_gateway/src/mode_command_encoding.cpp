#include "mode_command_encoding.hpp"

#include <optional>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/mavlink_gateway/px4_flight_mode.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway::detail {

std::optional<mavlink_message_t> encode_offboard_command() {
    const std::optional<Px4ModeParams> params = px4_mode_params(vehicle_api::FlightMode::Offboard);
    if (!params.has_value()) {
        return std::nullopt;
    }
    mavlink_message_t message{};
    mavlink_msg_command_long_pack_chan(kVehicleSysid, kOnboardCommandCompid, kCommandChannel,
                                       &message, kVehicleSysid, kFcCompid, MAV_CMD_DO_SET_MODE, 0,
                                       static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                       params->main_mode, params->sub_mode, 0.0F, 0.0F, 0.0F, 0.0F);
    return message;
}

std::optional<bool> decode_mode_command_ack(const mavlink_message_t& message) {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    if (ack.command != MAV_CMD_DO_SET_MODE) {
        return std::nullopt;
    }
    if (ack.target_component != 0 && ack.target_component != kOnboardCommandCompid) {
        return std::nullopt;
    }
    if (ack.result == MAV_RESULT_IN_PROGRESS) {
        return std::nullopt;
    }
    return ack.result == MAV_RESULT_ACCEPTED;
}

}  // namespace argus::mavlink_gateway::detail
