#include "argus/vehicle_api_bus/dds_types.hpp"

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "argus_msg/control_mode_requestPubSubTypes.hpp"
#include "argus_msg/control_setpointPubSubTypes.hpp"
#include "argus_msg/fc_attitudePubSubTypes.hpp"
#include "argus_msg/fc_mode_eventPubSubTypes.hpp"
#include "argus_msg/fc_statePubSubTypes.hpp"
#include "argus_msg/follow_commandPubSubTypes.hpp"
#include "argus_msg/follow_statusPubSubTypes.hpp"
#include "control_mode_request_idl.hpp"
#include "control_setpoint_idl.hpp"
#include "fc_telemetry_idl.hpp"
#include "follow_command_idl.hpp"
#include "follow_status_idl.hpp"

namespace argus::vehicle_api_bus {
namespace {

// C++ 값 타입 T 를 IDL 타입 PubSubType::type 에 대응시킨다. to_idl 은 항상 성공한다
template <typename T, typename PubSubType>
void add_type(bus_dds::DdsTypeTable& table, void (*to_idl)(const T&, typename PubSubType::type&),
              bool (*from_idl)(const typename PubSubType::type&, T&)) {
    bus_dds::add_dds_type<T, PubSubType>(
        table,
        [to_idl](const T& value, typename PubSubType::type& sample) {
            to_idl(value, sample);
            return true;
        },
        from_idl);
}

}  // namespace

bus_dds::DdsTypeTable dds_types() {
    bus_dds::DdsTypeTable table;
    add_type<vehicle_api::Attitude, idl::FcAttitudePubSubType>(table, &to_idl_fc_attitude,
                                                               &from_idl_fc_attitude);
    add_type<vehicle_api::VehicleState, idl::FcStatePubSubType>(table, &to_idl_fc_state,
                                                                &from_idl_fc_state);
    add_type<vehicle_api::FlightModeEvent, idl::FcModeEventPubSubType>(table, &to_idl_fc_mode_event,
                                                                       &from_idl_fc_mode_event);
    add_type<vehicle_api::VelocitySetpoint, idl::ControlSetpointPubSubType>(
        table, &to_idl_control_setpoint, &from_idl_control_setpoint);
    add_type<vehicle_api::FollowStatus, idl::FollowStatusPubSubType>(table, &to_idl_follow_status,
                                                                     &from_idl_follow_status);
    add_type<FlightModeRequest, idl::ControlModeRequestPubSubType>(
        table, &to_idl_flight_mode_request, &from_idl_flight_mode_request);
    add_type<vehicle_api::FlightModeReply, idl::ControlModeReplyPubSubType>(
        table, &to_idl_flight_mode_reply, &from_idl_flight_mode_reply);
    add_type<FollowCommandRequest, idl::FollowCommandPubSubType>(table, &to_idl_follow_command,
                                                                 &from_idl_follow_command);
    add_type<vehicle_api::FollowResult, idl::FollowResultPubSubType>(table, &to_idl_follow_result,
                                                                     &from_idl_follow_result);
    return table;
}

}  // namespace argus::vehicle_api_bus
