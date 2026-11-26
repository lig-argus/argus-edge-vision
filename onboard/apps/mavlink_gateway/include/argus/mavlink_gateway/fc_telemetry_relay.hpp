// FC 의 telemetry 를 접속한 모든 통제기에 전달
// 규약: docs/spec/icd.md 1.2, 2장 (FC → 통제기 메시지),
//      docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성한다
#ifndef ARGUS_ONBOARD_FC_TELEMETRY_RELAY_HPP_
#define ARGUS_ONBOARD_FC_TELEMETRY_RELAY_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"

namespace argus::mavlink_gateway {

// 통제기에 전달하는 FC 메시지 (icd 2장). 그 밖의 메시지는 Wi-Fi 로 보내지 않는다
inline constexpr std::array<std::uint32_t, 15> kRelayedFcMessages = {
    MAVLINK_MSG_ID_HEARTBEAT,          MAVLINK_MSG_ID_SYS_STATUS,
    MAVLINK_MSG_ID_GPS_RAW_INT,        MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_LOCAL_POSITION_NED, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_DISTANCE_SENSOR,    MAVLINK_MSG_ID_VFR_HUD,
    MAVLINK_MSG_ID_BATTERY_STATUS,     MAVLINK_MSG_ID_STATUSTEXT,
    MAVLINK_MSG_ID_EXTENDED_SYS_STATE, MAVLINK_MSG_ID_ESTIMATOR_STATUS,
    MAVLINK_MSG_ID_RC_CHANNELS,        MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    MAVLINK_MSG_ID_PARAM_VALUE,
};

// FC(sysid 1, compid 1)가 보낸 kRelayedFcMessages 의 메시지를 받은 그대로 broadcast 한다. 주기는
// 줄이지 않는다
class FcTelemetryRelay {
public:
    // connection, link 는 이 객체보다 오래 유지되어야 한다
    FcTelemetryRelay(IMavlinkConnection& connection, IGcsLink& link);

private:
    std::vector<core::Subscription> subscriptions_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_TELEMETRY_RELAY_HPP_
