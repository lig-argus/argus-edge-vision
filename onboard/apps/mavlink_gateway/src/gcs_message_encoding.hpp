// 통제기로 송신하는 COMMAND_ACK, STATUSTEXT, ARGUS_STATUS, ARGUS_SENSOR_STATUS 메시지와
// HEARTBEAT(통제기와 FC 에 송신)의 구성
// 규약: docs/spec/icd.md 2장 (HEARTBEAT), 3.1 (ARGUS_STATUS), 4.1 (COMMAND_ACK, STATUSTEXT),
//      5.1 (sysid 겹침 경고)
// 송신자는 mavlink_gateway 의 명령 compid (sysid 1, compid 195). 모듈 밖에서는 include 하지 않는다
#ifndef ARGUS_ONBOARD_SRC_GCS_MESSAGE_ENCODING_HPP_
#define ARGUS_ONBOARD_SRC_GCS_MESSAGE_ENCODING_HPP_

#include <cstdint>
#include <string>

#include "argus/mavlink.h"

namespace argus::mavlink_gateway::detail {

// target_system, target_component 는 명령을 보낸 쪽 (icd 4.1)
mavlink_message_t encode_command_ack(std::uint16_t command, MAV_RESULT result,
                                     std::uint8_t progress, std::uint8_t target_system,
                                     std::uint8_t target_component);

// text 가 STATUSTEXT 의 길이(50 바이트)를 넘으면 UTF-8 문자 경계에서 자른다
mavlink_message_t encode_statustext(MAV_SEVERITY severity, const std::string& text);

mavlink_message_t encode_argus_status(const mavlink_argus_status_t& status);

// type MAV_TYPE_ONBOARD_CONTROLLER, autopilot MAV_AUTOPILOT_INVALID, base_mode, custom_mode 0
mavlink_message_t encode_heartbeat(MAV_STATE system_status);

mavlink_message_t encode_argus_sensor_status(const mavlink_argus_sensor_status_t& status);

}  // namespace argus::mavlink_gateway::detail

#endif  // ARGUS_ONBOARD_SRC_GCS_MESSAGE_ENCODING_HPP_
