// 통제기 쪽 테스트가 함께 사용하는 MAVLink 메시지 생성과 해석
#ifndef ARGUS_ONBOARD_TEST_GCS_TEST_MESSAGES_HPP_
#define ARGUS_ONBOARD_TEST_GCS_TEST_MESSAGES_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"

namespace argus::mavlink_gateway::test_messages {

inline constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// sysid 통제기(compid 190)가 address:port 에서 보낸 것으로 하는 송신자
inline GcsPeer gcs_peer(std::uint8_t sysid, const std::string& address = "192.168.0.10",
                        std::uint16_t port = 14550, std::uint8_t compid = kGcsCompid) {
    return GcsPeer{sysid, compid, address, port};
}

inline mavlink_message_t heartbeat_from(const GcsPeer& peer, MAV_TYPE type = MAV_TYPE_GCS) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(peer.sysid, peer.compid, &message, type, MAV_AUTOPILOT_INVALID, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

// mavlink_gateway(1, 195)에 보낸 COMMAND_LONG. param7 은 명령 번호
inline mavlink_message_t command_long_from(const GcsPeer& peer, std::uint16_t command, float param7,
                                           float param1 = kNan, float param4 = kNan,
                                           std::uint8_t target_system = kVehicleSysid,
                                           std::uint8_t target_component = kOnboardCommandCompid) {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(peer.sysid, peer.compid, &message, target_system,
                                  target_component, command, 0, param1, kNan, kNan, param4, kNan,
                                  kNan, param7);
    return message;
}

inline mavlink_message_t command_int_from(const GcsPeer& peer, std::uint16_t command) {
    mavlink_message_t message{};
    mavlink_msg_command_int_pack(peer.sysid, peer.compid, &message, kVehicleSysid,
                                 kOnboardCommandCompid, MAV_FRAME_GLOBAL, command, 0, 0, kNan, kNan,
                                 kNan, kNan, 0, 0, kNan);
    return message;
}

inline mavlink_command_ack_t ack_of(const mavlink_message_t& message) {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    return ack;
}

inline std::string statustext_of(const mavlink_message_t& message) {
    mavlink_statustext_t text{};
    mavlink_msg_statustext_decode(&message, &text);
    return {text.text, strnlen(text.text, sizeof(text.text))};
}

}  // namespace argus::mavlink_gateway::test_messages

#endif  // ARGUS_ONBOARD_TEST_GCS_TEST_MESSAGES_HPP_
