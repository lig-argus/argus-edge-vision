#include "gcs_message_encoding.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "argus/mavlink_gateway/mavlink_ids.hpp"

namespace argus::mavlink_gateway::detail {
namespace {

constexpr std::size_t kStatustextLength = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;

// UTF-8 의 이어지는 바이트(10xxxxxx)인지
bool is_continuation_byte(char byte) { return (static_cast<std::uint8_t>(byte) & 0xC0U) == 0x80U; }

// text 를 limit 바이트 이하로, 문자 중간을 자르지 않게 줄인 길이
std::size_t utf8_prefix_length(const std::string& text, std::size_t limit) {
    if (text.size() <= limit) {
        return text.size();
    }
    std::size_t length = limit;
    while (length > 0 && is_continuation_byte(text[length])) {
        --length;
    }
    return length;
}

}  // namespace

mavlink_message_t encode_command_ack(std::uint16_t command, MAV_RESULT result,
                                     std::uint8_t progress, std::uint8_t target_system,
                                     std::uint8_t target_component) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack_chan(kVehicleSysid, kOnboardCommandCompid, kCommandChannel,
                                      &message, command, static_cast<std::uint8_t>(result),
                                      progress, 0, target_system, target_component);
    return message;
}

mavlink_message_t encode_statustext(MAV_SEVERITY severity, const std::string& text) {
    // 50 바이트를 모두 채우면 끝의 0 이 없어도 된다 (MAVLink STATUSTEXT 규칙)
    std::array<char, kStatustextLength + 1> buffer{};
    const std::size_t length = utf8_prefix_length(text, kStatustextLength);
    text.copy(buffer.data(), length);
    mavlink_message_t message{};
    mavlink_msg_statustext_pack_chan(kVehicleSysid, kOnboardCommandCompid, kCommandChannel,
                                     &message, static_cast<std::uint8_t>(severity), buffer.data(),
                                     0, 0);
    return message;
}

mavlink_message_t encode_argus_status(const mavlink_argus_status_t& status) {
    mavlink_message_t message{};
    mavlink_msg_argus_status_encode_chan(kVehicleSysid, kOnboardCommandCompid, kCommandChannel,
                                         &message, &status);
    return message;
}

mavlink_message_t encode_argus_sensor_status(const mavlink_argus_sensor_status_t& status) {
    mavlink_message_t message{};
    mavlink_msg_argus_sensor_status_encode_chan(kVehicleSysid, kOnboardCommandCompid,
                                                kCommandChannel, &message, &status);
    return message;
}

}  // namespace argus::mavlink_gateway::detail
