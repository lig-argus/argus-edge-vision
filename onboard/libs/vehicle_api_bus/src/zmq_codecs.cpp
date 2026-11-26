#include "argus/vehicle_api_bus/zmq_codecs.hpp"

#include <cstdint>
#include <optional>
#include <utility>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/message_types.hpp"
#include "codec_common.hpp"
#include "control_mode_request_codec.hpp"
#include "control_setpoint_codec.hpp"
#include "fc_telemetry_codec.hpp"
#include "follow_command_codec.hpp"
#include "follow_status_codec.hpp"

namespace argus::vehicle_api_bus {
namespace {

using bus_zmq::Bytes;

// 토픽 메시지. 발행 번호를 본문의 seq 에 싣는다. 바이너리가 있는 메시지는 형식이 맞지 않는 것이다
template <typename T>
void add_topic_codec(bus_zmq::ZmqCodecTable& table, Bytes (*encode)(const T&, std::uint64_t),
                     std::optional<Decoded<T>> (*decode)(const Bytes&)) {
    bus_zmq::add_zmq_codec<T>(
        table,
        [encode](const T& value, std::uint64_t sequence, Bytes& body, Bytes& /*binary*/) {
            body = encode(value, sequence);
            return true;
        },
        [decode](const Bytes& body, const Bytes& binary, T& value, std::uint64_t& sequence) {
            std::optional<Decoded<T>> decoded = binary.empty() ? decode(body) : std::nullopt;
            if (!decoded) {
                return false;
            }
            value = std::move(decoded->value);
            sequence = decoded->seq;
            return true;
        });
}

// 요청과 응답 메시지. 발행 번호를 싣지 않는다. 바이너리가 있는 메시지는 형식이 맞지 않는 것이다
template <typename T>
void add_request_reply_codec(bus_zmq::ZmqCodecTable& table, Bytes (*encode)(const T&),
                             std::optional<T> (*decode)(const Bytes&)) {
    bus_zmq::add_zmq_codec<T>(
        table,
        [encode](const T& value, std::uint64_t /*sequence*/, Bytes& body, Bytes& /*binary*/) {
            body = encode(value);
            return true;
        },
        [decode](const Bytes& body, const Bytes& binary, T& value, std::uint64_t& /*sequence*/) {
            std::optional<T> decoded = binary.empty() ? decode(body) : std::nullopt;
            if (!decoded) {
                return false;
            }
            value = std::move(*decoded);
            return true;
        });
}

}  // namespace

bus_zmq::ZmqCodecTable zmq_codecs() {
    bus_zmq::ZmqCodecTable table;
    add_topic_codec<vehicle_api::Attitude>(table, &encode_fc_attitude, &decode_fc_attitude);
    add_topic_codec<vehicle_api::VehicleState>(table, &encode_fc_state, &decode_fc_state);
    add_topic_codec<vehicle_api::FlightModeEvent>(table, &encode_fc_mode_event,
                                                  &decode_fc_mode_event);
    add_topic_codec<vehicle_api::VelocitySetpoint>(table, &encode_control_setpoint,
                                                   &decode_control_setpoint);
    add_topic_codec<vehicle_api::FollowStatus>(table, &encode_follow_status, &decode_follow_status);
    add_request_reply_codec<FlightModeRequest>(table, &encode_flight_mode_request,
                                               &decode_flight_mode_request);
    add_request_reply_codec<vehicle_api::FlightModeReply>(table, &encode_flight_mode_reply,
                                                          &decode_flight_mode_reply);
    add_request_reply_codec<FollowCommandRequest>(table, &encode_follow_command,
                                                  &decode_follow_command);
    add_request_reply_codec<vehicle_api::FollowResult>(table, &encode_follow_result,
                                                       &decode_follow_result);
    return table;
}

}  // namespace argus::vehicle_api_bus
