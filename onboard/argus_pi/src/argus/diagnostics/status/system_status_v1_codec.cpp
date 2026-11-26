// system/status의 필드 수·형식·상태를 고정하고 알 수 없는 필드는 거부한다.
#include "argus/diagnostics/status/include/system_status_v1_codec.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include "argus/common/serialization/include/message_pack.hpp"

namespace follow {
namespace {
using Object = msgpack::object;
std::string text(const Object& value, std::size_t limit) {
    return msgpack_read_string(value, limit, "invalid status string");
}
std::uint64_t u64(const Object& value) {
    return msgpack_read_u64(value, "invalid status unsigned integer");
}
std::int64_t i64(const Object& value) {
    return msgpack_read_i64(value, "invalid status integer", "status integer overflow");
}
bool boolean(const Object& value) { return msgpack_read_bool(value, "invalid status boolean"); }
double number(const Object& value) {
    return msgpack_read_finite_number(value, "invalid status number", "non-finite status number");
}
std::optional<std::uint64_t> optional_u64(const Object& value) {
    if (value.type == msgpack::type::NIL) {
        return std::nullopt;
    }
    return u64(value);
}
std::optional<std::int64_t> optional_i64(const Object& value) {
    if (value.type == msgpack::type::NIL) {
        return std::nullopt;
    }
    return i64(value);
}
void validate(const SystemStatusV1& m) {
    const bool key_valid = m.target_generation.has_value() == m.target_track_id.has_value() &&
                           (!m.target_track_id || *m.target_track_id > 0);
    const bool invalid_envelope =
        (m.schema != kSystemStatusV1Schema && m.schema != kSystemStatusV2Schema) ||
        m.source.empty() || m.monotonic_ns <= 0;
    const bool invalid_mission =
        m.mission.empty() || m.mission.size() > 32 || m.reason.size() > 128;
    const bool invalid_target = !key_valid || m.target_state.empty() ||
                                m.target_state.size() > 16 || !std::isfinite(m.age_since_real_s) ||
                                m.age_since_real_s < 0;
    const bool invalid_identity =
        m.schema == kSystemStatusV2Schema && m.identity_verified &&
        (m.fc_uid.size() != 36 ||
         m.fc_uid.find_first_not_of("0123456789abcdef") != std::string::npos);
    if (invalid_envelope || invalid_mission || invalid_target || invalid_identity) {
        throw std::invalid_argument("invalid system/status v1 message");
    }
}
}  // namespace

std::string SystemStatusV1Codec::encode(const SystemStatusV1& m) {
    validate(m);
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    p.pack_map(m.schema == kSystemStatusV2Schema ? 20 : 18);
    msgpack_pack_field(p, "schema", m.schema);
    msgpack_pack_field(p, "source", m.source);
    msgpack_pack_field(p, "monotonic_ns", m.monotonic_ns);
    msgpack_pack_field(p, "mission", m.mission);
    msgpack_pack_field(p, "reason", m.reason);
    msgpack_pack_field(p, "command_epoch", m.command_epoch);
    msgpack_pack_optional(p, "target_generation", m.target_generation);
    msgpack_pack_optional(p, "target_track_id", m.target_track_id);
    msgpack_pack_field(p, "target_state", m.target_state);
    msgpack_pack_field(p, "age_since_real_s", m.age_since_real_s);
    msgpack_pack_field(p, "connected", m.connected);
    msgpack_pack_field(p, "armed", m.armed);
    msgpack_pack_field(p, "in_air", m.in_air);
    msgpack_pack_field(p, "offboard", m.offboard);
    msgpack_pack_field(p, "hold", m.hold);
    msgpack_pack_field(p, "command_sent", m.command_sent);
    msgpack_pack_field(p, "offboard_start_requested", m.offboard_start_requested);
    msgpack_pack_field(p, "offboard_stop_requested", m.offboard_stop_requested);
    if (m.schema == kSystemStatusV2Schema) {
        msgpack_pack_field(p, "fc_uid", m.fc_uid);
        msgpack_pack_field(p, "identity_verified", m.identity_verified);
    }
    if (buffer.size() > kMaxMessageBytes) {
        throw std::length_error("system/status message too large");
    }
    return {buffer.data(), buffer.size()};
}

SystemStatusDecodeResult SystemStatusV1Codec::decode(std::string_view packed) {
    SystemStatusDecodeResult result;
    try {
        if (packed.empty() || packed.size() > kMaxMessageBytes) {
            throw std::runtime_error("invalid status size");
        }
        std::size_t offset = 0;
        const auto handle = msgpack::unpack(packed.data(), packed.size(), offset);
        if (offset != packed.size()) {
            throw std::runtime_error("trailing status bytes");
        }
        const auto& root = handle.get();
        if (root.type != msgpack::type::MAP ||
            (root.via.map.size != 18 && root.via.map.size != 20)) {
            throw std::runtime_error("status map must have 18(v1) or 20(v2) fields");
        }
        std::unordered_set<std::string> seen;
        for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
            const auto& pair = root.via.map.ptr[i];
            const auto key = text(pair.key, 64);
            if (!seen.insert(key).second) {
                throw std::runtime_error("duplicate status field");
            }
            const auto& v = pair.val;
            auto& m = result.message;
            if (key == "schema") {
                m.schema = text(v, 64);
            } else if (key == "source") {
                m.source = text(v, 128);
            } else if (key == "monotonic_ns") {
                m.monotonic_ns = i64(v);
            } else if (key == "mission") {
                m.mission = text(v, 32);
            } else if (key == "reason") {
                m.reason = text(v, 128);
            } else if (key == "command_epoch") {
                m.command_epoch = u64(v);
            } else if (key == "target_generation") {
                m.target_generation = optional_u64(v);
            } else if (key == "target_track_id") {
                m.target_track_id = optional_i64(v);
            } else if (key == "target_state") {
                m.target_state = text(v, 16);
            } else if (key == "age_since_real_s") {
                m.age_since_real_s = number(v);
            } else if (key == "connected") {
                m.connected = boolean(v);
            } else if (key == "armed") {
                m.armed = boolean(v);
            } else if (key == "in_air") {
                m.in_air = boolean(v);
            } else if (key == "offboard") {
                m.offboard = boolean(v);
            } else if (key == "hold") {
                m.hold = boolean(v);
            } else if (key == "command_sent") {
                m.command_sent = boolean(v);
            } else if (key == "offboard_start_requested") {
                m.offboard_start_requested = boolean(v);
            } else if (key == "offboard_stop_requested") {
                m.offboard_stop_requested = boolean(v);
            } else if (key == "fc_uid") {
                m.fc_uid = text(v, 36);
            } else if (key == "identity_verified") {
                m.identity_verified = boolean(v);
            } else {
                throw std::runtime_error("unknown status field");
            }
        }
        if (root.via.map.size != (result.message.schema == kSystemStatusV2Schema ? 20U : 18U)) {
            throw std::runtime_error("status schema/field count mismatch");
        }
        validate(result.message);
        result.valid = true;
    } catch (const std::exception& error) {
        result.error = error.what();
    }
    return result;
}
}  // namespace follow
