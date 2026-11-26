// command request/result map의 필드·크기·동작 문자열을 정확히 검증한다.
#include "argus/commands/target/include/target_command_codec.hpp"

#include <stdexcept>
#include <unordered_set>

#include "argus/common/serialization/include/message_pack.hpp"

namespace follow {
namespace {
constexpr char kRequestSchema[] = "argus.command.request.v1";
constexpr char kBoundRequestSchema[] = "argus.command.request.v2";
constexpr char kResultSchema[] = "argus.command.result.v1";
constexpr std::size_t kMaxBytes = 4096;
using Object = msgpack::object;
std::string read_string(const Object& value, std::size_t limit) {
    return msgpack_read_string(value, limit, "expected bounded command string");
}
std::uint64_t read_u64(const Object& value) {
    return msgpack_read_u64(value, "expected non-negative command integer");
}
std::int64_t read_i64(const Object& value) {
    return msgpack_read_i64(value, "expected signed command integer", "command integer overflow");
}
bool read_bool(const Object& value) { return msgpack_read_bool(value, "expected result boolean"); }
std::string action_name(TargetCommandAction action) {
    switch (action) {
        case TargetCommandAction::Select:
            return "SELECT";
        case TargetCommandAction::Start:
            return "START";
        case TargetCommandAction::Stop:
            return "STOP";
        case TargetCommandAction::SelectAndStart:
            throw std::invalid_argument("atomic target start requires native vMy request");
    }
    throw std::invalid_argument("unknown target command action");
}
TargetCommandAction parse_action(const std::string& action) {
    if (action == "SELECT") {
        return TargetCommandAction::Select;
    }
    if (action == "START") {
        return TargetCommandAction::Start;
    }
    if (action == "STOP") {
        return TargetCommandAction::Stop;
    }
    throw std::runtime_error("unsupported command action");
}
msgpack::object_handle unpack(std::string_view bytes) {
    if (bytes.empty() || bytes.size() > kMaxBytes) {
        throw std::runtime_error("invalid command message size");
    }
    std::size_t offset = 0;
    auto handle = msgpack::unpack(bytes.data(), bytes.size(), offset);
    if (offset != bytes.size()) {
        throw std::runtime_error("trailing command MessagePack bytes");
    }
    return handle;
}
}  // namespace

std::string TargetCommandCodec::encode_request(const TargetCommand& command) {
    if (command.class_id || command.label || command.sensor) {
        throw std::invalid_argument("class identity requires native vMy request");
    }
    if (command.request_id.empty() || command.request_id.size() > 128 ||
        (command.action == TargetCommandAction::Select && command.track_id <= 0)) {
        throw std::invalid_argument("invalid target command request");
    }
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    const bool bound = !command.expected_uid.empty();
    if (bound &&
        (command.expected_uid.size() != 36 ||
         command.expected_uid.find_first_not_of("0123456789abcdef") != std::string::npos)) {
        throw std::invalid_argument("invalid expected FC uid2");
    }
    packer.pack_map(bound ? 6 : 5);
    msgpack_pack_field(packer, "schema", bound ? kBoundRequestSchema : kRequestSchema);
    msgpack_pack_field(packer, "request_id", command.request_id);
    msgpack_pack_field(packer, "action", action_name(command.action));
    msgpack_pack_field(packer, "generation", command.generation);
    msgpack_pack_field(packer, "track_id", command.track_id);
    if (bound) {
        msgpack_pack_field(packer, "expected_uid", command.expected_uid);
    }
    if (buffer.size() > kMaxBytes) {
        throw std::length_error("command request too large");
    }
    return {buffer.data(), buffer.size()};
}

TargetCommandDecodeResult TargetCommandCodec::decode_request(std::string_view packed) {
    TargetCommandDecodeResult result;
    try {
        const auto handle = unpack(packed);
        const auto& root = handle.get();
        if (root.type != msgpack::type::MAP || (root.via.map.size != 5 && root.via.map.size != 6)) {
            throw std::runtime_error("command request must have 5(v1) or 6(v2) fields");
        }
        std::unordered_set<std::string> seen;
        std::string schema, action;
        for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
            const auto& pair = root.via.map.ptr[i];
            const auto key = read_string(pair.key, 64);
            if (!seen.insert(key).second) {
                throw std::runtime_error("duplicate command field");
            }
            if (key == "schema") {
                schema = read_string(pair.val, 96);
            } else if (key == "request_id") {
                result.command.request_id = read_string(pair.val, 128);
            } else if (key == "action") {
                action = read_string(pair.val, 16);
            } else if (key == "generation") {
                result.command.generation = read_u64(pair.val);
            } else if (key == "track_id") {
                result.command.track_id = read_i64(pair.val);
            } else if (key == "expected_uid") {
                result.command.expected_uid = read_string(pair.val, 36);
            } else {
                throw std::runtime_error("unknown command field");
            }
        }
        if ((schema != kRequestSchema && schema != kBoundRequestSchema) ||
            result.command.request_id.empty()) {
            throw std::runtime_error("unsupported request schema or empty request ID");
        }
        if ((schema == kBoundRequestSchema &&
             (root.via.map.size != 6 || result.command.expected_uid.size() != 36 ||
              result.command.expected_uid.find_first_not_of("0123456789abcdef") !=
                  std::string::npos)) ||
            (schema == kRequestSchema && root.via.map.size != 5)) {
            throw std::runtime_error("invalid request identity/schema binding");
        }
        result.command.action = parse_action(action);
        if (result.command.action == TargetCommandAction::Select && result.command.track_id <= 0) {
            throw std::runtime_error("selection request requires a positive track ID");
        }
        result.valid = true;
    } catch (const std::exception& exception) {
        result.error = exception.what();
    }
    return result;
}

std::string TargetCommandCodec::encode_result(const TargetCommandResult& result) {
    if (result.request_id.size() > 128 || result.status.size() > 32 || result.reason.size() > 128) {
        throw std::invalid_argument("command result string exceeds limit");
    }
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    packer.pack_map(5);
    msgpack_pack_field(packer, "schema", kResultSchema);
    msgpack_pack_field(packer, "request_id", result.request_id);
    msgpack_pack_field(packer, "accepted", result.accepted);
    msgpack_pack_field(packer, "status", result.status);
    msgpack_pack_field(packer, "reason", result.reason);
    if (buffer.size() > kMaxBytes) {
        throw std::length_error("command result too large");
    }
    return {buffer.data(), buffer.size()};
}

bool TargetCommandCodec::decode_result(std::string_view packed, TargetCommandResult& result,
                                       std::string& error) {
    try {
        const auto handle = unpack(packed);
        const auto& root = handle.get();
        if (root.type != msgpack::type::MAP || root.via.map.size != 5) {
            throw std::runtime_error("command result must have 5 fields");
        }
        std::unordered_set<std::string> seen;
        std::string schema;
        for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
            const auto& pair = root.via.map.ptr[i];
            const auto key = read_string(pair.key, 64);
            if (!seen.insert(key).second) {
                throw std::runtime_error("duplicate result field");
            }
            if (key == "schema") {
                schema = read_string(pair.val, 96);
            } else if (key == "request_id") {
                result.request_id = read_string(pair.val, 128);
            } else if (key == "accepted") {
                result.accepted = read_bool(pair.val);
            } else if (key == "status") {
                result.status = read_string(pair.val, 32);
            } else if (key == "reason") {
                result.reason = read_string(pair.val, 128);
            } else {
                throw std::runtime_error("unknown result field");
            }
        }
        if (schema != kResultSchema || result.request_id.empty() || result.status.empty()) {
            throw std::runtime_error("unsupported result schema or missing result value");
        }
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}
}  // namespace follow
