// GatewayFcProtobufTest 의 본문 해석과 만들기 (2026-11-25 batch 6a). 규칙은 gateway_fc_fixture.hpp.
#include "tests/gateway/gateway_fc_fixture.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>

#include "argus/transport/team_wire/include/team_wire.hpp"
#include "argus_msg/fc_state.pb.h"

namespace gateway_fc_fixture {

using follow::NativeJson;

Bytes read_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("fixture 없음: " + path);
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

bool decode(const std::string& topic, const Bytes& body, Bytes& packed, std::uint64_t& team_seq,
            std::string& error) {
    NativeJson value;
    if (!argus::team_wire::decode_body(topic, false, body, value, team_seq, error)) {
        return false;
    }
    packed = NativeJson::to_msgpack(value);
    return true;
}

std::string msgpack_difference(const Bytes& expected, const Bytes& actual) {
    const std::string left = NativeJson::from_msgpack(expected).dump();
    const std::string right = NativeJson::from_msgpack(actual).dump();
    return left == right ? std::string() : left + "\n  != " + right;
}

Bytes encode(const std::string& topic, const Bytes& packed) {
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (!argus::team_wire::encode_body(topic, false, NativeJson::from_msgpack(packed), 1, body,
                                       unknown, error)) {
        return {};
    }
    return body;
}

Bytes without_status(const Bytes& body) {
    argus::msg::FcState message;
    if (!message.ParseFromArray(body.data(), static_cast<int>(body.size()))) {
        throw std::runtime_error("fc/state parse");
    }
    message.clear_status();
    const std::string text = message.SerializeAsString();
    return {text.begin(), text.end()};
}

Bytes with_key(const Bytes& packed, const std::string& key, const Bytes& value_msgpack) {
    NativeJson value = NativeJson::from_msgpack(packed);
    value[key] = NativeJson::from_msgpack(value_msgpack);
    return NativeJson::to_msgpack(value);
}

}  // namespace gateway_fc_fixture
