// GatewayControlProtobufTest 의 본문 만들기 (2026-11-25 batch 6b). 규칙은
// gateway_control_fixture.hpp.
#include "tests/gateway/gateway_control_fixture.hpp"

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"
#include "argus_msg/pi_follow_status.pb.h"

namespace gateway_control_fixture {

using follow::NativeJson;

Bytes status_with_rows(const Bytes& body, std::size_t rows) {
    argus::msg::PiFollowStatus message;
    if (!message.ParseFromArray(body.data(), static_cast<int>(body.size()))) {
        throw std::runtime_error("follow/status parse");
    }
    auto* results = message.mutable_status()->mutable_command_results();
    if (results->empty()) {
        throw std::runtime_error("follow/status 에 command_results 행이 없다");
    }
    while (static_cast<std::size_t>(results->size()) < rows) {
        const argus::msg::PiFollowStatusCommandResult row = results->Get(results->size() - 1);
        *results->Add() = row;
    }
    const std::string text = message.SerializeAsString();
    return {text.begin(), text.end()};
}

Bytes mode_rpc_request(const std::string& action, bool mode, std::uint64_t sequence,
                       const std::string& onboard_session) {
    const std::string session = "a1b2c3d4e5f60718";
    NativeJson value{{"schema", "argus.follow.rpc.v1"},
                     {"source", "argus-follow-controller"},
                     {"request_id", session + "-" + std::to_string(sequence)},
                     {"client_session", session},
                     {"session_id", session},
                     {"command_id", session + "-" + std::to_string(sequence)},
                     {"action", action},
                     {"uid", "000600000000353733323333511700280027"},
                     {"clock_domain", "host_monotonic"},
                     {"onboard_session", onboard_session},
                     {"fc_epoch", std::uint64_t{0}},
                     {"owner_token", ""},
                     {"command_epoch", std::uint64_t{0}},
                     {"generation", std::uint64_t{0}},
                     {"track_id", std::int64_t{0}},
                     {"source_sample_monotonic_ns", std::int64_t{0}},
                     {"bench", false},
                     {"issued_monotonic_ns", std::int64_t{1000}},
                     {"seq", sequence},
                     {"revision", std::uint64_t{0}}};
    if (mode) {
        value["schema"] = "argus.control.mode_request.v1";
        if (action == "start_offboard") {
            value["mode"] = "OFFBOARD";
        } else if (action == "hold") {
            value["mode"] = "HOLD";
        } else {
            value["mode"] = "HOLD_HEADING";
        }
        value["valid_until_monotonic_ns"] = std::int64_t{2000};
    }
    return NativeJson::to_msgpack(value);
}

Bytes encode_mode_request(const Bytes& packed) {
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (!argus::team_wire::encode_body(argus::team_wire::kControlModeRequest, false,
                                       NativeJson::from_msgpack(packed), 0, body, unknown, error) ||
        !unknown.empty()) {
        return {};
    }
    return body;
}

bool decode_mode_reply(const Bytes& body, Bytes& packed, std::string& error) {
    NativeJson value;
    std::uint64_t ignored = 0;
    if (!argus::team_wire::decode_body(argus::team_wire::kControlModeRequest, true, body, value,
                                       ignored, error)) {
        return false;
    }
    try {
        packed = NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = failure.what();
        return false;
    }
    return true;
}

bool decode_follow_command(const Bytes& body, Bytes& packed, std::string& error) {
    NativeJson value;
    std::uint64_t ignored = 0;
    if (!argus::team_wire::decode_body(argus::team_wire::kFollowCommand, false, body, value,
                                       ignored, error)) {
        return false;
    }
    try {
        packed = NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = failure.what();
        return false;
    }
    return true;
}

Bytes encode_follow_result(const Bytes& packed) {
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (!argus::team_wire::encode_body(argus::team_wire::kFollowCommand, true,
                                       NativeJson::from_msgpack(packed), 0, body, unknown, error) ||
        !unknown.empty()) {
        return {};
    }
    return body;
}

}  // namespace gateway_control_fixture
