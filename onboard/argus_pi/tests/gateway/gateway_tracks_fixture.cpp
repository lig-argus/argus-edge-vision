// GatewayTracksProtobufTest 의 본문 만들기. 규칙은 gateway_tracks_fixture.hpp.
#include "tests/gateway/gateway_tracks_fixture.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"
#include "tests/team_wire/team_wire_test_util.hpp"

namespace gateway_tracks_fixture {
namespace {

Bytes encode(const team_wire_test::NativeJson& value, std::uint64_t sequence) {
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (!argus::team_wire::encode_body(argus::team_wire::kTrackingTracks, false, value, sequence,
                                       body, unknown, error)) {
        throw std::runtime_error("tracks fixture encode: " + error);
    }
    if (!unknown.empty()) {
        throw std::runtime_error("tracks fixture 에 proto 에 없는 키: " + unknown.front());
    }
    return body;
}

}  // namespace

std::vector<PiBody> pi_bodies() {
    std::vector<PiBody> bodies;
    std::uint64_t sequence = 0;
    for (const auto& value : team_wire_test::fixtures<argus::msg::TrackingTracks>()) {
        PiBody entry;
        entry.schema = value.at("schema").get<std::string>();
        entry.body = encode(value, ++sequence);
        entry.legacy_msgpack =
            team_wire_test::NativeJson::to_msgpack(team_wire_test::normalized(value));
        entry.generation_field = value.at("generation").get<std::uint64_t>() <= 0xffffffffULL;
        bodies.push_back(std::move(entry));
    }
    return bodies;
}

Bytes missing_pi_schema_body() {
    auto value = team_wire_test::fixtures<argus::msg::TrackingTracks>().front();
    value["schema"] = "";
    return encode(value, 1);
}

}  // namespace gateway_tracks_fixture
