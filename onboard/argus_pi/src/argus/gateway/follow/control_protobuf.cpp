// 게이트웨이의 6372 받기 (2026-11-25 batch 6b). 규칙은 include/control_protobuf.hpp.
#include "argus/gateway/follow/include/control_protobuf.hpp"

#include <exception>
#include <string_view>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

namespace argus::gateway {

namespace {

bool control_topic(std::string_view name) {
    return name == argus::team_wire::kControlSetpoint || name == argus::team_wire::kFollowStatus;
}

}  // namespace

ControlFrame control_msgpack(const std::string& name, const std::vector<std::uint8_t>& body,
                             std::vector<std::uint8_t>& packed, std::string& error) {
    packed.clear();
    error.clear();
    if (!control_topic(name)) {
        return ControlFrame::OtherName;
    }
    const auto* spec = argus::team_wire::find_wire_spec(name);
    if (spec == nullptr || body.size() > spec->max_body_bytes) {
        error = name + " 본문 상한 초과";
        return ControlFrame::Invalid;
    }
    follow::NativeJson value;
    std::uint64_t sequence = 0;
    if (!argus::team_wire::decode_body(name, false, body, value, sequence, error)) {
        return ControlFrame::Invalid;
    }
    try {
        packed = follow::NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = name + " msgpack: " + failure.what();
        packed.clear();
        return ControlFrame::Invalid;
    }
    return ControlFrame::Ok;
}

}  // namespace argus::gateway
