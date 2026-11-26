// 게이트웨이의 6373 요청-응답 (2026-11-25 batch 8). 규칙은 include/follow_command_protobuf.hpp.
#include "argus/gateway/follow/include/follow_command_protobuf.hpp"

#include <exception>
#include <string_view>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

namespace argus::gateway {

const std::string& follow_command_name() {
    static const std::string kName(argus::team_wire::kFollowCommand);
    return kName;
}

bool follow_command_request_body(const std::vector<std::uint8_t>& packed,
                                 std::vector<std::uint8_t>& body, std::vector<std::string>& unknown,
                                 std::string& error) {
    body.clear();
    unknown.clear();
    error.clear();
    follow::NativeJson value;
    try {
        value = follow::NativeJson::from_msgpack(packed);
    } catch (const std::exception& failure) {
        error = std::string("6373 요청 msgpack: ") + failure.what();
        return false;
    }
    const auto* spec = argus::team_wire::find_wire_spec(argus::team_wire::kFollowCommand);
    if (!argus::team_wire::encode_body(argus::team_wire::kFollowCommand, false, value, 0, body,
                                       unknown, error)) {
        body.clear();
        return false;
    }
    if (spec == nullptr || body.size() > spec->max_body_bytes) {
        error = "6373 요청 본문 상한 초과";
        body.clear();
        return false;
    }
    return true;
}

FollowReplyFrame follow_command_reply_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                                              std::vector<std::uint8_t>& packed,
                                              std::string& error) {
    packed.clear();
    error.clear();
    if (parts.size() != 2) {
        error = "6373 응답 프레임 수 " + std::to_string(parts.size());
        return FollowReplyFrame::BadShape;
    }
    const std::string_view name(reinterpret_cast<const char*>(parts.front().data()),
                                parts.front().size());
    if (name != argus::team_wire::kFollowCommand) {
        error = "6373 응답 이름이 다름";
        return FollowReplyFrame::BadShape;
    }
    const auto* spec = argus::team_wire::find_wire_spec(name);
    if (spec == nullptr || parts.back().size() > spec->max_body_bytes) {
        error = "6373 응답 본문 상한 초과";
        return FollowReplyFrame::DecodeFailed;
    }
    follow::NativeJson value;
    std::uint64_t sequence = 0;
    if (!argus::team_wire::decode_body(name, true, parts.back(), value, sequence, error)) {
        return FollowReplyFrame::DecodeFailed;
    }
    try {
        packed = follow::NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = std::string("6373 응답 msgpack: ") + failure.what();
        packed.clear();
        return FollowReplyFrame::DecodeFailed;
    }
    return FollowReplyFrame::Ok;
}

}  // namespace argus::gateway
