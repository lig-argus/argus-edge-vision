// 게이트웨이의 6371 요청-응답 (2026-11-25 batch 7). 규칙은 include/mode_rpc_protobuf.hpp.
#include "argus/gateway/follow/include/mode_rpc_protobuf.hpp"

#include <exception>
#include <string_view>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

namespace argus::gateway {

const std::string& mode_rpc_name() {
    static const std::string kName(argus::team_wire::kControlModeRequest);
    return kName;
}

ModeRpcFrame mode_rpc_request_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                                      std::vector<std::uint8_t>& packed, std::string& error) {
    packed.clear();
    error.clear();
    if (parts.size() != 2) {
        error = "6371 요청 프레임 수 " + std::to_string(parts.size());
        return ModeRpcFrame::Invalid;
    }
    const std::string_view name(reinterpret_cast<const char*>(parts.front().data()),
                                parts.front().size());
    if (name != argus::team_wire::kControlModeRequest) {
        error = "6371 요청 이름이 다름";
        return ModeRpcFrame::Invalid;
    }
    const auto* spec = argus::team_wire::find_wire_spec(name);
    if (spec == nullptr || parts.back().size() > spec->max_body_bytes) {
        error = "6371 요청 본문 상한 초과";
        return ModeRpcFrame::Invalid;
    }
    follow::NativeJson value;
    std::uint64_t sequence = 0;
    if (!argus::team_wire::decode_body(name, false, parts.back(), value, sequence, error)) {
        return ModeRpcFrame::Invalid;
    }
    try {
        packed = follow::NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = std::string("6371 요청 msgpack: ") + failure.what();
        packed.clear();
        return ModeRpcFrame::Invalid;
    }
    return ModeRpcFrame::Ok;
}

bool mode_rpc_reply_body(const std::vector<std::uint8_t>& packed, std::vector<std::uint8_t>& body,
                         std::vector<std::string>& unknown, std::string& error) {
    body.clear();
    unknown.clear();
    error.clear();
    follow::NativeJson value;
    try {
        value = follow::NativeJson::from_msgpack(packed);
    } catch (const std::exception& failure) {
        error = std::string("6371 응답 msgpack: ") + failure.what();
        return false;
    }
    if (!argus::team_wire::encode_body(argus::team_wire::kControlModeRequest, true, value, 0, body,
                                       unknown, error)) {
        body.clear();
        return false;
    }
    return true;
}

}  // namespace argus::gateway
