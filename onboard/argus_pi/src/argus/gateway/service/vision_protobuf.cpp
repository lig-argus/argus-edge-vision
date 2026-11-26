// 게이트웨이의 영상 6374 받기 (2026-11-25 batch 4). 규칙은 include/vision_protobuf.hpp.
#include "argus/gateway/service/include/vision_protobuf.hpp"

#include <string_view>

#include "argus/transport/team_wire/include/vision_body.hpp"

namespace argus::gateway {

VisionFrame vision_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                           std::vector<std::uint8_t>& packed, std::string& error) {
    packed.clear();
    error.clear();
    if (parts.size() < 2 || parts.size() > 3) {
        return VisionFrame::NotVision;
    }
    const auto& name = parts.front();
    const std::string_view topic(reinterpret_cast<const char*>(name.data()), name.size());
    if (!argus::team_wire::is_vision_topic(topic)) {
        return VisionFrame::NotVision;
    }
    const bool binary_required = argus::team_wire::vision_binary_required(topic);
    if (parts.size() != (binary_required ? 3U : 2U)) {
        return VisionFrame::NotVision;
    }
    if (binary_required && parts.back().empty()) {
        error = std::string(topic) + " 빈 바이너리";
        return VisionFrame::Invalid;
    }
    const auto& body = parts.at(1);
    switch (
        argus::team_wire::vision_body_to_msgpack(topic, body.data(), body.size(), packed, error)) {
        case argus::team_wire::VisionBodyStatus::Ok:
            return VisionFrame::Ok;
        case argus::team_wire::VisionBodyStatus::MissingPiSchema:
            return VisionFrame::MissingPiSchema;
        case argus::team_wire::VisionBodyStatus::NotVision:
            return VisionFrame::NotVision;
        case argus::team_wire::VisionBodyStatus::TooLarge:
        case argus::team_wire::VisionBodyStatus::DecodeFailed:
            break;
    }
    return VisionFrame::Invalid;
}

}  // namespace argus::gateway
