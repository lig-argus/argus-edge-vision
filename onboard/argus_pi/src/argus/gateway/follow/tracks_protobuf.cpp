#include "argus/gateway/follow/include/tracks_protobuf.hpp"

#include <string_view>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/tracks_body.hpp"

namespace argus::gateway {

TracksFrame tracks_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                           std::vector<std::uint8_t>& packed, std::string& error) {
    packed.clear();
    error.clear();
    if (parts.size() != 2) {
        return TracksFrame::NotTracks;
    }
    const auto& name = parts.front();
    const std::string_view topic(reinterpret_cast<const char*>(name.data()), name.size());
    if (topic != argus::team_wire::kTrackingTracks) {
        return TracksFrame::NotTracks;
    }
    const auto& body = parts.back();
    switch (argus::team_wire::tracks_body_to_msgpack(body.data(), body.size(), packed, error)) {
        case argus::team_wire::TracksBodyStatus::Ok:
            return TracksFrame::Ok;
        case argus::team_wire::TracksBodyStatus::MissingPiSchema:
            return TracksFrame::MissingPiSchema;
        case argus::team_wire::TracksBodyStatus::TooLarge:
        case argus::team_wire::TracksBodyStatus::DecodeFailed:
            break;
    }
    return TracksFrame::Invalid;
}

}  // namespace argus::gateway
