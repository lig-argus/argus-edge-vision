#pragma once
#include <cstdint>
#include <string>
namespace follow {
// Local typed view of existing wire identity; no automatic ID/session regeneration.
struct ModeRequestIdentity {
    std::string command_id, requester_session, onboard_session, uid;
    std::uint64_t fc_epoch{0};
    bool matches(const ModeRequestIdentity& other) const {
        return !command_id.empty() && !requester_session.empty() && !onboard_session.empty() &&
               !uid.empty() && command_id == other.command_id &&
               requester_session == other.requester_session &&
               onboard_session == other.onboard_session && uid == other.uid &&
               fc_epoch == other.fc_epoch;
    }
};
}  // namespace follow
