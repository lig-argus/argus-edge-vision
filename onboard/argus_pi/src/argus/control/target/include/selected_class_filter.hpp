#pragma once

#include <algorithm>
#include <cstdint>

#include "argus/tracking/contracts/include/tracks_v1.hpp"

namespace follow {

// Preserve the frame envelope and every ID in the selected class. The caller
// retains the original frame for operator display and selection.
inline TracksV1 selected_class_tracks(const TracksV1& frame, std::int32_t class_id) {
    auto filtered = frame;
    filtered.tracks.erase(
        std::remove_if(filtered.tracks.begin(), filtered.tracks.end(),
                       [class_id](const auto& track) { return track.class_id != class_id; }),
        filtered.tracks.end());
    return filtered;
}

}  // namespace follow
