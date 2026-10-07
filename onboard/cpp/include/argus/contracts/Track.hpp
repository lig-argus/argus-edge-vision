#pragma once
#include "argus/contracts/Observation.hpp"
#include <optional>
namespace argus {
enum class TrackState { Confirmed, Predicted, Lost };
struct Track {
    std::int64_t id{};
    FullBox full_box;
    TrackState state{TrackState::Lost};
    std::optional<std::uint64_t> last_real_detection_monotonic_ns;
    std::uint64_t ttl_ns{}; // zero is invalid/expired; never invent an operational TTL.
    bool stale(std::uint64_t now) const {
        return state == TrackState::Lost || !last_real_detection_monotonic_ns || !ttl_ns ||
               now < *last_real_detection_monotonic_ns || now - *last_real_detection_monotonic_ns >= ttl_ns;
    }
    Track with_prediction(FullBox box) const {
        Track next = *this;
        next.full_box = std::move(box);
        next.state = TrackState::Predicted;
        return next; // prediction MUST NOT refresh last-real timestamp or TTL.
    }
    Track with_real_detection(FullBox box, std::uint64_t stamp) const {
        if (last_real_detection_monotonic_ns && stamp < *last_real_detection_monotonic_ns)
            throw std::invalid_argument("out-of-order real detection");
        Track next = *this;
        next.full_box = std::move(box);
        next.state = TrackState::Confirmed;
        next.last_real_detection_monotonic_ns = stamp;
        return next;
    }
};
struct TrackBatch {
    std::uint64_t frame_id{}, frame_monotonic_ns{}, expires_monotonic_ns{};
    std::vector<Track> tracks;
    const Track *valid_track(std::int64_t id, std::uint64_t now) const {
        if (now < frame_monotonic_ns || now >= expires_monotonic_ns)
            return nullptr;
        for (const auto &track : tracks)
            if (track.id == id && !track.stale(now))
                return &track;
        return nullptr;
    }
};
} // namespace argus
