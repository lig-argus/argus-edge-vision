#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace follow {
struct CvHistoricalVelocity {
    std::uint64_t frame_id;
    std::int64_t source_ns, fit_ns;
    double north, east;
    bool unconfirmed;
};

// Stored velocity is evidence for CV only, never a fresh observation or identity proof.
class CvTenFrameMemory {
public:
    static constexpr std::uint64_t kLookbackFrames = 60;
    static constexpr std::size_t kHistoryCapacity = 96;
    void reset() {
        history_.clear();
        frozen_.reset();
        freeze_attempted_ = false;
    }
    void clear_frozen() {
        frozen_.reset();
        freeze_attempted_ = false;
    }
    void remember(std::uint64_t frame, std::int64_t source, std::int64_t fit, double n, double e,
                  bool trusted, bool moving, bool unconfirmed) {
        if (!trusted || !moving || source <= 0 || !std::isfinite(n) || !std::isfinite(e)) {
            return;
        }
        if (!history_.empty() &&
            (frame <= history_.back().frame_id || source <= history_.back().source_ns)) {
            reset();
        }
        history_.push_back({frame, source, fit, n, e, unconfirmed});
        while (history_.size() > kHistoryCapacity) {
            history_.pop_front();
        }
    }
    std::optional<CvHistoricalVelocity> historical_before(std::uint64_t frame) const {
        if (frame < kLookbackFrames) {
            return {};
        }
        for (const auto& sample : history_) {
            if (sample.frame_id == frame - kLookbackFrames) {
                return sample;
            }
        }
        return {};
    }
    void freeze_before(std::uint64_t frame) {
        frozen_ = historical_before(frame);
        freeze_attempted_ = true;
    }
    std::optional<CvHistoricalVelocity> selected(std::uint64_t last_frame) const {
        return freeze_attempted_ ? frozen_ : historical_before(last_frame);
    }

private:
    std::deque<CvHistoricalVelocity> history_;
    std::optional<CvHistoricalVelocity> frozen_;
    bool freeze_attempted_{false};
};
}  // namespace follow
