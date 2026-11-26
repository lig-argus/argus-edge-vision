#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

#include "argus/tracking/contracts/include/tracks_v1.hpp"

namespace follow {

// A comparison-only motion-quality hint, never an identity or detection decision.
class BoxMotionGuard {
public:
    void reset() {
        history_.clear();
        latched_.reset();
        restore_count_ = 0;
    }

    bool suspect(const TracksV1& frame, const TrackRecordV1& track, double threshold = .8) {
        if (!frame.ground_pose || !track.ground) {
            return latched_.has_value();
        }
        const double dn = (*track.ground)[0] - (*frame.ground_pose)[0];
        const double de = (*track.ground)[1] - (*frame.ground_pose)[1];
        const double range = std::hypot(dn, de);
        const double w = (track.x2 - track.x1) * range;
        const double h = (track.y2 - track.y1) * range;
        if (!std::isfinite(range) || range < .2 || !std::isfinite(w) || !std::isfinite(h) ||
            w <= 0 || h <= 0 || frame.sample_monotonic_ns <= 0) {
            return latched_.has_value();
        }
        const auto now = frame.sample_monotonic_ns;
        if (!history_.empty() && now <= history_.back().stamp) {
            reset();
        }
        while (!history_.empty() && now - history_.front().stamp > 750000000LL) {
            history_.pop_front();
        }
        if (latched_) {
            const bool restored =
                w >= threshold * latched_->width && h >= threshold * latched_->height;
            restore_count_ = restored ? restore_count_ + 1 : 0;
            if (restore_count_ < 2) {
                return true;
            }
            reset();
        }
        if (!history_.empty()) {
            const auto baseline = *std::max_element(
                history_.begin(), history_.end(), [](const Entry& a, const Entry& b) {
                    return a.width * a.height < b.width * b.height;
                });
            if (w < threshold * baseline.width || h < threshold * baseline.height) {
                latched_ = baseline;
                restore_count_ = 0;
                return true;
            }
        }
        history_.push_back({now, w, h});
        while (history_.size() > 128) {
            history_.pop_front();
        }
        return false;
    }

private:
    struct Entry {
        std::int64_t stamp;
        double width, height;
    };
    std::deque<Entry> history_;
    std::optional<Entry> latched_;
    unsigned restore_count_{0};
};
}  // namespace follow
