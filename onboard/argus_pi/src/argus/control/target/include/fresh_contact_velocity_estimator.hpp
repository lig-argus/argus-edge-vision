#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <tuple>

#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/tracking/motion/include/short_motion_estimator.hpp"

namespace follow {

// Same real-bottom quality gates as R5, warmed before clipping; no predicted samples or new TTL.
class FreshContactVelocityEstimator {
public:
    FreshContactVelocityEstimator() : estimator_(config()) {}

    void reset() {
        estimator_.reset();
        latest_ = {};
        widths_.clear();
        identity_.reset();
        last_motion_ns_ = 0;
        confirmed_moving_ = false;
        can_refresh_ = false;
    }

    void observe(const TracksV1& frame, const TrackRecordV1& track, std::int64_t received_ns) {
        can_refresh_ = false;
        latest_ = {};
        if (!track.real_detection || !track.ground || !frame.ground_pose || !track.ground_anchor ||
            (*track.ground_anchor)[0] != 1. || frame.clock_domain != "host_monotonic" ||
            !frame.motion_sample_ns || *frame.motion_sample_ns <= 0 ||
            frame.sample_monotonic_ns <= 0 || received_ns < frame.sample_monotonic_ns ||
            received_ns - frame.sample_monotonic_ns >= 250000000 ||
            track.last_real_detection_monotonic_ns != frame.sample_monotonic_ns ||
            !std::isfinite((*track.ground)[0] + (*track.ground)[1] + (*track.ground)[2] +
                           (*track.ground)[4] + (*track.ground)[5]) ||
            (*track.ground)[2] < 0 || (*track.ground)[2] > .25 || (*track.ground)[4] < 0 ||
            (*track.ground)[4] > .25 || (*track.ground)[5] < 0 || (*track.ground)[5] > 50 ||
            frame.image_width <= 4 || frame.image_height <= 4 ||
            !std::isfinite(track.x1 + track.x2 + track.y1 + track.y2) || track.x1 < 2 ||
            track.x2 > frame.image_width - 2 || track.y2 < 2 || track.y2 > frame.image_height - 2 ||
            track.x2 <= track.x1 || track.y2 - track.y1 < 8) {
            reset();
            return;
        }
        const Identity identity{frame.source,   frame.fc_epoch, frame.generation,
                                track.track_id, track.class_id, track.label};
        const auto stamp = *frame.motion_sample_ns;
        if ((identity_ && *identity_ != identity) ||
            (last_motion_ns_ > 0 &&
             (stamp < last_motion_ns_ || stamp - last_motion_ns_ > 250000000))) {
            reset();
        }
        if (last_motion_ns_ > 0 && stamp == last_motion_ns_) {
            return;
        }
        identity_ = identity;
        last_motion_ns_ = stamp;
        widths_.push_back({stamp, track.x2 - track.x1});
        while (!widths_.empty() && stamp - widths_.front().first > 500000000) {
            widths_.pop_front();
        }
        const auto bounds =
            std::minmax_element(widths_.begin(), widths_.end(),
                                [](const auto& a, const auto& b) { return a.second < b.second; });
        if (bounds.first->second < .8 * bounds.second->second) {
            reset();
            return;
        }
        latest_ = estimator_.update(stamp, {(*track.ground)[0], (*track.ground)[1]});
        if (!latest_.valid) {
            return;
        }
        if (latest_.phase == ObservedMotionPhase::Stopped && !latest_.departure_pending) {
            confirmed_moving_ = false;
            can_refresh_ = true;
        } else if (latest_.phase == ObservedMotionPhase::Moving && latest_.direction_valid) {
            if (latest_.feedforward_valid) {
                confirmed_moving_ = true;
            }
            // Hysteresis can retain a confirmed movement through deceleration below resume speed.
            can_refresh_ = confirmed_moving_;
        }
    }

    bool can_refresh() const { return can_refresh_; }
    const ShortMotionEstimate& latest() const { return latest_; }

private:
    using Identity = std::tuple<std::string, std::optional<std::uint64_t>, std::uint64_t,
                                std::int64_t, std::int32_t, std::string>;
    static ShortMotionConfig config() {
        ShortMotionConfig c;
        c.enabled = true;
        c.window_s = .5;
        c.minimum_span_s = .3;
        c.minimum_samples = 6;
        c.confirmation_s = .1;
        c.stop_speed_m_s = .08;
        c.resume_speed_m_s = .25;
        c.maximum_rmse_m = .05;
        c.maximum_sample_gap_s = .25;
        return c;
    }
    ShortMotionEstimator estimator_;
    ShortMotionEstimate latest_;
    std::deque<std::pair<std::int64_t, double>> widths_;
    std::optional<Identity> identity_;
    std::int64_t last_motion_ns_{0};
    bool confirmed_moving_{false}, can_refresh_{false};
};
}  // namespace follow
