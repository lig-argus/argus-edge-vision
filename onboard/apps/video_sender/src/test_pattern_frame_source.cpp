#include "argus/video_sender/test_pattern_frame_source.hpp"

#include <utility>

namespace argus::video_sender {

TestPatternFrameSource::TestPatternFrameSource(const core::IClock& clock, UnixClock unix_clock,
                                               TestPatternConfig config)
    : clock_(clock), unix_clock_(std::move(unix_clock)), config_(std::move(config)) {}

core::Subscription TestPatternFrameSource::subscribe(FrameHandler handler) {
    return handlers_.subscribe(std::move(handler));
}

void TestPatternFrameSource::emit_frame() { handlers_.notify(make_frame(next_frame_id_++)); }

perception_api::Frame TestPatternFrameSource::make_frame(std::uint64_t frame_id) const {
    perception_api::Frame frame;
    frame.source = config_.source;
    frame.session_id = config_.session_id;
    frame.frame_id = frame_id;
    frame.unix_ns = unix_clock_();
    frame.received_monotonic_ns = clock_.now_monotonic_ns();
    frame.width = config_.width;
    frame.height = config_.height;
    frame.stride = config_.width;
    frame.pixels = test_pattern_pixels(config_, frame_id);
    return frame;
}

}  // namespace argus::video_sender
