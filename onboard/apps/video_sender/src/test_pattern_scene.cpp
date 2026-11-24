#include "argus/video_sender/test_pattern_scene.hpp"

#include <algorithm>
#include <cstddef>

namespace argus::video_sender {
namespace {

// 사각형의 크기는 영상 크기의 1/8, 프레임마다 이동하는 거리는 영상 폭의 1/160 (640 에서 4 픽셀)
constexpr std::uint32_t kBoxSizeDivisor = 8;
constexpr std::uint32_t kStepDivisor = 160;
constexpr std::uint8_t kBoxBrightness = 255;
constexpr std::uint32_t kBackgroundLevels = 128;  // 배경 밝기는 0~127 로 사각형보다 어둡다

// 시험용 객체의 식별 값. class_id 0 은 video.md 8장의 tank 이다
constexpr std::uint32_t kTrackId = 1;
constexpr std::uint32_t kGeneration = 1;
constexpr std::int32_t kClassId = 0;
constexpr const char* kLabel = "tank";
constexpr double kConfidence = 0.9;

perception_api::Classification test_pattern_classification() {
    perception_api::Classification classification;
    classification.class_id = kClassId;
    classification.label = kLabel;
    classification.confidence = kConfidence;
    return classification;
}

}  // namespace

perception_api::PixelBox test_pattern_box(const TestPatternConfig& config, std::uint64_t frame_id) {
    if (config.width == 0 || config.height == 0) {
        return {};
    }
    const std::uint32_t box_width = std::max<std::uint32_t>(config.width / kBoxSizeDivisor, 1);
    const std::uint32_t box_height = std::max<std::uint32_t>(config.height / kBoxSizeDivisor, 1);
    const std::uint32_t step = std::max<std::uint32_t>(config.width / kStepDivisor, 1);
    // 사각형의 왼쪽 변이 이동하는 범위. 사각형이 영상 밖으로 나가지 않는다
    const std::uint32_t travel = config.width - box_width + 1;
    const auto x1 = static_cast<std::uint32_t>((frame_id * step) % travel);
    const std::uint32_t y1 = (config.height - box_height) / 2;
    perception_api::PixelBox box;
    box.x1 = x1;
    box.y1 = y1;
    box.x2 = x1 + box_width;
    box.y2 = y1 + box_height;
    return box;
}

std::vector<std::uint8_t> test_pattern_pixels(const TestPatternConfig& config,
                                              std::uint64_t frame_id) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(config.width) * config.height);
    const perception_api::PixelBox box = test_pattern_box(config, frame_id);
    for (std::uint32_t y = 0; y < config.height; ++y) {
        for (std::uint32_t x = 0; x < config.width; ++x) {
            const bool in_box = x >= box.x1 && x < box.x2 && y >= box.y1 && y < box.y2;
            const auto background = static_cast<std::uint8_t>(x * kBackgroundLevels / config.width);
            pixels[static_cast<std::size_t>(y) * config.width + x] =
                in_box ? kBoxBrightness : background;
        }
    }
    return pixels;
}

perception_api::Observations test_pattern_observations(const TestPatternConfig& config,
                                                       const perception_api::Frame& frame) {
    perception_api::Observations observations;
    observations.source = frame.source;
    observations.frame_id = frame.frame_id;
    observations.sample_monotonic_ns = frame.received_monotonic_ns;
    observations.image_width = frame.width;
    observations.image_height = frame.height;
    perception_api::Detection detection;
    detection.classification = test_pattern_classification();
    detection.box = test_pattern_box(config, frame.frame_id);
    observations.detections.push_back(detection);
    return observations;
}

perception_api::Tracks test_pattern_tracks(const TestPatternConfig& config,
                                           const perception_api::Frame& frame) {
    perception_api::Tracks tracks;
    tracks.source = frame.source;
    tracks.session_id = frame.session_id;
    tracks.frame_id = frame.frame_id;
    tracks.generation = kGeneration;
    tracks.sample_monotonic_ns = frame.received_monotonic_ns;
    tracks.image_width = frame.width;
    tracks.image_height = frame.height;
    perception_api::Track track;
    track.track_id = kTrackId;
    track.classification = test_pattern_classification();
    track.box = test_pattern_box(config, frame.frame_id);
    track.state = perception_api::TrackState::Confirmed;
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = frame.received_monotonic_ns;
    tracks.tracks.push_back(track);
    return tracks;
}

}  // namespace argus::video_sender
