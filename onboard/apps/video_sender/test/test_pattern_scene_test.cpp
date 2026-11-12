#include "argus/video_sender/test_pattern_scene.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "argus/perception_api/types.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Frame;
using perception_api::PixelBox;

constexpr std::uint8_t kBoxBrightness = 255;
constexpr std::uint64_t kLaterFrameId = 25;  // 사각형이 왼쪽 끝에서 떨어진 프레임

std::uint8_t pixel_at(const std::vector<std::uint8_t>& pixels, const TestPatternConfig& config,
                      double x, double y) {
    return pixels.at(static_cast<std::size_t>(y) * config.width + static_cast<std::size_t>(x));
}

Frame make_frame(const TestPatternConfig& config, std::uint64_t frame_id) {
    Frame frame;
    frame.source = config.source;
    frame.session_id = config.session_id;
    frame.frame_id = frame_id;
    frame.received_monotonic_ns = 5'000'000'000;
    frame.width = config.width;
    frame.height = config.height;
    return frame;
}

TEST(TestPatternSceneTest, BoxMovesRightEachFrame) {
    const TestPatternConfig config;

    const PixelBox first = test_pattern_box(config, 0);
    const PixelBox second = test_pattern_box(config, 1);

    EXPECT_EQ(first.x1, 0.0);
    EXPECT_EQ(second.x1, 4.0);  // 640 / 160
    EXPECT_EQ(second.y1, first.y1);
    EXPECT_EQ(first.width(), 80.0);   // 640 / 8
    EXPECT_EQ(first.height(), 60.0);  // 480 / 8
}

TEST(TestPatternSceneTest, BoxReturnsToLeftEdgeAndStaysInsideImage) {
    const TestPatternConfig config;
    bool returned_to_left = false;
    double previous_x1 = 0.0;

    for (std::uint64_t frame_id = 0; frame_id < 1000; ++frame_id) {
        const PixelBox box = test_pattern_box(config, frame_id);
        EXPECT_LE(box.x2, config.width);
        EXPECT_LE(box.y2, config.height);
        returned_to_left = returned_to_left || box.x1 < previous_x1;
        previous_x1 = box.x1;
    }

    EXPECT_TRUE(returned_to_left);
}

TEST(TestPatternSceneTest, PixelsHaveBrightRectangleAtBoxOfThatFrame) {
    const TestPatternConfig config;

    const std::vector<std::uint8_t> pixels = test_pattern_pixels(config, kLaterFrameId);
    const PixelBox box = test_pattern_box(config, kLaterFrameId);

    ASSERT_EQ(pixels.size(), static_cast<std::size_t>(config.width) * config.height);
    ASSERT_GT(box.x1, 0.0);
    EXPECT_EQ(pixel_at(pixels, config, box.x1, box.y1), kBoxBrightness);
    EXPECT_EQ(pixel_at(pixels, config, box.x2 - 1, box.y2 - 1), kBoxBrightness);
    EXPECT_LT(pixel_at(pixels, config, box.x1 - 1, box.y1), kBoxBrightness);  // 왼쪽 밖
    EXPECT_LT(pixel_at(pixels, config, box.x2, box.y1), kBoxBrightness);      // 오른쪽 밖
    EXPECT_LT(pixel_at(pixels, config, box.x1, box.y1 - 1), kBoxBrightness);  // 위쪽 밖
    EXPECT_LT(pixel_at(pixels, config, box.x1, box.y2), kBoxBrightness);      // 아래쪽 밖
}

TEST(TestPatternSceneTest, TracksAndObservationsMatchFrame) {
    const TestPatternConfig config;
    const Frame frame = make_frame(config, kLaterFrameId);
    const PixelBox box = test_pattern_box(config, kLaterFrameId);

    const perception_api::Tracks tracks = test_pattern_tracks(config, frame);
    EXPECT_EQ(tracks.source, frame.source);
    EXPECT_EQ(tracks.session_id, frame.session_id);
    EXPECT_EQ(tracks.frame_id, frame.frame_id);
    EXPECT_EQ(tracks.image_width, frame.width);
    EXPECT_EQ(tracks.image_height, frame.height);
    ASSERT_EQ(tracks.tracks.size(), 1U);
    EXPECT_EQ(tracks.tracks[0].box.x1, box.x1);
    EXPECT_EQ(tracks.tracks[0].box.y2, box.y2);

    const perception_api::Observations observations = test_pattern_observations(config, frame);
    EXPECT_EQ(observations.source, frame.source);
    EXPECT_EQ(observations.frame_id, frame.frame_id);
    ASSERT_EQ(observations.detections.size(), 1U);
    EXPECT_EQ(observations.detections[0].box.x1, box.x1);
}

TEST(TestPatternSceneTest, ZeroSizeGivesNoPixelsAndEmptyBox) {
    TestPatternConfig config;
    config.width = 0;
    config.height = 0;

    EXPECT_TRUE(test_pattern_pixels(config, 3).empty());
    EXPECT_EQ(test_pattern_box(config, 3).width(), 0.0);
}

}  // namespace
}  // namespace argus::video_sender
