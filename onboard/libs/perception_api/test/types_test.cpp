#include "argus/perception_api/types.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace argus::perception_api {
namespace {

TEST(FrameTest, ExpectedPixelBytesIsStrideTimesHeight) {
    Frame frame;
    frame.width = 640;
    frame.height = 480;
    frame.stride = 640;

    EXPECT_EQ(frame.expected_pixel_bytes(), 640U * 480U);
    EXPECT_TRUE(frame.pixels.empty());  // 픽셀은 어댑터가 채운다
}

TEST(FrameTest, StrideMayExceedWidth) {
    Frame frame;
    frame.width = 640;
    frame.height = 2;
    frame.stride = 704;  // 64 바이트 정렬

    EXPECT_EQ(frame.expected_pixel_bytes(), 704U * 2U);
}

TEST(PixelBoxTest, WidthAndHeightFromCorners) {
    const PixelBox box{100.0, 300.0, 150.0, 380.0};

    EXPECT_DOUBLE_EQ(box.width(), 50.0);
    EXPECT_DOUBLE_EQ(box.height(), 80.0);
}

TEST(ObservationsTest, DefaultsToNoDetections) {
    const Observations observations;

    EXPECT_TRUE(observations.detections.empty());
    EXPECT_EQ(observations.frame_id, 0U);
}

TEST(TrackTest, GroundProjectionIsEmptyUntilProvided) {
    const Track track;

    EXPECT_FALSE(track.ground.has_value());
    EXPECT_FALSE(track.ground_anchor.has_value());
    EXPECT_EQ(track.state, TrackState::Confirmed);
    EXPECT_FALSE(track.real_detection);
}

TEST(TracksTest, GroundPoseIsEmptyUntilProvidedAndTracksMayBeEmpty) {
    const Tracks tracks;

    EXPECT_FALSE(tracks.ground_pose.has_value());
    EXPECT_TRUE(tracks.tracks.empty());
    EXPECT_EQ(tracks.generation, 0U);
}

TEST(DetectionAndTrackTest, ShareClassificationAndBox) {
    Detection detection;
    detection.classification = {3, "tank", 0.9};
    detection.box = {10.0, 20.0, 30.0, 40.0};

    Track track;
    track.track_id = 12;
    track.classification =
        detection.classification;  // 추적기가 탐지의 분류·박스에 track_id 를 붙인다
    track.box = detection.box;

    EXPECT_EQ(track.classification.class_id, 3);
    EXPECT_EQ(track.classification.label, "tank");
    EXPECT_DOUBLE_EQ(track.box.width(), detection.box.width());
    EXPECT_EQ(track.track_id, 12U);
}

TEST(TrackingStatusTest, DefaultsToNotFreshAndNoCmc) {
    const TrackingStatus status;

    EXPECT_FALSE(status.fc_fresh);
    EXPECT_FALSE(status.cmc_valid);
    EXPECT_DOUBLE_EQ(status.fps, 0.0);
}

}  // namespace
}  // namespace argus::perception_api
