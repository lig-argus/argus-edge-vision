// dds_types() 의 변환 테스트. IDL 타입이 .proto 의 필드를 빠짐없이 나르는지 확인한다: 값을 IDL
// 표본으로 바꿨다가 되돌린 결과를 zmq_codecs() 로 protobuf 본문과 바이너리로 만들어, 원래 값의
// 결과와 바이트가 같은지 비교한다
#include "argus/perception_api_bus/dds_types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/zmq_codecs.hpp"
#include "argus_msg/perception_observations.hpp"
#include "argus_msg/tracking_status.hpp"
#include "argus_msg/tracking_tracks.hpp"
#include "argus_msg/vision_frame_gray8.hpp"

namespace argus::perception_api_bus {
namespace {

using bus_zmq::Bytes;

class DdsTypesTest : public ::testing::Test {
protected:
    // value 를 IDL 표본(Idl)으로 바꿨다가 되돌린다. 어느 쪽이든 실패하면 nullopt
    template <typename T, typename Idl>
    std::optional<T> through_idl(const T& value) const {
        const bus_dds::DdsType& type = dds_.at(std::type_index(typeid(T)));
        Idl sample;
        if (!type.to_idl(&value, &sample)) {
            return std::nullopt;
        }
        return from_sample<T>(sample);
    }

    // 표본을 값으로 바꾼다. 실패하면 nullopt
    template <typename T, typename Idl>
    std::optional<T> from_sample(const Idl& sample) const {
        const bus_dds::DdsType& type = dds_.at(std::type_index(typeid(T)));
        T value{};
        if (!type.from_idl(&sample, &value)) {
            return std::nullopt;
        }
        return value;
    }

    // value 의 protobuf 본문과 바이너리 (발행 번호 1)
    template <typename T>
    std::pair<Bytes, Bytes> zmq_frames(const T& value) const {
        Bytes body;
        Bytes binary;
        EXPECT_TRUE(zmq_.at(std::type_index(typeid(T))).encode(&value, 1, body, binary));
        return {body, binary};
    }

    // value 를 IDL 로 왕복시킨 값의 본문과 바이너리가 value 의 것과 같은지 확인한다
    template <typename T, typename Idl>
    void expect_same_zmq_frames(const T& value) const {
        const std::optional<T> returned = through_idl<T, Idl>(value);
        ASSERT_TRUE(returned.has_value());
        EXPECT_EQ(zmq_frames(returned.value_or(T{})), zmq_frames(value));
    }

    bus_dds::DdsTypeTable dds_ = dds_types();
    bus_zmq::ZmqCodecTable zmq_ = zmq_codecs();
};

perception_api::Frame sample_frame() {
    perception_api::Frame frame;
    frame.source = "ir";
    frame.session_id = "session-a";
    frame.frame_id = 42;
    frame.unix_ns = 1'700'000'000'000'000'000;
    frame.received_monotonic_ns = 5'000'000'000;
    frame.width = 4;
    frame.height = 2;
    frame.stride = 5;
    frame.pixels = {1, 2, 3, 4, 0, 5, 6, 7, 8, 0};
    return frame;
}

perception_api::Track sample_track() {
    perception_api::Track track;
    track.track_id = 7;
    track.classification = perception_api::Classification{1, "person", 0.5};
    track.box = perception_api::PixelBox{10.5, 20.25, 30.0, 40.75};
    track.state = perception_api::TrackState::Predicted;
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = 4'900'000'000;
    track.ground = perception_api::GroundEstimate{1.0, 2.0, 0.1, 0.01, 0.2, 0.005};
    track.ground_anchor = perception_api::GroundAnchor{0.9, 0.0};
    return track;
}

TEST_F(DdsTypesTest, FrameKeepsHeaderAndPixels) {
    expect_same_zmq_frames<perception_api::Frame, idl::VisionFrameGray8>(sample_frame());
}

TEST_F(DdsTypesTest, FrameWithStrideSmallerThanWidthCannotBeConverted) {
    perception_api::Frame frame = sample_frame();
    frame.stride = 3;
    frame.pixels.assign(6, 0);

    EXPECT_FALSE((through_idl<perception_api::Frame, idl::VisionFrameGray8>(frame)).has_value());
}

TEST_F(DdsTypesTest, FrameWithWrongPixelCountCannotBeConverted) {
    perception_api::Frame frame = sample_frame();
    frame.pixels.pop_back();

    EXPECT_FALSE((through_idl<perception_api::Frame, idl::VisionFrameGray8>(frame)).has_value());
}

TEST_F(DdsTypesTest, ObservationsKeepEveryField) {
    perception_api::Observations observations;
    observations.source = "ir";
    observations.session_id = "session-a";
    observations.frame_id = 42;
    observations.sample_monotonic_ns = 5'000'000'000;
    observations.model = "yolov8n";
    observations.image_width = 640;
    observations.image_height = 480;
    observations.detections.push_back(
        perception_api::Detection{perception_api::Classification{1, "person", 0.75},
                                  perception_api::PixelBox{10, 20, 30, 40}});
    observations.detections.push_back(perception_api::Detection{
        perception_api::Classification{2, "car", 0.5}, perception_api::PixelBox{1, 2, 3, 4}});

    expect_same_zmq_frames<perception_api::Observations, idl::PerceptionObservations>(observations);
}

TEST_F(DdsTypesTest, EmptySessionIdBecomesNoValue) {
    perception_api::Observations observations;
    observations.session_id = std::string{};

    const std::optional<perception_api::Observations> returned =
        through_idl<perception_api::Observations, idl::PerceptionObservations>(observations);

    ASSERT_TRUE(returned.has_value());
    EXPECT_EQ(returned.value_or(perception_api::Observations{}).session_id, std::nullopt);
}

TEST_F(DdsTypesTest, TracksKeepEveryField) {
    perception_api::Tracks tracks;
    tracks.source = "ir";
    tracks.session_id = "session-a";
    tracks.frame_id = 42;
    tracks.fc_epoch = 3;
    tracks.generation = 2;
    tracks.sample_monotonic_ns = 5'000'000'000;
    tracks.image_width = 640;
    tracks.image_height = 480;
    tracks.ground_pose = perception_api::GroundPose{1.0, 2.0, -3.0, 0.1, -0.1, 1.5};
    tracks.tracks.push_back(sample_track());
    perception_api::Track without_ground = sample_track();
    without_ground.track_id = 8;
    without_ground.state = perception_api::TrackState::Confirmed;
    without_ground.ground.reset();
    without_ground.ground_anchor.reset();
    tracks.tracks.push_back(without_ground);

    expect_same_zmq_frames<perception_api::Tracks, idl::TrackingTracks>(tracks);
}

TEST_F(DdsTypesTest, TracksWithUnspecifiedStateCannotBeConverted) {
    idl::TrackingTracks sample;
    idl::TrackingTracks_Track track;
    track.state(idl::TrackState::TRACK_STATE_UNSPECIFIED);
    sample.tracks().push_back(track);

    EXPECT_FALSE(from_sample<perception_api::Tracks>(sample).has_value());
}

TEST_F(DdsTypesTest, TrackingStatusKeepsEveryField) {
    perception_api::TrackingStatus status;
    status.source = "eo";
    status.published_monotonic_ns = 5'000'000'000;
    status.generation = 4;
    status.fc_fresh = true;
    status.cmc_valid = false;
    status.fps = 14.25;

    expect_same_zmq_frames<perception_api::TrackingStatus, idl::TrackingStatus>(status);
}

}  // namespace
}  // namespace argus::perception_api_bus
