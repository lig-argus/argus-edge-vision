// 변환 표가 값 타입과 protobuf 본문 사이를 규칙대로 바꾸는지 확인한다
#include "argus/perception_api_bus/zmq_codecs.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <typeindex>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/perception_api/types.hpp"
#include "argus_msg/perception_observations.pb.h"
#include "argus_msg/tracking_status.pb.h"
#include "argus_msg/tracking_tracks.pb.h"
#include "argus_msg/vision_frame_gray8.pb.h"

namespace argus::perception_api_bus {
namespace {

using bus_zmq::Bytes;

template <typename Message>
Bytes to_bytes(const Message& message) {
    Bytes body(message.ByteSizeLong());
    EXPECT_TRUE(message.SerializeToArray(body.data(), static_cast<std::int32_t>(body.size())));
    return body;
}

template <typename Message>
Message parse(const Bytes& body) {
    Message message;
    EXPECT_TRUE(message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size())));
    return message;
}

class ZmqCodecsTest : public ::testing::Test {
protected:
    template <typename T>
    const bus_zmq::ZmqCodec& codec() const {
        return table_.at(std::type_index(typeid(T)));
    }

    template <typename T>
    bool encode(const T& value, std::uint64_t sequence, Bytes& body, Bytes& binary) const {
        return codec<T>().encode(&value, sequence, body, binary);
    }

    // 실패하면 nullopt
    template <typename T>
    std::optional<T> decode(const Bytes& body, const Bytes& binary,
                            std::uint64_t* sequence = nullptr) const {
        T value;
        std::uint64_t decoded_sequence = 0;
        if (!codec<T>().decode(body, binary, &value, decoded_sequence)) {
            return std::nullopt;
        }
        if (sequence != nullptr) {
            *sequence = decoded_sequence;
        }
        return value;
    }

    bus_zmq::ZmqCodecTable table_ = zmq_codecs();
};

perception_api::Frame sample_frame() {
    perception_api::Frame frame;
    frame.source = "ir";
    frame.session_id = "session-a";
    frame.frame_id = 42;
    frame.unix_ns = 1'700'000'000'000'000'000;
    frame.received_monotonic_ns = 5'000'000'000;
    frame.width = 3;
    frame.height = 2;
    frame.stride = 4;
    frame.pixels = {1, 2, 3, 0, 4, 5, 6, 0};
    return frame;
}

TEST_F(ZmqCodecsTest, FrameHeaderGoesToBodyAndPixelsToBinary) {
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(sample_frame(), 99, body, binary));

    const auto message = parse<msg::VisionFrameGray8>(body);
    EXPECT_EQ(message.source(), "ir");
    EXPECT_EQ(message.session_id(), "session-a");
    EXPECT_EQ(message.frame_id(), 42U);
    EXPECT_EQ(message.seq(), 42U);  // 발행 번호(99)가 아니라 frame_id
    EXPECT_EQ(message.stride(), 4U);
    EXPECT_EQ(binary, sample_frame().pixels);
}

TEST_F(ZmqCodecsTest, FrameRoundTrip) {
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(sample_frame(), 0, body, binary));

    std::uint64_t sequence = 0;
    const auto frame = decode<perception_api::Frame>(body, binary, &sequence);

    ASSERT_TRUE(frame.has_value());
    const perception_api::Frame value = frame.value_or(perception_api::Frame{});
    EXPECT_EQ(value.source, "ir");
    EXPECT_EQ(value.session_id, "session-a");
    EXPECT_EQ(value.frame_id, 42U);
    EXPECT_EQ(value.unix_ns, 1'700'000'000'000'000'000);
    EXPECT_EQ(value.received_monotonic_ns, 5'000'000'000);
    EXPECT_EQ(value.width, 3U);
    EXPECT_EQ(value.height, 2U);
    EXPECT_EQ(value.stride, 4U);
    EXPECT_EQ(value.pixels, sample_frame().pixels);
    EXPECT_EQ(sequence, 42U);
}

TEST_F(ZmqCodecsTest, FrameWithPixelCountOtherThanStrideTimesHeightIsRejected) {
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(sample_frame(), 0, body, binary));
    binary.pop_back();

    EXPECT_FALSE(decode<perception_api::Frame>(body, binary).has_value());
}

TEST_F(ZmqCodecsTest, FrameWithStrideSmallerThanWidthIsRejected) {
    perception_api::Frame frame = sample_frame();
    frame.stride = 2;
    frame.pixels.resize(frame.expected_pixel_bytes());
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(frame, 0, body, binary));

    EXPECT_FALSE(decode<perception_api::Frame>(body, binary).has_value());
}

TEST_F(ZmqCodecsTest, ObservationsRoundTripWithIntegerPixelBoxes) {
    perception_api::Observations observations;
    observations.source = "ir";
    observations.session_id = "session-a";
    observations.frame_id = 42;
    observations.sample_monotonic_ns = 5'000'000'000;
    observations.model = "tank_v1";
    observations.image_width = 640;
    observations.image_height = 480;
    perception_api::Detection& detection = observations.detections.emplace_back();
    detection.classification = {0, "tank", 0.875};
    detection.box = {10.4, 20.6, 110.0, 220.0};
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(observations, 7, body, binary));
    EXPECT_TRUE(binary.empty());
    EXPECT_EQ(parse<msg::PerceptionObservations>(body).seq(), 7U);

    std::uint64_t sequence = 0;
    const auto decoded = decode<perception_api::Observations>(body, binary, &sequence);

    ASSERT_TRUE(decoded.has_value());
    const perception_api::Observations value = decoded.value_or(perception_api::Observations{});
    EXPECT_EQ(value.session_id.value_or(""), "session-a");
    EXPECT_EQ(value.frame_id, 42U);
    EXPECT_EQ(value.sample_monotonic_ns, 5'000'000'000);
    EXPECT_EQ(value.model, "tank_v1");
    EXPECT_EQ(value.image_width, 640U);
    ASSERT_EQ(value.detections.size(), 1U);
    EXPECT_EQ(value.detections[0].classification.label, "tank");
    EXPECT_DOUBLE_EQ(value.detections[0].classification.confidence, 0.875);
    EXPECT_DOUBLE_EQ(value.detections[0].box.x1, 10.0);  // 원본 영상의 정수 픽셀
    EXPECT_DOUBLE_EQ(value.detections[0].box.y1, 21.0);
    EXPECT_EQ(sequence, 7U);
}

TEST_F(ZmqCodecsTest, EmptySessionIdBecomesNoValue) {
    msg::PerceptionObservations observations;
    observations.set_frame_id(1);
    msg::TrackingTracks tracks;
    tracks.set_frame_id(1);

    const auto decoded_observations =
        decode<perception_api::Observations>(to_bytes(observations), Bytes{});
    const auto decoded_tracks = decode<perception_api::Tracks>(to_bytes(tracks), Bytes{});

    ASSERT_TRUE(decoded_observations.has_value());
    ASSERT_TRUE(decoded_tracks.has_value());
    EXPECT_FALSE(
        decoded_observations.value_or(perception_api::Observations{}).session_id.has_value());
    EXPECT_FALSE(decoded_tracks.value_or(perception_api::Tracks{}).session_id.has_value());
}

TEST_F(ZmqCodecsTest, MessagesWithBinaryOrBrokenBodyAreRejected) {
    msg::PerceptionObservations observations;
    const Bytes body = to_bytes(observations);
    const Bytes broken{0x0A, 0x05};  // 필드 1, 길이 5 인데 내용이 없음

    EXPECT_FALSE(decode<perception_api::Observations>(body, Bytes{1}).has_value());
    EXPECT_FALSE(decode<perception_api::Tracks>(body, Bytes{1}).has_value());
    EXPECT_FALSE(decode<perception_api::TrackingStatus>(body, Bytes{1}).has_value());
    EXPECT_FALSE(decode<perception_api::TrackingStatus>(broken, Bytes{}).has_value());
    EXPECT_FALSE(decode<perception_api::Observations>(broken, Bytes{}).has_value());
    EXPECT_FALSE(decode<perception_api::Frame>(broken, Bytes{}).has_value());
}

perception_api::Tracks sample_tracks() {
    perception_api::Tracks tracks;
    tracks.source = "ir";
    tracks.session_id = "session-a";
    tracks.frame_id = 42;
    tracks.fc_epoch = 3;
    tracks.generation = 5;
    tracks.sample_monotonic_ns = 5'000'000'000;
    tracks.image_width = 640;
    tracks.image_height = 480;
    tracks.ground_pose = perception_api::GroundPose{1.0, 2.0, -30.0, 0.1, 0.2, 0.3};
    perception_api::Track& track = tracks.tracks.emplace_back();
    track.track_id = 17;
    track.classification = {0, "tank", 0.75};
    track.box = {10.5, 20.5, 110.25, 220.25};
    track.state = perception_api::TrackState::Predicted;
    track.real_detection = false;
    track.last_real_detection_monotonic_ns = 4'900'000'000;
    track.ground = perception_api::GroundEstimate{100.0, 200.0, 1.0, 0.5, 2.0, 0.01};
    track.ground_anchor = perception_api::GroundAnchor{0.9, 0.0};
    tracks.tracks.emplace_back().track_id = 18;  // 지면 투영이 없는 트랙
    return tracks;
}

TEST_F(ZmqCodecsTest, TracksRoundTrip) {
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(sample_tracks(), 7, body, binary));

    const auto decoded = decode<perception_api::Tracks>(body, binary);

    ASSERT_TRUE(decoded.has_value());
    const perception_api::Tracks value = decoded.value_or(perception_api::Tracks{});
    EXPECT_EQ(value.session_id.value_or(""), "session-a");
    EXPECT_EQ(value.fc_epoch, 3U);
    EXPECT_EQ(value.generation, 5U);
    ASSERT_TRUE(value.ground_pose.has_value());
    EXPECT_DOUBLE_EQ(value.ground_pose.value_or(perception_api::GroundPose{}).down_m, -30.0);
    ASSERT_EQ(value.tracks.size(), 2U);
    const perception_api::Track& first = value.tracks[0];
    EXPECT_EQ(first.track_id, 17U);
    EXPECT_EQ(first.classification.label, "tank");
    EXPECT_DOUBLE_EQ(first.box.x2, 110.25);
    EXPECT_EQ(first.state, perception_api::TrackState::Predicted);
    EXPECT_EQ(first.last_real_detection_monotonic_ns, 4'900'000'000);
    ASSERT_TRUE(first.ground.has_value());
    EXPECT_DOUBLE_EQ(first.ground.value_or(perception_api::GroundEstimate{}).east_m, 200.0);
    ASSERT_TRUE(first.ground_anchor.has_value());
    EXPECT_DOUBLE_EQ(first.ground_anchor.value_or(perception_api::GroundAnchor{}).anchor_y_ratio,
                     0.9);
    EXPECT_FALSE(value.tracks[1].ground.has_value());
    EXPECT_FALSE(value.tracks[1].ground_anchor.has_value());
}

TEST_F(ZmqCodecsTest, TracksWithUnspecifiedTrackStateAreRejected) {
    msg::TrackingTracks message;
    message.add_tracks()->set_state(msg::TRACK_STATE_UNSPECIFIED);

    EXPECT_FALSE(decode<perception_api::Tracks>(to_bytes(message), Bytes{}).has_value());
}

TEST_F(ZmqCodecsTest, TrackingStatusRoundTripKeepsEveryFieldAndSequence) {
    perception_api::TrackingStatus status;
    status.source = "ir";
    status.published_monotonic_ns = 5'000'000'000;
    status.generation = 3;
    status.fc_fresh = true;
    status.cmc_valid = true;
    status.fps = 29.5;
    Bytes body;
    Bytes binary;
    ASSERT_TRUE(encode(status, 9, body, binary));
    EXPECT_TRUE(binary.empty());
    msg::TrackingStatus message;
    ASSERT_TRUE(message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size())));
    EXPECT_EQ(message.seq(), 9U);

    std::uint64_t sequence = 0;
    const perception_api::TrackingStatus value =
        decode<perception_api::TrackingStatus>(body, binary, &sequence)
            .value_or(perception_api::TrackingStatus{});

    EXPECT_EQ(sequence, 9U);
    EXPECT_EQ(value.source, "ir");
    EXPECT_EQ(value.published_monotonic_ns, 5'000'000'000);
    EXPECT_EQ(value.generation, 3U);
    EXPECT_TRUE(value.fc_fresh);
    EXPECT_TRUE(value.cmc_valid);
    EXPECT_DOUBLE_EQ(value.fps, 29.5);
}

}  // namespace
}  // namespace argus::perception_api_bus
