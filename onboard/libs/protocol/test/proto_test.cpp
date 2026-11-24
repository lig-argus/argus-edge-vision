#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "argus_msg/control_setpoint.pb.h"
#include "argus_msg/perception_observations.pb.h"
#include "argus_msg/tracking_tracks.pb.h"

namespace argus::protocol {
namespace {

TEST(ProtoTest, SerializedMessageParsesToSameValues) {
    msg::ControlSetpoint sent;
    sent.set_seq(7);
    sent.set_forward_m_s(1.5);
    sent.set_right_m_s(-0.25);
    sent.set_down_m_s(0.0);
    sent.set_yaw_rate_rad_s(0.1745);
    sent.set_valid_until_monotonic_ns(123'456'789'000);

    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));

    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.seq(), 7U);
    EXPECT_DOUBLE_EQ(received.forward_m_s(), 1.5);
    EXPECT_DOUBLE_EQ(received.right_m_s(), -0.25);
    EXPECT_DOUBLE_EQ(received.down_m_s(), 0.0);
    EXPECT_DOUBLE_EQ(received.yaw_rate_rad_s(), 0.1745);
    EXPECT_EQ(received.valid_until_monotonic_ns(), 123'456'789'000);
}

// 발행 쪽이 필드를 추가해도(같은 버전, process_interfaces.md 3장) 받는 쪽은 모르는 필드를 무시하고
// 파싱한다
TEST(ProtoTest, UnknownFieldIsIgnored) {
    msg::ControlSetpoint sent;
    sent.set_seq(1);
    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));
    // 필드 번호 100, wire type varint(0) → tag 0x0320, 값 5
    bytes.push_back(static_cast<char>(0xA0));
    bytes.push_back(static_cast<char>(0x06));
    bytes.push_back(static_cast<char>(0x05));

    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.seq(), 1U);
}

// 보내지 않은 필드는 기본값(0)으로 읽힌다
TEST(ProtoTest, MissingFieldReadsAsDefault) {
    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(std::string{}));
    EXPECT_EQ(received.seq(), 0U);
    EXPECT_DOUBLE_EQ(received.forward_m_s(), 0.0);
    EXPECT_EQ(received.valid_until_monotonic_ns(), 0);
}

// 탐지 한 건은 detection.proto 의 Detection 이다 (process_interfaces.md 5.1.4)
TEST(ProtoTest, PerceptionObservationsParsesToSameValues) {
    msg::PerceptionObservations sent;
    sent.set_source("ir");
    sent.set_session_id("session-a");
    sent.set_frame_id(42);
    sent.set_seq(7);
    sent.set_image_width(640);
    sent.set_image_height(480);
    sent.set_model("model-a");
    sent.set_inference_ms(12.5);
    sent.set_received_unix_ns(1'700'000'000'123'456'789);
    sent.set_received_monotonic_ns(987'654'321);
    sent.set_published_monotonic_ns(987'700'000);
    msg::Detection* detection = sent.add_detections();
    detection->set_class_id(2);
    detection->set_label("car");
    detection->set_confidence(0.75);
    detection->set_x1(10);
    detection->set_y1(20);
    detection->set_x2(110);
    detection->set_y2(80);

    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));

    msg::PerceptionObservations received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.source(), "ir");
    EXPECT_EQ(received.session_id(), "session-a");
    EXPECT_EQ(received.frame_id(), 42U);
    EXPECT_EQ(received.seq(), 7U);
    EXPECT_EQ(received.image_width(), 640U);
    EXPECT_EQ(received.image_height(), 480U);
    EXPECT_EQ(received.model(), "model-a");
    EXPECT_DOUBLE_EQ(received.inference_ms(), 12.5);
    EXPECT_EQ(received.received_unix_ns(), 1'700'000'000'123'456'789);
    EXPECT_EQ(received.received_monotonic_ns(), 987'654'321);
    EXPECT_EQ(received.published_monotonic_ns(), 987'700'000);
    // 카메라가 제공하지 않은 시각은 없음으로 구분된다
    EXPECT_FALSE(received.has_source_receive_monotonic_ns());
    EXPECT_FALSE(received.has_exposure_monotonic_ns());
    ASSERT_EQ(received.detections_size(), 1);
    EXPECT_EQ(received.detections(0).class_id(), 2);
    EXPECT_EQ(received.detections(0).label(), "car");
    EXPECT_DOUBLE_EQ(received.detections(0).confidence(), 0.75);
    EXPECT_EQ(received.detections(0).x1(), 10);
    EXPECT_EQ(received.detections(0).y1(), 20);
    EXPECT_EQ(received.detections(0).x2(), 110);
    EXPECT_EQ(received.detections(0).y2(), 80);
}

// 값이 없을 수 있는 필드(ground_pose, ground, ground_anchor)는 has_ 로 구분된다
// (process_interfaces.md 3장, 5.1.1)
TEST(ProtoTest, TrackingTracksParsesToSameValues) {
    msg::TrackingTracks sent;
    sent.set_source("ir");
    sent.set_session_id("session-a");
    sent.set_frame_id(42);
    sent.set_fc_epoch(3);
    sent.set_generation(5);
    sent.mutable_ground_pose()->set_yaw_rad(1.25);
    msg::TrackingTracks::Track* track = sent.add_tracks();
    track->set_track_id(16'777'215);
    track->mutable_classification()->set_class_id(2);
    track->mutable_box()->set_x2(110.5F);
    track->set_state(msg::TRACK_STATE_CONFIRMED);
    track->set_real_detection(true);
    track->mutable_ground()->set_north_m(12.5);
    track->mutable_ground()->set_alignment_s(0.004);

    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));

    msg::TrackingTracks received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.session_id(), "session-a");
    EXPECT_EQ(received.frame_id(), 42U);
    EXPECT_EQ(received.fc_epoch(), 3U);
    EXPECT_EQ(received.generation(), 5U);
    ASSERT_TRUE(received.has_ground_pose());
    EXPECT_DOUBLE_EQ(received.ground_pose().yaw_rad(), 1.25);
    ASSERT_EQ(received.tracks_size(), 1);
    EXPECT_EQ(received.tracks(0).track_id(), 16'777'215U);
    EXPECT_EQ(received.tracks(0).classification().class_id(), 2);
    EXPECT_FLOAT_EQ(received.tracks(0).box().x2(), 110.5F);
    EXPECT_EQ(received.tracks(0).state(), msg::TRACK_STATE_CONFIRMED);
    EXPECT_TRUE(received.tracks(0).real_detection());
    ASSERT_TRUE(received.tracks(0).has_ground());
    EXPECT_DOUBLE_EQ(received.tracks(0).ground().north_m(), 12.5);
    EXPECT_DOUBLE_EQ(received.tracks(0).ground().alignment_s(), 0.004);
    // 지면 투영이 없는 값은 없음으로 구분된다
    EXPECT_FALSE(received.tracks(0).has_ground_anchor());
}

}  // namespace
}  // namespace argus::protocol
