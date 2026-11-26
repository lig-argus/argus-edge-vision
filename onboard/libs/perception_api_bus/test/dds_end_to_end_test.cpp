// 구독자를 DdsBus 와 dds_types() 에 붙여, 프레임(헤더와 픽셀), 탐지 결과, 추적 결과가 IDL 타입
// 토픽으로 오가는지 확인한다. 발행은 bus::make_publisher 로 한다 (이 라이브러리는 발행 어댑터를
// 두지 않는다). 보내는 쪽과 받는 쪽은 서로 다른 DdsBus(DomainParticipant)다
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "argus/bus/typed.hpp"
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/core/poll_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/bus_tracking_status_subscriber.hpp"
#include "argus/perception_api_bus/bus_vision_status_subscriber.hpp"
#include "argus/perception_api_bus/dds_types.hpp"
#include "argus/perception_api_bus/message_types.hpp"
#include "argus/perception_api_bus/subscribers.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {
namespace {

using std::chrono::milliseconds;

// 같은 컴퓨터의 다른 프로그램, 다른 DDS 테스트와 섞이지 않도록 기본값(0)이 아닌 domain 을 사용한다
constexpr std::uint32_t kTestDomain = 79;
constexpr std::int32_t kDepth = 4;

class DdsEndToEndTest : public ::testing::Test {
protected:
    void SetUp() override {
        sender_ = bus_dds::DdsBus::create(poller_, poller_, clock_, kTestDomain, dds_types());
        receiver_ = bus_dds::DdsBus::create(poller_, poller_, clock_, kTestDomain, dds_types());
        ASSERT_NE(sender_, nullptr);
        ASSERT_NE(receiver_, nullptr);
    }

    // condition 이 참이 될 때까지 step 과 이벤트 처리를 반복한다 (최대 약 4 초). DDS 의 발견이
    // 끝나기 전에 보낸 표본은 전달되지 않으므로, 보내는 동작을 step 에 넣어 받을 때까지 다시 보낸다
    bool pump_until(const std::function<void()>& step, const std::function<bool()>& condition) {
        constexpr std::int32_t kMaxRounds = 400;
        for (std::int32_t round = 0; round < kMaxRounds && !condition(); ++round) {
            step();
            poller_.process_events(milliseconds{10}, milliseconds{50});
        }
        return condition();
    }

    core::SteadyClock clock_;
    core::PollPoller poller_{clock_};
    std::unique_ptr<bus_dds::DdsBus> sender_;
    std::unique_ptr<bus_dds::DdsBus> receiver_;
};

TEST_F(DdsEndToEndTest, FrameArrivesWithHeaderAndPixels) {
    const auto publisher = bus::make_publisher<perception_api::Frame>(
        *sender_, protocol::kVisionFrameGray8, kVisionFrameGray8Type, kDepth);
    const auto subscriber = create_frame_subscriber(*receiver_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::Frame> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const perception_api::Frame& frame,
                                  const bus::MessageInfo& /*info*/) { received.push_back(frame); });
    // 640×480 프레임 (실제 크기)
    perception_api::Frame frame;
    frame.source = "ir";
    frame.session_id = "session-a";
    frame.frame_id = 42;
    frame.width = 640;
    frame.height = 480;
    frame.stride = 640;
    frame.pixels.resize(frame.expected_pixel_bytes());
    for (std::size_t i = 0; i < frame.pixels.size(); ++i) {
        frame.pixels[i] = static_cast<std::uint8_t>(i % 251);
    }

    ASSERT_TRUE(pump_until([&] { publisher->publish(frame); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].source, "ir");
    EXPECT_EQ(received[0].session_id, "session-a");
    EXPECT_EQ(received[0].frame_id, 42U);
    EXPECT_EQ(received[0].width, 640U);
    EXPECT_EQ(received[0].pixels, frame.pixels);
}

TEST_F(DdsEndToEndTest, ObservationsArrive) {
    const auto publisher = bus::make_publisher<perception_api::Observations>(
        *sender_, protocol::kPerceptionObservations, kPerceptionObservationsType, kDepth);
    const auto subscriber = create_observations_subscriber(*receiver_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::Observations> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const perception_api::Observations& observations, const bus::MessageInfo& /*info*/) {
            received.push_back(observations);
        });
    perception_api::Observations observations;
    observations.source = "ir";
    observations.frame_id = 42;
    observations.detections.push_back(
        perception_api::Detection{perception_api::Classification{1, "person", 0.75},
                                  perception_api::PixelBox{10, 20, 30, 40}});

    ASSERT_TRUE(
        pump_until([&] { publisher->publish(observations); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].frame_id, 42U);
    ASSERT_EQ(received[0].detections.size(), 1U);
    EXPECT_EQ(received[0].detections[0].classification.label, "person");
    EXPECT_DOUBLE_EQ(received[0].detections[0].box.x2, 30.0);
}

TEST_F(DdsEndToEndTest, TracksArrive) {
    const auto publisher = bus::make_publisher<perception_api::Tracks>(
        *sender_, protocol::kTrackingTracks, kTrackingTracksType, kDepth);
    const auto subscriber = create_tracks_subscriber(*receiver_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::Tracks> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const perception_api::Tracks& tracks, const bus::MessageInfo& /*info*/) {
            received.push_back(tracks);
        });
    perception_api::Tracks tracks;
    tracks.source = "ir";
    tracks.frame_id = 42;
    tracks.generation = 2;
    perception_api::Track track;
    track.track_id = 7;
    track.box = perception_api::PixelBox{10.5, 20.5, 30.5, 40.5};
    tracks.tracks.push_back(track);

    ASSERT_TRUE(pump_until([&] { publisher->publish(tracks); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].generation, 2U);
    ASSERT_EQ(received[0].tracks.size(), 1U);
    EXPECT_EQ(received[0].tracks[0].track_id, 7U);
    EXPECT_FLOAT_EQ(static_cast<float>(received[0].tracks[0].box.x1), 10.5F);
}

TEST_F(DdsEndToEndTest, TrackingStatusArrives) {
    // 추적기의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
    const auto publisher = bus::make_publisher<perception_api::TrackingStatus>(
        *sender_, protocol::kTrackingStatus, kTrackingStatusType, kDepth);
    const auto subscriber = BusTrackingStatusSubscriber::create(*receiver_, clock_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::ReceivedTrackingStatus> received;
    const core::Subscription subscription = subscriber->subscribe(
        [&](const perception_api::ReceivedTrackingStatus& r) { received.push_back(r); });
    perception_api::TrackingStatus status;
    status.source = "ir";
    status.generation = 2;
    status.cmc_valid = true;
    const std::int64_t sent_after_ns = clock_.now_monotonic_ns();

    ASSERT_TRUE(pump_until([&] { publisher->publish(status); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].status.source, "ir");
    EXPECT_EQ(received[0].status.generation, 2U);
    EXPECT_TRUE(received[0].status.cmc_valid);
    EXPECT_GE(received[0].arrived_monotonic_ns, sent_after_ns);
}

TEST_F(DdsEndToEndTest, VisionStatusArrives) {
    // 탐지의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
    const auto publisher = bus::make_publisher<perception_api::VisionStatus>(
        *sender_, protocol::kVisionStatus, kVisionStatusType, kDepth);
    const auto subscriber = BusVisionStatusSubscriber::create(*receiver_, clock_);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    perception_api::VisionStatus status;
    status.source = "camera-ir";
    status.inference_ok = true;
    status.model_ver = 2;

    ASSERT_TRUE(pump_until([&] { publisher->publish(status); },
                           [&] { return subscriber->latest("camera-ir").has_value(); }));

    const perception_api::ReceivedVisionStatus received =
        subscriber->latest("camera-ir").value_or(perception_api::ReceivedVisionStatus{});
    EXPECT_TRUE(received.status.inference_ok);
    EXPECT_EQ(received.status.model_ver, 2U);
}

}  // namespace
}  // namespace argus::perception_api_bus
