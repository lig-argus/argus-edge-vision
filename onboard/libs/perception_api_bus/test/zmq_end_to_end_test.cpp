// 구독자를 ZmqBus 와 zmq_codecs() 에 붙여 프레임(헤더와 픽셀)과 추적 결과가 multipart 로 오가는지
// 확인한다. 발행은 bus::make_publisher 로 한다 (이 라이브러리는 발행 어댑터를 두지 않는다)
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <zmq.hpp>

#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/message_types.hpp"
#include "argus/perception_api_bus/subscribers.hpp"
#include "argus/perception_api_bus/zmq_codecs.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {
namespace {

using std::chrono::milliseconds;

constexpr milliseconds kBudget{1000};
constexpr std::int32_t kDepth = 4;

class ZmqEndToEndTest : public ::testing::Test {
protected:
    // 테스트마다 다른 inproc 엔드포인트를 만든다 (같은 context 안에서 이름이 겹치지 않게)
    std::string endpoint() const {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        return std::string("inproc://") + info->name();
    }

    // condition 이 참이 될 때까지 publish 와 이벤트 처리를 반복한다. 연결이 끝나기 전의 메시지는
    // 도착하지 않으므로 받을 때까지 다시 보낸다
    bool publish_until(const std::function<void()>& publish,
                       const std::function<bool()>& condition) {
        for (std::int32_t i = 0; i < 400 && !condition(); ++i) {
            publish();
            poller_.process_events(milliseconds{5}, kBudget);
        }
        return condition();
    }

    zmq::context_t context_;
    core_testing::FakeClock clock_;
    bus_zmq::ZmqPoller poller_{clock_};
};

TEST_F(ZmqEndToEndTest, FrameArrivesWithHeaderAndPixels) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kVisionFrameGray8), endpoint()}}, zmq_codecs());
    const auto publisher = bus::make_publisher<perception_api::Frame>(
        bus, protocol::kVisionFrameGray8, kVisionFrameGray8Type, kDepth);
    const auto subscriber = create_frame_subscriber(bus);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscriber, nullptr);
    std::vector<perception_api::Frame> received;
    const core::Subscription subscription =
        subscriber->subscribe([&](const perception_api::Frame& frame,
                                  const bus::MessageInfo& /*info*/) { received.push_back(frame); });
    perception_api::Frame frame;
    frame.source = "ir";
    frame.session_id = "session-a";
    frame.frame_id = 42;
    frame.width = 640;
    frame.height = 480;
    frame.stride = 640;
    frame.pixels.assign(frame.expected_pixel_bytes(), 0x7F);

    ASSERT_TRUE(
        publish_until([&] { publisher->publish(frame); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].frame_id, 42U);
    EXPECT_EQ(received[0].session_id, "session-a");
    EXPECT_EQ(received[0].pixels, frame.pixels);
}

TEST_F(ZmqEndToEndTest, TracksArriveFromTrackingTracksTopic) {
    bus_zmq::ZmqBus bus(context_, poller_, clock_,
                        {{std::string(protocol::kTrackingTracks), endpoint()}}, zmq_codecs());
    const auto publisher = bus::make_publisher<perception_api::Tracks>(
        bus, protocol::kTrackingTracks, kTrackingTracksType, kDepth);
    const auto subscriber = create_tracks_subscriber(bus);
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
    tracks.tracks.emplace_back().track_id = 17;

    ASSERT_TRUE(
        publish_until([&] { publisher->publish(tracks); }, [&] { return !received.empty(); }));

    EXPECT_EQ(received[0].frame_id, 42U);
    ASSERT_EQ(received[0].tracks.size(), 1U);
    EXPECT_EQ(received[0].tracks[0].track_id, 17U);
}

}  // namespace
}  // namespace argus::perception_api_bus
