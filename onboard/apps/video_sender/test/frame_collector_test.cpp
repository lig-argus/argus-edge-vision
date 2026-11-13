#include "argus/video_sender/frame_collector.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/collected_frame.hpp"
#include "fake_frame_source.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Frame;
using perception_api::Observations;
using perception_api::Tracks;

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kMaxWaitNs = 100 * kNsPerMs;
constexpr std::size_t kMaxPending = 3;
constexpr std::string_view kSource = "camera-ir";
constexpr std::string_view kSession = "session-1";

Frame make_frame(std::uint64_t frame_id, std::string_view session_id = kSession,
                 std::string_view source = kSource) {
    Frame frame;
    frame.source = std::string{source};
    frame.session_id = std::string{session_id};
    frame.frame_id = frame_id;
    return frame;
}

Observations make_observations(std::uint64_t frame_id, std::string_view source = kSource) {
    Observations observations;
    observations.source = std::string{source};
    observations.frame_id = frame_id;
    return observations;
}

Tracks make_tracks(std::uint64_t frame_id, std::optional<std::string_view> session_id = kSession,
                   std::string_view source = kSource) {
    Tracks tracks;
    tracks.source = std::string{source};
    if (session_id.has_value()) {
        tracks.session_id = std::string{session_id.value()};
    }
    tracks.frame_id = frame_id;
    return tracks;
}

class FrameCollectorTest : public ::testing::Test {
protected:
    static FrameCollectorConfig config() {
        FrameCollectorConfig config;
        config.max_wait = std::chrono::milliseconds{100};
        config.max_pending = kMaxPending;
        return config;
    }

    std::vector<std::uint64_t> collected_frame_ids() const {
        std::vector<std::uint64_t> ids;
        ids.reserve(collected_.size());
        for (const CollectedFrame& collected : collected_) {
            ids.push_back(collected.frame.frame_id);
        }
        return ids;
    }

    core_testing::FakeClock clock_;
    FrameCollector collector_{clock_, config()};
    std::vector<CollectedFrame> collected_;
    core::Subscription subscription_ = collector_.subscribe(
        [this](const CollectedFrame& collected) { collected_.push_back(collected); });
};

TEST_F(FrameCollectorTest, PublishesImmediatelyWhenFrameArrivesFirst) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));
    EXPECT_TRUE(collected_.empty());

    collector_.on_tracks(make_tracks(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.frame_id, 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, PublishesImmediatelyWhenFrameArrivesLast) {
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));
    EXPECT_TRUE(collected_.empty());

    collector_.on_frame(make_frame(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, PublishesImmediatelyWhenFrameArrivesBetweenResults) {
    collector_.on_tracks(make_tracks(1));
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, DoesNotPublishBeforeMaxWait) {
    collector_.on_frame(make_frame(1));

    clock_.advance_ns(kMaxWaitNs - 1);
    collector_.process_timeouts();

    EXPECT_TRUE(collected_.empty());
}

TEST_F(FrameCollectorTest, PublishesFrameAloneAfterMaxWait) {
    collector_.on_frame(make_frame(1));

    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.frame_id, 1U);
    EXPECT_FALSE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, PublishesFrameWithObservationsOnlyAfterMaxWait) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));

    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, MaxWaitStartsAtFirstMessageOfTheFrame) {
    collector_.on_observations(make_observations(1));
    clock_.advance_ns(kMaxWaitNs - 1);
    collector_.on_frame(make_frame(1));
    EXPECT_TRUE(collected_.empty());

    clock_.advance_ns(1);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
}

TEST_F(FrameCollectorTest, OnMessageAlsoHandlesExpiredFrames) {
    collector_.on_frame(make_frame(1));
    clock_.advance_ns(kMaxWaitNs);

    collector_.on_frame(make_frame(2));  // process_timeouts() 호출 없이

    EXPECT_EQ(collected_frame_ids(), (std::vector<std::uint64_t>{1}));
}

TEST_F(FrameCollectorTest, DiscardsResultsWithoutFrameAfterMaxWait) {
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();
    EXPECT_TRUE(collected_.empty());

    // 폐기된 뒤 도착한 프레임은 탐지 결과와 추적 결과 없이 출력된다
    collector_.on_frame(make_frame(1));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_FALSE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, PublishesOldestFrameWhenPendingExceedsLimit) {
    for (std::uint64_t frame_id = 1; frame_id <= kMaxPending; ++frame_id) {
        collector_.on_frame(make_frame(frame_id));
    }
    EXPECT_TRUE(collected_.empty());

    collector_.on_frame(make_frame(kMaxPending + 1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.frame_id, 1U);
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, DiscardsOldestResultsWithoutFrameWhenPendingExceedsLimit) {
    for (std::uint64_t frame_id = 1; frame_id <= kMaxPending + 1; ++frame_id) {
        collector_.on_observations(make_observations(frame_id));
    }
    EXPECT_TRUE(collected_.empty());

    // frame_id 1 의 탐지 결과는 폐기되었고 2 의 탐지 결과는 남아 있다
    collector_.on_tracks(make_tracks(2));
    collector_.on_frame(make_frame(2));
    collector_.on_tracks(make_tracks(1));
    collector_.on_frame(make_frame(1));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 2U);
    EXPECT_EQ(collected_[0].frame.frame_id, 2U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_EQ(collected_[1].frame.frame_id, 1U);
    EXPECT_FALSE(collected_[1].observations.has_value());
}

TEST_F(FrameCollectorTest, PublishesOlderFrameOfSameSourceBeforeCompleteFrame) {
    collector_.on_frame(make_frame(1));
    collector_.on_frame(make_frame(2));

    collector_.on_observations(make_observations(2));
    collector_.on_tracks(make_tracks(2));

    ASSERT_EQ(collected_frame_ids(), (std::vector<std::uint64_t>{1, 2}));
    EXPECT_FALSE(collected_[0].tracks.has_value());
    EXPECT_TRUE(collected_[1].tracks.has_value());
}

TEST_F(FrameCollectorTest, CompleteFrameDoesNotReleaseOtherSource) {
    collector_.on_frame(make_frame(1, kSession, "camera-eo"));
    collector_.on_frame(make_frame(1));

    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.source, kSource);
}

TEST_F(FrameCollectorTest, DoesNotJoinSameFrameIdOfDifferentSource) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1, "camera-eo"));
    collector_.on_tracks(make_tracks(1, kSession, "camera-eo"));
    EXPECT_TRUE(collected_.empty());

    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_FALSE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, JoinsTracksWithoutSessionIdBySourceAndFrameId) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1, std::nullopt));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, JoinsFrameWithEmptySessionIdBySourceAndFrameId) {
    collector_.on_tracks(make_tracks(1));
    collector_.on_observations(make_observations(1));
    collector_.on_frame(make_frame(1, ""));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, DoesNotJoinTracksOfDifferentSessionId) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1, "session-0"));
    EXPECT_TRUE(collected_.empty());

    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_TRUE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, FrameWithNewSessionIdDiscardsPendingOfThatSource) {
    collector_.on_frame(make_frame(7));
    collector_.on_observations(make_observations(1));  // session_id 없음
    collector_.on_frame(make_frame(1, kSession, "camera-eo"));

    collector_.on_frame(make_frame(1, "session-2"));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    // 이전 session_id 의 프레임 7 은 출력되지 않고, 신규 session_id 의 프레임 1 은 그 전에 도착한
    // 탐지 결과 없이 출력된다
    ASSERT_EQ(collected_.size(), 2U);
    EXPECT_EQ(collected_[0].frame.source, "camera-eo");
    EXPECT_EQ(collected_[1].frame.session_id, "session-2");
    EXPECT_FALSE(collected_[1].observations.has_value());
}

TEST_F(FrameCollectorTest, FrameWithNewSessionIdKeepsResultsOfNewSession) {
    collector_.on_frame(make_frame(7));
    collector_.on_tracks(make_tracks(1, "session-2"));

    collector_.on_frame(make_frame(1, "session-2"));
    collector_.on_observations(make_observations(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.session_id, "session-2");
    EXPECT_TRUE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, OnSessionChangeDiscardsAllPending) {
    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(2));
    collector_.on_tracks(make_tracks(2));

    collector_.on_session_change();
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();
    EXPECT_TRUE(collected_.empty());

    collector_.on_frame(make_frame(2));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_FALSE(collected_[0].observations.has_value());
    EXPECT_FALSE(collected_[0].tracks.has_value());
}

TEST_F(FrameCollectorTest, EveryHandlerReceivesCollectedFrame) {
    int second_handler_calls = 0;
    const core::Subscription second =
        collector_.subscribe([&](const CollectedFrame&) { ++second_handler_calls; });

    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));

    EXPECT_EQ(collected_.size(), 1U);
    EXPECT_EQ(second_handler_calls, 1);
}

TEST_F(FrameCollectorTest, ReleasedSubscriptionStopsDelivery) {
    subscription_.reset();

    collector_.on_frame(make_frame(1));
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));

    EXPECT_TRUE(collected_.empty());
}

TEST_F(FrameCollectorTest, HandlerMayCallCollectorAgain) {
    const core::Subscription reentrant = collector_.subscribe([this](const CollectedFrame& c) {
        if (c.frame.frame_id == 1) {
            collector_.on_frame(make_frame(2));
        }
    });

    collector_.on_frame(make_frame(1));
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();
    clock_.advance_ns(kMaxWaitNs);
    collector_.process_timeouts();

    EXPECT_EQ(collected_frame_ids(), (std::vector<std::uint64_t>{1, 2}));
}

TEST_F(FrameCollectorTest, MaxPendingBelowOneIsHandledAsOne) {
    FrameCollectorConfig config;
    config.max_pending = 0;
    FrameCollector collector{clock_, config};
    std::vector<std::uint64_t> ids;
    const core::Subscription subscription = collector.subscribe(
        [&](const CollectedFrame& collected) { ids.push_back(collected.frame.frame_id); });

    collector.on_frame(make_frame(1));
    EXPECT_TRUE(ids.empty());

    collector.on_frame(make_frame(2));
    EXPECT_EQ(ids, (std::vector<std::uint64_t>{1}));
}

TEST(FrameCollectorConfigTest, DefaultsAre100MsAnd8) {
    const FrameCollectorConfig config;

    EXPECT_EQ(config.max_wait, std::chrono::milliseconds{100});
    EXPECT_EQ(config.max_pending, 8U);
}

// IFrameSource 의 handler 에서 on_frame() 을 호출하는 연결 (VideoSender 가 하는 일)
TEST_F(FrameCollectorTest, CollectsFramesDeliveredByFrameSource) {
    core_testing::FakePoller poller{clock_};
    FakeFrameSource source{poller};
    const core::Subscription frames =
        source.subscribe([this](const Frame& frame) { collector_.on_frame(frame); });

    source.simulate_frame_received(make_frame(1));
    poller.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    collector_.on_observations(make_observations(1));
    collector_.on_tracks(make_tracks(1));

    ASSERT_EQ(collected_.size(), 1U);
    EXPECT_EQ(collected_[0].frame.frame_id, 1U);
}

}  // namespace
}  // namespace argus::video_sender
