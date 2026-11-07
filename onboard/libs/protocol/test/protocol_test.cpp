#include <gtest/gtest.h>

#include <array>
#include <set>
#include <string_view>

#include "argus/protocol/topics.hpp"
#include "argus/protocol/zmq_endpoints.hpp"

namespace argus::protocol {
namespace {

constexpr std::array kAllNames = {
    kVisionFrameGray8, kVisionOverlayJpeg, kVisionNpuInputJpeg, kPerceptionObservations,
    kTrackingTracks,   kTrackingStatus,    kFcAttitude,         kFcState,
    kFcModeEvent,      kControlSetpoint,   kFollowStatus,       kControlModeRequest,
    kFollowCommand,
};

constexpr std::array kAllEndpoints = {
    kDefaultFcStateEndpoint,      kDefaultControlModeRequestEndpoint,
    kDefaultControlStateEndpoint, kDefaultFollowCommandEndpoint,
    kDefaultVisionEndpoint,       kDefaultTrackingEndpoint,
};

constexpr std::array kAllQueueDepths = {
    kVisionFrameGray8QueueDepth,   kVisionOverlayJpegQueueDepth,
    kVisionNpuInputJpegQueueDepth, kPerceptionObservationsQueueDepth,
    kTrackingTracksQueueDepth,     kTrackingStatusQueueDepth,
    kFcAttitudeQueueDepth,         kFcStateQueueDepth,
    kFcModeEventQueueDepth,        kControlSetpointQueueDepth,
    kFollowStatusQueueDepth,
};

TEST(ProtocolTest, NamesAreUnique) {
    const std::set<std::string_view> unique(kAllNames.begin(), kAllNames.end());
    EXPECT_EQ(unique.size(), kAllNames.size());
}

// ZeroMQ 구현(ZmqBus)의 구독 필터는 앞부분 일치라, 한 이름이 다른 이름의 앞부분이면 원하지 않는
// 메시지까지 받는다
TEST(ProtocolTest, NoNameIsPrefixOfAnother) {
    for (const auto a : kAllNames) {
        for (const auto b : kAllNames) {
            if (a != b) {
                EXPECT_NE(b.substr(0, a.size()), a) << a << " 가 " << b << " 의 앞부분";
            }
        }
    }
}

TEST(ProtocolTest, NamesHaveGroupAndName) {
    for (const auto name : kAllNames) {
        const auto slash = name.find('/');
        EXPECT_NE(slash, std::string_view::npos) << name;
        EXPECT_GT(slash, 0U) << name;
        EXPECT_NE(name.back(), '/') << name;
    }
}

TEST(ProtocolTest, EndpointsAreUniqueLocalTcp) {
    const std::set<std::string_view> unique(kAllEndpoints.begin(), kAllEndpoints.end());
    EXPECT_EQ(unique.size(), kAllEndpoints.size());
    for (const auto endpoint : kAllEndpoints) {
        EXPECT_EQ(endpoint.substr(0, 16), "tcp://127.0.0.1:") << endpoint;
    }
}

TEST(ProtocolTest, QueueDepthsArePositive) {
    for (const auto depth : kAllQueueDepths) {
        EXPECT_GE(depth, 1);
    }
}

}  // namespace
}  // namespace argus::protocol
