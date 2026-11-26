#include "argus/mavlink_gateway/track_start_handler.hpp"

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/latest_tracks.hpp"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/tracking_status_source.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api/vision_status_source.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "follow_command_handler_rig.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using testing::kMs;
using testing::kSecond;

template <typename Received>
class FakeStatusSource {
public:
    std::optional<Received> find(const std::string& source) const {
        const auto found = by_source.find(source);
        if (found == by_source.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::map<std::string, Received> by_source;
};

class FakeTrackingStatusSource : public perception_api::ITrackingStatusSource,
                                 public FakeStatusSource<perception_api::ReceivedTrackingStatus> {
public:
    std::optional<perception_api::ReceivedTrackingStatus> latest(
        const std::string& source) const override {
        return find(source);
    }
};

class FakeVisionStatusSource : public perception_api::IVisionStatusSource,
                               public FakeStatusSource<perception_api::ReceivedVisionStatus> {
public:
    std::optional<perception_api::ReceivedVisionStatus> latest(
        const std::string& source) const override {
        return find(source);
    }
};

class FakeLatestTracks : public ILatestTracks {
public:
    std::optional<perception_api::Tracks> latest(const std::string& source) const override {
        const auto found = by_source.find(source);
        if (found == by_source.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::map<std::string, perception_api::Tracks> by_source;
};

// track_id 7, class_id 2, generation 3
mavlink_command_long_t track_start(float sensor = ARGUS_SENSOR_IR, float track_id = 7) {
    mavlink_command_long_t command{};
    command.command = MAV_CMD_ARGUS_TRACK_START;
    command.param1 = track_id;
    command.param2 = sensor;
    command.param3 = 2;
    command.param4 = 3;
    command.param5 = test_messages::kNan;
    command.param6 = test_messages::kNan;
    command.param7 = 1;
    return command;
}

class TrackStartHandlerTest : public ::testing::Test {
protected:
    TrackStartHandlerTest() {
        // camera-ir: tracking/status 를 받았고 표적 7 이 최신 트랙에 있음
        perception_api::ReceivedTrackingStatus tracking;
        tracking.arrived_monotonic_ns = rig_.clock.now_monotonic_ns();
        tracking_.by_source["camera-ir"] = tracking;
        perception_api::Tracks tracks;
        tracks.source = "camera-ir";
        tracks.generation = 3;
        tracks.sample_monotonic_ns = rig_.clock.now_monotonic_ns() - 50 * kMs;
        perception_api::Track track;
        track.track_id = 7;
        track.classification.class_id = 2;
        track.classification.label = "vehicle";
        tracks.tracks.push_back(track);
        tracks_.by_source["camera-ir"] = tracks;
    }

    MAV_RESULT last_result() const { return rig_.acks.back().result; }

    testing::FollowCommandHandlerRig rig_;
    SensorSourceMap sensors_{{{"camera-ir", ARGUS_SENSOR_IR, true}}};
    FakeTrackingStatusSource tracking_;
    FakeVisionStatusSource vision_;
    FakeLatestTracks tracks_;
    TrackStartHandler handler_{TrackStartHandlerInputs{rig_.clock, sensors_, tracking_, vision_,
                                                       tracks_, rig_.headers, rig_.follow_control,
                                                       rig_.tracker}};
};

TEST_F(TrackStartHandlerTest, RulesAreNumberedControlledStart) {
    EXPECT_EQ(handler_.command(), MAV_CMD_ARGUS_TRACK_START);
    EXPECT_TRUE(TrackStartHandler::kRules.numbered);
    EXPECT_TRUE(TrackStartHandler::kRules.requires_control);
    EXPECT_EQ(TrackStartHandler::kRules.kind, CommandKind::Start);
}

TEST_F(TrackStartHandlerTest, ValidTargetSendsInProgressAndTrackStartWithLabel) {
    rig_.handle(handler_, track_start());

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(rig_.acks[0].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(rig_.acks[0].progress, kProgressUnknown);
    ASSERT_EQ(rig_.follow_control.commands().size(), 1U);
    const auto& record = rig_.follow_control.commands()[0];
    EXPECT_EQ(record.kind, vehicle_api_testing::FollowCommandKind::TrackStart);
    ASSERT_TRUE(record.target.has_value());
    const vehicle_api::TrackTarget target = record.target.value_or(vehicle_api::TrackTarget{});
    EXPECT_EQ(target.sensor, vehicle_api::Sensor::Ir);
    EXPECT_EQ(target.track_id, 7U);
    EXPECT_EQ(target.class_id, 2);
    EXPECT_EQ(target.generation, 3U);
    EXPECT_EQ(target.label, "vehicle");
    EXPECT_EQ(record.header.id.session_id, "session-g");
}

TEST_F(TrackStartHandlerTest, OutOfRangeParamIsDeniedWithoutRequest) {
    rig_.handle(handler_, track_start(ARGUS_SENSOR_IR, 0));

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(last_result(), MAV_RESULT_DENIED);
    EXPECT_FALSE(rig_.acks[0].reason.empty());
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

TEST_F(TrackStartHandlerTest, UnconfiguredSensorIsTemporarilyRejected) {
    rig_.handle(handler_, track_start(ARGUS_SENSOR_EO));

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(last_result(), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

TEST_F(TrackStartHandlerTest, NotFollowableIsTemporarilyRejected) {
    rig_.clock.advance_ns(3 * kSecond + 1);  // tracking/status 두절
    tracks_.by_source["camera-ir"].sample_monotonic_ns = rig_.clock.now_monotonic_ns();

    rig_.handle(handler_, track_start());

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(last_result(), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_TRUE(rig_.acks[0].reason.empty());
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

TEST_F(TrackStartHandlerTest, StaleTracksAreDenied) {
    rig_.clock.advance_ns(250 * kMs);

    rig_.handle(handler_, track_start());

    ASSERT_EQ(rig_.acks.size(), 1U);
    EXPECT_EQ(last_result(), MAV_RESULT_DENIED);
    EXPECT_TRUE(rig_.follow_control.commands().empty());
}

}  // namespace
}  // namespace argus::mavlink_gateway
