#include "argus/mavlink_gateway/follow_command_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <variant>

#include "argus/mavlink.h"
#include "argus/perception_api/types.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kSecond = 1'000'000'000;
constexpr double kPi = 3.14159265358979323846;
constexpr float kDegToRad = static_cast<float>(kPi / 180.0);

mavlink_command_long_t command(float p1, float p2, float p3, float p4) {
    mavlink_command_long_t result{};
    result.param1 = p1;
    result.param2 = p2;
    result.param3 = p3;
    result.param4 = p4;
    result.param5 = kNan;
    result.param6 = kNan;
    result.param7 = 1;
    return result;
}

// track_id 7, IR, class_id 2, generation 3
mavlink_command_long_t track_start(float track_id = 7, float sensor = ARGUS_SENSOR_IR,
                                   float class_id = 2, float generation = 3) {
    return command(track_id, sensor, class_id, generation);
}

MAV_RESULT rejection_of(const std::variant<TrackStartParams, CommandRejection>& parsed) {
    const auto* rejection = std::get_if<CommandRejection>(&parsed);
    return rejection != nullptr ? rejection->result : MAV_RESULT_ACCEPTED;
}

TEST(ParseTrackStartTest, ConvertsParams) {
    const auto parsed = parse_track_start(track_start());

    const auto* params = std::get_if<TrackStartParams>(&parsed);
    ASSERT_NE(params, nullptr);
    EXPECT_EQ(params->track_id, 7U);
    EXPECT_EQ(params->sensor, ARGUS_SENSOR_IR);
    EXPECT_EQ(params->class_id, 2);
    EXPECT_EQ(params->generation, 3U);
}

TEST(ParseTrackStartTest, AcceptsIntegerBounds) {
    EXPECT_EQ(rejection_of(parse_track_start(track_start(1, ARGUS_SENSOR_EO, 0, 1))),
              MAV_RESULT_ACCEPTED);
    EXPECT_EQ(rejection_of(parse_track_start(
                  track_start(16'777'215, ARGUS_SENSOR_IR, 16'777'215, 16'777'215))),
              MAV_RESULT_ACCEPTED);
}

TEST(ParseTrackStartTest, DeniesOutOfRangeParams) {
    EXPECT_EQ(rejection_of(parse_track_start(track_start(0))), MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(16'777'216))), MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(1.5F))), MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(kNan))), MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(7, ARGUS_SENSOR_UNKNOWN))),
              MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(7, 3))), MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(7, ARGUS_SENSOR_IR, -1))),
              MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(7, ARGUS_SENSOR_IR, 2, 0))),
              MAV_RESULT_DENIED);
    EXPECT_EQ(rejection_of(parse_track_start(track_start(7, ARGUS_SENSOR_IR, 2, kNan))),
              MAV_RESULT_DENIED);
}

TEST(FindSensorTest, FindsConfiguredSensor) {
    const SensorSourceMap sensors({{"camera-ir", ARGUS_SENSOR_IR, true}});

    EXPECT_EQ(find_sensor(sensors, ARGUS_SENSOR_IR).value_or(SensorSource{}).source, "camera-ir");
    EXPECT_FALSE(find_sensor(sensors, ARGUS_SENSOR_EO).has_value());
}

class SelectTrackTargetTest : public ::testing::Test {
protected:
    SelectTrackTargetTest() {
        selection_.params = TrackStartParams{ARGUS_SENSOR_IR, 7, 2, 3};
        selection_.followable = true;
        selection_.received_ns = 100 * kSecond;
        tracks_.source = "camera-ir";
        tracks_.generation = 3;
        tracks_.sample_monotonic_ns = selection_.received_ns - 100 * kMs;
        perception_api::Track track;
        track.track_id = 7;
        track.classification.class_id = 2;
        track.classification.label = "person";
        track.state = perception_api::TrackState::Confirmed;
        tracks_.tracks.push_back(track);
    }

    perception_api::Track& track() { return tracks_.tracks.front(); }

    std::variant<vehicle_api::TrackTarget, CommandRejection> select() const {
        TrackSelection selection = selection_;
        if (has_tracks_) {
            selection.tracks = tracks_;
        }
        return select_track_target(selection);
    }

    // 거부 결과. 거부하지 않았으면 ACCEPTED
    MAV_RESULT result() const {
        const auto selected = select();
        const auto* rejected = std::get_if<CommandRejection>(&selected);
        return rejected != nullptr ? rejected->result : MAV_RESULT_ACCEPTED;
    }

    TrackSelection selection_;
    perception_api::Tracks tracks_;
    bool has_tracks_ = true;
};

TEST_F(SelectTrackTargetTest, ReturnsTargetWithLabel) {
    const auto selected = select();

    const auto* target = std::get_if<vehicle_api::TrackTarget>(&selected);
    ASSERT_NE(target, nullptr);
    EXPECT_EQ(target->sensor, vehicle_api::Sensor::Ir);
    EXPECT_EQ(target->generation, 3U);
    EXPECT_EQ(target->class_id, 2);
    EXPECT_EQ(target->track_id, 7U);
    EXPECT_EQ(target->label, "person");
}

TEST_F(SelectTrackTargetTest, NotFollowableIsTemporarilyRejected) {
    selection_.followable = false;

    EXPECT_EQ(result(), MAV_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(SelectTrackTargetTest, FrameAt250MsIsLatest) {
    tracks_.sample_monotonic_ns = selection_.received_ns - 250 * kMs;

    EXPECT_EQ(result(), MAV_RESULT_ACCEPTED);
}

TEST_F(SelectTrackTargetTest, FrameOlderThan250MsIsDenied) {
    tracks_.sample_monotonic_ns = selection_.received_ns - 250 * kMs - 1;

    EXPECT_EQ(result(), MAV_RESULT_DENIED);
}

TEST_F(SelectTrackTargetTest, NoTracksIsDenied) {
    has_tracks_ = false;

    EXPECT_EQ(result(), MAV_RESULT_DENIED);
}

TEST_F(SelectTrackTargetTest, GenerationMismatchIsDenied) {
    tracks_.generation = 4;

    EXPECT_EQ(result(), MAV_RESULT_DENIED);
}

TEST_F(SelectTrackTargetTest, PredictedTrackIsDenied) {
    track().state = perception_api::TrackState::Predicted;

    EXPECT_EQ(result(), MAV_RESULT_DENIED);
}

TEST_F(SelectTrackTargetTest, ClassMismatchIsDenied) {
    track().classification.class_id = 5;

    EXPECT_EQ(result(), MAV_RESULT_DENIED);
}

std::optional<vehicle_api::ScanRequest> scan_of(const mavlink_command_long_t& command) {
    const auto parsed = parse_scan_start(command);
    if (const auto* scan = std::get_if<vehicle_api::ScanRequest>(&parsed)) {
        return *scan;
    }
    return std::nullopt;
}

TEST(ParseScanStartTest, NanParamsUseDefaults) {
    const std::optional<vehicle_api::ScanRequest> parsed = scan_of(command(kNan, kNan, kNan, kNan));

    ASSERT_TRUE(parsed.has_value());
    const vehicle_api::ScanRequest scan = parsed.value_or(vehicle_api::ScanRequest{});
    EXPECT_DOUBLE_EQ(scan.yaw_range_rad.value_or(0.0), 2.0 * kPi);
    EXPECT_DOUBLE_EQ(scan.yaw_rate_rad_s.value_or(0.0), 10.0 * kPi / 180.0);
    EXPECT_DOUBLE_EQ(scan.timeout_s.value_or(0.0), 60.0);
}

TEST(ParseScanStartTest, AcceptsFloatUpperBounds) {
    const auto two_pi = static_cast<float>(2.0 * kPi);

    EXPECT_TRUE(scan_of(command(two_pi, 75.0F * kDegToRad, 10.0F, kNan)).has_value());
    EXPECT_TRUE(scan_of(command(1.0F, -75.0F * kDegToRad, 10.0F, kNan)).has_value());
}

TEST(ParseScanStartTest, DeniesOutOfRangeParams) {
    EXPECT_FALSE(scan_of(command(0.0F, kNan, kNan, kNan)).has_value());
    EXPECT_FALSE(scan_of(command(7.0F, kNan, kNan, kNan)).has_value());
    EXPECT_FALSE(scan_of(command(kNan, 0.0F, kNan, kNan)).has_value());
    EXPECT_FALSE(scan_of(command(kNan, 76.0F * kDegToRad, kNan, kNan)).has_value());
    EXPECT_FALSE(
        scan_of(command(kNan, kNan, std::numeric_limits<float>::infinity(), kNan)).has_value());
}

TEST(ParseScanStartTest, TimeoutMustExceedRotationTime) {
    EXPECT_FALSE(scan_of(command(2.0F, 1.0F, 2.0F, kNan)).has_value());
    EXPECT_TRUE(scan_of(command(2.0F, 1.0F, 2.5F, kNan)).has_value());
    // 기본값 2π 를 1 deg/s 로 돌면 360 s 이므로 기본 제한 시간 60 s 로는 범위 밖
    EXPECT_FALSE(scan_of(command(kNan, kDegToRad, kNan, kNan)).has_value());
}

TEST(ScanProgressPercentTest, ConvertsDegreesToPercentOfRange) {
    EXPECT_EQ(scan_progress_percent(90.0, 2.0 * kPi), 25);
    EXPECT_EQ(scan_progress_percent(0.0, 2.0 * kPi), 0);
    EXPECT_EQ(scan_progress_percent(400.0, 2.0 * kPi), 100);
    EXPECT_EQ(scan_progress_percent(-10.0, 2.0 * kPi), 0);
}

std::optional<vehicle_api::ReceivedFollowStatus> status(vehicle_api::MissionState state,
                                                        std::int64_t arrived_ns) {
    vehicle_api::ReceivedFollowStatus received;
    received.status.mission_state = state;
    received.arrived_monotonic_ns = arrived_ns;
    return received;
}

TEST(FollowStopNeededTest, StopIsNeededOutsideIdleAndHoldWait) {
    const std::int64_t now = 100 * kSecond;
    using vehicle_api::MissionState;

    EXPECT_FALSE(follow_stop_needed(status(MissionState::Idle, now), now));
    EXPECT_FALSE(follow_stop_needed(status(MissionState::HoldWait, now), now));
    EXPECT_TRUE(follow_stop_needed(status(MissionState::FollowActive, now), now));
    EXPECT_TRUE(follow_stop_needed(status(MissionState::ScanActive, now), now));
    EXPECT_TRUE(follow_stop_needed(status(MissionState::Manual, now), now));
    EXPECT_TRUE(follow_stop_needed(status(MissionState::Fault, now), now));
}

TEST(FollowStopNeededTest, StaleOrMissingStatusNeedsNoStop) {
    const std::int64_t now = 100 * kSecond;

    EXPECT_FALSE(follow_stop_needed(std::nullopt, now));
    EXPECT_TRUE(follow_stop_needed(
        status(vehicle_api::MissionState::FollowActive, now - 3 * kSecond), now));
    EXPECT_FALSE(follow_stop_needed(
        status(vehicle_api::MissionState::FollowActive, now - 3 * kSecond - 1), now));
}

}  // namespace
}  // namespace argus::mavlink_gateway
