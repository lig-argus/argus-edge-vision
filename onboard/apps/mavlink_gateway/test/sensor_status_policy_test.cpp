#include "argus/mavlink_gateway/sensor_status_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kNow = 100 * kSecond;

SensorObservation observation(bool followable = true) {
    SensorObservation value;
    value.now_ns = kNow;
    value.sensor = SensorSource{"camera-ir", ARGUS_SENSOR_IR, followable};
    return value;
}

perception_api::ReceivedVisionStatus vision(std::int64_t age_ns, bool camera_ok = true,
                                            bool inference_ok = true) {
    perception_api::ReceivedVisionStatus received;
    received.status.source = "camera-ir";
    received.status.camera_ok = camera_ok;
    received.status.inference_ok = inference_ok;
    received.status.infer_fps = 14.5;
    received.status.model_ver = 3;
    received.status.image_width = 640;
    received.status.image_height = 512;
    received.arrived_monotonic_ns = kNow - age_ns;
    return received;
}

perception_api::ReceivedTrackingStatus tracking(std::int64_t age_ns) {
    perception_api::ReceivedTrackingStatus received;
    received.status.source = "camera-ir";
    received.arrived_monotonic_ns = kNow - age_ns;
    return received;
}

TEST(SensorStatusPolicyTest, NothingReceivedGivesOnlySensorId) {
    const mavlink_argus_sensor_status_t status = compose_sensor_status(observation());

    EXPECT_EQ(status.sensor_id, ARGUS_SENSOR_IR);
    EXPECT_EQ(status.flags, 0);
    EXPECT_TRUE(std::isnan(status.infer_fps));
    EXPECT_EQ(status.model_ver, 0);
    EXPECT_EQ(status.img_w, 0);
    EXPECT_EQ(status.img_h, 0);
}

TEST(SensorStatusPolicyTest, FreshStatusesGiveEveryFlagAndValue) {
    SensorObservation value = observation();
    value.vision = vision(kSensorStatusTimeoutNs);
    value.tracking = tracking(kSensorStatusTimeoutNs);

    const mavlink_argus_sensor_status_t status = compose_sensor_status(value);

    constexpr std::uint32_t kAllFlags =
        static_cast<std::uint32_t>(ARGUS_SENSOR_FLAGS_CAMERA_OK) |
        static_cast<std::uint32_t>(ARGUS_SENSOR_FLAGS_INFERENCE_OK) |
        static_cast<std::uint32_t>(ARGUS_SENSOR_FLAGS_TRACKING) |
        static_cast<std::uint32_t>(ARGUS_SENSOR_FLAGS_FOLLOWABLE);
    EXPECT_EQ(status.flags, kAllFlags);
    EXPECT_FLOAT_EQ(status.infer_fps, 14.5F);
    EXPECT_EQ(status.model_ver, 3);
    EXPECT_EQ(status.img_w, 640);
    EXPECT_EQ(status.img_h, 512);
}

TEST(SensorStatusPolicyTest, FlagsFollowReportedValues) {
    SensorObservation value = observation();
    value.vision = vision(0, true, false);

    EXPECT_EQ(compose_sensor_status(value).flags, ARGUS_SENSOR_FLAGS_CAMERA_OK);
}

TEST(SensorStatusPolicyTest, OldStatusesAreNotUsed) {
    SensorObservation value = observation();
    value.vision = vision(kSensorStatusTimeoutNs + 1);
    value.tracking = tracking(kSensorStatusTimeoutNs + 1);

    const mavlink_argus_sensor_status_t status = compose_sensor_status(value);

    EXPECT_EQ(status.flags, 0);
    EXPECT_TRUE(std::isnan(status.infer_fps));
    EXPECT_EQ(status.model_ver, 0);
}

TEST(SensorStatusPolicyTest, FollowableNeedsTrackingAndPermission) {
    SensorObservation not_allowed = observation(false);
    not_allowed.tracking = tracking(0);
    EXPECT_EQ(compose_sensor_status(not_allowed).flags, ARGUS_SENSOR_FLAGS_TRACKING);

    SensorObservation no_tracking = observation(true);
    no_tracking.vision = vision(0);
    EXPECT_EQ(compose_sensor_status(no_tracking).flags & ARGUS_SENSOR_FLAGS_FOLLOWABLE, 0);
}

TEST(SensorStatusPolicyTest, ValuesBeyondUint16AreLimited) {
    SensorObservation value = observation();
    perception_api::ReceivedVisionStatus received = vision(0);
    received.status.model_ver = 70'000;
    value.vision = received;

    EXPECT_EQ(compose_sensor_status(value).model_ver, 65'535);
}

}  // namespace
}  // namespace argus::mavlink_gateway
