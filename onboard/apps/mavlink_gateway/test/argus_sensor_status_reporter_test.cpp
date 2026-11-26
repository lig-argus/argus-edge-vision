#include "argus/mavlink_gateway/argus_sensor_status_reporter.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/tracking_status_source.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api/vision_status_source.hpp"
#include "fake_gcs_link.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kSecond = 1'000'000'000;

class FakeTrackingStatusSource : public perception_api::ITrackingStatusSource {
public:
    std::optional<perception_api::ReceivedTrackingStatus> latest(
        const std::string& source) const override {
        const auto found = by_source.find(source);
        if (found == by_source.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::map<std::string, perception_api::ReceivedTrackingStatus> by_source;
};

class FakeVisionStatusSource : public perception_api::IVisionStatusSource {
public:
    std::optional<perception_api::ReceivedVisionStatus> latest(
        const std::string& source) const override {
        const auto found = by_source.find(source);
        if (found == by_source.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::map<std::string, perception_api::ReceivedVisionStatus> by_source;
};

class ArgusSensorStatusReporterTest : public ::testing::Test {
protected:
    void advance_ms(std::int64_t ms) { clock_.advance_ns(ms * kMs); }

    void receive_tracking(const std::string& source) {
        perception_api::ReceivedTrackingStatus received;
        received.status.source = source;
        received.arrived_monotonic_ns = clock_.now_monotonic_ns();
        tracking_.by_source[source] = received;
    }

    // 보낸 ARGUS_SENSOR_STATUS 들
    std::vector<mavlink_argus_sensor_status_t> sent() const {
        std::vector<mavlink_argus_sensor_status_t> result;
        for (const FakeGcsLink::Sent& item : link_.sent()) {
            EXPECT_FALSE(item.to.has_value());  // 모든 통제기에 broadcast
            mavlink_argus_sensor_status_t status{};
            mavlink_msg_argus_sensor_status_decode(&item.message, &status);
            result.push_back(status);
        }
        return result;
    }

    core_testing::FakeClock clock_{100 * kSecond};
    FakeGcsLink link_;
    FakeTrackingStatusSource tracking_;
    FakeVisionStatusSource vision_;
    SensorSourceMap sensors_ = SensorSourceMap::default_map();
    ArgusSensorStatusReporter reporter_{
        ArgusSensorStatusReporterInputs{link_, clock_, tracking_, vision_, sensors_}};
};

TEST_F(ArgusSensorStatusReporterTest, SendsEverySensorOncePerSecond) {
    reporter_.tick();
    ASSERT_EQ(sent().size(), 2U);
    EXPECT_EQ(sent()[0].sensor_id, ARGUS_SENSOR_IR);
    EXPECT_EQ(sent()[1].sensor_id, ARGUS_SENSOR_EO);
    EXPECT_EQ(sent()[0].flags, 0);

    advance_ms(900);
    reporter_.tick();
    EXPECT_EQ(sent().size(), 2U);

    advance_ms(100);
    reporter_.tick();
    EXPECT_EQ(sent().size(), 4U);
}

TEST_F(ArgusSensorStatusReporterTest, SendsAtOnceOnlyTheSensorWhoseFlagsChanged) {
    reporter_.tick();
    advance_ms(100);
    receive_tracking("camera-ir");

    reporter_.tick();

    ASSERT_EQ(sent().size(), 3U);
    EXPECT_EQ(sent()[2].sensor_id, ARGUS_SENSOR_IR);
    EXPECT_EQ(sent()[2].flags, ARGUS_SENSOR_FLAGS_TRACKING | ARGUS_SENSOR_FLAGS_FOLLOWABLE);
}

}  // namespace
}  // namespace argus::mavlink_gateway
