#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "argus/bus/typed.hpp"
#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/bus_vision_status_subscriber.hpp"
#include "argus/perception_api_bus/message_types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {
namespace {

using perception_api::ReceivedVisionStatus;
using perception_api::VisionStatus;

constexpr std::int64_t kMs = 1'000'000;

VisionStatus status_of(const std::string& source, std::uint32_t model_ver) {
    VisionStatus status;
    status.source = source;
    status.camera_ok = true;
    status.model_ver = model_ver;
    return status;
}

class VisionStatusSubscriberTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 탐지의 발행 클래스는 이 모듈에 없으므로 typed 발행자로 보낸다
        publisher_ = bus::make_publisher<VisionStatus>(
            bus_, protocol::kVisionStatus, kVisionStatusType, protocol::kVisionStatusQueueDepth);
        subscriber_ = BusVisionStatusSubscriber::create(bus_, clock_);
        ASSERT_NE(publisher_, nullptr);
        ASSERT_NE(subscriber_, nullptr);
    }

    void deliver_at(std::int64_t at_ns, const VisionStatus& status) {
        clock_.advance_ns(at_ns - clock_.now_monotonic_ns());
        ASSERT_TRUE(publisher_->publish(status));
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
    std::unique_ptr<bus::Publisher<VisionStatus>> publisher_;
    std::unique_ptr<BusVisionStatusSubscriber> subscriber_;
};

TEST_F(VisionStatusSubscriberTest, LatestIsEmptyUntilThatCameraReports) {
    deliver_at(10 * kMs, status_of("camera-ir", 1));

    EXPECT_FALSE(subscriber_->latest("camera-eo").has_value());
}

TEST_F(VisionStatusSubscriberTest, KeepsLatestStatusPerCameraWithArrivalTime) {
    VisionStatus first = status_of("camera-ir", 1);
    first.published_monotonic_ns = 1;  // 발행자가 채운 값은 받은 시각으로 사용하지 않는다
    deliver_at(10 * kMs, first);
    deliver_at(20 * kMs, status_of("camera-eo", 5));
    deliver_at(30 * kMs, status_of("camera-ir", 2));

    const ReceivedVisionStatus ir =
        subscriber_->latest("camera-ir").value_or(ReceivedVisionStatus{});
    const ReceivedVisionStatus eo =
        subscriber_->latest("camera-eo").value_or(ReceivedVisionStatus{});
    EXPECT_EQ(ir.status.model_ver, 2U);
    EXPECT_EQ(ir.arrived_monotonic_ns, 30 * kMs);
    EXPECT_EQ(eo.status.model_ver, 5U);
    EXPECT_EQ(eo.arrived_monotonic_ns, 20 * kMs);
}

}  // namespace
}  // namespace argus::perception_api_bus
