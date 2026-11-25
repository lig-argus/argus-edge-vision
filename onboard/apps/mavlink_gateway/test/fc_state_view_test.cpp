// FcStateView 를 실제 FcLinkTracker, FcTelemetryReceiver 와 연결한 테스트. 값의 삭제와 두절 판단의
// 세부 동작은 fc_link_tracker_test, fc_telemetry_receiver_test 가 확인한다
#include "argus/mavlink_gateway/fc_state_view.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/fc_telemetry_receiver.hpp"
#include "argus/mavlink_gateway/fc_time_converter.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/vehicle_api/types.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kStartNs = 5'000'000'000;

// FC 시각의 변환값이 없는 상태 (이 테스트는 변환을 사용하지 않음)
class NoFcTimeConverter : public IFcTimeConverter {
public:
    std::optional<std::int64_t> to_monotonic_ns(std::uint32_t /*fc_time_boot_ms*/) const override {
        return std::nullopt;
    }
};

class FcStateViewTest : public ::testing::Test {
protected:
    void receive_attitude(std::uint32_t time_boot_ms) {
        mavlink_message_t message{};
        mavlink_msg_attitude_pack(kVehicleSysid, kFcCompid, &message, time_boot_ms, 0.1F, -0.2F,
                                  1.5F, 0.0F, 0.0F, 0.0F);
        connection_.simulate_message_received(message);
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    NoFcTimeConverter time_converter_;
    HealthState health_;
    FcLinkTracker link_{clock_, health_};
    FcTelemetryReceiver telemetry_{connection_, link_, time_converter_};
    FcStateView view_{link_, telemetry_};
};

TEST_F(FcStateViewTest, IsDisconnectedWithoutValuesBeforeFcMessages) {
    EXPECT_FALSE(view_.fc_connected());
    EXPECT_EQ(view_.fc_epoch(), 0U);
    EXPECT_FALSE(view_.latest().attitude.has_value());
}

TEST_F(FcStateViewTest, ReportsLinkStateAndLatestValuesAtCallTime) {
    receive_attitude(1000);

    EXPECT_TRUE(view_.fc_connected());
    EXPECT_EQ(view_.fc_epoch(), 1U);
    const std::optional<vehicle_api::Attitude> attitude = view_.latest().attitude;
    ASSERT_TRUE(attitude.has_value());
    EXPECT_FLOAT_EQ(static_cast<float>(attitude.value_or(vehicle_api::Attitude{}).yaw_rad), 1.5F);

    // check_timeout() 을 호출하지 않아도 호출 시각으로 두절을 판단한다
    clock_.advance_ns(kFcLinkTimeoutNs + 1);
    EXPECT_FALSE(view_.fc_connected());
}

}  // namespace
}  // namespace argus::mavlink_gateway
