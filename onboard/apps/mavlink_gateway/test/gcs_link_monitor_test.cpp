#include "argus/mavlink_gateway/gcs_link_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_fc_epoch_source.hpp"
#include "fake_gcs_link.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::gcs_peer;
using test_messages::heartbeat_from;

constexpr std::int64_t kSecond = 1'000'000'000;

class FakeControlAuthority : public IControlAuthority {
public:
    std::optional<std::uint8_t> controller() const override { return owner; }
    bool is_controller(std::uint8_t sysid) const override { return owner == sysid; }

    std::optional<std::uint8_t> owner;
};

class GcsLinkMonitorTest : public ::testing::Test {
protected:
    GcsLinkMonitorTest() {
        clock_.advance_ns(100 * kSecond);
        authority_.owner = 250;
    }

    void heartbeat() { link_.simulate_message_received(heartbeat_from(owner_), owner_); }

    void set_mission_state(vehicle_api::MissionState state) {
        vehicle_api::FollowStatus status;
        status.mission_state = state;
        follow_status_.simulate_status(status, clock_.now_monotonic_ns());
    }

    bool gcs_link() const { return health_.is_set(ARGUS_HEALTH_GCS_LINK); }

    const GcsPeer owner_ = gcs_peer(250);
    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    FakeGcsLink link_;
    GcsRegistry registry_{link_, clock_};
    FakeControlAuthority authority_;
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    vehicle_api_testing::FakeFollowControlClient follow_control_{poller_, clock_};
    FakeFcEpochSource epoch_;
    RequestHeaderFactory headers_{"session-g", clock_, epoch_};
    HealthState health_;
    GcsLinkMonitor monitor_{GcsLinkMonitorInputs{registry_, authority_, clock_, follow_status_,
                                                 follow_control_, headers_, health_}};
};

TEST_F(GcsLinkMonitorTest, LinkIsUpWhileControllerHeartbeatIsRecent) {
    heartbeat();
    clock_.advance_ns(3 * kSecond);

    monitor_.tick();

    EXPECT_TRUE(gcs_link());
}

TEST_F(GcsLinkMonitorTest, LinkIsDownWithoutController) {
    heartbeat();
    authority_.owner = std::nullopt;

    monitor_.tick();

    EXPECT_FALSE(gcs_link());
}

TEST_F(GcsLinkMonitorTest, LossWhileFollowingRequestsLinkLossStopOnce) {
    heartbeat();
    monitor_.tick();
    clock_.advance_ns(3 * kSecond + 1);
    set_mission_state(vehicle_api::MissionState::FollowActive);  // follow/status 는 계속 받는다

    monitor_.tick();
    monitor_.tick();

    EXPECT_FALSE(gcs_link());
    ASSERT_EQ(follow_control_.commands().size(), 1U);
    const vehicle_api_testing::FollowCommandRecord& stop = follow_control_.commands()[0];
    EXPECT_EQ(stop.kind, vehicle_api_testing::FollowCommandKind::Stop);
    EXPECT_EQ(stop.reason,
              std::optional<vehicle_api::StopReason>{vehicle_api::StopReason::LinkLoss});
    EXPECT_EQ(stop.header.id.session_id, "session-g");
}

TEST_F(GcsLinkMonitorTest, LossWhileIdleRequestsNoStop) {
    heartbeat();
    monitor_.tick();
    clock_.advance_ns(3 * kSecond + 1);
    set_mission_state(vehicle_api::MissionState::Idle);  // follow/status 는 계속 받는다

    monitor_.tick();

    EXPECT_FALSE(gcs_link());
    EXPECT_TRUE(follow_control_.commands().empty());
}

TEST_F(GcsLinkMonitorTest, RecoveryRestoresFlagOnly) {
    heartbeat();
    monitor_.tick();
    clock_.advance_ns(3 * kSecond + 1);
    set_mission_state(vehicle_api::MissionState::ScanActive);  // follow/status 는 계속 받는다
    monitor_.tick();

    heartbeat();
    monitor_.tick();

    EXPECT_TRUE(gcs_link());
    EXPECT_EQ(follow_control_.commands().size(), 1U);
}

}  // namespace
}  // namespace argus::mavlink_gateway
