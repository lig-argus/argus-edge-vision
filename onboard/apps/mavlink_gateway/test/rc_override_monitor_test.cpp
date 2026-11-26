#include "argus/mavlink_gateway/rc_override_monitor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/fc_rc_status_source.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_fc_state_source.hpp"
#include "fake_forwarded_command_events.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kTick = 50'000'000;

class FakeFcRcStatusSource : public IFcRcStatusSource {
public:
    std::optional<RcChannels> rc_channels() const override { return channels; }
    std::optional<FcSensorHealth> sensor_health() const override { return health; }

    std::optional<RcChannels> channels;
    std::optional<FcSensorHealth> health;
};

class FakeFcCommandStatus : public IFcCommandStatus {
public:
    bool sending_setpoints() const override { return false; }
    bool entering() const override { return entering_now; }
    std::optional<std::int64_t> last_setpoint_timeout_ns() const override { return std::nullopt; }

    bool entering_now = false;
};

class RcOverrideMonitorTest : public ::testing::Test {
protected:
    RcOverrideMonitorTest() {
        clock_.advance_ns(100 * kSecond);
        set_mode(FlightMode::Offboard);
        set_sticks(1500);
        FcSensorHealth health;
        health.rc_receiver_ok = true;
        health.local_position_ok = true;
        health.attitude_ok = true;
        health.received_monotonic_ns = clock_.now_monotonic_ns();
        health.fc_epoch = fc_state_.epoch;
        rc_status_.health = health;
        monitor_.tick();
    }

    void set_mode(FlightMode mode, bool armed = true) {
        fc_state_.set_status(mode, clock_.now_monotonic_ns(), armed);
    }

    // 롤 채널(1)의 값을 roll_us 로, 나머지 축은 중앙으로 받은 것으로 한다
    void set_sticks(std::uint16_t roll_us) {
        RcChannels channels;
        channels.channel_count = 8;
        channels.raw_us.fill(1500);
        channels.raw_us[0] = roll_us;
        channels.received_monotonic_ns = clock_.now_monotonic_ns();
        channels.fc_epoch = fc_state_.epoch;
        rc_status_.channels = channels;
        if (rc_status_.health.has_value()) {
            rc_status_.health->received_monotonic_ns = clock_.now_monotonic_ns();
        }
    }

    // duration_ns 동안 tick 하며 스틱과 SYS_STATUS 를 계속 받는다
    void run_for(std::int64_t duration_ns, std::uint16_t roll_us = 1500) {
        for (std::int64_t elapsed = 0; elapsed < duration_ns; elapsed += kTick) {
            clock_.advance_ns(kTick);
            set_sticks(roll_us);
            monitor_.tick();
        }
    }

    void pilot_takes_over() {
        clock_.advance_ns(kTick);
        set_mode(FlightMode::Position);
        monitor_.tick();
    }

    core_testing::FakeClock clock_;
    FakeFcStateSource fc_state_;
    FakeFcRcStatusSource rc_status_;
    FakeForwardedCommandEvents forwarded_;
    CommandActivity activity_;
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    FakeFcCommandStatus command_status_;
    RcOverrideMonitor monitor_{RcOverrideMonitorInputs{fc_state_, rc_status_, forwarded_, clock_,
                                                       activity_, follow_status_, command_status_}};
};

TEST_F(RcOverrideMonitorTest, ManualModeChangeByPilotIsTakeover) {
    EXPECT_FALSE(monitor_.pilot_in_control());

    pilot_takes_over();

    EXPECT_TRUE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, ModeChangeRequestedByGcsIsNotTakeover) {
    forwarded_.simulate_forwarded(MAV_CMD_DO_SET_MODE, FlightMode::Position);

    pilot_takes_over();

    EXPECT_FALSE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, GcsCommandOlderThanWindowDoesNotExplainChange) {
    forwarded_.simulate_forwarded(MAV_CMD_DO_SET_MODE, FlightMode::Position);
    clock_.advance_ns(3 * kSecond);

    pilot_takes_over();

    EXPECT_TRUE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, FailsafeModeIsNotTakeover) {
    clock_.advance_ns(kTick);
    set_mode(FlightMode::Land);
    monitor_.tick();

    EXPECT_FALSE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, FirstStatusOrNewFcEpochIsNotAChange) {
    fc_state_.epoch = 2;
    set_mode(FlightMode::Position);
    monitor_.tick();

    EXPECT_FALSE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, DisarmEndsTakeover) {
    pilot_takes_over();
    clock_.advance_ns(kTick);
    set_mode(FlightMode::Position, false);
    monitor_.tick();

    EXPECT_FALSE(monitor_.pilot_in_control());
}

TEST_F(RcOverrideMonitorTest, ReturnNeedsSticksNeutralForHoldTime) {
    pilot_takes_over();
    run_for(500'000'000);
    EXPECT_EQ(monitor_.check_return().block, ReturnBlock::Sticks);

    run_for(600'000'000);
    const ReturnCheck check = monitor_.check_return();
    EXPECT_EQ(check.block, ReturnBlock::None);
    EXPECT_EQ(check.fc_mode, FlightMode::Position);
}

TEST_F(RcOverrideMonitorTest, MovedStickRestartsHoldTime) {
    pilot_takes_over();
    run_for(900'000'000);
    run_for(kTick, 1700);
    run_for(900'000'000);

    EXPECT_EQ(monitor_.check_return().block, ReturnBlock::Sticks);
}

TEST_F(RcOverrideMonitorTest, StaleSticksAreNotNeutral) {
    pilot_takes_over();
    for (std::int32_t count = 0; count < 30; ++count) {
        clock_.advance_ns(kTick);
        monitor_.tick();  // RC_CHANNELS 를 새로 받지 않음
    }

    EXPECT_EQ(monitor_.check_return().block, ReturnBlock::Sticks);
    EXPECT_FALSE(monitor_.return_still_allowed());
}

TEST_F(RcOverrideMonitorTest, ReturnIsBlockedWhileBusy) {
    pilot_takes_over();
    run_for(1'100'000'000);
    command_status_.entering_now = true;
    EXPECT_EQ(monitor_.check_return().block, ReturnBlock::Busy);
    // 복귀 절차 중에는 자신의 진입과 명령은 보지 않는다
    EXPECT_TRUE(monitor_.return_still_allowed());
}

TEST_F(RcOverrideMonitorTest, EndOverrideClearsTakeover) {
    pilot_takes_over();

    monitor_.end_override();

    EXPECT_FALSE(monitor_.pilot_in_control());
    EXPECT_EQ(monitor_.check_return().block, ReturnBlock::NotPilot);
}

}  // namespace
}  // namespace argus::mavlink_gateway
