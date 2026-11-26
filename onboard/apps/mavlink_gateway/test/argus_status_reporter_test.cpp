#include "argus/mavlink_gateway/argus_status_reporter.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/cpu_temperature.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/mavlink_gateway/stop_requester.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_gcs_link.hpp"

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::MissionState;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kSecond = 1'000'000'000;

class FakeControlAuthority : public IControlAuthority {
public:
    std::optional<std::uint8_t> controller() const override { return owner; }
    bool is_controller(std::uint8_t sysid) const override { return owner == sysid; }

    std::optional<std::uint8_t> owner;
};

class FakeFcCommandStatus : public IFcCommandStatus {
public:
    bool sending_setpoints() const override { return false; }
    bool entering() const override { return false; }
    std::optional<std::int64_t> last_setpoint_timeout_ns() const override { return timeout_ns; }

    std::optional<std::int64_t> timeout_ns;
};

class FakeStopRequester : public IStopRequester {
public:
    bool stop_from_onboard() override {
        ++stops;
        return true;
    }

    std::int32_t stops = 0;
};

class FakeCpuTemperature : public ICpuTemperature {
public:
    std::optional<double> celsius() const override { return 50.0; }
};

class ArgusStatusReporterTest : public ::testing::Test {
protected:
    void advance_ms(std::int64_t ms) { clock_.advance_ns(ms * kMs); }

    void receive(MissionState state) {
        vehicle_api::FollowStatus status;
        status.mission_state = state;
        follow_.simulate_status(status, clock_.now_monotonic_ns());
    }

    // 보낸 ARGUS_STATUS 들
    std::vector<mavlink_argus_status_t> sent() const {
        std::vector<mavlink_argus_status_t> result;
        for (const FakeGcsLink::Sent& item : link_.sent()) {
            EXPECT_FALSE(item.to.has_value());  // 모든 통제기에 broadcast
            mavlink_argus_status_t status{};
            mavlink_msg_argus_status_decode(&item.message, &status);
            result.push_back(status);
        }
        return result;
    }

    std::uint8_t last_mission_state() const {
        const std::vector<mavlink_argus_status_t> all = sent();
        return all.empty() ? 255 : all.back().mission_state;
    }

    core_testing::FakeClock clock_{100 * kSecond};  // 리포터 생성 시각 = 시작 시각
    FakeGcsLink link_;
    vehicle_api_testing::FakeFollowStatusSource follow_;
    FakeControlAuthority authority_;
    HealthState health_;
    FakeFcCommandStatus fc_status_;
    FakeStopRequester stop_;
    FakeCpuTemperature cpu_;
    ArgusStatusReporter reporter_{ArgusStatusReporterInputs{
        link_, clock_, follow_, follow_, authority_, health_, fc_status_, stop_, cpu_}};
};

TEST_F(ArgusStatusReporterTest, SendsOncePerSecond) {
    reporter_.tick();
    advance_ms(900);
    reporter_.tick();
    EXPECT_EQ(sent().size(), 1U);

    advance_ms(100);
    reporter_.tick();
    ASSERT_EQ(sent().size(), 2U);
    EXPECT_EQ(sent()[1].time_boot_ms, 1000U);
    EXPECT_EQ(link_.sent()[0].message.compid, kOnboardCommandCompid);
}

TEST_F(ArgusStatusReporterTest, SendsAtOnceWhenKeyFieldChanges) {
    reporter_.tick();
    receive(MissionState::Idle);  // IDLE 그대로: 바로 보내지 않는다
    EXPECT_EQ(sent().size(), 1U);

    receive(MissionState::ScanActive);
    EXPECT_EQ(sent().size(), 2U);
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_SCAN_ACTIVE);

    health_.set(ARGUS_HEALTH_GCS_LINK, true);
    ASSERT_EQ(sent().size(), 3U);
    EXPECT_EQ(sent().back().health, ARGUS_HEALTH_GCS_LINK);
}

TEST_F(ArgusStatusReporterTest, LostStatusWhileFollowingStopsOnceAndReportsFault) {
    receive(MissionState::FollowActive);
    advance_ms(3001);

    reporter_.tick();
    reporter_.tick();

    EXPECT_EQ(stop_.stops, 1);
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_FAULT);
}

TEST_F(ArgusStatusReporterTest, LostStatusWhileIdleReportsFaultWithoutStop) {
    receive(MissionState::Idle);
    advance_ms(3001);

    reporter_.tick();

    EXPECT_EQ(stop_.stops, 0);
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_FAULT);
}

TEST_F(ArgusStatusReporterTest, NewStatusAfterLossEndsFault) {
    receive(MissionState::FollowActive);
    advance_ms(3001);
    reporter_.tick();

    receive(MissionState::Idle);

    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_IDLE);
}

TEST_F(ArgusStatusReporterTest, SetpointTimeoutFaultEndsOnceAndDoesNotReturn) {
    receive(MissionState::FollowActive);
    advance_ms(10);
    fc_status_.timeout_ns = clock_.now_monotonic_ns();
    reporter_.tick();
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_FAULT);

    advance_ms(10);
    receive(MissionState::HoldWait);  // 추종 제어가 정지를 반영함
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_HOLD_WAIT);

    advance_ms(10);
    receive(MissionState::FollowActive);  // 새 명령으로 다시 추종
    EXPECT_EQ(last_mission_state(), ARGUS_MISSION_STATE_FOLLOW_ACTIVE);
}

}  // namespace
}  // namespace argus::mavlink_gateway
