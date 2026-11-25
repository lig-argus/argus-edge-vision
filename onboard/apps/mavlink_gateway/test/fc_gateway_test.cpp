// FcGateway 의 연결 테스트: FC 메시지 → bus 발행, bus 의 입력 → FC 송신, 주기 작업.
// Fake 는 FC 와의 연결(FakeMavlinkConnection), bus(FakeBus), 시각(FakeClock, FakePoller)뿐이다.
// 추종 제어의 요청은 실제 BusFlightModeControlClient 로 보낸다. 판단의 세부 동작은 fc_control_test,
// fc_command_scenario_test 가 확인한다
#include "argus/mavlink_gateway/fc_gateway.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_client.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kStartNs = 10'000 * kNsPerMs;
constexpr std::int64_t kTickNs = 50 * kNsPerMs;  // kFcControlTickPeriod
constexpr std::chrono::milliseconds kHandlerBudget{10};
constexpr std::chrono::milliseconds kRequestTimeout{500};
constexpr std::int64_t kRequestValidNs = 1'000 * kNsPerMs;
constexpr std::int64_t kSetpointValidNs = 200 * kNsPerMs;

// PX4 의 custom_mode (icd.md 부록 A)
constexpr std::uint32_t kOffboardCustomMode = 6U << 16U;
constexpr std::uint32_t kHoldCustomMode = (4U << 16U) | (3U << 24U);
constexpr std::uint8_t kArmedBaseMode =
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;

// FcControl 이 진입 요청을 받으려면 setpoint 를 1 초 이상 끊김 없이 송신하고 있어야 한다
constexpr std::int32_t kPrestreamTicks = 22;

constexpr double kForwardMps = 1.5;

class FcGatewayTest : public ::testing::Test {
protected:
    void SetUp() override {
        gateway_ = FcGateway::create(bus_, connection_, clock_, loop_, health_);
        ASSERT_NE(gateway_, nullptr);
        client_ = vehicle_api_bus::BusFlightModeControlClient::create(bus_);
        ASSERT_NE(client_, nullptr);
    }

    std::uint32_t fc_time_boot_ms() const {
        return static_cast<std::uint32_t>((clock_.now_monotonic_ns() - kStartNs) / kNsPerMs + 1);
    }

    // 아래 receive_* 는 FC 가 보낸 메시지 하나를 도착한 것으로 넣는다 (다음 run_once() 에서 처리)
    void receive_attitude() {
        mavlink_message_t message{};
        mavlink_msg_attitude_pack(kVehicleSysid, kFcCompid, &message, fc_time_boot_ms(), 0.1F,
                                  -0.2F, 1.5F, 0.0F, 0.0F, 0.0F);
        connection_.simulate_message_received(message);
    }

    void receive_local_position() {
        mavlink_message_t message{};
        mavlink_msg_local_position_ned_pack(kVehicleSysid, kFcCompid, &message, fc_time_boot_ms(),
                                            5.0F, 6.0F, -7.0F, 0.0F, 0.0F, 0.0F);
        connection_.simulate_message_received(message);
    }

    void receive_heartbeat(std::uint32_t custom_mode) {
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(kVehicleSysid, kFcCompid, &message, MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_PX4, kArmedBaseMode, custom_mode,
                                   MAV_STATE_ACTIVE);
        connection_.simulate_message_received(message);
    }

    void receive_entry_ack() {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack(kVehicleSysid, kFcCompid, &message, MAV_CMD_DO_SET_MODE,
                                     MAV_RESULT_ACCEPTED, 0, 0, kVehicleSysid,
                                     kOnboardCommandCompid);
        connection_.simulate_message_received(message);
    }

    // FC 연결(ATTITUDE)을 확인한 뒤 FC 의 비행 상태(HEARTBEAT)를 받는다. HEARTBEAT 를 먼저 받으면
    // 첫 ATTITUDE 에서 fc_epoch 가 바뀌어 그 비행 상태가 삭제된다
    void connect_fc(std::uint32_t custom_mode) {
        receive_attitude();
        receive_heartbeat(custom_mode);
        loop_.run_once();
    }

    // 추종 제어가 전진 속도 kForwardMps 의 setpoint 를 발행한 것으로 넣는다
    void receive_setpoint() {
        vehicle_api::VelocitySetpoint setpoint;
        setpoint.forward_m_s = kForwardMps;
        setpoint.valid_until_monotonic_ns = clock_.now_monotonic_ns() + kSetpointValidNs;
        bus_.simulate_message_received(protocol::kControlSetpoint, setpoint);
    }

    // 다음 주기 작업 시각까지 진행한다. 그 전에 도착한 메시지를 먼저 처리한다
    void run_one_tick() {
        const std::int64_t target_ns = clock_.now_monotonic_ns() + kTickNs;
        while (clock_.now_monotonic_ns() < target_ns) {
            loop_.run_once();
        }
    }

    // 한 주기: FC 의 자세와 local 위치, 추종 제어의 setpoint(with_setpoint 인 경우)가 도착한 뒤
    // 주기 작업을 실행한다
    void step(bool with_setpoint = true) {
        receive_attitude();
        receive_local_position();
        if (with_setpoint) {
            receive_setpoint();
        }
        run_one_tick();
    }

    // 추종 제어가 control/mode_request 로 요청하고 응답을 받는다. 응답이 없으면 nullopt
    std::optional<vehicle_api::FlightModeReply> request(bool offboard, std::uint64_t command_id) {
        vehicle_api::RequestHeader header;
        header.id.command_id = vehicle_api::CommandId{command_id};
        header.id.session_id = "follow";
        header.id.fc_epoch = gateway_->fc_state().fc_epoch();
        header.valid_until_monotonic_ns = clock_.now_monotonic_ns() + kRequestValidNs;
        std::optional<vehicle_api::FlightModeReply> received;
        const auto on_reply = [&received](std::optional<vehicle_api::FlightModeReply> reply) {
            received = std::move(reply);
        };
        const core::Subscription pending =
            offboard ? client_->request_offboard(header, kRequestTimeout, on_reply)
                     : client_->request_hold(header, kRequestTimeout, on_reply);
        loop_.run_once();
        return received;
    }

    bool accepted(const std::optional<vehicle_api::FlightModeReply>& reply) const {
        return reply.has_value() && reply->accepted;
    }

    std::vector<mavlink_message_t> sent(std::uint32_t msgid) const {
        std::vector<mavlink_message_t> messages;
        for (const mavlink_message_t& message : connection_.sent()) {
            if (message.msgid == msgid) {
                messages.push_back(message);
            }
        }
        return messages;
    }

    std::vector<vehicle_api::VehicleState> published_states() const {
        return bus_.published<vehicle_api::VehicleState>(protocol::kFcState);
    }

    // 마지막으로 발행한 fc/state 의 비행 상태. fc/state 를 발행하지 않았거나 비행 상태가 없으면
    // nullopt
    std::optional<vehicle_api::FlightStatus> last_published_status() const {
        const std::vector<vehicle_api::VehicleState> states = published_states();
        if (states.empty()) {
            return std::nullopt;
        }
        return states.back().status;
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    bus_testing::FakeBus bus_{poller_, clock_};
    core::EventLoop loop_{poller_, clock_, kHandlerBudget};
    HealthState health_;
    std::unique_ptr<vehicle_api_bus::BusFlightModeControlClient> client_;
    std::unique_ptr<FcGateway> gateway_;
};

TEST_F(FcGatewayTest, PublishesFcAttitudeWhenFcSendsAttitude) {
    receive_attitude();
    loop_.run_once();

    const std::vector<vehicle_api::Attitude> attitudes =
        bus_.published<vehicle_api::Attitude>(protocol::kFcAttitude);
    ASSERT_EQ(attitudes.size(), 1U);
    EXPECT_FLOAT_EQ(static_cast<float>(attitudes[0].yaw_rad), 1.5F);
    EXPECT_EQ(attitudes[0].fc_epoch, 1U);
    EXPECT_TRUE(published_states().empty());  // 자세는 fc/state 로 발행하지 않음
}

// 전제: FC 연결(ATTITUDE)과 FC 의 비행 상태(HEARTBEAT, Hold)를 받음
TEST_F(FcGatewayTest, FcStateCarriesOffboardHoverOfFcControl) {
    connect_fc(kHoldCustomMode);
    step();
    const std::optional<vehicle_api::FlightStatus> hold = last_published_status();
    ASSERT_TRUE(hold.has_value());
    EXPECT_FALSE(hold.value_or(vehicle_api::FlightStatus{}).offboard_hover);

    // 추종 제어가 Offboard 로 진입시킨 뒤 정지를 요청하면 FcControl 이 제자리 유지를 보낸다
    for (std::int32_t tick = 0; tick < kPrestreamTicks; ++tick) {
        step();
    }
    ASSERT_TRUE(accepted(request(true, 1)));
    receive_entry_ack();
    receive_heartbeat(kOffboardCustomMode);
    step();
    ASSERT_TRUE(accepted(request(false, 2)));
    step(false);

    receive_heartbeat(kOffboardCustomMode);
    loop_.run_once();

    const std::optional<vehicle_api::FlightStatus> hover = last_published_status();
    ASSERT_TRUE(hover.has_value());
    const vehicle_api::FlightStatus status = hover.value_or(vehicle_api::FlightStatus{});
    EXPECT_EQ(status.mode, vehicle_api::FlightMode::Offboard);
    EXPECT_TRUE(status.offboard_hover);
}

// 전제: FC 연결(ATTITUDE)과 Offboard 가 아닌 FC 의 비행 상태(HEARTBEAT, Hold)를 받음
TEST_F(FcGatewayTest, SendsSetpointFromBusToFcOnNextTick) {
    connect_fc(kHoldCustomMode);
    step(false);
    ASSERT_TRUE(sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED).empty());

    step();

    const std::vector<mavlink_message_t> setpoints =
        sent(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    ASSERT_EQ(setpoints.size(), 1U);
    mavlink_set_position_target_local_ned_t target{};
    mavlink_msg_set_position_target_local_ned_decode(&setpoints[0], &target);
    EXPECT_FLOAT_EQ(target.vx, static_cast<float>(kForwardMps));
}

// 전제: FC 연결(ATTITUDE)과 FC 의 비행 상태(HEARTBEAT, Hold)를 받음
TEST_F(FcGatewayTest, RepliesToModeRequestFromBus) {
    connect_fc(kHoldCustomMode);
    step(false);

    const std::optional<vehicle_api::FlightModeReply> reply = request(false, 7);

    ASSERT_TRUE(reply.has_value());
    const vehicle_api::FlightModeReply value = reply.value_or(vehicle_api::FlightModeReply{});
    EXPECT_TRUE(value.accepted);
    ASSERT_TRUE(value.id.has_value());
    EXPECT_EQ(value.id.value_or(vehicle_api::RequestId{}).command_id, vehicle_api::CommandId{7});
}

// 전제: FC 메시지를 받기 전에는 FC 연결 비트가 0
TEST_F(FcGatewayTest, PeriodicLinkCheckClearsFcConnectedAfterAttitudeStops) {
    EXPECT_FALSE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
    receive_attitude();
    run_one_tick();
    EXPECT_TRUE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));

    // 마지막 ATTITUDE 후 두절 기준(0.5 s)이 지난 다음 주기 작업에서 0 으로 바뀐다
    const std::int64_t last_attitude_ns = clock_.now_monotonic_ns() - kTickNs;
    while (clock_.now_monotonic_ns() - last_attitude_ns <= kFcLinkTimeoutNs) {
        EXPECT_TRUE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
        run_one_tick();
    }
    EXPECT_FALSE(health_.is_set(ARGUS_HEALTH_FC_CONNECTED));
}

}  // namespace
}  // namespace argus::mavlink_gateway
