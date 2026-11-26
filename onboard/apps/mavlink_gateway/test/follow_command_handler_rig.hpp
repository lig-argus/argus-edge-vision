// 추종 명령 handler 테스트가 함께 사용하는 객체와 응답 기록
#ifndef ARGUS_ONBOARD_TEST_FOLLOW_COMMAND_HANDLER_RIG_HPP_
#define ARGUS_ONBOARD_TEST_FOLLOW_COMMAND_HANDLER_RIG_HPP_

#include <chrono>
#include <cstdint>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/follow_command_tracker.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"
#include "fake_fc_epoch_source.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway::testing {

inline constexpr std::int64_t kMs = 1'000'000;
inline constexpr std::int64_t kSecond = 1'000'000'000;

class FollowCommandHandlerRig {
public:
    // handler 에 COMMAND_LONG 을 넘기고 응답을 acks 에 모은다
    void handle(IMavlinkCommandHandler& handler, const mavlink_command_long_t& command) {
        CommandRequest request;
        request.sender = test_messages::gcs_peer(250);
        request.command = command;
        handler.handle(request,
                       CommandResponder([this](const CommandAck& ack) { acks.push_back(ack); }));
    }

    void set_mission_state(vehicle_api::MissionState state) {
        vehicle_api::FollowStatus status;
        status.onboard_session = headers.session_id();
        status.mission_state = state;
        follow_status.simulate_status(status, clock.now_monotonic_ns());
    }

    // tracker 가 소멸할 때 남은 명령의 FAILED 를 기록하므로 tracker 보다 앞에 둔다
    std::vector<CommandAck> acks;
    core_testing::FakeClock clock{100 * kSecond};
    core_testing::FakePoller poller{clock};
    vehicle_api_testing::FakeFollowControlClient follow_control{poller, clock};
    vehicle_api_testing::FakeFollowStatusSource follow_status;
    FakeFcEpochSource epoch;
    RequestHeaderFactory headers{"session-g", clock, epoch};
    FollowCommandTracker tracker{clock, follow_status, follow_status, headers.session_id()};
};

}  // namespace argus::mavlink_gateway::testing

#endif  // ARGUS_ONBOARD_TEST_FOLLOW_COMMAND_HANDLER_RIG_HPP_
