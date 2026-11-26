#include "argus/mavlink_gateway/follow_command_tracker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control_client.hpp"
#include "argus/vehicle_api_testing/fake_follow_status_source.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kSession = "session-g";

class FollowCommandTrackerTest : public ::testing::Test {
protected:
    // command_id 의 track_start 를 FollowCommandTracker 로 보낸다
    void track(std::uint64_t command_id, std::optional<double> scan_range = std::nullopt) {
        vehicle_api::RequestHeader header;
        header.id.command_id = vehicle_api::CommandId{command_id};
        tracker_.track(header.id.command_id, scan_range,
                       CommandResponder([this](const CommandAck& ack) { acks_.push_back(ack); }),
                       [this, header](vehicle_api::IFollowControlClient::ResultHandler handler) {
                           return client_.track_start(header, vehicle_api::TrackTarget{},
                                                      kFollowCommandReplyTimeout,
                                                      std::move(handler));
                       });
    }

    void reply(vehicle_api::FollowOutcome outcome, const std::string& reason = "") {
        vehicle_api::FollowResult result;
        result.outcome = outcome;
        result.request_accepted = outcome != vehicle_api::FollowOutcome::Rejected;
        result.reason = reason;
        ASSERT_TRUE(client_.simulate_reply(result));
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{0});
    }

    // command_id 의 결과가 들어 있는 follow/status 를 받는다
    void status_with(std::uint64_t command_id, vehicle_api::FollowOutcome outcome,
                     const std::string& reason = "", const std::string& session = kSession,
                     std::optional<double> scan_progress_deg = std::nullopt) {
        vehicle_api::FollowStatus status;
        status.onboard_session = session;
        vehicle_api::FollowCommandResult result;
        result.command_id = vehicle_api::CommandId{command_id};
        result.outcome = outcome;
        result.reason = reason;
        status.command_results.push_back(result);
        if (scan_progress_deg.has_value()) {
            status.scan = vehicle_api::ScanProgress{};
            status.scan->progress_deg = scan_progress_deg.value();
        }
        follow_status_.simulate_status(status, clock_.now_monotonic_ns());
    }

    void advance_s(std::int64_t seconds) { clock_.advance_ns(seconds * kSecond); }

    // tracker_ 가 소멸할 때 남은 명령의 FAILED 를 기록하므로 tracker_ 보다 앞에 둔다
    std::vector<CommandAck> acks_;
    core_testing::FakeClock clock_{100 * kSecond};
    core_testing::FakePoller poller_{clock_};
    vehicle_api_testing::FakeFollowControlClient client_{poller_, clock_};
    vehicle_api_testing::FakeFollowStatusSource follow_status_;
    FollowCommandTracker tracker_{clock_, follow_status_, follow_status_, kSession};
};

TEST_F(FollowCommandTrackerTest, SendsInProgressAndRequests) {
    track(1);

    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(acks_[0].progress, kProgressUnknown);
    EXPECT_EQ(client_.commands().size(), 1U);
}

TEST_F(FollowCommandTrackerTest, StatusSucceededIsAccepted) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);
    EXPECT_EQ(acks_.size(), 1U);

    status_with(1, vehicle_api::FollowOutcome::Succeeded);

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_ACCEPTED);
}

TEST_F(FollowCommandTrackerTest, StatusFailedIsFailedWithReason) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);

    status_with(1, vehicle_api::FollowOutcome::Failed, "Offboard 진입 실패");

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_FAILED);
    EXPECT_EQ(acks_[1].reason, "Offboard 진입 실패");
}

TEST_F(FollowCommandTrackerTest, RejectedReplyIsTemporarilyRejectedWithoutReason) {
    track(1);

    reply(vehicle_api::FollowOutcome::Rejected, "비행 중 아님");

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_TRUE(acks_[1].reason.empty());
}

TEST_F(FollowCommandTrackerTest, SucceededReplyIsAccepted) {
    track(1);

    reply(vehicle_api::FollowOutcome::Succeeded);

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_ACCEPTED);
}

TEST_F(FollowCommandTrackerTest, NoReplyIsFailed) {
    track(1);

    poller_.process_events(kFollowCommandReplyTimeout, std::chrono::milliseconds{0});

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_FAILED);
    EXPECT_EQ(acks_[1].reason, "추종 제어 응답 없음");
}

TEST_F(FollowCommandTrackerTest, ResultArrivedBeforeReplyIsUsed) {
    track(1);
    status_with(1, vehicle_api::FollowOutcome::Succeeded);
    EXPECT_EQ(acks_.size(), 1U);

    reply(vehicle_api::FollowOutcome::InProgress);

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_ACCEPTED);
}

TEST_F(FollowCommandTrackerTest, IgnoresResultsOfOtherSession) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);

    status_with(1, vehicle_api::FollowOutcome::Succeeded, "", "session-old");

    EXPECT_EQ(acks_.size(), 1U);
}

TEST_F(FollowCommandTrackerTest, IgnoresResultsOfOtherCommand) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);

    status_with(2, vehicle_api::FollowOutcome::Succeeded);

    EXPECT_EQ(acks_.size(), 1U);
}

TEST_F(FollowCommandTrackerTest, FollowStatusLossIsFailed) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);
    status_with(1, vehicle_api::FollowOutcome::InProgress);

    advance_s(3);
    tracker_.tick();
    EXPECT_EQ(acks_.size(), 1U);

    clock_.advance_ns(1);
    tracker_.tick();

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_FAILED);
    EXPECT_EQ(acks_[1].reason, "추종 제어 상태 두절");
}

TEST_F(FollowCommandTrackerTest, NoStatusAfterReplyIsLossFromReplyTime) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);

    advance_s(4);
    tracker_.tick();

    ASSERT_EQ(acks_.size(), 2U);
    EXPECT_EQ(acks_[1].result, MAV_RESULT_FAILED);
}

TEST_F(FollowCommandTrackerTest, ScanSendsProgressFromStatus) {
    track(1, 2.0 * kPi);
    ASSERT_EQ(acks_.size(), 1U);
    EXPECT_EQ(acks_[0].progress, 0);
    reply(vehicle_api::FollowOutcome::InProgress);

    status_with(1, vehicle_api::FollowOutcome::InProgress, "", kSession, 90.0);
    status_with(1, vehicle_api::FollowOutcome::InProgress, "", kSession, 90.0);

    ASSERT_EQ(acks_.size(), 2U);  // 진행률이 바뀔 때만
    EXPECT_EQ(acks_[1].result, MAV_RESULT_IN_PROGRESS);
    EXPECT_EQ(acks_[1].progress, 25);
}

TEST_F(FollowCommandTrackerTest, CompletedCommandIsForgotten) {
    track(1);
    reply(vehicle_api::FollowOutcome::InProgress);
    status_with(1, vehicle_api::FollowOutcome::Succeeded);

    status_with(1, vehicle_api::FollowOutcome::Failed);
    advance_s(10);
    tracker_.tick();

    EXPECT_EQ(acks_.size(), 2U);
}

}  // namespace
}  // namespace argus::mavlink_gateway
