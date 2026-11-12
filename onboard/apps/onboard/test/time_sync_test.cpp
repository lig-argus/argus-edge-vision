#include "argus/onboard/time_sync.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "fake_fc_epoch_source.hpp"
#include "fake_mavlink_connection.hpp"

namespace argus::onboard {
namespace {

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kStartNs = 100'000 * kNsPerMs;   // RPi 시각 100 s
constexpr std::int64_t kFcAheadNs = 40'000 * kNsPerMs;  // 테스트의 FC 시각 = RPi 시각 - 40 s
constexpr std::uint8_t kFcSysid = 1;
constexpr std::uint8_t kFcCompid = 1;

class TimeSyncTest : public ::testing::Test {
protected:
    void process() {
        poller_.process_events(std::chrono::milliseconds{0}, std::chrono::milliseconds{5});
    }

    // 마지막으로 송신된 요청
    mavlink_timesync_t last_request() const {
        mavlink_timesync_t request{};
        if (!connection_.sent().empty()) {
            mavlink_msg_timesync_decode(&connection_.sent().back(), &request);
        }
        return request;
    }

    // sysid, compid 가 tc1, ts1 의 TIMESYNC 를 송신한 것으로 등록하고 처리한다
    void receive(std::int64_t tc1, std::int64_t ts1, std::uint8_t sysid = kFcSysid,
                 std::uint8_t compid = kFcCompid) {
        mavlink_message_t message{};
        mavlink_msg_timesync_pack(sysid, compid, &message, tc1, ts1, 0, 0);
        connection_.simulate_message_received(message);
        process();
    }

    // 요청을 송신하고 round_trip_ns 후에 FC 의 응답을 받는다. FC 는 왕복의 중간 시점에 응답한다
    void exchange(std::int64_t round_trip_ns) {
        ASSERT_TRUE(time_sync_.send_request());
        const std::int64_t request_ns = clock_.now_monotonic_ns();
        clock_.advance_ns(round_trip_ns);
        receive(request_ns + round_trip_ns / 2 - kFcAheadNs, request_ns);
    }

    core_testing::FakeClock clock_{kStartNs};
    core_testing::FakePoller poller_{clock_};
    FakeMavlinkConnection connection_{poller_};
    FakeFcEpochSource epoch_source_;
    TimeSync time_sync_{connection_, clock_, epoch_source_};
};

TEST_F(TimeSyncTest, HasNoConversionBeforeAnyResponse) {
    EXPECT_FALSE(time_sync_.valid());
    EXPECT_FALSE(time_sync_.to_monotonic_ns(1000).has_value());
}

TEST_F(TimeSyncTest, SendsRequestFromObserverComponentToFc) {
    ASSERT_TRUE(time_sync_.send_request());

    ASSERT_EQ(connection_.sent().size(), 1U);
    const mavlink_message_t& message = connection_.sent().back();
    EXPECT_EQ(message.msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_TIMESYNC));
    EXPECT_EQ(message.sysid, 1);
    EXPECT_EQ(message.compid, MAV_COMP_ID_ONBOARD_COMPUTER);
    const mavlink_timesync_t request = last_request();
    EXPECT_EQ(request.tc1, 0);
    EXPECT_EQ(request.ts1, kStartNs);
    EXPECT_EQ(request.target_system, 1);
    EXPECT_EQ(request.target_component, MAV_COMP_ID_AUTOPILOT1);
}

TEST_F(TimeSyncTest, ReportsSendFailure) {
    connection_.set_send_result(false);
    EXPECT_FALSE(time_sync_.send_request());
}

TEST_F(TimeSyncTest, FailedSendDoesNotReplacePendingRequest) {
    ASSERT_TRUE(time_sync_.send_request());
    const std::int64_t request_ns = clock_.now_monotonic_ns();
    clock_.advance_ns(kNsPerMs);
    connection_.set_send_result(false);
    ASSERT_FALSE(time_sync_.send_request());
    clock_.advance_ns(kNsPerMs);

    receive(request_ns + kNsPerMs - kFcAheadNs, request_ns);

    EXPECT_TRUE(time_sync_.valid());
}

TEST_F(TimeSyncTest, ConvertsFcTimeUsingMatchedResponse) {
    exchange(4 * kNsPerMs);

    EXPECT_TRUE(time_sync_.valid());
    // FC 시각 70 s 는 RPi 시각 110 s
    const std::optional<std::int64_t> converted = time_sync_.to_monotonic_ns(70'000);
    ASSERT_TRUE(converted.has_value());
    EXPECT_EQ(converted.value_or(0), 110'000 * kNsPerMs);
}

TEST_F(TimeSyncTest, IgnoresResponseWithDifferentTs1) {
    ASSERT_TRUE(time_sync_.send_request());
    clock_.advance_ns(kNsPerMs);
    receive(60'000 * kNsPerMs, kStartNs + 1);

    EXPECT_FALSE(time_sync_.valid());
    EXPECT_EQ(time_sync_.rejected_count(), 1U);
}

TEST_F(TimeSyncTest, IgnoresResponseWhenNoRequestIsPending) {
    receive(60'000 * kNsPerMs, kStartNs);

    EXPECT_FALSE(time_sync_.valid());
    EXPECT_EQ(time_sync_.rejected_count(), 1U);
}

TEST_F(TimeSyncTest, IgnoresResponseToReplacedRequest) {
    ASSERT_TRUE(time_sync_.send_request());
    const std::int64_t first_request_ns = clock_.now_monotonic_ns();
    clock_.advance_ns(kNsPerMs);
    ASSERT_TRUE(time_sync_.send_request());
    receive(60'000 * kNsPerMs, first_request_ns);

    EXPECT_FALSE(time_sync_.valid());
}

TEST_F(TimeSyncTest, UsesEachResponseOnlyOnce) {
    exchange(4 * kNsPerMs);
    const std::int64_t request_ns = last_request().ts1;
    // 같은 응답이 다시 도착한 경우. FC 시각을 다르게 넣어 사용 여부를 확인한다
    receive(1, request_ns);

    const std::optional<std::int64_t> converted = time_sync_.to_monotonic_ns(70'000);
    ASSERT_TRUE(converted.has_value());
    EXPECT_EQ(converted.value_or(0), 110'000 * kNsPerMs);
    EXPECT_EQ(time_sync_.rejected_count(), 1U);
}

TEST_F(TimeSyncTest, IgnoresMessageNotFromFc) {
    ASSERT_TRUE(time_sync_.send_request());
    receive(60'000 * kNsPerMs, kStartNs, 250, 190);
    receive(60'000 * kNsPerMs, kStartNs, kFcSysid, MAV_COMP_ID_ONBOARD_COMPUTER);

    EXPECT_FALSE(time_sync_.valid());
}

TEST_F(TimeSyncTest, IgnoresRequestFromFc) {
    ASSERT_TRUE(time_sync_.send_request());
    receive(0, kStartNs);

    EXPECT_FALSE(time_sync_.valid());
    EXPECT_EQ(time_sync_.rejected_count(), 0U);
}

TEST_F(TimeSyncTest, RejectsResponseWithLongRoundTrip) {
    exchange(TimeSyncConfig{}.max_round_trip_ns + 1);

    EXPECT_FALSE(time_sync_.valid());
    EXPECT_EQ(time_sync_.rejected_count(), 1U);
}

TEST_F(TimeSyncTest, AcceptsResponseAtRoundTripLimit) {
    exchange(TimeSyncConfig{}.max_round_trip_ns);
    EXPECT_TRUE(time_sync_.valid());
}

TEST_F(TimeSyncTest, ConversionExpiresWithoutNewResponse) {
    exchange(4 * kNsPerMs);

    clock_.advance_ns(TimeSyncConfig{}.max_age_ns);
    EXPECT_TRUE(time_sync_.valid());
    clock_.advance_ns(1);
    EXPECT_FALSE(time_sync_.valid());
    EXPECT_FALSE(time_sync_.to_monotonic_ns(70'000).has_value());
}

TEST_F(TimeSyncTest, NewFcEpochDiscardsConversionUntilNextResponse) {
    exchange(4 * kNsPerMs);
    ASSERT_TRUE(time_sync_.valid());

    epoch_source_.simulate_epoch_change();
    EXPECT_FALSE(time_sync_.valid());
    EXPECT_FALSE(time_sync_.to_monotonic_ns(300).has_value());

    exchange(4 * kNsPerMs);
    EXPECT_TRUE(time_sync_.valid());
}

TEST_F(TimeSyncTest, NewFcEpochDiscardsPendingRequest) {
    ASSERT_TRUE(time_sync_.send_request());
    epoch_source_.simulate_epoch_change();
    // 왕복 시간 상한 안에 도착한, 폐기된 요청의 응답
    clock_.advance_ns(kNsPerMs);
    receive(60'000 * kNsPerMs, kStartNs);

    EXPECT_FALSE(time_sync_.valid());
    EXPECT_EQ(time_sync_.rejected_count(), 1U);
}

}  // namespace
}  // namespace argus::onboard
