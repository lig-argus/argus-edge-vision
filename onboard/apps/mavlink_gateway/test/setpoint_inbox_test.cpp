#include "argus/mavlink_gateway/setpoint_inbox.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::int64_t kMaxValidNs = 1'000'000'000;
constexpr std::int64_t kMaxGapNs = 200'000'000;
constexpr std::int64_t kNowNs = 5'000'000'000;

vehicle_api::VelocitySetpoint setpoint(double forward, std::int64_t valid_until_ns) {
    vehicle_api::VelocitySetpoint value;
    value.forward_m_s = forward;
    value.valid_until_monotonic_ns = valid_until_ns;
    return value;
}

TEST(SetpointInboxTest, HasNoValidValueBeforeFirstSetpoint) {
    const SetpointInbox inbox{kMaxValidNs};

    EXPECT_FALSE(inbox.valid_received_ns(kNowNs).has_value());
}

TEST(SetpointInboxTest, ValueIsValidUntilValidUntilInclusive) {
    SetpointInbox inbox{kMaxValidNs};
    inbox.put(setpoint(1.0, kNowNs + 100), kNowNs);

    EXPECT_EQ(inbox.valid_received_ns(kNowNs + 100).value_or(0), kNowNs);
    EXPECT_FALSE(inbox.valid_received_ns(kNowNs + 101).has_value());
    EXPECT_DOUBLE_EQ(inbox.setpoint().forward_m_s, 1.0);
}

TEST(SetpointInboxTest, IgnoresSetpointAlreadyExpiredOnArrival) {
    SetpointInbox inbox{kMaxValidNs};
    inbox.put(setpoint(1.0, kNowNs + 100), kNowNs);
    inbox.put(setpoint(2.0, kNowNs - 1), kNowNs);

    EXPECT_DOUBLE_EQ(inbox.setpoint().forward_m_s, 1.0);
    EXPECT_TRUE(inbox.valid_received_ns(kNowNs).has_value());
}

TEST(SetpointInboxTest, IgnoresSetpointThatCannotBeSentAndKeepsEarlierValue) {
    SetpointInbox inbox{kMaxValidNs};
    inbox.put(setpoint(1.0, kNowNs + 100), kNowNs);

    inbox.put(setpoint(std::nan(""), kNowNs + 500), kNowNs + 50);
    inbox.put(setpoint(std::numeric_limits<double>::infinity(), kNowNs + 500), kNowNs + 50);
    inbox.put(setpoint(1e300, kNowNs + 500), kNowNs + 50);

    EXPECT_DOUBLE_EQ(inbox.setpoint().forward_m_s, 1.0);
    EXPECT_EQ(inbox.valid_received_ns(kNowNs + 100).value_or(0), kNowNs);
    EXPECT_FALSE(inbox.valid_received_ns(kNowNs + 101).has_value());
}

TEST(SetpointInboxTest, LimitsHowLongOneSetpointStaysValid) {
    SetpointInbox inbox{kMaxValidNs};
    inbox.put(setpoint(1.0, kNowNs + 10 * kMaxValidNs), kNowNs);

    EXPECT_TRUE(inbox.valid_received_ns(kNowNs + kMaxValidNs).has_value());
    EXPECT_FALSE(inbox.valid_received_ns(kNowNs + kMaxValidNs + 1).has_value());
}

TEST(SetpointInboxTest, NewerSetpointReplacesEarlierOne) {
    SetpointInbox inbox{kMaxValidNs};
    inbox.put(setpoint(1.0, kNowNs + 100), kNowNs);
    inbox.put(setpoint(2.0, kNowNs + 300), kNowNs + 50);

    EXPECT_DOUBLE_EQ(inbox.setpoint().forward_m_s, 2.0);
    EXPECT_EQ(inbox.valid_received_ns(kNowNs + 300).value_or(0), kNowNs + 50);
}

TEST(SetpointStreamRecordTest, IsNotStreamingBeforeFirstSend) {
    const SetpointStreamRecord stream{kMaxGapNs};

    EXPECT_EQ(stream.streaming_for_ns(kNowNs), 0);
}

TEST(SetpointStreamRecordTest, ReportsTimeFromFirstToLastSend) {
    SetpointStreamRecord stream{kMaxGapNs};
    for (std::int64_t offset = 0; offset <= 1'000'000'000; offset += 50'000'000) {
        stream.record_sent(kNowNs + offset);
    }

    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 1'000'000'000), 1'000'000'000);
    // 마지막 송신 후 간격 한도까지는 송신 중으로 본다
    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 1'000'000'000 + kMaxGapNs), 1'000'000'000);
    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 1'000'000'000 + kMaxGapNs + 1), 0);
}

TEST(SetpointStreamRecordTest, GapLongerThanLimitRestartsDuration) {
    SetpointStreamRecord stream{kMaxGapNs};
    stream.record_sent(kNowNs);
    stream.record_sent(kNowNs + kMaxGapNs);
    stream.record_sent(kNowNs + 2 * kMaxGapNs + 1);
    stream.record_sent(kNowNs + 2 * kMaxGapNs + 101);

    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 2 * kMaxGapNs + 101), 100);
}

TEST(SetpointStreamRecordTest, ResetRestartsDuration) {
    SetpointStreamRecord stream{kMaxGapNs};
    stream.record_sent(kNowNs);
    stream.record_sent(kNowNs + 100);
    stream.reset();

    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 100), 0);
    stream.record_sent(kNowNs + 150);
    stream.record_sent(kNowNs + 170);
    EXPECT_EQ(stream.streaming_for_ns(kNowNs + 170), 20);
}

}  // namespace
}  // namespace argus::mavlink_gateway
