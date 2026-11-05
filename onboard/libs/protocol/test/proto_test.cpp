#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "control_setpoint.pb.h"

namespace argus::protocol {
namespace {

TEST(ProtoTest, SerializedMessageParsesToSameValues) {
    msg::ControlSetpoint sent;
    sent.set_seq(7);
    sent.set_forward_m_s(1.5);
    sent.set_right_m_s(-0.25);
    sent.set_down_m_s(0.0);
    sent.set_yaw_rate_rad_s(0.1745);
    sent.set_valid_until_monotonic_ns(123'456'789'000);

    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));

    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.seq(), 7U);
    EXPECT_DOUBLE_EQ(received.forward_m_s(), 1.5);
    EXPECT_DOUBLE_EQ(received.right_m_s(), -0.25);
    EXPECT_DOUBLE_EQ(received.down_m_s(), 0.0);
    EXPECT_DOUBLE_EQ(received.yaw_rate_rad_s(), 0.1745);
    EXPECT_EQ(received.valid_until_monotonic_ns(), 123'456'789'000);
}

// 발행 쪽이 필드를 추가해도(같은 버전, process_interfaces.md 3장) 받는 쪽은 모르는 필드를 무시하고
// 파싱한다
TEST(ProtoTest, UnknownFieldIsIgnored) {
    msg::ControlSetpoint sent;
    sent.set_seq(1);
    std::string bytes;
    ASSERT_TRUE(sent.SerializeToString(&bytes));
    // 필드 번호 100, wire type varint(0) → tag 0x0320, 값 5
    bytes.push_back(static_cast<char>(0xA0));
    bytes.push_back(static_cast<char>(0x06));
    bytes.push_back(static_cast<char>(0x05));

    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(bytes));
    EXPECT_EQ(received.seq(), 1U);
}

// 보내지 않은 필드는 기본값(0)으로 읽힌다
TEST(ProtoTest, MissingFieldReadsAsDefault) {
    msg::ControlSetpoint received;
    ASSERT_TRUE(received.ParseFromString(std::string{}));
    EXPECT_EQ(received.seq(), 0U);
    EXPECT_DOUBLE_EQ(received.forward_m_s(), 0.0);
    EXPECT_EQ(received.valid_until_monotonic_ns(), 0);
}

}  // namespace
}  // namespace argus::protocol
