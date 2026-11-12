#include "argus/onboard/udp_mavlink_connection.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "argus/core/fd_watcher.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "poll_fd_watcher.hpp"

namespace argus::onboard {
namespace {

constexpr std::chrono::milliseconds kWait{1000};

// mavlink-router 의 역할을 하는 UDP 소켓. 127.0.0.1 의 임의 포트에 bind 한다
class Peer {
public:
    Peer() : fd_(socket(AF_INET, SOCK_DGRAM, 0)) {
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        local.sin_port = 0;
        bound_ =
            fd_ >= 0 && bind(fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == 0;
    }
    ~Peer() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    Peer(const Peer&) = delete;
    Peer& operator=(const Peer&) = delete;
    Peer(Peer&&) = delete;
    Peer& operator=(Peer&&) = delete;

    bool bound() const { return bound_; }

    bool send_to(std::uint16_t port, const std::uint8_t* data, std::size_t size) const {
        sockaddr_in target{};
        target.sin_family = AF_INET;
        target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        target.sin_port = htons(port);
        const ssize_t sent =
            sendto(fd_, data, size, 0, reinterpret_cast<const sockaddr*>(&target), sizeof(target));
        return sent == static_cast<ssize_t>(size);
    }

    bool send_to(std::uint16_t port, const std::vector<std::uint8_t>& bytes) const {
        return send_to(port, bytes.data(), bytes.size());
    }

    // datagram 하나를 최대 kWait 동안 기다려 받는다. 받지 못한 경우 빈 vector
    std::vector<std::uint8_t> receive() const {
        pollfd waiting{fd_, POLLIN, 0};
        if (poll(&waiting, 1, static_cast<int>(kWait.count())) <= 0) {
            return {};
        }
        std::array<std::uint8_t, 2048> buffer{};
        const ssize_t received = recv(fd_, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            return {};
        }
        return {buffer.begin(), buffer.begin() + received};
    }

private:
    int fd_;
    bool bound_ = false;
};

std::vector<std::uint8_t> to_bytes(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return {buffer.begin(), buffer.begin() + size};
}

mavlink_message_t attitude_message(std::uint32_t time_boot_ms) {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(1, 1, &message, time_boot_ms, 0.1F, 0.2F, 0.3F, 0.0F, 0.0F, 0.0F);
    return message;
}

mavlink_message_t heartbeat_message() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0,
                               MAV_STATE_ACTIVE);
    return message;
}

class UdpMavlinkConnectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(peer_.bound());
        UdpMavlinkConfig config;
        config.local_port = 0;
        connection_ = UdpMavlinkConnection::create(watcher_, config);
        ASSERT_NE(connection_, nullptr);
    }

    std::uint16_t port() const { return connection_->local_port(); }

    PollFdWatcher watcher_;
    Peer peer_;
    std::unique_ptr<UdpMavlinkConnection> connection_;
};

TEST_F(UdpMavlinkConnectionTest, DeliversReceivedMessageToHandlerOfItsMsgid) {
    std::vector<std::uint32_t> attitude_times;
    int heartbeat_count = 0;
    const core::Subscription attitude = connection_->subscribe(
        MAVLINK_MSG_ID_ATTITUDE, [&attitude_times](const mavlink_message_t& message) {
            attitude_times.push_back(mavlink_msg_attitude_get_time_boot_ms(&message));
        });
    const core::Subscription heartbeat =
        connection_->subscribe(MAVLINK_MSG_ID_HEARTBEAT,
                               [&heartbeat_count](const mavlink_message_t&) { ++heartbeat_count; });

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(attitude_message(1234))));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(attitude_times, std::vector<std::uint32_t>{1234});
    EXPECT_EQ(heartbeat_count, 0);
}

TEST_F(UdpMavlinkConnectionTest, AssemblesMessageSplitAcrossDatagrams) {
    int count = 0;
    const core::Subscription subscription = connection_->subscribe(
        MAVLINK_MSG_ID_ATTITUDE, [&count](const mavlink_message_t&) { ++count; });
    const std::vector<std::uint8_t> bytes = to_bytes(attitude_message(1));
    const std::size_t half = bytes.size() / 2;

    ASSERT_TRUE(peer_.send_to(port(), bytes.data(), half));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_EQ(count, 0);

    ASSERT_TRUE(peer_.send_to(port(), bytes.data() + half, bytes.size() - half));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_EQ(count, 1);
}

TEST_F(UdpMavlinkConnectionTest, DeliversEveryMessageInOneDatagram) {
    std::vector<std::uint32_t> times;
    const core::Subscription subscription =
        connection_->subscribe(MAVLINK_MSG_ID_ATTITUDE, [&times](const mavlink_message_t& message) {
            times.push_back(mavlink_msg_attitude_get_time_boot_ms(&message));
        });
    std::vector<std::uint8_t> bytes = to_bytes(attitude_message(1));
    const std::vector<std::uint8_t> second = to_bytes(attitude_message(2));
    bytes.insert(bytes.end(), second.begin(), second.end());

    ASSERT_TRUE(peer_.send_to(port(), bytes));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(times, (std::vector<std::uint32_t>{1, 2}));
}

TEST_F(UdpMavlinkConnectionTest, DiscardsMessageWithWrongCrcAndDeliversNextMessage) {
    std::vector<std::uint32_t> times;
    const core::Subscription subscription =
        connection_->subscribe(MAVLINK_MSG_ID_ATTITUDE, [&times](const mavlink_message_t& message) {
            times.push_back(mavlink_msg_attitude_get_time_boot_ms(&message));
        });
    std::vector<std::uint8_t> corrupted = to_bytes(attitude_message(1));
    corrupted.back() ^= 0xFFU;  // CRC 의 마지막 바이트를 변경

    ASSERT_TRUE(peer_.send_to(port(), corrupted));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_TRUE(times.empty());
    EXPECT_EQ(connection_->discarded_count(), 1U);

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(attitude_message(2))));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_EQ(times, std::vector<std::uint32_t>{2});
}

TEST_F(UdpMavlinkConnectionTest, CallsHandlersOfSameMsgidInRegistrationOrder) {
    std::vector<int> order;
    const core::Subscription first = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&order](const mavlink_message_t&) { order.push_back(1); });
    const core::Subscription second = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&order](const mavlink_message_t&) { order.push_back(2); });

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(order, (std::vector<int>{1, 2}));
}

TEST_F(UdpMavlinkConnectionTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    int count = 0;
    core::Subscription subscription = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&count](const mavlink_message_t&) { ++count; });
    subscription.reset();

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(count, 0);
}

TEST_F(UdpMavlinkConnectionTest, DoesNotCallHandlerReleasedByEarlierHandlerOfSameMessage) {
    int second_count = 0;
    core::Subscription second;
    const core::Subscription first = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&second](const mavlink_message_t&) { second.reset(); });
    second = connection_->subscribe(MAVLINK_MSG_ID_HEARTBEAT,
                                    [&second_count](const mavlink_message_t&) { ++second_count; });

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(second_count, 0);
}

TEST_F(UdpMavlinkConnectionTest, SendFailsBeforeAnyDatagramIsReceived) {
    EXPECT_FALSE(connection_->send(heartbeat_message()));
}

TEST_F(UdpMavlinkConnectionTest, SendsToAddressOfLastReceivedDatagram) {
    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    const mavlink_message_t sent = attitude_message(77);
    EXPECT_TRUE(connection_->send(sent));

    EXPECT_EQ(peer_.receive(), to_bytes(sent));
}

TEST_F(UdpMavlinkConnectionTest, SendsToSenderOfMostRecentValidMessage) {
    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    const Peer second;
    ASSERT_TRUE(second.bound());
    ASSERT_TRUE(second.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    const mavlink_message_t sent = attitude_message(78);
    EXPECT_TRUE(connection_->send(sent));

    EXPECT_EQ(second.receive(), to_bytes(sent));
}

TEST_F(UdpMavlinkConnectionTest, DiscardsPartialMessageWhenSenderChanges) {
    std::vector<std::uint32_t> times;
    const core::Subscription subscription =
        connection_->subscribe(MAVLINK_MSG_ID_ATTITUDE, [&times](const mavlink_message_t& message) {
            times.push_back(mavlink_msg_attitude_get_time_boot_ms(&message));
        });
    const std::vector<std::uint8_t> partial = to_bytes(attitude_message(1));
    ASSERT_TRUE(peer_.send_to(port(), partial.data(), partial.size() / 2));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    // 다른 송신 주소의 메시지는 앞의 미완성 바이트와 이어서 조립하지 않는다
    const Peer second;
    ASSERT_TRUE(second.bound());
    ASSERT_TRUE(second.send_to(port(), to_bytes(attitude_message(2))));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(times, std::vector<std::uint32_t>{2});
}

TEST_F(UdpMavlinkConnectionTest, SubscriptionReleasedAfterConnectionIsDestroyedIsSafe) {
    core::Subscription subscription =
        connection_->subscribe(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t&) {});
    connection_.reset();
    subscription.reset();
    SUCCEED();
}

TEST_F(UdpMavlinkConnectionTest, UnregistersFromWatcherWhenDestroyed) {
    EXPECT_EQ(watcher_.registered_count(), 1U);
    connection_.reset();
    EXPECT_EQ(watcher_.registered_count(), 0U);
}

TEST_F(UdpMavlinkConnectionTest, DiscardsMessageWhoseCrcDoesNotMatchItsDefinition) {
    int count = 0;
    const core::Subscription subscription = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&count](const mavlink_message_t&) { ++count; });
    // HEARTBEAT 의 정의와 다른 CRC_EXTRA 로 만든 메시지 (정의가 다른 송신자의 메시지)
    mavlink_message_t message = heartbeat_message();
    mavlink_finalize_message(&message, 1, 1, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN,
                             MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC + 1);

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(message)));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_EQ(count, 0);
    EXPECT_EQ(connection_->discarded_count(), 1U);
}

TEST_F(UdpMavlinkConnectionTest, DatagramWithoutValidMessageDoesNotSetSendAddress) {
    const std::vector<std::uint8_t> garbage{0x01, 0x02, 0x03};
    ASSERT_TRUE(peer_.send_to(port(), garbage));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_FALSE(connection_->send(heartbeat_message()));
}

TEST_F(UdpMavlinkConnectionTest, DatagramWithoutValidMessageDoesNotReplaceSendAddress) {
    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    const Peer other;
    ASSERT_TRUE(other.bound());
    const std::vector<std::uint8_t> garbage{0x01, 0x02, 0x03};
    ASSERT_TRUE(other.send_to(port(), garbage));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    const mavlink_message_t sent = attitude_message(5);
    EXPECT_TRUE(connection_->send(sent));
    EXPECT_EQ(peer_.receive(), to_bytes(sent));
}

TEST_F(UdpMavlinkConnectionTest, HandlerCanSendReplyToFirstReceivedMessage) {
    const mavlink_message_t reply = attitude_message(9);
    bool sent = false;
    const core::Subscription subscription =
        connection_->subscribe(MAVLINK_MSG_ID_HEARTBEAT,
                               [&](const mavlink_message_t&) { sent = connection_->send(reply); });

    ASSERT_TRUE(peer_.send_to(port(), to_bytes(heartbeat_message())));
    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);

    EXPECT_TRUE(sent);
    EXPECT_EQ(peer_.receive(), to_bytes(reply));
}

TEST_F(UdpMavlinkConnectionTest, ReadsAtMostLimitDatagramsPerCallAndRestOnNextCall) {
    int count = 0;
    const core::Subscription subscription = connection_->subscribe(
        MAVLINK_MSG_ID_HEARTBEAT, [&count](const mavlink_message_t&) { ++count; });
    constexpr int kExtra = 5;
    const std::vector<std::uint8_t> bytes = to_bytes(heartbeat_message());
    for (int index = 0; index < UdpMavlinkConnection::kMaxDatagramsPerEvent + kExtra; ++index) {
        ASSERT_TRUE(peer_.send_to(port(), bytes));
    }

    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_EQ(count, UdpMavlinkConnection::kMaxDatagramsPerEvent);

    EXPECT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    EXPECT_EQ(count, UdpMavlinkConnection::kMaxDatagramsPerEvent + kExtra);
}

TEST(UdpMavlinkConnectionCreateTest, ReturnsNullWhenPortIsAlreadyBound) {
    PollFdWatcher watcher;
    UdpMavlinkConfig config;
    config.local_port = 0;
    const auto first = UdpMavlinkConnection::create(watcher, config);
    ASSERT_NE(first, nullptr);

    config.local_port = first->local_port();
    EXPECT_EQ(UdpMavlinkConnection::create(watcher, config), nullptr);
}

TEST(UdpMavlinkConnectionCreateTest, ReturnsNullWhenAddressIsInvalid) {
    PollFdWatcher watcher;
    UdpMavlinkConfig config;
    config.local_address = "not-an-address";
    EXPECT_EQ(UdpMavlinkConnection::create(watcher, config), nullptr);
}

// 등록하지 못하고 빈 Subscription 을 반환하는 IFdWatcher
class RejectingFdWatcher : public core::IFdWatcher {
public:
    [[nodiscard]] core::Subscription add_fd(int /*fd*/, Handler /*on_readable*/) override {
        return core::Subscription{};
    }
};

TEST(UdpMavlinkConnectionCreateTest, ReturnsNullWhenWatcherRejectsRegistration) {
    RejectingFdWatcher watcher;
    UdpMavlinkConfig config;
    config.local_port = 0;
    EXPECT_EQ(UdpMavlinkConnection::create(watcher, config), nullptr);
}

TEST(UdpMavlinkConnectionCreateTest, WatcherMayBeDestroyedBeforeConnection) {
    auto watcher = std::make_unique<PollFdWatcher>();
    UdpMavlinkConfig config;
    config.local_port = 0;
    auto connection = UdpMavlinkConnection::create(*watcher, config);
    ASSERT_NE(connection, nullptr);

    watcher.reset();
    connection.reset();
    SUCCEED();
}

}  // namespace
}  // namespace argus::onboard
