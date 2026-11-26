#include "argus/mavlink_gateway/udp_gcs_link.hpp"

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
#include <optional>
#include <string>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "gcs_test_messages.hpp"
#include "poll_fd_watcher.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr std::chrono::milliseconds kWait{1000};

// 통제기 역할을 하는 UDP 소켓. 127.0.0.1 의 임의 포트에 bind 한다
class GcsSocket {
public:
    GcsSocket() : fd_(socket(AF_INET, SOCK_DGRAM, 0)) {
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bound_ =
            fd_ >= 0 && bind(fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == 0;
        socklen_t size = sizeof(local);
        if (bound_ && getsockname(fd_, reinterpret_cast<sockaddr*>(&local), &size) == 0) {
            port_ = ntohs(local.sin_port);
        }
    }
    ~GcsSocket() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    GcsSocket(const GcsSocket&) = delete;
    GcsSocket& operator=(const GcsSocket&) = delete;
    GcsSocket(GcsSocket&&) = delete;
    GcsSocket& operator=(GcsSocket&&) = delete;

    bool bound() const { return bound_; }
    std::uint16_t port() const { return port_; }

    bool send_to(std::uint16_t port, const mavlink_message_t& message) const {
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
        sockaddr_in target{};
        target.sin_family = AF_INET;
        target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        target.sin_port = htons(port);
        return sendto(fd_, buffer.data(), size, 0, reinterpret_cast<const sockaddr*>(&target),
                      sizeof(target)) == static_cast<ssize_t>(size);
    }

    // 메시지 하나를 최대 kWait 동안 기다려 받는다. 받지 못하면 nullopt
    std::optional<mavlink_message_t> receive() const {
        pollfd waiting{fd_, POLLIN, 0};
        if (poll(&waiting, 1, static_cast<std::int32_t>(kWait.count())) <= 0) {
            return std::nullopt;
        }
        std::array<std::uint8_t, 2048> buffer{};
        const ssize_t received = recv(fd_, buffer.data(), buffer.size(), 0);
        mavlink_message_t message{};
        mavlink_status_t status{};
        for (ssize_t index = 0; index < received; ++index) {
            if (mavlink_parse_char(MAVLINK_COMM_3, buffer[static_cast<std::size_t>(index)],
                                   &message, &status) != 0) {
                return message;
            }
        }
        return std::nullopt;
    }

private:
    std::int32_t fd_;
    bool bound_ = false;
    std::uint16_t port_ = 0;
};

class UdpGcsLinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(first_.bound());
        ASSERT_TRUE(second_.bound());
        UdpGcsConfig config;
        config.local_address = "127.0.0.1";
        config.local_port = 0;
        link_ = UdpGcsLink::create(watcher_, config);
        ASSERT_NE(link_, nullptr);
        subscription_ =
            link_->subscribe(MAVLINK_MSG_ID_HEARTBEAT,
                             [this](const mavlink_message_t& /*message*/, const GcsPeer& sender) {
                                 senders_.push_back(sender);
                             });
    }

    // socket 이 sysid 로 HEARTBEAT 를 보내고 link 가 받을 때까지 기다린다
    void heartbeat_from(const GcsSocket& socket, std::uint8_t sysid) {
        const GcsPeer peer = test_messages::gcs_peer(sysid);
        ASSERT_TRUE(socket.send_to(link_->local_port(), test_messages::heartbeat_from(peer)));
        ASSERT_EQ(watcher_.wait_and_dispatch(kWait), 1);
    }

    PollFdWatcher watcher_;
    GcsSocket first_;
    GcsSocket second_;
    std::unique_ptr<UdpGcsLink> link_;
    std::vector<GcsPeer> senders_;
    core::Subscription subscription_;
};

TEST_F(UdpGcsLinkTest, DeliversMessageWithSenderIdsAndAddress) {
    heartbeat_from(first_, 250);

    ASSERT_EQ(senders_.size(), 1U);
    EXPECT_EQ(senders_[0].sysid, 250);
    EXPECT_EQ(senders_[0].compid, kGcsCompid);
    EXPECT_EQ(senders_[0].address, "127.0.0.1");
    EXPECT_EQ(senders_[0].port, first_.port());
}

TEST_F(UdpGcsLinkTest, SendToReachesOnlyThatGcs) {
    heartbeat_from(first_, 250);
    heartbeat_from(second_, 251);

    ASSERT_TRUE(link_->send_to(senders_[1], test_messages::heartbeat_from(senders_[1])));

    EXPECT_TRUE(second_.receive().has_value());
    EXPECT_FALSE(first_.receive().has_value());
}

TEST_F(UdpGcsLinkTest, BroadcastReachesEveryGcsThatSentAMessage) {
    EXPECT_FALSE(link_->broadcast(test_messages::heartbeat_from(test_messages::gcs_peer(1))));
    heartbeat_from(first_, 250);
    heartbeat_from(second_, 251);

    EXPECT_TRUE(link_->broadcast(test_messages::heartbeat_from(test_messages::gcs_peer(1))));

    EXPECT_TRUE(first_.receive().has_value());
    EXPECT_TRUE(second_.receive().has_value());
}

TEST_F(UdpGcsLinkTest, LocalAddressIsTheAddressUsedToReachTheGcs) {
    heartbeat_from(first_, 250);

    EXPECT_EQ(link_->local_address(senders_[0]), std::optional<std::string>{"127.0.0.1"});
    EXPECT_EQ(link_->local_address(GcsPeer{250, kGcsCompid, "not an address", 1}), std::nullopt);
}

TEST_F(UdpGcsLinkTest, SendToRejectsInvalidAddress) {
    EXPECT_FALSE(link_->send_to(GcsPeer{250, kGcsCompid, "not an address", 1},
                                test_messages::heartbeat_from(test_messages::gcs_peer(250))));
}

}  // namespace
}  // namespace argus::mavlink_gateway
