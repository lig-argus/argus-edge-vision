#include "argus/bus_zmq/zmq_poller.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <zmq.hpp>

#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"

namespace argus::bus_zmq {
namespace {

using std::chrono::milliseconds;

constexpr milliseconds kWait{200};     // 이벤트가 올 때까지 기다리는 최대 시간
constexpr milliseconds kBudget{1000};  // 테스트에서 budget 에 걸리지 않을 만큼 큰 값
constexpr std::int64_t kNsPerMs = 1'000'000;

// 같은 프로세스 안에서 연결된 PAIR 소켓 한 쌍. sender 로 보내면 receiver 에 읽을 메시지가 생긴다
struct SocketPair {
    SocketPair(zmq::context_t& context, const std::string& name)
        : receiver(context, zmq::socket_type::pair), sender(context, zmq::socket_type::pair) {
        // inproc 은 같은 context 안에서만 통하는 ZeroMQ 전송 방식이다 (포트를 쓰지 않는다)
        const std::string endpoint = "inproc://" + name;
        receiver.bind(endpoint);
        sender.connect(endpoint);
    }

    void send(const std::string& text) {
        ASSERT_TRUE(sender.send(zmq::buffer(text), zmq::send_flags::dontwait).has_value());
    }

    // 메시지 하나를 읽는다. 없으면 빈 문자열
    std::string receive_one() {
        zmq::message_t message;
        if (!receiver.recv(message, zmq::recv_flags::dontwait).has_value()) {
            return {};
        }
        return message.to_string();
    }

    zmq::socket_t receiver;
    zmq::socket_t sender;
};

// 127.0.0.1 의 임의 포트에 bind 한 UDP 소켓. send_to_self() 로 자기 포트에 데이터그램을 보낸다.
// 소켓 생성이나 bind 에 실패하면 fd() 가 -1 이다
class UdpLoopbackSocket {
public:
    UdpLoopbackSocket() : fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {
        if (fd_ < 0) {
            return;
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        socklen_t length = sizeof(address_);
        if (::bind(fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
            ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address_), &length) != 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    ~UdpLoopbackSocket() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }
    UdpLoopbackSocket(const UdpLoopbackSocket&) = delete;
    UdpLoopbackSocket& operator=(const UdpLoopbackSocket&) = delete;
    UdpLoopbackSocket(UdpLoopbackSocket&&) = delete;
    UdpLoopbackSocket& operator=(UdpLoopbackSocket&&) = delete;

    int fd() const { return fd_; }

    bool send_to_self(const std::string& text) const {
        const auto* address = reinterpret_cast<const sockaddr*>(&address_);
        return ::sendto(fd_, text.data(), text.size(), 0, address, sizeof(address_)) ==
               static_cast<ssize_t>(text.size());
    }

    std::string receive_one() const {
        std::string buffer(64, '\0');
        const ssize_t size = ::recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
        buffer.resize(size > 0 ? static_cast<std::size_t>(size) : 0);
        return buffer;
    }

private:
    int fd_;
    sockaddr_in address_{};
};

class ZmqPollerTest : public ::testing::Test {
protected:
    zmq::context_t context_;
    core::SteadyClock clock_;
    ZmqPoller poller_{clock_};
};

TEST_F(ZmqPollerTest, CallsHandlerWhenSocketHasMessage) {
    SocketPair pair(context_, "socket_event");
    std::vector<std::string> received;
    const core::Subscription subscription =
        poller_.add_socket(pair.receiver, [&] { received.push_back(pair.receive_one()); });
    ASSERT_TRUE(subscription.active());

    pair.send("hello");
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(received, std::vector<std::string>{"hello"});
}

TEST_F(ZmqPollerTest, CallsHandlerWhenFdIsReadable) {
    UdpLoopbackSocket udp;
    ASSERT_GE(udp.fd(), 0);
    std::vector<std::string> received;
    const core::Subscription subscription =
        poller_.add_fd(udp.fd(), [&] { received.push_back(udp.receive_one()); });
    ASSERT_TRUE(subscription.active());

    ASSERT_TRUE(udp.send_to_self("datagram"));
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(received, std::vector<std::string>{"datagram"});
}

TEST_F(ZmqPollerTest, WaitsForZmqSocketAndFdTogether) {
    SocketPair pair(context_, "together");
    UdpLoopbackSocket udp;
    ASSERT_GE(udp.fd(), 0);
    int socket_calls = 0;
    int fd_calls = 0;
    const core::Subscription socket_subscription = poller_.add_socket(pair.receiver, [&] {
        pair.receive_one();
        ++socket_calls;
    });
    const core::Subscription fd_subscription = poller_.add_fd(udp.fd(), [&] {
        udp.receive_one();
        ++fd_calls;
    });

    pair.send("a");
    ASSERT_TRUE(udp.send_to_self("b"));
    // 두 이벤트의 도착 시점이 다를 수 있어 둘 다 처리될 때까지 몇 번 돌린다
    for (int i = 0; i < 10 && (socket_calls == 0 || fd_calls == 0); ++i) {
        poller_.process_events(kWait, kBudget);
    }

    EXPECT_EQ(socket_calls, 1);
    EXPECT_EQ(fd_calls, 1);
}

TEST_F(ZmqPollerTest, ReturnsAfterWaitWhenNothingArrives) {
    SocketPair pair(context_, "nothing");
    int calls = 0;
    const core::Subscription subscription = poller_.add_socket(pair.receiver, [&] { ++calls; });

    const std::int64_t before_ns = clock_.now_monotonic_ns();
    poller_.process_events(milliseconds{20}, kBudget);
    const std::int64_t waited_ns = clock_.now_monotonic_ns() - before_ns;

    EXPECT_EQ(calls, 0);
    EXPECT_GE(waited_ns, 15 * kNsPerMs);  // 타이머 정밀도 여유 5 ms
}

TEST_F(ZmqPollerTest, WaitsWithoutAnyRegistration) {
    const std::int64_t before_ns = clock_.now_monotonic_ns();
    poller_.process_events(milliseconds{20}, kBudget);
    EXPECT_GE(clock_.now_monotonic_ns() - before_ns, 15 * kNsPerMs);
}

TEST_F(ZmqPollerTest, HandlesAllQueuedMessagesInOneCall) {
    SocketPair pair(context_, "queued");
    std::vector<std::string> received;
    // handler 는 한 번에 하나만 읽는다. 남은 메시지는 같은 process_events() 안에서 다시 불려 읽는다
    const core::Subscription subscription =
        poller_.add_socket(pair.receiver, [&] { received.push_back(pair.receive_one()); });

    pair.send("1");
    pair.send("2");
    pair.send("3");
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(received, (std::vector<std::string>{"1", "2", "3"}));
}

TEST_F(ZmqPollerTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    SocketPair pair(context_, "released");
    int calls = 0;
    core::Subscription subscription = poller_.add_socket(pair.receiver, [&] {
        pair.receive_one();
        ++calls;
    });

    subscription.reset();
    pair.send("ignored");
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(calls, 0);
}

TEST_F(ZmqPollerTest, HandlerMayReleaseItsOwnSubscription) {
    SocketPair pair(context_, "self_release");
    int calls = 0;
    core::Subscription subscription;
    subscription = poller_.add_socket(pair.receiver, [&] {
        pair.receive_one();
        ++calls;
        subscription.reset();
    });

    pair.send("first");
    pair.send("second");
    poller_.process_events(kWait, kBudget);
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(calls, 1);
}

TEST_F(ZmqPollerTest, HandlerMayReleaseAnotherSubscription) {
    SocketPair first(context_, "release_other_first");
    SocketPair second(context_, "release_other_second");
    int second_calls = 0;
    core::Subscription second_subscription;
    const core::Subscription first_subscription = poller_.add_socket(first.receiver, [&] {
        first.receive_one();
        second_subscription.reset();
    });
    second_subscription = poller_.add_socket(second.receiver, [&] {
        second.receive_one();
        ++second_calls;
    });

    // 둘 다 읽을 메시지가 있는 상태에서 먼저 불린 handler 가 뒤 등록을 해제한다
    second.send("b");
    first.send("a");
    poller_.process_events(kWait, kBudget);
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(second_calls, 0);
}

TEST_F(ZmqPollerTest, HandlerMayAddRegistration) {
    SocketPair first(context_, "add_first");
    SocketPair second(context_, "add_second");
    int second_calls = 0;
    core::Subscription second_subscription;
    const core::Subscription first_subscription = poller_.add_socket(first.receiver, [&] {
        first.receive_one();
        second_subscription = poller_.add_socket(second.receiver, [&] {
            second.receive_one();
            ++second_calls;
        });
    });

    second.send("b");
    first.send("a");
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(second_calls, 1);
}

TEST_F(ZmqPollerTest, RejectsInvalidRegistration) {
    SocketPair pair(context_, "invalid");
    EXPECT_FALSE(poller_.add_socket(zmq::socket_ref(), [] {}).active());
    EXPECT_FALSE(poller_.add_socket(pair.receiver, nullptr).active());
    EXPECT_FALSE(poller_.add_fd(-1, [] {}).active());
    EXPECT_FALSE(poller_.add_fd(0, nullptr).active());
}

TEST(ZmqPollerLifetimeTest, SubscriptionMayOutlivePoller) {
    zmq::context_t context;
    core::SteadyClock clock;
    SocketPair pair(context, "outlive");
    core::Subscription subscription;
    {
        ZmqPoller poller(clock);
        subscription = poller.add_socket(pair.receiver, [] {});
    }
    subscription.reset();  // poller 가 먼저 사라져도 해제가 안전하다
    EXPECT_FALSE(subscription.active());
}

// budget 테스트는 FakeClock 으로 handler 가 걸린 시간을 정한다 (실제로 기다리지 않는다)
class ZmqPollerBudgetTest : public ::testing::Test {
protected:
    // 메시지를 하나 읽고 FakeClock 을 5 ms 앞으로 옮기는 handler 를 등록한다
    core::Subscription add_slow_handler(SocketPair& pair, const std::string& label) {
        // label 을 const 가 아닌 복사본으로 잡는다 (const 멤버가 있으면 람다를 이동할 수 없다)
        return poller_.add_socket(pair.receiver, [this, &pair, text = label] {
            pair.receive_one();
            clock_.advance_ns(5 * kNsPerMs);
            served_.push_back(text);
        });
    }

    zmq::context_t context_;
    core_testing::FakeClock clock_;
    ZmqPoller poller_{clock_};
    std::vector<std::string> served_;
};

TEST_F(ZmqPollerBudgetTest, LeavesRestForNextCallWhenBudgetIsUsedUp) {
    SocketPair first(context_, "budget_first");
    SocketPair second(context_, "budget_second");
    const core::Subscription first_subscription = add_slow_handler(first, "first");
    const core::Subscription second_subscription = add_slow_handler(second, "second");
    first.send("a");
    second.send("b");

    poller_.process_events(kWait, milliseconds{5});
    EXPECT_EQ(served_, std::vector<std::string>{"first"});

    poller_.process_events(kWait, milliseconds{5});
    EXPECT_EQ(served_, (std::vector<std::string>{"first", "second"}));
}

TEST_F(ZmqPollerBudgetTest, CallsAtLeastOneHandlerWithZeroBudget) {
    SocketPair pair(context_, "zero_budget");
    const core::Subscription subscription = add_slow_handler(pair, "only");
    pair.send("a");
    pair.send("b");

    poller_.process_events(kWait, milliseconds{0});

    EXPECT_EQ(served_, std::vector<std::string>{"only"});
}

TEST_F(ZmqPollerBudgetTest, TakesTurnsWhenBudgetCutsEveryCall) {
    SocketPair first(context_, "turns_first");
    SocketPair second(context_, "turns_second");
    const core::Subscription first_subscription = add_slow_handler(first, "first");
    const core::Subscription second_subscription = add_slow_handler(second, "second");
    first.send("a1");
    first.send("a2");
    second.send("b1");
    second.send("b2");

    for (int i = 0; i < 4; ++i) {
        poller_.process_events(kWait, milliseconds{5});
    }

    // 먼저 등록한 소켓에 메시지가 계속 있어도 뒤 소켓이 밀리지 않는다
    EXPECT_EQ(served_, (std::vector<std::string>{"first", "second", "first", "second"}));
}

TEST_F(ZmqPollerBudgetTest, CallsTimerOnceWhenDeadlineIsReached) {
    int calls = 0;
    const core::Subscription timer = poller_.add_timer_at(10 * kNsPerMs, [&] { ++calls; });
    ASSERT_TRUE(timer.active());

    poller_.process_events(milliseconds{0}, kBudget);
    EXPECT_EQ(calls, 0);

    clock_.advance_ns(10 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);
    EXPECT_EQ(calls, 1);

    clock_.advance_ns(10 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);
    EXPECT_EQ(calls, 1);
}

TEST_F(ZmqPollerBudgetTest, CallsDueTimersInDeadlineOrder) {
    const core::Subscription late =
        poller_.add_timer_at(20 * kNsPerMs, [&] { served_.emplace_back("late"); });
    const core::Subscription early =
        poller_.add_timer_at(10 * kNsPerMs, [&] { served_.emplace_back("early"); });

    clock_.advance_ns(30 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);

    EXPECT_EQ(served_, (std::vector<std::string>{"early", "late"}));
}

TEST_F(ZmqPollerBudgetTest, DoesNotCallReleasedTimer) {
    int calls = 0;
    core::Subscription timer = poller_.add_timer_at(10 * kNsPerMs, [&] { ++calls; });

    timer.reset();
    clock_.advance_ns(20 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);

    EXPECT_EQ(calls, 0);
    EXPECT_FALSE(poller_.add_timer_at(0, nullptr).active());
}

TEST_F(ZmqPollerTest, WakesUpAtTimerDeadlineBeforeWaitEnds) {
    int calls = 0;
    const std::int64_t before_ns = clock_.now_monotonic_ns();
    const core::Subscription timer =
        poller_.add_timer_at(before_ns + 20 * kNsPerMs, [&] { ++calls; });

    poller_.process_events(milliseconds{2000}, kBudget);

    EXPECT_EQ(calls, 1);
    EXPECT_LT(clock_.now_monotonic_ns() - before_ns, 1000 * kNsPerMs);
}

}  // namespace
}  // namespace argus::bus_zmq
