#include "argus/core/poll_poller.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "argus/core/fd_watcher.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core/timer_scheduler.hpp"
#include "argus/core_testing/fake_clock.hpp"

namespace argus::core {
namespace {

using std::chrono::milliseconds;

constexpr milliseconds kWait{200};     // 이벤트가 올 때까지 기다리는 최대 시간
constexpr milliseconds kBudget{1000};  // 테스트에서 budget 에 걸리지 않을 만큼 큰 값
constexpr std::int64_t kNsPerMs = 1'000'000;

// pipe 한 쌍. write_byte() 로 쓰면 read_fd() 에 읽을 데이터가 생긴다.
// 생성에 실패하면 read_fd() 가 -1 이다
class Pipe {
public:
    Pipe() {
        std::array<int, 2> fds{-1, -1};
        if (::pipe(fds.data()) == 0) {
            read_fd_ = fds[0];
            write_fd_ = fds[1];
        }
    }
    ~Pipe() {
        if (read_fd_ >= 0) {
            ::close(read_fd_);
            ::close(write_fd_);
        }
    }
    Pipe(const Pipe&) = delete;
    Pipe& operator=(const Pipe&) = delete;
    Pipe(Pipe&&) = delete;
    Pipe& operator=(Pipe&&) = delete;

    int read_fd() const { return read_fd_; }

    bool write_byte(char value) const { return ::write(write_fd_, &value, 1) == 1; }

    // 1 바이트를 읽는다. 읽지 못하면 '\0'
    char read_byte() const {
        char value = '\0';
        return ::read(read_fd_, &value, 1) == 1 ? value : '\0';
    }

private:
    int read_fd_ = -1;
    int write_fd_ = -1;
};

class PollPollerTest : public ::testing::Test {
protected:
    SteadyClock clock_;
    PollPoller poller_{clock_};
};

TEST_F(PollPollerTest, CallsHandlerWhenFdIsReadable) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    std::vector<char> received;
    const Subscription subscription =
        poller_.add_fd(pipe.read_fd(), [&] { received.push_back(pipe.read_byte()); });
    ASSERT_TRUE(subscription.active());

    ASSERT_TRUE(pipe.write_byte('a'));
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(received, std::vector<char>{'a'});
}

// 통신 구현은 PollPoller 를 모르고 IFdWatcher, ITimerScheduler 로만 등록한다
TEST_F(PollPollerTest, WorksThroughFdWatcherAndTimerSchedulerInterfaces) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    IFdWatcher& watcher = poller_;
    ITimerScheduler& scheduler = poller_;
    std::int32_t fd_calls = 0;
    std::int32_t timer_calls = 0;
    const Subscription fd_subscription = watcher.add_fd(pipe.read_fd(), [&] {
        pipe.read_byte();
        ++fd_calls;
    });
    const Subscription timer =
        scheduler.add_timer_at(clock_.now_monotonic_ns(), [&] { ++timer_calls; });

    ASSERT_TRUE(pipe.write_byte('a'));
    poller_.process_events(kWait, kBudget);

    EXPECT_EQ(fd_calls, 1);
    EXPECT_EQ(timer_calls, 1);
}

TEST_F(PollPollerTest, ReturnsAfterWaitWhenNothingArrives) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    std::int32_t calls = 0;
    const Subscription subscription = poller_.add_fd(pipe.read_fd(), [&] { ++calls; });

    const std::int64_t before_ns = clock_.now_monotonic_ns();
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(calls, 0);
    EXPECT_GE(clock_.now_monotonic_ns() - before_ns, 15 * kNsPerMs);
}

TEST_F(PollPollerTest, WaitsWithoutAnyRegistration) {
    const std::int64_t before_ns = clock_.now_monotonic_ns();
    poller_.process_events(milliseconds{20}, kBudget);
    EXPECT_GE(clock_.now_monotonic_ns() - before_ns, 15 * kNsPerMs);
}

TEST_F(PollPollerTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    std::int32_t calls = 0;
    Subscription subscription = poller_.add_fd(pipe.read_fd(), [&] {
        pipe.read_byte();
        ++calls;
    });

    subscription.reset();
    ASSERT_TRUE(pipe.write_byte('a'));
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(calls, 0);
}

TEST_F(PollPollerTest, HandlerMayReleaseItsOwnSubscription) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    std::int32_t calls = 0;
    Subscription subscription;
    subscription = poller_.add_fd(pipe.read_fd(), [&] {
        ++calls;
        subscription.reset();  // 읽지 않고 해제한다
    });

    ASSERT_TRUE(pipe.write_byte('a'));
    poller_.process_events(kWait, kBudget);
    poller_.process_events(milliseconds{20}, kBudget);

    EXPECT_EQ(calls, 1);
}

TEST_F(PollPollerTest, RejectsInvalidRegistration) {
    EXPECT_FALSE(poller_.add_fd(-1, [] {}).active());
    EXPECT_FALSE(poller_.add_fd(0, nullptr).active());
    EXPECT_FALSE(poller_.add_timer_at(0, nullptr).active());
}

TEST_F(PollPollerTest, WakesUpAtTimerDeadlineBeforeWaitEnds) {
    std::int32_t calls = 0;
    const std::int64_t before_ns = clock_.now_monotonic_ns();
    const Subscription timer = poller_.add_timer_at(before_ns + 20 * kNsPerMs, [&] { ++calls; });

    poller_.process_events(milliseconds{1000}, kBudget);

    EXPECT_EQ(calls, 1);
    EXPECT_LT(clock_.now_monotonic_ns() - before_ns, 500 * kNsPerMs);
}

TEST(PollPollerLifetimeTest, SubscriptionMayOutlivePoller) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    SteadyClock clock;
    Subscription fd_subscription;
    Subscription timer;
    {
        PollPoller poller(clock);
        fd_subscription = poller.add_fd(pipe.read_fd(), [] {});
        timer = poller.add_timer_at(0, [] {});
    }
    fd_subscription.reset();  // poller 가 먼저 사라져도 해제가 안전하다
    timer.reset();
    EXPECT_FALSE(fd_subscription.active());
}

// budget 과 타이머 테스트는 FakeClock 으로 handler 가 걸린 시간과 현재 시각을 정한다
class PollPollerBudgetTest : public ::testing::Test {
protected:
    // 1 바이트를 읽고 FakeClock 을 5 ms 앞으로 옮기는 handler 를 등록한다
    Subscription add_slow_handler(const Pipe& pipe, const std::string& label) {
        return poller_.add_fd(pipe.read_fd(), [this, &pipe, text = label] {
            pipe.read_byte();
            clock_.advance_ns(5 * kNsPerMs);
            served_.push_back(text);
        });
    }

    core_testing::FakeClock clock_;
    PollPoller poller_{clock_};
    std::vector<std::string> served_;
};

TEST_F(PollPollerBudgetTest, LeavesRestForNextCallWhenBudgetIsUsedUp) {
    Pipe first;
    Pipe second;
    ASSERT_GE(first.read_fd(), 0);
    ASSERT_GE(second.read_fd(), 0);
    const Subscription first_subscription = add_slow_handler(first, "first");
    const Subscription second_subscription = add_slow_handler(second, "second");
    ASSERT_TRUE(first.write_byte('a'));
    ASSERT_TRUE(second.write_byte('b'));

    poller_.process_events(kWait, milliseconds{5});
    EXPECT_EQ(served_, std::vector<std::string>{"first"});

    poller_.process_events(kWait, milliseconds{5});
    EXPECT_EQ(served_, (std::vector<std::string>{"first", "second"}));
}

TEST_F(PollPollerBudgetTest, CallsAtLeastOneHandlerWithZeroBudget) {
    Pipe pipe;
    ASSERT_GE(pipe.read_fd(), 0);
    const Subscription subscription = add_slow_handler(pipe, "only");
    ASSERT_TRUE(pipe.write_byte('a'));
    ASSERT_TRUE(pipe.write_byte('b'));

    poller_.process_events(kWait, milliseconds{0});

    EXPECT_EQ(served_, std::vector<std::string>{"only"});
}

TEST_F(PollPollerBudgetTest, TakesTurnsWhenBudgetCutsEveryCall) {
    Pipe first;
    Pipe second;
    ASSERT_GE(first.read_fd(), 0);
    ASSERT_GE(second.read_fd(), 0);
    const Subscription first_subscription = add_slow_handler(first, "first");
    const Subscription second_subscription = add_slow_handler(second, "second");
    for (const char value : {'a', 'b'}) {
        ASSERT_TRUE(first.write_byte(value));
        ASSERT_TRUE(second.write_byte(value));
    }

    for (std::int32_t i = 0; i < 4; ++i) {
        poller_.process_events(kWait, milliseconds{5});
    }

    // 먼저 등록한 fd 에 데이터가 계속 있어도 뒤 fd 가 밀리지 않는다
    EXPECT_EQ(served_, (std::vector<std::string>{"first", "second", "first", "second"}));
}

TEST_F(PollPollerBudgetTest, CallsTimerOnceWhenDeadlineIsReached) {
    std::int32_t calls = 0;
    const Subscription timer = poller_.add_timer_at(10 * kNsPerMs, [&] { ++calls; });
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

TEST_F(PollPollerBudgetTest, CallsDueTimersInDeadlineOrder) {
    const Subscription late =
        poller_.add_timer_at(20 * kNsPerMs, [&] { served_.emplace_back("late"); });
    const Subscription early =
        poller_.add_timer_at(10 * kNsPerMs, [&] { served_.emplace_back("early"); });

    clock_.advance_ns(30 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);

    EXPECT_EQ(served_, (std::vector<std::string>{"early", "late"}));
}

TEST_F(PollPollerBudgetTest, DoesNotCallReleasedTimer) {
    std::int32_t calls = 0;
    Subscription timer = poller_.add_timer_at(10 * kNsPerMs, [&] { ++calls; });

    timer.reset();
    clock_.advance_ns(20 * kNsPerMs);
    poller_.process_events(milliseconds{0}, kBudget);

    EXPECT_EQ(calls, 0);
}

}  // namespace
}  // namespace argus::core
