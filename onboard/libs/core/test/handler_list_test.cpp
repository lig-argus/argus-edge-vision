#include "argus/core/handler_list.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "argus/core/subscription.hpp"

namespace argus::core {
namespace {

TEST(HandlerListTest, NotifyWithoutHandlersDoesNothing) {
    HandlerList<std::int32_t> handlers;

    handlers.notify(1);
}

TEST(HandlerListTest, HandlerReceivesArguments) {
    HandlerList<std::int32_t, const std::string&> handlers;
    std::int32_t received_number = 0;
    std::string received_text;
    const Subscription subscription =
        handlers.subscribe([&](std::int32_t number, const std::string& text) {
            received_number = number;
            received_text = text;
        });

    handlers.notify(7, "frame");

    EXPECT_EQ(received_number, 7);
    EXPECT_EQ(received_text, "frame");
}

TEST(HandlerListTest, CallsHandlersInSubscribeOrder) {
    HandlerList<> handlers;
    std::vector<std::int32_t> calls;
    const Subscription first = handlers.subscribe([&] { calls.push_back(1); });
    const Subscription second = handlers.subscribe([&] { calls.push_back(2); });
    const Subscription third = handlers.subscribe([&] { calls.push_back(3); });

    handlers.notify();

    EXPECT_EQ(calls, (std::vector<std::int32_t>{1, 2, 3}));
}

TEST(HandlerListTest, ReleasedHandlerIsNotCalled) {
    HandlerList<> handlers;
    std::int32_t released_calls = 0;
    std::int32_t kept_calls = 0;
    Subscription released = handlers.subscribe([&] { ++released_calls; });
    const Subscription kept = handlers.subscribe([&] { ++kept_calls; });

    released.reset();
    handlers.notify();

    EXPECT_EQ(released_calls, 0);
    EXPECT_EQ(kept_calls, 1);
}

TEST(HandlerListTest, HandlerMayReleaseLaterHandlerDuringNotify) {
    HandlerList<> handlers;
    std::int32_t later_calls = 0;
    Subscription later;
    const Subscription earlier = handlers.subscribe([&] { later.reset(); });
    later = handlers.subscribe([&] { ++later_calls; });

    handlers.notify();

    EXPECT_EQ(later_calls, 0);
}

TEST(HandlerListTest, HandlerMayReleaseItselfDuringNotify) {
    HandlerList<> handlers;
    std::int32_t calls = 0;
    std::int32_t other_calls = 0;
    Subscription self;
    // handler 가 보관한 값을 해제 뒤에도 읽는다. 실행 중인 함수가 유지되지 않으면 잘못된 값을
    // 읽는다
    const auto marker = std::make_shared<std::int32_t>(42);
    std::int32_t value_after_release = 0;
    self = handlers.subscribe([&, marker] {
        ++calls;
        self.reset();
        value_after_release = *marker;
    });
    const Subscription other = handlers.subscribe([&] { ++other_calls; });

    handlers.notify();
    handlers.notify();

    EXPECT_EQ(calls, 1);
    EXPECT_EQ(value_after_release, 42);
    EXPECT_EQ(other_calls, 2);
}

TEST(HandlerListTest, HandlerSubscribedDuringNotifyIsCalledFromNextNotify) {
    HandlerList<> handlers;
    std::int32_t added_calls = 0;
    Subscription added;
    const Subscription adder = handlers.subscribe([&] {
        if (!added.active()) {
            added = handlers.subscribe([&] { ++added_calls; });
        }
    });

    handlers.notify();
    EXPECT_EQ(added_calls, 0);

    handlers.notify();
    EXPECT_EQ(added_calls, 1);
}

TEST(HandlerListTest, SubscriptionMayOutliveHandlerList) {
    Subscription subscription;
    {
        HandlerList<> handlers;
        subscription = handlers.subscribe([] {});
        EXPECT_TRUE(subscription.active());
    }

    subscription.reset();  // 목록이 없으므로 아무것도 하지 않는다

    EXPECT_FALSE(subscription.active());
}

}  // namespace
}  // namespace argus::core
