#include "argus/onboard/handler_list.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "argus/core/subscription.hpp"

namespace argus::onboard {
namespace {

TEST(HandlerListTest, CallWithoutHandlersDoesNothing) {
    const HandlerList<int> list;
    EXPECT_TRUE(list.empty());
    list.call(1);
    SUCCEED();
}

TEST(HandlerListTest, PassesArgumentsToHandlersInRegistrationOrder) {
    HandlerList<int> list;
    std::vector<int> calls;
    const core::Subscription first = list.add([&calls](int value) { calls.push_back(value); });
    const core::Subscription second =
        list.add([&calls](int value) { calls.push_back(value * 10); });

    list.call(7);

    EXPECT_EQ(calls, (std::vector<int>{7, 70}));
    EXPECT_FALSE(list.empty());
}

TEST(HandlerListTest, DoesNotCallHandlerAfterSubscriptionIsReleased) {
    HandlerList<> list;
    int count = 0;
    core::Subscription subscription = list.add([&count] { ++count; });
    subscription.reset();

    list.call();

    EXPECT_EQ(count, 0);
    EXPECT_TRUE(list.empty());
}

TEST(HandlerListTest, DoesNotCallHandlerReleasedByEarlierHandler) {
    HandlerList<> list;
    int second_count = 0;
    core::Subscription second;
    const core::Subscription first = list.add([&second] { second.reset(); });
    second = list.add([&second_count] { ++second_count; });

    list.call();

    EXPECT_EQ(second_count, 0);
}

TEST(HandlerListTest, HandlerMayReleaseItsOwnSubscription) {
    HandlerList<> list;
    int count = 0;
    core::Subscription subscription;
    subscription = list.add([&] {
        ++count;
        subscription.reset();
    });

    list.call();
    list.call();

    EXPECT_EQ(count, 1);
}

TEST(HandlerListTest, HandlerAddedDuringCallIsIncludedFromNextCall) {
    HandlerList<> list;
    int added_count = 0;
    core::Subscription added;
    const core::Subscription first = list.add([&] {
        if (!added.active()) {
            added = list.add([&added_count] { ++added_count; });
        }
    });

    list.call();
    EXPECT_EQ(added_count, 0);

    list.call();
    EXPECT_EQ(added_count, 1);
}

TEST(HandlerListTest, SubscriptionReleasedAfterListIsDestroyedIsSafe) {
    auto list = std::make_unique<HandlerList<>>();
    core::Subscription subscription = list->add([] {});
    list.reset();
    subscription.reset();
    SUCCEED();
}

TEST(HandlerListTest, HandlerMayDestroyListDuringCall) {
    auto list = std::make_unique<HandlerList<>>();
    int second_count = 0;
    const core::Subscription first = list->add([&list] { list.reset(); });
    const core::Subscription second = list->add([&second_count] { ++second_count; });

    list->call();

    EXPECT_EQ(list, nullptr);
    EXPECT_EQ(second_count, 1);  // 목록은 call 이 끝날 때까지 유지되며 등록도 해제되지 않았음
}

}  // namespace
}  // namespace argus::onboard
