// Client Fake(FakeFollowControlClient, FakeFlightModeControlClient)가 공유하는 응답 대기 목록.
// 요청마다 handler 와 시간 제한을 기억해 두고, 테스트가 넣은 응답 또는 시간 초과(nullopt)를
// FakePoller 를 거쳐 process_events() 안에서 handler 에 전달한다. 실제 구현(bus::IRpcClient)과
// 같이 handler 는 정확히 한 번 불리고, Subscription 이 먼저 사라지면 불리지 않는다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_PENDING_REPLIES_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_PENDING_REPLIES_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::vehicle_api_testing {

template <typename Reply>
class FakePendingReplies {
public:
    using Handler = std::function<void(std::optional<Reply>)>;

    FakePendingReplies(core_testing::FakePoller& poller, const core::IClock& clock)
        : poller_(poller), clock_(clock), state_(std::make_shared<State>()) {}

    // 요청 하나를 등록한다. timeout 이 지날 때까지 reply() 가 없으면 handler(nullopt)
    [[nodiscard]] core::Subscription add(std::chrono::milliseconds timeout, Handler handler) {
        const std::uint64_t id = state_->next_id++;
        state_->pending[id] = std::move(handler);
        const std::weak_ptr<State> weak = state_;
        poller_.post_at(clock_.now_monotonic_ns() + timeout.count() * kNsPerMs,
                        [weak, id] { finish(weak, id, std::nullopt); });
        return core::Subscription([weak, id] {
            if (const auto state = weak.lock()) {
                state->pending.erase(id);
            }
        });
    }

    // 아직 응답을 넣지 않은 요청 중 가장 오래 기다린 것에 응답을 넣는다. 다음 process_events() 에서
    // handler 에 전달된다. 연달아 두 번 부르면 두 요청에 순서대로 응답한다. 남은 요청이 없으면
    // false
    bool reply(Reply reply) {
        const auto next = state_->pending.lower_bound(state_->next_reply_id);
        if (next == state_->pending.end()) {
            return false;
        }
        const std::uint64_t id = next->first;
        state_->next_reply_id = id + 1;
        const std::weak_ptr<State> weak = state_;
        poller_.post([weak, id, reply = std::move(reply)] { finish(weak, id, reply); });
        return true;
    }

    // 응답을 아직 받지 못한 요청 수
    std::size_t pending_count() const { return state_->pending.size(); }

private:
    static constexpr std::int64_t kNsPerMs = 1'000'000;

    struct State {
        std::map<std::uint64_t, Handler> pending;
        std::uint64_t next_id = 1;
        std::uint64_t next_reply_id = 1;  // reply() 가 다음에 응답할 요청 id 의 하한
    };

    static void finish(const std::weak_ptr<State>& weak, std::uint64_t id,
                       const std::optional<Reply>& reply) {
        const auto state = weak.lock();
        if (!state) {
            return;
        }
        const auto found = state->pending.find(id);
        if (found == state->pending.end()) {
            return;  // 이미 응답했거나 Subscription 이 먼저 사라짐
        }
        const Handler handler = std::move(found->second);
        state->pending.erase(found);
        handler(reply);
    }

    core_testing::FakePoller& poller_;
    const core::IClock& clock_;
    std::shared_ptr<State> state_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_PENDING_REPLIES_HPP_
