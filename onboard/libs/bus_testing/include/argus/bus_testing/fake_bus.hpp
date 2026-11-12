// 테스트용 IBus. 한 FakeBus 가 만든 발행자, 구독자, 요청하는 쪽, 응답하는 쪽은 같은 bus 에 붙은
// 것처럼 동작한다. 발행하거나 simulate_message_received() 로 도착한 것으로 등록한 메시지는
// FakePoller 에 쌓였다가 process_events() 안에서 구독자에게 간다. 요청은 같은 FakeBus 에 serve() 로
// 등록한 handler 가 응답하고, 없으면 timeout 뒤 nullopt 가 된다.
// queue_depth (실제 구현과 같은 contract): 큐는 구독자마다 따로 있다. process_events() 가 아직
// 전달하지 않은 메시지가 그 구독자의 queue_depth 개를 넘으면 가장 오래된 메시지부터 폐기하고 최근
// queue_depth 개를 받은 순서대로 전달한다. publish() 는 큐 길이와 관계없이 true 를 반환한다. 발행
// 실패를 처리하는 코드는 simulate_publish_failure() 로 테스트한다
#ifndef ARGUS_BUS_TESTING_FAKE_BUS_HPP_
#define ARGUS_BUS_TESTING_FAKE_BUS_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "argus/bus/bus.hpp"
#include "argus/bus/message.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::bus_testing {

class FakeBus : public bus::IBus {
public:
    FakeBus(core_testing::FakePoller& poller, const core::IClock& clock)
        : state_(std::make_shared<State>(poller, clock)) {}

    std::unique_ptr<bus::ITopicPublisher> create_publisher(std::string_view topic,
                                                           int queue_depth) override {
        if (!check_name(topic) || !check_depth(queue_depth)) {
            return nullptr;
        }
        return std::make_unique<Publisher>(state_, std::string(topic));
    }

    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(std::string_view topic,
                                                             int queue_depth) override {
        if (!check_name(topic) || !check_depth(queue_depth)) {
            return nullptr;
        }
        return std::make_unique<Subscriber>(state_, std::string(topic), queue_depth);
    }

    std::unique_ptr<bus::IRequester> create_requester(std::string_view name) override {
        if (!check_name(name)) {
            return nullptr;
        }
        return std::make_unique<Requester>(state_, std::string(name));
    }

    std::unique_ptr<bus::IReplier> create_replier(std::string_view name) override {
        if (!check_name(name)) {
            return nullptr;
        }
        return std::make_unique<Replier>(state_, std::string(name));
    }

    // 테스트용: 다른 프로그램이 보낸 메시지가 도착한 것처럼 넣는다
    void simulate_message_received(const bus::BusMessage& message) {
        State::deliver(state_, message);
    }

    // 테스트용: true 로 설정한 동안 모든 발행자의 publish() 가 false 를 반환하고 메시지를 보내지
    // 않는다 (누구에게도 보내지 못한 경우)
    void simulate_publish_failure(bool fail) { state_->publish_fails = fail; }

    // 테스트용: publish() 로 발행된 메시지 기록 (모든 토픽, 발행 순서)
    const std::vector<bus::BusMessage>& published() const { return state_->published; }

private:
    static constexpr std::int64_t kNsPerMs = 1'000'000;

    using Handler = bus::ITopicSubscriber::Handler;
    using RequestHandler = bus::IReplier::RequestHandler;
    using ReplyHandler = bus::IRequester::ReplyHandler;

    // 구독자 하나의 큐와 handler 들
    struct SubscriberQueue {
        std::size_t depth = 1;
        std::deque<bus::BusMessage> pending;        // 받은 순서. 아직 전달하지 않은 메시지
        std::map<std::uint64_t, Handler> handlers;  // 등록한 순서
        bool drain_posted = false;                  // 전달 이벤트를 poller 에 이미 넣었음
    };

    struct ReplierEntry {
        std::uint64_t id = 0;
        RequestHandler handler;
    };

    // 만든 객체와 Subscription 이 FakeBus 보다 오래 살아도 안전하도록 등록 정보는 공유 객체에 둔다
    struct State {
        State(core_testing::FakePoller& p, const core::IClock& c) : poller(p), clock(c) {}

        core_testing::FakePoller& poller;
        const core::IClock& clock;
        std::uint64_t next_id = 1;
        // 토픽 → (구독자 번호 → 큐). 구독자를 만든 순서대로 전달한다
        std::map<std::string, std::map<std::uint64_t, std::shared_ptr<SubscriberQueue>>>
            subscribers;
        bool publish_fails = false;
        std::map<std::string, ReplierEntry> repliers;
        std::map<std::uint64_t, ReplyHandler> pending_requests;
        std::vector<bus::BusMessage> published;

        // 메시지를 그 토픽의 구독자마다 큐에 넣고, 다음 process_events() 에서 전달하도록 예약한다.
        // 큐에 depth 개가 있으면 가장 오래된 메시지를 폐기한다
        static void deliver(const std::shared_ptr<State>& state, const bus::BusMessage& message) {
            const auto found = state->subscribers.find(message.topic);
            if (found == state->subscribers.end()) {
                return;
            }
            for (const auto& [id, queue] : found->second) {
                if (queue->pending.size() == queue->depth) {
                    queue->pending.pop_front();
                }
                queue->pending.push_back(message);
                if (!queue->drain_posted) {
                    queue->drain_posted = true;
                    const std::weak_ptr<SubscriberQueue> weak = queue;
                    state->poller.post([weak] { drain(weak); });
                }
            }
        }

        // 구독자의 큐에 있는 메시지를 받은 순서대로 handler 에 전달한다
        static void drain(const std::weak_ptr<SubscriberQueue>& weak) {
            const auto queue = weak.lock();
            if (!queue) {
                return;  // 구독자가 사라짐
            }
            queue->drain_posted = false;
            std::deque<bus::BusMessage> messages;
            messages.swap(queue->pending);
            for (const bus::BusMessage& message : messages) {
                // handler 가 구독을 풀어도 안전하도록 복사해서 부른다
                const auto handlers = queue->handlers;
                for (const auto& [id, handler] : handlers) {
                    handler(message);
                }
            }
        }

        template <typename Release>
        static core::Subscription release(const std::shared_ptr<State>& state, Release remove) {
            const std::weak_ptr<State> weak = state;
            return core::Subscription([weak, remove = std::move(remove)] {
                if (const auto locked = weak.lock()) {
                    remove(*locked);
                }
            });
        }
    };

    class Publisher : public bus::ITopicPublisher {
    public:
        Publisher(std::shared_ptr<State> state, std::string topic)
            : state_(std::move(state)), topic_(std::move(topic)) {}

        std::string_view topic() const override { return topic_; }

        bool publish(const bus::Bytes& body, const bus::Bytes& binary) override {
            if (state_->publish_fails) {
                return false;
            }
            const bus::BusMessage message{topic_, body, binary};
            state_->published.push_back(message);
            State::deliver(state_, message);
            return true;
        }

    private:
        std::shared_ptr<State> state_;
        std::string topic_;
    };

    class Subscriber : public bus::ITopicSubscriber {
    public:
        Subscriber(std::shared_ptr<State> state, std::string topic, int queue_depth)
            : state_(std::move(state)),
              topic_(std::move(topic)),
              id_(state_->next_id++),
              queue_(std::make_shared<SubscriberQueue>()) {
            queue_->depth = static_cast<std::size_t>(queue_depth);
            state_->subscribers[topic_][id_] = queue_;
        }
        // 소멸하면 아직 전달하지 않은 메시지는 폐기된다 (실제 구현과 같은 contract)
        ~Subscriber() override {
            const auto found = state_->subscribers.find(topic_);
            if (found != state_->subscribers.end()) {
                found->second.erase(id_);
            }
        }
        Subscriber(const Subscriber&) = delete;
        Subscriber& operator=(const Subscriber&) = delete;
        Subscriber(Subscriber&&) = delete;
        Subscriber& operator=(Subscriber&&) = delete;

        std::string_view topic() const override { return topic_; }

        [[nodiscard]] core::Subscription subscribe(Handler handler) override {
            const std::uint64_t id = state_->next_id++;
            queue_->handlers[id] = std::move(handler);
            const std::weak_ptr<SubscriberQueue> weak = queue_;
            return core::Subscription([weak, id] {
                if (const auto queue = weak.lock()) {
                    queue->handlers.erase(id);
                }
            });
        }

    private:
        std::shared_ptr<State> state_;
        std::string topic_;
        std::uint64_t id_;
        std::shared_ptr<SubscriberQueue> queue_;
    };

    class Requester : public bus::IRequester {
    public:
        Requester(std::shared_ptr<State> state, std::string name)
            : state_(std::move(state)), name_(std::move(name)) {}

        std::string_view name() const override { return name_; }

        [[nodiscard]] core::Subscription request(const bus::Bytes& request,
                                                 std::chrono::milliseconds timeout,
                                                 ReplyHandler on_reply) override {
            const std::uint64_t id = state_->next_id++;
            state_->pending_requests[id] = std::move(on_reply);
            const std::weak_ptr<State> weak = state_;
            const auto finish = [weak, id](const std::optional<bus::BusMessage>& reply) {
                const auto state = weak.lock();
                if (!state) {
                    return;
                }
                const auto found = state->pending_requests.find(id);
                if (found == state->pending_requests.end()) {
                    return;  // Subscription 이 먼저 사라짐
                }
                const ReplyHandler handler = std::move(found->second);
                state->pending_requests.erase(found);
                handler(reply);
            };

            const auto replier = state_->repliers.find(name_);
            if (replier == state_->repliers.end()) {
                state_->poller.post_at(
                    state_->clock.now_monotonic_ns() + timeout.count() * kNsPerMs,
                    [finish] { finish(std::nullopt); });
            } else {
                const auto shared_request =
                    std::make_shared<const bus::BusMessage>(bus::BusMessage{name_, request, {}});
                state_->poller.post(
                    [finish, handler = replier->second.handler, shared_request, name = name_] {
                        finish(bus::BusMessage{name, handler(*shared_request), {}});
                    });
            }
            return State::release(state_, [id](State& state) { state.pending_requests.erase(id); });
        }

    private:
        std::shared_ptr<State> state_;
        std::string name_;
    };

    class Replier : public bus::IReplier {
    public:
        Replier(std::shared_ptr<State> state, std::string name)
            : state_(std::move(state)), name_(std::move(name)) {}

        std::string_view name() const override { return name_; }

        [[nodiscard]] core::Subscription serve(RequestHandler handler) override {
            const std::uint64_t id = state_->next_id++;
            state_->repliers[name_] = ReplierEntry{id, std::move(handler)};
            return State::release(state_, [name = name_, id](State& state) {
                const auto found = state.repliers.find(name);
                if (found != state.repliers.end() && found->second.id == id) {
                    state.repliers.erase(found);  // 다른 등록이 대신했으면 그대로 둔다
                }
            });
        }

    private:
        std::shared_ptr<State> state_;
        std::string name_;
    };

    // 실패 이유는 로그로 남기는 것이 contract 이지만 테스트용이므로 남기지 않는다
    static bool check_name(std::string_view name) { return !name.empty(); }
    static bool check_depth(int queue_depth) { return queue_depth >= 1; }

    std::shared_ptr<State> state_;
};

}  // namespace argus::bus_testing

#endif  // ARGUS_BUS_TESTING_FAKE_BUS_HPP_
