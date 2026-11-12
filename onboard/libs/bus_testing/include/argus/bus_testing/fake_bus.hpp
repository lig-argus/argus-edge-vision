// 테스트용 IBus. 한 FakeBus 가 만든 발행자, 구독자, 요청하는 쪽, 응답하는 쪽은 같은 bus 에 붙은
// 것처럼 동작한다. 발행하거나 simulate_message_received() 로 도착한 것으로 등록한 메시지는
// FakePoller 에 쌓였다가 process_events() 안에서 구독자에게 간다. 요청은 같은 FakeBus 에 serve() 로
// 등록한 handler 가 응답하고, 없으면 timeout 뒤 nullopt 가 된다. 발행자의 queue_depth:
// process_events() 가 아직 전달하지 않은 메시지가 queue_depth 개이면 publish() 가 false 를 반환한다
// (실제 구현과 같은 contract).
#ifndef ARGUS_BUS_TESTING_FAKE_BUS_HPP_
#define ARGUS_BUS_TESTING_FAKE_BUS_HPP_

#include <chrono>
#include <cstdint>
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
        return std::make_unique<Publisher>(state_, std::string(topic), queue_depth);
    }

    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(std::string_view topic,
                                                             int queue_depth) override {
        if (!check_name(topic) || !check_depth(queue_depth)) {
            return nullptr;
        }
        return std::make_unique<Subscriber>(state_, std::string(topic));
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

    // 테스트용: publish() 로 발행된 메시지 기록 (모든 토픽, 발행 순서)
    const std::vector<bus::BusMessage>& published() const { return state_->published; }

private:
    static constexpr std::int64_t kNsPerMs = 1'000'000;

    using Handler = bus::ITopicSubscriber::Handler;
    using RequestHandler = bus::IReplier::RequestHandler;
    using ReplyHandler = bus::IRequester::ReplyHandler;

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
        std::map<std::string, std::map<std::uint64_t, Handler>> subscribers;
        std::map<std::string, ReplierEntry> repliers;
        std::map<std::uint64_t, ReplyHandler> pending_requests;
        std::vector<bus::BusMessage> published;

        // 메시지를 다음 process_events() 에서 구독자에게 전달하도록 예약한다.
        // on_taken 은 전달 직전에 불린다 (발행자의 큐 개수 감소용)
        static void deliver(const std::shared_ptr<State>& state, const bus::BusMessage& message,
                            std::function<void()> on_taken = nullptr) {
            const std::weak_ptr<State> weak = state;
            const auto shared_message = std::make_shared<const bus::BusMessage>(message);
            state->poller.post([weak, shared_message, on_taken = std::move(on_taken)] {
                if (on_taken) {
                    on_taken();
                }
                const auto locked = weak.lock();
                if (!locked) {
                    return;
                }
                const auto found = locked->subscribers.find(shared_message->topic);
                if (found == locked->subscribers.end()) {
                    return;
                }
                // handler 가 구독을 풀어도 안전하도록 복사해서 부른다
                const auto handlers = found->second;
                for (const auto& [id, handler] : handlers) {
                    handler(*shared_message);
                }
            });
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
        Publisher(std::shared_ptr<State> state, std::string topic, int queue_depth)
            : state_(std::move(state)),
              topic_(std::move(topic)),
              queue_depth_(queue_depth),
              queued_(std::make_shared<int>(0)) {}

        std::string_view topic() const override { return topic_; }

        bool publish(const bus::Bytes& body, const bus::Bytes& binary) override {
            if (*queued_ >= queue_depth_) {
                return false;  // 큐가 가득 참. 이 메시지는 유실
            }
            const bus::BusMessage message{topic_, body, binary};
            state_->published.push_back(message);
            ++*queued_;
            State::deliver(state_, message, [queued = queued_] { --*queued; });
            return true;
        }

    private:
        std::shared_ptr<State> state_;
        std::string topic_;
        int queue_depth_;
        std::shared_ptr<int> queued_;  // 예약됐지만 아직 전달되지 않은 메시지 수
    };

    class Subscriber : public bus::ITopicSubscriber {
    public:
        Subscriber(std::shared_ptr<State> state, std::string topic)
            : state_(std::move(state)), topic_(std::move(topic)) {}

        std::string_view topic() const override { return topic_; }

        [[nodiscard]] core::Subscription subscribe(Handler handler) override {
            const std::uint64_t id = state_->next_id++;
            state_->subscribers[topic_][id] = std::move(handler);
            return State::release(
                state_, [topic = topic_, id](State& state) { state.subscribers[topic].erase(id); });
        }

    private:
        std::shared_ptr<State> state_;
        std::string topic_;
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
