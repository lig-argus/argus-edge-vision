// 테스트용 bus. 한 객체가 토픽 발행, 구독, RPC 요청, 응답을 모두 흉내 낸다.
// 발행하거나 넣은(inject) 메시지는 FakePoller 에 쌓였다가 process_events() 안에서 구독자에게 간다.
// RPC 요청은 같은 FakeBus 에 serve() 로 등록한 handler 가 응답하고, 없으면 timeout 뒤 nullopt 가
// 된다.
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

#include "argus/bus/message.hpp"
#include "argus/bus/rpc_client.hpp"
#include "argus/bus/rpc_server.hpp"
#include "argus/bus/topic_publisher.hpp"
#include "argus/bus/topic_subscriber.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"

namespace argus::bus_testing {

class FakeBus : public bus::ITopicPublisher,
                public bus::ITopicSubscriber,
                public bus::IRpcClient,
                public bus::IRpcServer {
public:
    FakeBus(core_testing::FakePoller& poller, const core::IClock& clock)
        : poller_(poller), clock_(clock), state_(std::make_shared<State>()) {}

    // 발행한 메시지는 기록되고, 같은 FakeBus 의 구독자에게도 간다 (한 프로세스 안의 bus 처럼)
    void publish(const bus::BusMessage& message) override {
        published_.push_back(message);
        deliver(message);
    }

    [[nodiscard]] core::Subscription subscribe(std::string_view topic, Handler handler) override {
        const std::uint64_t id = state_->next_id++;
        state_->subscribers[std::string(topic)][id] = std::move(handler);
        return release(
            [topic = std::string(topic), id](State& state) { state.subscribers[topic].erase(id); });
    }

    [[nodiscard]] core::Subscription serve(std::string_view name, RequestHandler handler) override {
        const std::uint64_t id = state_->next_id++;
        state_->servers[std::string(name)] = Server{id, std::move(handler)};
        return release([name = std::string(name), id](State& state) {
            const auto found = state.servers.find(name);
            if (found != state.servers.end() && found->second.id == id) {
                state.servers.erase(found);  // 다른 등록이 대신했으면 그대로 둔다
            }
        });
    }

    [[nodiscard]] core::Subscription call(const bus::BusMessage& request,
                                          std::chrono::milliseconds timeout,
                                          ReplyHandler on_reply) override {
        const std::uint64_t id = state_->next_id++;
        state_->pending_calls[id] = std::move(on_reply);
        const std::weak_ptr<State> weak = state_;
        const auto finish = [weak, id](const std::optional<bus::BusMessage>& reply) {
            const auto state = weak.lock();
            if (!state) {
                return;
            }
            const auto found = state->pending_calls.find(id);
            if (found == state->pending_calls.end()) {
                return;  // Subscription 이 먼저 사라짐
            }
            const ReplyHandler handler = std::move(found->second);
            state->pending_calls.erase(found);
            handler(reply);
        };

        const auto server = state_->servers.find(request.topic);
        if (server == state_->servers.end()) {
            poller_.post_at(clock_.now_monotonic_ns() + timeout.count() * kNsPerMs,
                            [finish] { finish(std::nullopt); });
        } else {
            const auto shared_request = std::make_shared<const bus::BusMessage>(request);
            poller_.post([finish, handler = server->second.handler, shared_request] {
                finish(handler(*shared_request));
            });
        }
        return release([id](State& state) { state.pending_calls.erase(id); });
    }

    // 테스트용: 다른 프로그램이 보낸 메시지가 도착한 것처럼 넣는다
    void inject(const bus::BusMessage& message) { deliver(message); }

    // 테스트용: publish() 로 발행된 메시지 기록
    const std::vector<bus::BusMessage>& published() const { return published_; }

private:
    static constexpr std::int64_t kNsPerMs = 1'000'000;

    struct Server {
        std::uint64_t id = 0;
        RequestHandler handler;
    };
    // Subscription 이 FakeBus 보다 오래 살아도 안전하도록 등록 정보는 공유 객체에 둔다
    struct State {
        std::uint64_t next_id = 1;
        std::map<std::string, std::map<std::uint64_t, Handler>> subscribers;
        std::map<std::string, Server> servers;
        std::map<std::uint64_t, ReplyHandler> pending_calls;
    };

    template <typename Release>
    core::Subscription release(Release remove) {
        const std::weak_ptr<State> weak = state_;
        return core::Subscription([weak, remove = std::move(remove)] {
            if (const auto state = weak.lock()) {
                remove(*state);
            }
        });
    }

    void deliver(const bus::BusMessage& message) {
        const std::weak_ptr<State> weak = state_;
        const auto shared_message = std::make_shared<const bus::BusMessage>(message);
        poller_.post([weak, shared_message] {
            const bus::BusMessage& message = *shared_message;
            const auto state = weak.lock();
            if (!state) {
                return;
            }
            const auto found = state->subscribers.find(message.topic);
            if (found == state->subscribers.end()) {
                return;
            }
            // handler 가 구독을 풀어도 안전하도록 복사해서 부른다
            const auto handlers = found->second;
            for (const auto& [id, handler] : handlers) {
                handler(message);
            }
        });
    }

    core_testing::FakePoller& poller_;
    const core::IClock& clock_;
    std::shared_ptr<State> state_;
    std::vector<bus::BusMessage> published_;
};

}  // namespace argus::bus_testing

#endif  // ARGUS_BUS_TESTING_FAKE_BUS_HPP_
