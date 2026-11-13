// IBus 를 값 타입으로 쓰는 래퍼. 호출하는 코드는 const void* 를 직접 다루지 않고 이 래퍼를 쓴다.
// 래퍼는 타입마다 컴파일되므로, 생성 때와 다른 타입의 값을 넘기는 실수는 컴파일 오류가 된다.
// 사용:
//   auto publisher = bus::make_publisher<VelocitySetpoint>(bus, topic, kSetpointType, depth);
//   if (publisher) { publisher->publish(setpoint); }
#ifndef ARGUS_BUS_TYPED_HPP_
#define ARGUS_BUS_TYPED_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string_view>
#include <typeindex>
#include <utility>

#include "argus/bus/bus.hpp"
#include "argus/bus/message_type.hpp"
#include "argus/core/subscription.hpp"

namespace argus::bus {

template <typename T>
class Publisher {
public:
    explicit Publisher(std::unique_ptr<ITopicPublisher> publisher)
        : publisher_(std::move(publisher)) {}

    std::string_view topic() const { return publisher_->topic(); }
    bool publish(const T& message) { return publisher_->publish(&message); }

private:
    std::unique_ptr<ITopicPublisher> publisher_;
};

template <typename T>
class Subscriber {
public:
    using Handler = std::function<void(const T& message, const MessageInfo& info)>;

    explicit Subscriber(std::unique_ptr<ITopicSubscriber> subscriber)
        : subscriber_(std::move(subscriber)) {}

    std::string_view topic() const { return subscriber_->topic(); }

    // handler 가 비어 있으면 등록하지 않고 빈 Subscription 을 반환한다
    [[nodiscard]] core::Subscription subscribe(Handler handler) {
        if (!handler) {
            return {};
        }
        return subscriber_->subscribe(
            [handler = std::move(handler)](const void* message, const MessageInfo& info) {
                handler(*static_cast<const T*>(message), info);
            });
    }

private:
    std::unique_ptr<ITopicSubscriber> subscriber_;
};

template <typename Request, typename Reply>
class Requester {
public:
    // 응답, 또는 timeout 안에 응답이 없으면 nullopt
    using ReplyHandler = std::function<void(std::optional<Reply>)>;

    explicit Requester(std::unique_ptr<IRequester> requester) : requester_(std::move(requester)) {}

    std::string_view name() const { return requester_->name(); }

    // on_reply 가 비어 있으면 요청을 보내지 않고 빈 Subscription 을 반환한다
    [[nodiscard]] core::Subscription request(const Request& request,
                                             std::chrono::milliseconds timeout,
                                             ReplyHandler on_reply) {
        if (!on_reply) {
            return {};
        }
        return requester_->request(&request, timeout,
                                   [on_reply = std::move(on_reply)](const void* reply) {
                                       if (reply == nullptr) {
                                           on_reply(std::nullopt);
                                       } else {
                                           on_reply(*static_cast<const Reply*>(reply));
                                       }
                                   });
    }

private:
    std::unique_ptr<IRequester> requester_;
};

template <typename Request, typename Reply>
class Replier {
public:
    using RequestHandler = std::function<Reply(const Request&)>;

    explicit Replier(std::unique_ptr<IReplier> replier) : replier_(std::move(replier)) {}

    std::string_view name() const { return replier_->name(); }

    // handler 가 비어 있으면 등록하지 않고 빈 Subscription 을 반환한다 (앞의 등록은 그대로 남는다)
    [[nodiscard]] core::Subscription serve(RequestHandler handler) {
        if (!handler) {
            return {};
        }
        return replier_->serve([handler = std::move(handler)](const void* request, void* reply) {
            *static_cast<Reply*>(reply) = handler(*static_cast<const Request*>(request));
        });
    }

private:
    std::unique_ptr<IReplier> replier_;
};

// 아래 넷은 IBus 의 create_*() 를 부르고 결과를 래퍼로 감싼다. type 이 T 의 타입이 아니거나 bus 가
// 만들지 못하면 nullptr 를 반환한다
template <typename T>
std::unique_ptr<Publisher<T>> make_publisher(IBus& bus, std::string_view topic,
                                             const MessageType& type, int queue_depth) {
    if (type.type != std::type_index(typeid(T))) {
        return nullptr;
    }
    auto publisher = bus.create_publisher(topic, type, queue_depth);
    return publisher ? std::make_unique<Publisher<T>>(std::move(publisher)) : nullptr;
}

template <typename T>
std::unique_ptr<Subscriber<T>> make_subscriber(IBus& bus, std::string_view topic,
                                               const MessageType& type, int queue_depth) {
    if (type.type != std::type_index(typeid(T))) {
        return nullptr;
    }
    auto subscriber = bus.create_subscriber(topic, type, queue_depth);
    return subscriber ? std::make_unique<Subscriber<T>>(std::move(subscriber)) : nullptr;
}

template <typename Request, typename Reply>
std::unique_ptr<Requester<Request, Reply>> make_requester(IBus& bus, std::string_view name,
                                                          const MessageType& request,
                                                          const MessageType& reply) {
    if (request.type != std::type_index(typeid(Request)) ||
        reply.type != std::type_index(typeid(Reply))) {
        return nullptr;
    }
    auto requester = bus.create_requester(name, request, reply);
    return requester ? std::make_unique<Requester<Request, Reply>>(std::move(requester)) : nullptr;
}

template <typename Request, typename Reply>
std::unique_ptr<Replier<Request, Reply>> make_replier(IBus& bus, std::string_view name,
                                                      const MessageType& request,
                                                      const MessageType& reply) {
    if (request.type != std::type_index(typeid(Request)) ||
        reply.type != std::type_index(typeid(Reply))) {
        return nullptr;
    }
    auto replier = bus.create_replier(name, request, reply);
    return replier ? std::make_unique<Replier<Request, Reply>>(std::move(replier)) : nullptr;
}

}  // namespace argus::bus

#endif  // ARGUS_BUS_TYPED_HPP_
