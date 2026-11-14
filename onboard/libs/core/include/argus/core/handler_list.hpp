// handler 를 등록해 두고 한 번에 호출하는 목록. subscribe(handler) 로 Subscription 을 반환하는
// 클래스(FrameCollector 등)가 멤버로 사용한다
// 규약: docs/design/uml/class_onboard.md 1절 (Subscription)
//
// 규칙
// - notify() 는 등록된 handler 를 등록 순서대로 호출한다
// - notify() 도중 handler 가 Subscription 을 해제하면, 해제된 handler 는 그 뒤로 호출하지 않는다.
//   handler 가 자신의 Subscription 을 해제해도 된다
// - notify() 도중 등록한 handler 는 그 notify() 에서는 호출하지 않는다
// - HandlerList 가 Subscription 보다 먼저 소멸해도 된다. 그 뒤 Subscription 의 해제는 아무것도
//   하지 않는다
//
// 스레드: 모든 함수와 반환된 Subscription 의 해제는 한 스레드에서 호출한다
#ifndef ARGUS_CORE_HANDLER_LIST_HPP_
#define ARGUS_CORE_HANDLER_LIST_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "argus/core/subscription.hpp"

namespace argus::core {

template <typename... Args>
class HandlerList {
public:
    using Handler = std::function<void(Args...)>;

    HandlerList() = default;
    ~HandlerList() = default;

    // 복사하면 같은 handler 가 두 목록에서 호출되므로 금지한다
    HandlerList(const HandlerList&) = delete;
    HandlerList& operator=(const HandlerList&) = delete;
    HandlerList(HandlerList&&) noexcept = default;
    HandlerList& operator=(HandlerList&&) noexcept = default;

    // handler 를 등록한다. 반환된 Subscription 이 소멸하면 등록이 해제된다
    [[nodiscard]] Subscription subscribe(Handler handler) {
        const Id id = registrations_->next_id++;
        registrations_->handlers.emplace(id, std::move(handler));
        return Subscription([weak = std::weak_ptr<Registrations>(registrations_), id] {
            if (const auto registrations = weak.lock()) {
                registrations->handlers.erase(id);
            }
        });
    }

    void notify(Args... args) {
        // handler 가 등록과 해제를 할 수 있으므로 호출할 대상을 먼저 정한다
        for (const Id id : registered_ids()) {
            const auto found = registrations_->handlers.find(id);
            if (found == registrations_->handlers.end()) {
                continue;  // 앞에서 호출한 handler 가 해제했다
            }
            // handler 가 자신의 Subscription 을 해제해도 실행 중인 함수가 유지되도록 복사해서
            // 호출한다
            const Handler handler = found->second;
            handler(args...);
        }
    }

private:
    using Id = std::uint64_t;

    // 등록 순서가 id 의 크기 순서와 같도록 id 는 증가하는 값으로 발급한다
    struct Registrations {
        std::map<Id, Handler> handlers;
        Id next_id = 1;
    };

    std::vector<Id> registered_ids() const {
        std::vector<Id> ids;
        ids.reserve(registrations_->handlers.size());
        for (const auto& [id, handler] : registrations_->handlers) {
            ids.push_back(id);
        }
        return ids;
    }

    // Subscription 이 HandlerList 보다 오래 유지될 수 있으므로 공유 객체에 두고, Subscription 은
    // weak_ptr 로 참조한다
    std::shared_ptr<Registrations> registrations_ = std::make_shared<Registrations>();
};

}  // namespace argus::core

#endif  // ARGUS_CORE_HANDLER_LIST_HPP_
