// handler 목록의 등록, 해제, 호출. 등록 함수가 core::Subscription 을 반환하는 클래스들이 함께
// 사용하여 호출 규칙을 같게 한다
#ifndef ARGUS_ONBOARD_HANDLER_LIST_HPP_
#define ARGUS_ONBOARD_HANDLER_LIST_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "argus/core/subscription.hpp"

namespace argus::onboard {

template <typename... Args>
class HandlerList {
public:
    using Handler = std::function<void(Args...)>;

    HandlerList() : entries_(std::make_shared<Entries>()) {}
    ~HandlerList() = default;
    HandlerList(const HandlerList&) = delete;
    HandlerList& operator=(const HandlerList&) = delete;
    HandlerList(HandlerList&&) noexcept = default;
    HandlerList& operator=(HandlerList&&) noexcept = default;

    // handler 를 등록한다. 반환된 Subscription 이 소멸하면 등록이 해제된다.
    // Subscription 은 이 객체보다 늦게 소멸하여도 된다
    [[nodiscard]] core::Subscription add(Handler handler) {
        const std::uint64_t id = entries_->next_id++;
        entries_->by_id[id] = std::move(handler);
        // Subscription 은 목록을 소유하지 않고 weak_ptr 로 참조한다
        const std::weak_ptr<Entries> weak = entries_;
        return core::Subscription([weak, id] {
            if (const auto entries = weak.lock()) {
                entries->by_id.erase(id);
            }
        });
    }

    // 등록된 handler 를 등록한 순서대로 호출한다.
    // handler 는 호출 중에 등록을 추가하거나 해제하여도 된다. 앞선 handler 가 해제한 handler 는
    // 호출하지 않으며, 호출 중에 추가된 handler 는 다음 호출부터 포함한다
    void call(Args... args) const {
        // handler 가 이 객체를 소멸시키는 경우에도 순회가 유효하도록 목록을 공유한다
        const std::shared_ptr<Entries> entries = entries_;
        std::vector<std::uint64_t> ids;
        ids.reserve(entries->by_id.size());
        for (const auto& [id, handler] : entries->by_id) {
            ids.push_back(id);
        }
        for (const std::uint64_t id : ids) {
            const auto entry = entries->by_id.find(id);
            if (entry == entries->by_id.end()) {
                continue;
            }
            const Handler handler =
                entry->second;  // 호출 중에 자기 등록을 해제하여도 유효하도록 복사
            handler(args...);
        }
    }

    bool empty() const { return entries_->by_id.empty(); }

private:
    struct Entries {
        std::uint64_t next_id = 1;
        std::map<std::uint64_t, Handler> by_id;
    };

    std::shared_ptr<Entries> entries_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_HANDLER_LIST_HPP_
