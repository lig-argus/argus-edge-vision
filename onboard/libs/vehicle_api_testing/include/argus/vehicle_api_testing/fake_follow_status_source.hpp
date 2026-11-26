// 테스트용 IFollowStatusSource, IFollowStatusEvents. 테스트가 받은 값과 받은 시각을 직접 넣는다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_STATUS_SOURCE_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_STATUS_SOURCE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/follow_status_events.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_testing {

class FakeFollowStatusSource : public vehicle_api::IFollowStatusSource,
                               public vehicle_api::IFollowStatusEvents {
public:
    FakeFollowStatusSource() : handlers_(std::make_shared<Handlers>()) {}

    std::optional<vehicle_api::ReceivedFollowStatus> latest() const override { return latest_; }

    [[nodiscard]] core::Subscription subscribe(FollowStatusHandler handler) override {
        const std::uint64_t id = handlers_->next_id++;
        handlers_->by_id[id] = std::move(handler);
        const std::weak_ptr<Handlers> weak = handlers_;
        return core::Subscription([weak, id] {
            if (const auto handlers = weak.lock()) {
                handlers->by_id.erase(id);
            }
        });
    }

    // 테스트용 조작
    // 받은 값으로 남기고 등록된 handler 를 부른다. 실제 구현은 process_events() 안에서 handler 를
    // 부르지만 이 Fake 는 테스트가 부른 자리에서 바로 부른다. handler 가 등록을 해제하거나 추가해도
    // 안전하도록 복사본을 돈다
    void simulate_status(const vehicle_api::FollowStatus& status,
                         std::int64_t arrived_monotonic_ns) {
        latest_ = vehicle_api::ReceivedFollowStatus{status, arrived_monotonic_ns};
        const vehicle_api::ReceivedFollowStatus received = *latest_;
        const auto snapshot = handlers_->by_id;
        for (const auto& [id, handler] : snapshot) {
            if (handlers_->by_id.count(id) != 0) {
                handler(received);
            }
        }
    }

private:
    struct Handlers {
        std::map<std::uint64_t, FollowStatusHandler> by_id;
        std::uint64_t next_id = 1;
    };

    std::optional<vehicle_api::ReceivedFollowStatus> latest_;
    std::shared_ptr<Handlers> handlers_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_STATUS_SOURCE_HPP_
