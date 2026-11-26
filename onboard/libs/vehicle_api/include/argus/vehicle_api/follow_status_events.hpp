// 추종 제어의 상태(`follow/status`)를 받을 때마다 알리는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1.2, docs/design/uml/class_mavlink_gateway.md
// 구현: BusFollowStatusSubscriber (vehicle_api_bus. 받은 시각은 core::IClock),
//      테스트용 FakeFollowStatusSource (vehicle_api_testing)
// 사용: mavlink_gateway (ARGUS_STATUS 의 변경 시 보고, 명령 결과 전달)
// 최신 값만 조회하는 쪽은 IFollowStatusSource 를 사용한다
#ifndef ARGUS_VEHICLE_API_FOLLOW_STATUS_EVENTS_HPP_
#define ARGUS_VEHICLE_API_FOLLOW_STATUS_EVENTS_HPP_

#include <functional>

#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

class IFollowStatusEvents {
public:
    using FollowStatusHandler = std::function<void(const ReceivedFollowStatus&)>;

    IFollowStatusEvents() = default;
    virtual ~IFollowStatusEvents() = default;
    IFollowStatusEvents(const IFollowStatusEvents&) = delete;
    IFollowStatusEvents& operator=(const IFollowStatusEvents&) = delete;
    IFollowStatusEvents(IFollowStatusEvents&&) = delete;
    IFollowStatusEvents& operator=(IFollowStatusEvents&&) = delete;

    // 값을 받을 때마다 handler 를 부른다. 여러 번 등록하면 모두 부른다.
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 다른 메시지 처리가 밀리지 않게 오래 걸리는 일을 하지 않는다.
    // 반환된 Subscription 이 사라지면 그 handler 는 더 불리지 않는다 (등록 해제이지 취소가 아니다).
    [[nodiscard]] virtual core::Subscription subscribe(FollowStatusHandler handler) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FOLLOW_STATUS_EVENTS_HPP_
