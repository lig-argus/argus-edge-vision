// 추적기 상태(`tracking/status`)를 받을 때마다 알리는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1.3, docs/design/uml/class_mavlink_gateway.md
// 구현: BusTrackingStatusSubscriber (perception_api_bus. 받은 시각은 core::IClock)
// 사용: mavlink_gateway (ARGUS_SENSOR_STATUS 의 flags 변경 시 보고)
// 최신 값만 조회하는 쪽은 ITrackingStatusSource 를 사용한다
#ifndef ARGUS_PERCEPTION_API_TRACKING_STATUS_EVENTS_HPP_
#define ARGUS_PERCEPTION_API_TRACKING_STATUS_EVENTS_HPP_

#include <functional>

#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::perception_api {

class ITrackingStatusEvents {
public:
    using TrackingStatusHandler = std::function<void(const ReceivedTrackingStatus&)>;

    ITrackingStatusEvents() = default;
    virtual ~ITrackingStatusEvents() = default;
    ITrackingStatusEvents(const ITrackingStatusEvents&) = delete;
    ITrackingStatusEvents& operator=(const ITrackingStatusEvents&) = delete;
    ITrackingStatusEvents(ITrackingStatusEvents&&) = delete;
    ITrackingStatusEvents& operator=(ITrackingStatusEvents&&) = delete;

    // 값을 받을 때마다(모든 카메라) handler 를 부른다. 여러 번 등록하면 모두 부른다.
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 다른 메시지 처리가 밀리지 않게 오래 걸리는 일을 하지 않는다.
    // 반환된 Subscription 이 사라지면 그 handler 는 더 불리지 않는다 (등록 해제이지 취소가 아니다).
    [[nodiscard]] virtual core::Subscription subscribe(TrackingStatusHandler handler) = 0;
};

}  // namespace argus::perception_api

#endif  // ARGUS_PERCEPTION_API_TRACKING_STATUS_EVENTS_HPP_
