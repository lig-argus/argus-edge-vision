// 추적기 상태(`tracking/status`)를 카메라마다 조회하는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1.3, docs/design/uml/class_mavlink_gateway.md
// 구현: BusTrackingStatusSubscriber (perception_api_bus. 받은 시각은 core::IClock)
// 사용: mavlink_gateway (ARGUS_SENSOR_STATUS 보고)
// 값을 받을 때마다 알림을 받는 쪽은 ITrackingStatusEvents 를 사용한다
#ifndef ARGUS_PERCEPTION_API_TRACKING_STATUS_SOURCE_HPP_
#define ARGUS_PERCEPTION_API_TRACKING_STATUS_SOURCE_HPP_

#include <optional>
#include <string>

#include "argus/perception_api/types.hpp"

namespace argus::perception_api {

class ITrackingStatusSource {
public:
    ITrackingStatusSource() = default;
    virtual ~ITrackingStatusSource() = default;
    ITrackingStatusSource(const ITrackingStatusSource&) = delete;
    ITrackingStatusSource& operator=(const ITrackingStatusSource&) = delete;
    ITrackingStatusSource(ITrackingStatusSource&&) = delete;
    ITrackingStatusSource& operator=(ITrackingStatusSource&&) = delete;

    // source(카메라 식별 문자열)의 가장 최근 값. 아직 받지 못했으면 nullopt.
    // 값이 얼마나 오래됐는지는 호출자가 arrived_monotonic_ns 로 판단한다
    virtual std::optional<ReceivedTrackingStatus> latest(const std::string& source) const = 0;
};

}  // namespace argus::perception_api

#endif  // ARGUS_PERCEPTION_API_TRACKING_STATUS_SOURCE_HPP_
