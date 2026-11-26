// 추종 제어의 최신 상태(`follow/status`)를 조회하는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1.2, docs/design/uml/class_mavlink_gateway.md
// 구현: BusFollowStatusSubscriber (vehicle_api_bus. 받은 시각은 core::IClock),
//      테스트용 FakeFollowStatusSource (vehicle_api_testing)
// 사용: mavlink_gateway (제어권 판단, 통제기 두절 처리)
// 값을 받을 때마다 알림을 받는 쪽은 IFollowStatusEvents 를 사용한다
#ifndef ARGUS_VEHICLE_API_FOLLOW_STATUS_SOURCE_HPP_
#define ARGUS_VEHICLE_API_FOLLOW_STATUS_SOURCE_HPP_

#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

class IFollowStatusSource {
public:
    IFollowStatusSource() = default;
    virtual ~IFollowStatusSource() = default;
    IFollowStatusSource(const IFollowStatusSource&) = delete;
    IFollowStatusSource& operator=(const IFollowStatusSource&) = delete;
    IFollowStatusSource(IFollowStatusSource&&) = delete;
    IFollowStatusSource& operator=(IFollowStatusSource&&) = delete;

    // 가장 최근에 받은 값. 아직 하나도 받지 못했으면 nullopt.
    // 값이 얼마나 오래됐는지는 호출자가 arrived_monotonic_ns 로 판단한다
    virtual std::optional<ReceivedFollowStatus> latest() const = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FOLLOW_STATUS_SOURCE_HPP_
