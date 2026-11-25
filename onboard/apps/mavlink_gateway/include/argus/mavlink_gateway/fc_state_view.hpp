// FcLinkTracker 의 연결 상태와 FcTelemetryReceiver 의 최신 값을 IFcStateSource 로 제공한다
// 규약: docs/spec/icd.md 6.2 (FC 연결 두절), docs/spec/process_interfaces.md 3장 (fc_epoch)
// 사용: FcGateway 가 하나를 생성하여 FcControl 과 통제기 명령 처리(다음 작업)에 전달
#ifndef ARGUS_ONBOARD_FC_STATE_VIEW_HPP_
#define ARGUS_ONBOARD_FC_STATE_VIEW_HPP_

#include <cstdint>

#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/fc_telemetry_receiver.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 값을 보관하지 않고 호출마다 두 객체의 현재 값을 전달한다
class FcStateView : public IFcStateSource {
public:
    // link 와 telemetry 는 이 객체보다 오래 유지되어야 한다
    FcStateView(const FcLinkTracker& link, const FcTelemetryReceiver& telemetry)
        : link_(link), telemetry_(telemetry) {}

    bool fc_connected() const override { return link_.connected(); }
    std::uint64_t fc_epoch() const override { return link_.fc_epoch(); }
    vehicle_api::VehicleState latest() const override { return telemetry_.latest(); }

private:
    const FcLinkTracker& link_;
    const FcTelemetryReceiver& telemetry_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_STATE_VIEW_HPP_
