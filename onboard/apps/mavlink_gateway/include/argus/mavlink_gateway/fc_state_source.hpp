// FC 의 최신 상태를 읽는 인터페이스
// 구현: mavlink_gateway 의 main 이 FC telemetry 수신 클래스와 연결,
//      테스트용 FakeFcStateSource (mavlink_gateway test/)
// 사용: FcControl
#ifndef ARGUS_ONBOARD_FC_STATE_SOURCE_HPP_
#define ARGUS_ONBOARD_FC_STATE_SOURCE_HPP_

#include <cstdint>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

class IFcStateSource {
public:
    IFcStateSource() = default;
    virtual ~IFcStateSource() = default;
    IFcStateSource(const IFcStateSource&) = delete;
    IFcStateSource& operator=(const IFcStateSource&) = delete;
    IFcStateSource(IFcStateSource&&) = delete;
    IFcStateSource& operator=(IFcStateSource&&) = delete;

    // FC 와 연결되어 있는지 (docs/spec/icd.md 6.2 의 두절 판단)
    virtual bool fc_connected() const = 0;

    // FC 연결의 현재 epoch (docs/spec/process_interfaces.md 3장). 요청의 fc_epoch 와 비교한다
    virtual std::uint64_t fc_epoch() const = 0;

    // 가장 최근 값들. FC 에서 아직 받지 못한 항목은 nullopt. fc_epoch 가 바뀐 경우 이전 fc_epoch
    // 의 값은 삭제되어, 새 fc_epoch 에서 받기 전까지 nullopt 이다. 값이 받은 후 지난 시간은
    // 호출자가 각 값의 received_monotonic_ns 로 판단한다
    virtual vehicle_api::VehicleState latest() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_STATE_SOURCE_HPP_
