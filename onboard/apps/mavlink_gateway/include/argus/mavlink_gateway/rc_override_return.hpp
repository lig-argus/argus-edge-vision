// 조종사 개입 뒤 통제기로의 복귀 조건을 확인하고 개입을 끝내는 인터페이스
// 규약: docs/spec/icd.md 4.4, 5.4
// 구현: RcOverrideMonitor
// 사용: EndRcOverrideHandler
#ifndef ARGUS_ONBOARD_RC_OVERRIDE_RETURN_HPP_
#define ARGUS_ONBOARD_RC_OVERRIDE_RETURN_HPP_

#include "argus/mavlink_gateway/rc_override_policy.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

struct ReturnCheck {
    ReturnBlock block = ReturnBlock::NotPilot;  // None 이면 모든 조건을 충족
    vehicle_api::FlightMode fc_mode = vehicle_api::FlightMode::Unknown;  // 확인한 FC 모드
};

class IRcOverrideReturn {
public:
    IRcOverrideReturn() = default;
    virtual ~IRcOverrideReturn() = default;
    IRcOverrideReturn(const IRcOverrideReturn&) = delete;
    IRcOverrideReturn& operator=(const IRcOverrideReturn&) = delete;
    IRcOverrideReturn(IRcOverrideReturn&&) = delete;
    IRcOverrideReturn& operator=(IRcOverrideReturn&&) = delete;

    // 복귀 조건 전체를 확인한다 (45011 을 받은 때)
    virtual ReturnCheck check_return() const = 0;

    // 복귀 절차 중(Offboard 진입 명령을 보내기 전)에 이어 가도 되는지. 조종사 개입 중이고, 스틱
    // 4축이 지금 중립이며, RC 수신기와 추정 상태가 정상이면 true. 진행 중인 명령(복귀 명령 자신)과
    // Offboard 진입(복귀 절차 자신)은 보지 않는다
    virtual bool return_still_allowed() const = 0;

    // 복귀 절차를 마침. 조종사 개입을 끝낸다
    virtual void end_override() = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_RC_OVERRIDE_RETURN_HPP_
