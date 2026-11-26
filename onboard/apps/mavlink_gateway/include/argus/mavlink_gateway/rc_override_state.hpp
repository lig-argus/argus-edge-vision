// 조종사(RC)의 개입 상태를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.2 (전달 조건의 조종사 개입 중), 5.4
// 구현: InactiveRcOverride (조종사 개입 판정을 구현하기 전), RcOverrideMonitor
// 사용: GcsFcForwarder, MavlinkCommandReceiver, ControlAuthority, ArgusStatusReporter
#ifndef ARGUS_ONBOARD_RC_OVERRIDE_STATE_HPP_
#define ARGUS_ONBOARD_RC_OVERRIDE_STATE_HPP_

namespace argus::mavlink_gateway {

class IRcOverrideState {
public:
    IRcOverrideState() = default;
    virtual ~IRcOverrideState() = default;
    IRcOverrideState(const IRcOverrideState&) = delete;
    IRcOverrideState& operator=(const IRcOverrideState&) = delete;
    IRcOverrideState(IRcOverrideState&&) = delete;
    IRcOverrideState& operator=(IRcOverrideState&&) = delete;

    // 조종사 개입 중이면 true
    virtual bool pilot_in_control() const = 0;
};

// 조종사 개입 판정이 없는 동안 연결하는 구현. 개입을 판정하는 기능이 없으므로 항상 false
class InactiveRcOverride : public IRcOverrideState {
public:
    bool pilot_in_control() const override { return false; }
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_RC_OVERRIDE_STATE_HPP_
