// 착륙 잠금 상태를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.1 (제어권 요청의 거부 조건), 5.3 (착륙 잠금)
// 구현: LandingLock
// 사용: ControlAuthority, MavlinkCommandReceiver, GcsFcForwarder, ArgusStatusReporter
#ifndef ARGUS_ONBOARD_LANDING_LOCK_STATE_HPP_
#define ARGUS_ONBOARD_LANDING_LOCK_STATE_HPP_

namespace argus::mavlink_gateway {

class ILandingLockState {
public:
    ILandingLockState() = default;
    virtual ~ILandingLockState() = default;
    ILandingLockState(const ILandingLockState&) = delete;
    ILandingLockState& operator=(const ILandingLockState&) = delete;
    ILandingLockState(ILandingLockState&&) = delete;
    ILandingLockState& operator=(ILandingLockState&&) = delete;

    // 착륙 잠금 상태이면 true
    virtual bool active() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_LANDING_LOCK_STATE_HPP_
