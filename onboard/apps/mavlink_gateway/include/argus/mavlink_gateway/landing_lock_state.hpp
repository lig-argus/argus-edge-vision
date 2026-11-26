// 착륙 잠금 상태를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.1 (제어권 요청의 거부 조건), 5.3 (착륙 잠금)
// 구현: InactiveLandingLock (착륙 잠금 기능을 구현하기 전), LandingLock (다음 작업)
// 사용: ControlAuthority
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

// 착륙 잠금 기능이 없는 동안 main 이 연결하는 구현. 잠금을 시작하는 기능이 없으므로 항상 false 가
// 실제 상태와 같다
class InactiveLandingLock : public ILandingLockState {
public:
    bool active() const override { return false; }
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_LANDING_LOCK_STATE_HPP_
