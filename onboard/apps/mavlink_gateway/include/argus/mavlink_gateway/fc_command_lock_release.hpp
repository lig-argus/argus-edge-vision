// FC 가 요청 없이 Offboard 를 벗어난 후의 송신 잠금을 해제하는 인터페이스
// 규약: docs/spec/icd.md 4.2 (운용자의 명시적 TRACK_STOP), 6장 (FAULT, MANUAL 에서의 복구)
// 구현: FcControl
// 사용: 통제기 명령 처리 (다음 PR)
#ifndef ARGUS_ONBOARD_FC_COMMAND_LOCK_RELEASE_HPP_
#define ARGUS_ONBOARD_FC_COMMAND_LOCK_RELEASE_HPP_

namespace argus::mavlink_gateway {

class IFcCommandLockRelease {
public:
    IFcCommandLockRelease() = default;
    virtual ~IFcCommandLockRelease() = default;
    IFcCommandLockRelease(const IFcCommandLockRelease&) = delete;
    IFcCommandLockRelease& operator=(const IFcCommandLockRelease&) = delete;
    IFcCommandLockRelease(IFcCommandLockRelease&&) = delete;
    IFcCommandLockRelease& operator=(IFcCommandLockRelease&&) = delete;

    // true = 잠겨 있지 않거나 해제함. false = 해제하지 못함 (FC 의 비행 상태가 최신이 아님)
    virtual bool release_lock() = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_COMMAND_LOCK_RELEASE_HPP_
