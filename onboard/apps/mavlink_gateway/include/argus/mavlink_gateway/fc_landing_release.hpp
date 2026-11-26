// 통제기의 착륙 명령을 FC 로 전달할 때 mavlink_gateway 의 FC 제어를 내려놓는 인터페이스
// 규약: docs/spec/icd.md 5.3 (착륙 잠금), 4.2 (setpoint 송신을 끝내는 조건)
// 구현: FcControl
// 사용: LandingLock
#ifndef ARGUS_ONBOARD_FC_LANDING_RELEASE_HPP_
#define ARGUS_ONBOARD_FC_LANDING_RELEASE_HPP_

namespace argus::mavlink_gateway {

class IFcLandingRelease {
public:
    IFcLandingRelease() = default;
    virtual ~IFcLandingRelease() = default;
    IFcLandingRelease(const IFcLandingRelease&) = delete;
    IFcLandingRelease& operator=(const IFcLandingRelease&) = delete;
    IFcLandingRelease(IFcLandingRelease&&) = delete;
    IFcLandingRelease& operator=(IFcLandingRelease&&) = delete;

    // FC 제어 명령 송신권과 진행 중인 Offboard 진입(복귀 명령의 진입 포함)을 폐기하고 추종 제어의
    // setpoint 를 FC 로 보내지 않는다. FC 가 Offboard 일 수 있는 동안은 제자리 유지 setpoint 를
    // 계속 보내고, FC 가 Offboard 가 아님을 비행 상태로 확인하면 송신을 끝낸다. 그 사이의 Offboard
    // 진입 요청은 거부한다
    virtual void release_for_landing() = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_LANDING_RELEASE_HPP_
