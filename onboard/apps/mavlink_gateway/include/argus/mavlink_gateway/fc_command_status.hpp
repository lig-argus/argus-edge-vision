// mavlink_gateway 의 FC 명령 송신 상태를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.1 (제어권 부여 조건의 setpoint 송신),
// docs/design/uml/class_mavlink_gateway.md 구현: FcControl 사용: ControlAuthority
#ifndef ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_
#define ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_

namespace argus::mavlink_gateway {

class IFcCommandStatus {
public:
    IFcCommandStatus() = default;
    virtual ~IFcCommandStatus() = default;
    IFcCommandStatus(const IFcCommandStatus&) = delete;
    IFcCommandStatus& operator=(const IFcCommandStatus&) = delete;
    IFcCommandStatus(IFcCommandStatus&&) = delete;
    IFcCommandStatus& operator=(IFcCommandStatus&&) = delete;

    // 마지막 주기 작업에서 FC 로 setpoint 를 보낼 차례였으면 true. Offboard 진입 전의 사전
    // 송신(prestream)도 포함한다
    virtual bool sending_setpoints() const = 0;

    // Offboard 진입의 확인을 기다리는 중이면 true
    virtual bool entering() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_
