// mavlink_gateway 의 FC 명령 송신 상태를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.1 (제어권 부여 조건의 setpoint 송신), 6.4 ⓐ,
//      docs/design/uml/class_mavlink_gateway.md
// 구현: FcControl
// 사용: ControlAuthority (setpoint 송신), ArgusStatusReporter (setpoint 유효 시간 경과)
#ifndef ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_
#define ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_

#include <cstdint>
#include <optional>

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

    // 추종 중 setpoint 의 유효 시간이 지나 정지한 마지막 시각 (RPi monotonic clock, icd 6.4 ⓐ).
    // 그런 적이 없으면 nullopt
    virtual std::optional<std::int64_t> last_setpoint_timeout_ns() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_COMMAND_STATUS_HPP_
