// mavlink_gateway 가 스스로 기체의 정지를 요청하는 인터페이스
// 규약: docs/spec/icd.md 6.1, 6.4
// 구현: FcControl
// 사용: 통제기 명령 처리, 통제기 두절과 추종 제어 상태 보고 두절의 처리 (다음 PR)
#ifndef ARGUS_ONBOARD_STOP_REQUESTER_HPP_
#define ARGUS_ONBOARD_STOP_REQUESTER_HPP_

namespace argus::mavlink_gateway {

// 추종 제어의 정지 요청(IFlightModeControl::request_hold)과 구분한다. 이 요청은 세션과 fc_epoch
// 값이 없으며 송신권을 확인하지 않는다. bus 로는 호출할 수 없다
class IStopRequester {
public:
    IStopRequester() = default;
    virtual ~IStopRequester() = default;
    IStopRequester(const IStopRequester&) = delete;
    IStopRequester& operator=(const IStopRequester&) = delete;
    IStopRequester(IStopRequester&&) = delete;
    IStopRequester& operator=(IStopRequester&&) = delete;

    // 기체를 정지시킨다. true = 정지를 시작함 (또는 정지시킬 대상이 아님: FC 가 Offboard 가 아니고
    // Offboard 진입을 기다리는 중도 아님).
    // false = 정지를 시작하지 못함. 외부의 모드 변경이 기록되어 정지 setpoint 를 송신할 수 없는
    // 경우이다 (FC 연결 두절이나 오래된 비행 상태에서는 정지를 시작한다). 기록이 해제되기 전에는
    // 다시 요청하여도 결과가 같다
    virtual bool stop_from_onboard() = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_STOP_REQUESTER_HPP_
