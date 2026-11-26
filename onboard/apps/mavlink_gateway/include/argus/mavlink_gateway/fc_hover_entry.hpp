// 조종사 개입 뒤 통제기의 복귀 명령(45011)으로 Offboard 제자리 유지에 들어가는 인터페이스
// 규약: docs/spec/icd.md 4.4, 5.4 (넘겨받은 뒤의 상태는 제자리 유지)
// 구현: FcControl
// 사용: EndRcOverrideHandler
#ifndef ARGUS_ONBOARD_FC_HOVER_ENTRY_HPP_
#define ARGUS_ONBOARD_FC_HOVER_ENTRY_HPP_

#include <functional>
#include <string_view>

namespace argus::mavlink_gateway {

class IFcHoverEntry {
public:
    // 진입 명령을 보내기 전에 절차를 이어 가도 되는지. false 이면 진입 명령을 보내지 않고 실패로
    // 끝낸다 (절차 중 조종사의 개입)
    using StillAllowed = std::function<bool()>;

    // 절차의 결과. true = Offboard 제자리 유지에 들어감, false = 실패
    using Done = std::function<void(bool entered)>;

    IFcHoverEntry() = default;
    virtual ~IFcHoverEntry() = default;
    IFcHoverEntry(const IFcHoverEntry&) = delete;
    IFcHoverEntry& operator=(const IFcHoverEntry&) = delete;
    IFcHoverEntry(IFcHoverEntry&&) = delete;
    IFcHoverEntry& operator=(IFcHoverEntry&&) = delete;

    // 지금 위치와 yaw 를 목표로 위치 setpoint 를 보내기 시작하고, 진입 전 송신 시간(1 s)이 지나면
    // Offboard 진입 명령을 보낸다. FC 가 Offboard 인 것을 비행 상태로 확인하면 done(true).
    // FC 의 거부, 확인 시간 초과, 진입 명령 전의 FC 모드 변경이나 still_allowed() 가 false,
    // 착륙 명령 전달, 정지 요청이면 done(false). FC 가 요청 없이 Offboard 를 벗어난 기록(Locked)은
    // 이 요청으로 해제한다.
    // 시작하지 못하면 거부 원인(로그용 영문)을 반환하고 done 을 부르지 않는다: FC 를 제어하는 중,
    // FC 의 모드를 모르거나 Offboard 임, 자세와 local 위치가 최신이 아님. 시작하면 빈 문자열.
    // still_allowed 와 done 은 주기 작업과 요청 처리 안에서 불린다
    virtual std::string_view request_hover_entry(StillAllowed still_allowed, Done done) = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_HOVER_ENTRY_HPP_
