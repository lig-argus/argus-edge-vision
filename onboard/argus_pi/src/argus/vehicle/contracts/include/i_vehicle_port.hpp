// 계산된 기체 속도 명령을 실제 전송 어댑터에 넘기는 공통 출력 인터페이스다.
#pragma once
#include "argus/common/time/include/time.hpp"
#include "argus/control/contracts/include/control_intent.hpp"
namespace follow {
// 모든 속도 설정값은 이 경계를 통과하며 SDK 자료형은 구체 어댑터에만 둔다.
class IVehiclePort {
public:
    virtual ~IVehiclePort() = default;
    // 전송 처리의 성공 여부를 반환한다. 무장·이륙·Offboard 전환은 실행부에서 별도로 수행한다.
    virtual bool send(const FollowCommand&) = 0;
    // 비동기 송신 어댑터는 후보의 수명을 반복 packet에도 적용한다.
    virtual bool send_until(const FollowCommand& command, Clock::time_point valid_until) {
        (void)valid_until;
        return send(command);
    }
    // Cached asynchronous transports override this to stop every pending repeat.
    virtual void revoke_commands() {}
};
}  // namespace follow
