// 두 SITL 예제가 공통으로 사용하는 속도 명령 자료형이다.
// vx/vy/vz는 기체 전방·우측·하방 속도(m/s), yaw_rate는 회전 속도(deg/s)다.
#pragma once
#include <string>
namespace follow {
// 현재 SITL 속도 설정값이다. 흐름도의 만료 시각 등을 포함한 풍부한 의도 계약은 아직 구현되지
// 않았다.
struct FollowCommand {
    double vx_m_s{0.0};
    double vy_m_s{0.0};
    double vz_m_s{0.0};
    double yaw_rate_deg_s{0.0};
    // mode는 진단용 문자열이며 MAVSDK 비행 모드를 직접 전환하지 않는다.
    std::string mode{"LOST_HOVER"};
};
}  // namespace follow
