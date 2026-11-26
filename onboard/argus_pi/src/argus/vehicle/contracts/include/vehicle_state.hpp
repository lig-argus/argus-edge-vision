// SITL_IMAGE 제어에 필요한 기체 NED 위치(m), Euler 자세(deg), 각 수신 시각을 보관한다.
#pragma once
#include <cstdint>
#include <string>

#include "argus/common/time/include/time.hpp"
namespace follow {
struct Pose {
    double north_m{0}, east_m{0}, down_m{0};
    double roll_deg{0}, pitch_deg{0}, yaw_deg{0};
    Clock::time_point position_at{}, attitude_at{};
    bool have_position{false}, have_attitude{false};
    // host monotonic 수신 시각. 센서 노출/FC 내부 측정 시각으로 간주하지 않는다.
    std::int64_t position_monotonic_ns{0}, attitude_monotonic_ns{0};
    std::string clock_domain{"host_monotonic"};
    // 로컬 producer가 잠금 아래 복사를 마친 시각이다. 원래 위치/자세 표본 시각은 바꾸지 않는다.
    Clock::time_point snapshot_at{};
    // 같은 PositionVelocityNed 표본의 FC 추정 속도. GT 또는 명령 속도가 아니다.
    double velocity_north_m_s{0}, velocity_east_m_s{0}, velocity_down_m_s{0};
    bool have_velocity{false};
    // 위치와 자세를 모두 받은 뒤 각각 1초 미만 경과했는지 확인한다. 두 표본의 동시성 검사는 아니다.
    bool fresh(Clock::time_point now = Clock::now()) const {
        return have_position && have_attitude && now - position_at < 1s && now - attitude_at < 1s;
    }
};
}  // namespace follow
