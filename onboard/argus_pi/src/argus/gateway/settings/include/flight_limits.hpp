// 대상 FC 운용 범위와 실내·실외 EKF 파라미터 묶음 (Pi tools/real_fc/config.py와 같은 값).
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace argus::gateway {

// 파라미터 이름과 값. 순서는 원본 dict 순서와 같다 (확인·기록 순서가 같아야 한다).
using ParameterProfile = std::vector<std::pair<std::string, double>>;

inline const ParameterProfile& indoor_profile() {
    static const ParameterProfile kProfile{
        {"SYS_HAS_GPS", 0},  {"EKF2_GPS_CTRL", 0},  {"EKF2_OF_CTRL", 1},     {"EKF2_RNG_CTRL", 2},
        {"EKF2_HGT_REF", 2}, {"SYS_HAS_NUM_OF", 1}, {"SYS_HAS_NUM_DIST", 1},
    };
    return kProfile;
}

inline const ParameterProfile& height_profile() {
    static const ParameterProfile kProfile{{"SENS_FLOW_MAXHGT", 5.0}};
    return kProfile;
}

// dict(INDOOR, SYS_HAS_GPS=1, SYS_HAS_MAG=1, EKF2_GPS_CTRL=7, EKF2_HGT_REF=2, EKF2_BARO_CTRL=1,
// EKF2_RNG_CTRL=2): 기존 키는 자리를 지키고 새 키는 뒤에 붙는다.
inline const ParameterProfile& outdoor_profile() {
    static const ParameterProfile kProfile{
        {"SYS_HAS_GPS", 1},      {"EKF2_GPS_CTRL", 7}, {"EKF2_OF_CTRL", 1},
        {"EKF2_RNG_CTRL", 2},    {"EKF2_HGT_REF", 2},  {"SYS_HAS_NUM_OF", 1},
        {"SYS_HAS_NUM_DIST", 1}, {"SYS_HAS_MAG", 1},   {"EKF2_BARO_CTRL", 1},
    };
    return kProfile;
}

// 사용자가 정한 초기 운용 범위 (단위: m, s, m/s)
struct FlightLimits {
    double minimum{0.5};
    double takeoff_maximum{4.8};
    double ceiling{5.0};
    double brake_margin{0.1};
    double heartbeat_ttl{2.5};
    double sensor_ttl{2.0};  // PixEagle 텔레메트리 신선도 기본값 2초
    double lease_ttl{2.0};
    double climb_speed{0.25};
    double target_tolerance{0.10};
    // 2026-11-20 사용자 지시: 이륙 완료 판정만 허용 오차를 넓힌다. 실외 이륙(11-20 10:29)에서 FC
    // 추정 높이는 목표 4.0 m에 닿았지만 레이저는 3.83~3.90 m라 0.10 m 기준을 못 넘겨 30초 뒤
    // 착륙했다. 고도 변경(set_height)과 이륙 거리 검사는 target_tolerance 그대로.
    double takeoff_tolerance{0.30};
    double takeoff_timeout{30.0};
    // 2026-11-24 사용자 지시: 이륙 완료를 못 봐도 실제로 떠서 상승이 멈췄으면 지금 높이에서
    // 유지로 완료한다 (메인 GUI가 결과를 기다리며 다음 버튼을 막고, 실패면 착륙하던 문제).
    // 조건: 상승 setpoint가 목표에 닿은 뒤, 레이저 높이 >= 하한(데모 범위면 그 하한), |vz| < 속도가
    // 유지 시간 동안 계속. 목표 setpoint는 바꾸지 않는다.
    double takeoff_stall_min_height{0.5};
    double takeoff_stall_speed{0.10};
    double takeoff_stall_hold_s{2.0};

    double brake_height() const { return ceiling - brake_margin; }
};

}  // namespace argus::gateway
