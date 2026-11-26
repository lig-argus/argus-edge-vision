// 각도 감기. 방위와 방향 차를 한 곳에서 같은 방식으로 감아 경계(0°/360°, ±180°)의 결과를 맞춘다
#ifndef ARGUS_GCS_LINK_ANGLES_HPP_
#define ARGUS_GCS_LINK_ANGLES_HPP_

#include <cmath>

namespace argus::gcs::link {

// 0 이상 360 미만 (방위, 방향)
inline double wrap360(double deg) {
    const double r = std::fmod(deg, 360.0);
    return r < 0.0 ? r + 360.0 : r;
}

// -180 이상 180 이하 (두 방위의 차, 가까운 쪽으로 도는 각)
inline double wrap180(double deg) { return std::remainder(deg, 360.0); }

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_ANGLES_HPP_
