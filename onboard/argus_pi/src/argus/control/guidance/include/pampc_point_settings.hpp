#pragma once
#include <stdexcept>

namespace follow {
// V3 (userpc SITL 검증본) PAMPC point 모드 가중치 묶음. 원 SITL 비교에서는 환경변수로
// 켰고, 메인에서는 이동 추종 설정 파일(v3_pampc_point_profile)에서만 켠다.
struct PampcPointSettings {
    int profile{0};
    double center{12}, image_rate{.25}, range{200}, height{.3};
    double self_velocity{.08}, acceleration{.06}, control{.03}, slew{.12};
    double attitude{12.5};  // Small-angle equivalent of quaternion weight50.
    double center_deadband{.08}, range_deadband{.3};
    bool enabled() const { return profile > 0; }
};
// 0: 끔. 1: V3 검증 묶음. 2..4: V3 비교 실험 묶음 (검증 범위 밖).
inline PampcPointSettings pampc_point_settings(int profile) {
    if (profile < 0 || profile > 4) {
        throw std::invalid_argument("PAMPC point profile must be 0..4");
    }
    PampcPointSettings result;
    result.profile = profile;
    if (profile >= 2) {
        result.center = 24;
    }
    if (profile == 3) {
        result.image_rate = 1;
    }
    if (profile == 4) {
        result.range = 80;
        result.height = .1;
        result.slew = .2;
    }
    return result;
}
}  // namespace follow
