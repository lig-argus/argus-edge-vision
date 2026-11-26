// 2026-11-12 실외 장착각 모드 시험: 모드 이름 규칙, 실내 제한, 30°/45° 지면 투영.
// 합성 자세와 보정값만 쓴다 (카메라·FC 없음). 배포 프로필을 넘기면 모드 이름과 장착각 일치도 본다.
#include <array>
#include <cassert>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"

using namespace follow;

namespace {
constexpr double kPi = 3.14159265358979323846;

std::array<double, 9> mount(double down_deg) {
    const double c = std::cos(down_deg * kPi / 180.);
    const double s = std::sin(down_deg * kPi / 180.);
    return {0, -s, c, 1, 0, 0, 0, c, s};
}

CameraGroundCalibration calibration(double down_deg) {
    CameraGroundCalibration value;
    value.image_width = 640;
    value.image_height = 480;
    value.fx = 946.653562180606;
    value.fy = 853.010927704285;
    value.cx = 320;
    value.cy = 240;
    value.camera_to_body = mount(down_deg);
    value.camera_offset_body_m = {0.125, 0, -0.242};
    return value;
}

// 수평 자세, 높이 h에서 화면 세로 중앙이 닿는 지면 거리 (카메라 위치 보정 포함)
double center_range(double height_m, double down_deg) {
    return 0.125 + ((height_m + 0.242) / std::tan(down_deg * kPi / 180.));
}

double image_y(const CalibratedGroundGeometry& geometry, double height_m, double range_m) {
    Pose pose;
    pose.down_m = -height_m;
    const auto projected = geometry.image_projection(pose, {range_m, 0, 0});
    assert(projected.has_value());
    return projected.value_or(std::array<double, 2>{9, 9}).at(1);
}

void naming_test() {
    for (const char* name :
         {"predictive", "predictive_low", "predictive_m30", "predictive_low_m30"}) {
        assert(valid_follow_controller_profile(name));
    }
    assert(!valid_follow_controller_profile("pid"));
    assert(!valid_follow_controller_profile("predictive_30"));
    // 2026-11-18 실내 데모 모드: 이름은 유효, 45° 장착
    assert(valid_follow_controller_profile("demo"));
    assert(demo_follow_controller_profile("demo"));
    assert(!demo_follow_controller_profile("predictive"));
    assert(!mount30_follow_controller_profile("demo"));
    assert(!valid_follow_controller_profile("demo_m30"));
    assert(mount30_follow_controller_profile("predictive_m30"));
    assert(mount30_follow_controller_profile("predictive_low_m30"));
    assert(!mount30_follow_controller_profile("predictive_low"));
    std::cout << "PASS outdoor profile names" << '\n';
}

void geometry_test() {
    for (const double down : {30., 45.}) {
        const CalibratedGroundGeometry geometry(calibration(down));
        for (const double height : {3., 4.}) {
            const double range = center_range(height, down);
            assert(std::abs(image_y(geometry, height, range)) < 1e-6);
            // 더 가까운 점은 화면 아래(+), 더 먼 점은 위(-)
            assert(image_y(geometry, height, range - 1) > 0);
            assert(image_y(geometry, height, range + 1) < 0);
        }
    }
    // 30°: 높이 4 m -> 약 7.4 m(설정 6.9 m는 카메라 위치 보정 전 높이/tan30), 3 m -> 약 5.7 m
    assert(std::abs(center_range(4, 30) - 7.476) < 0.01);
    assert(std::abs(center_range(3, 30) - 5.743) < 0.01);
    std::cout << "PASS 30/45 degree ground projection" << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    try {
        naming_test();
        geometry_test();
        // 배포 프로필(materialize된 것): 모드 이름이 있고, 장착각이 이름과 맞아야 한다
        for (int i = 1; i < argc; ++i) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic) C++17 main 인자
            const auto config = load_bus_following_config(argv[i]);
            assert(valid_follow_controller_profile(config.follow_controller_profile));
            const auto& r = config.geometry.camera_to_body;
            const double tilt = std::atan2(r.at(8), r.at(2)) * 180. / kPi;
            const double expected =
                mount30_follow_controller_profile(config.follow_controller_profile) ? 30. : 45.;
            assert(std::abs(tilt - expected) < 1.);
        }
        std::cout << "PASS deployed profiles (" << (argc - 1) << ")\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
