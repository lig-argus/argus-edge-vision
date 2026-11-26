#include "argus/perception/geometry/include/attitude_interpolation.hpp"

#include <cassert>
#include <cmath>
#include <exception>
using namespace follow;
int main() try {
    constexpr std::int64_t kStart = 1000000000;
    const auto make = [](std::int64_t time, double yaw, std::size_t index) {
        return TimedAttitudeSample{time, time + 1000000, attitude_rotation(0., 0., yaw), index};
    };
    const double pi = std::acos(-1.);
    const auto left = make(kStart, 179. * pi / 180., 0);
    const auto right = make(kStart + 16000000, -179. * pi / 180., 1);
    auto result = interpolate_attitude({right, left}, kStart + 8000000, kStart + 20000000);
    if (!result) {
        return 1;
    }
    assert(result && std::abs(std::abs(result.value().rpy(2)) - pi) < 1e-10);
    assert(result.value().before_index == 0 && result.value().after_index == 1 &&
           result.value().fraction == .5);
    assert(result.value().endpoint_delta_ns == 8000000);
    result = interpolate_attitude({left, right}, kStart, kStart + 20000000);
    if (!result) {
        return 1;
    }
    assert(result && result.value().before_index == result.value().after_index);
    assert(!interpolate_attitude({left}, kStart + 8000000, kStart + 20000000));
    assert(!interpolate_attitude({right}, kStart + 8000000, kStart + 20000000));
    assert(!interpolate_attitude({left, right}, kStart + 8000000, kStart + 300000000));
    assert(!interpolate_attitude({left, right}, kStart + 8000000, kStart + 15000000));
    assert(!interpolate_attitude({left, make(kStart + 60000000, 0., 1)}, kStart + 30000000,
                                 kStart + 70000000));
    auto duplicate = left;
    duplicate.received_ns = kStart + 300000000;
    assert(!interpolate_attitude({duplicate, left, right}, kStart, kStart + 301000000));
    auto tilted_left = left;
    auto tilted_right = right;
    tilted_left.rotation = attitude_rotation(.12, -.18, .4);
    tilted_right.rotation = attitude_rotation(-.08, .25, .6);
    result = interpolate_attitude({tilted_left, tilted_right}, kStart + 8000000, kStart + 20000000);
    if (!result) {
        return 1;
    }
    assert(result);
    const auto expected = tilted_left.rotation.slerp(.5, tilted_right.rotation);
    const auto actual =
        attitude_rotation(result.value().rpy(0), result.value().rpy(1), result.value().rpy(2));
    assert(expected.angularDistance(actual) < 1e-10);
} catch (const std::exception&) {
    return 1;
}
